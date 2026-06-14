import os
import threading
import time

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray, String
from ultralytics import YOLO

DEFAULT_MODEL = "best.pt"
FALLBACK_MODEL = "tennis_v2.pt"
# Smaller inference size — much faster on GPU; bear/knob still detectable at 416.
YOLO_IMGSZ = 416


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__("yolo_detection_node")

        self.bridge = CvBridge()
        self.latest_depth_image_raw = None
        self.latest_depth_image_compressed = None

        share = get_package_share_directory("yolo_example_pkg")
        model_path = os.path.join(share, "models", DEFAULT_MODEL)
        if not os.path.exists(model_path):
            self.get_logger().warn(
                f"{DEFAULT_MODEL} not found, falling back to {FALLBACK_MODEL}"
            )
            model_path = os.path.join(share, "models", FALLBACK_MODEL)

        force_cpu = os.environ.get("YOLO_FORCE_CPU", "0").strip().lower()
        if force_cpu in ("1", "true", "yes", "on"):
            device = "cpu"
        else:
            device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"[yolo] Using device: {device}", flush=True)

        self.conf_threshold = 0.5
        self.x_num_splits = 20

        self.model = YOLO(model_path)
        self._device = device
        # Let Ultralytics handle FP16 via half=True at predict time only.
        # Calling model.half() manually causes Half vs float mismatches on YOLO26.
        self._use_half = device == "cuda"
        if device == "cuda":
            torch.backends.cudnn.benchmark = True

        # Warm up CUDA kernels so the first real frame is not a multi-second stall.
        try:
            dummy = np.zeros((YOLO_IMGSZ, YOLO_IMGSZ, 3), dtype=np.uint8)
            self._run_inference(dummy)
            print("[yolo] warmup complete", flush=True)
        except Exception as e:
            print(f"[yolo] warmup failed ({e}) — retrying without half", flush=True)
            self._use_half = False
            try:
                self._run_inference(dummy)
                print("[yolo] warmup complete (fp32)", flush=True)
            except Exception as e2:
                print(f"[yolo] warmup skipped: {e2}", flush=True)

        self._frame_lock = threading.Lock()
        self._pending_frame = None
        self._infer_stop = threading.Event()
        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()
        self._fps_count = 0
        self._fps_t = time.time()

        self.image_sub = self.create_subscription(
            CompressedImage, "/camera/image/compressed", self.image_callback, 1
        )
        self.depth_sub_raw = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback_raw, 1
        )
        self.depth_sub_compressed = self.create_subscription(
            CompressedImage,
            "/camera/depth/compressed",
            self.depth_callback_compressed,
            1,
        )

        self.image_pub = self.create_publisher(
            CompressedImage, "/yolo/detection/compressed", 10
        )
        self.target_pub = self.create_publisher(
            Float32MultiArray, "/yolo/target_info", 10
        )
        self.x_multi_depth_pub = self.create_publisher(
            Float32MultiArray, "/camera/x_multi_depth_values", 10
        )

        self.allowed_labels = set()
        self.target_label_sub = self.create_subscription(
            String, "/target_label", self.target_label_callback, 10
        )

    def target_label_callback(self, msg):
        raw = (msg.data or "").strip()
        if not raw or raw.lower() in ("any", "all", "*"):
            self.allowed_labels = set()
            self.get_logger().info("[target_label] cleared -- matching any class")
            return
        labels = {tok.strip().lower() for tok in raw.split(",") if tok.strip()}
        self.allowed_labels = labels
        self.get_logger().info(f"[target_label] now filtering to {labels}")

    def depth_callback_raw(self, msg):
        try:
            self.latest_depth_image_raw = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
        except Exception as e:
            self.get_logger().error(f"Could not convert raw depth image: {e}")

    def depth_callback_compressed(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            depth_img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            if depth_img is not None:
                self.latest_depth_image_compressed = depth_img
        except Exception as e:
            self.get_logger().error(f"Could not convert compressed depth image: {e}")

    def image_callback(self, msg):
        """Queue the latest camera frame; inference runs on a worker thread."""
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        with self._frame_lock:
            self._pending_frame = cv_image

    def _run_inference(self, frame):
        kw = dict(conf=self.conf_threshold, verbose=False, imgsz=YOLO_IMGSZ, device=self._device)
        if self._use_half:
            kw["half"] = True
        return self.model(frame, **kw)

    def _infer_loop(self):
        """Always process the newest frame only — never backlog behind Unity."""
        while not self._infer_stop.is_set():
            frame = None
            with self._frame_lock:
                if self._pending_frame is not None:
                    frame = self._pending_frame
                    self._pending_frame = None
            if frame is None:
                time.sleep(0.005)
                continue

            try:
                results = self._run_inference(frame)
            except Exception as e:
                if self._use_half:
                    self.get_logger().warn(f"half inference failed ({e}), falling back to fp32")
                    self._use_half = False
                    try:
                        results = self._run_inference(frame)
                    except Exception as e2:
                        self.get_logger().error(f"Error during YOLO detection: {e2}")
                        continue
                else:
                    self.get_logger().error(f"Error during YOLO detection: {e}")
                    continue

            processed_image = self.draw_bounding_boxes(frame, results)
            self.publish_x_multi_depths(processed_image)
            self.publish_image(processed_image)

            self._fps_count += 1
            now = time.time()
            if now - self._fps_t >= 5.0:
                hz = self._fps_count / (now - self._fps_t)
                print(f"[yolo] inference ~{hz:.1f} Hz ({self._device})", flush=True)
                self._fps_count = 0
                self._fps_t = now

    def draw_cross(self, image):
        height, width = image.shape[:2]
        cx_center = width // 2
        cy_center = height // 2
        cv2.line(image, (0, cy_center), (width, cy_center), (0, 0, 255), 2)
        cv2.line(
            image,
            (cx_center, cy_center - 10),
            (cx_center, cy_center + 10),
            (0, 0, 255),
            2,
        )

        segment_length = width // self.x_num_splits
        points = [
            (i * segment_length, cy_center) for i in range(self.x_num_splits + 1)
        ]
        for x, y in points:
            cv2.line(image, (x, y - 10), (x, y + 10), (0, 0, 0), 2)

        return image, points

    def draw_bounding_boxes(self, image, results):
        """Draw detections; lock /yolo/target_info to the closest filtered target."""
        found_target = 0
        target_distance = 0.0
        delta_x = 0.0
        det_image = image.copy()
        overlay, points = self.draw_cross(det_image)
        img_cx = image.shape[1] / 2
        cy_center = image.shape[0] // 2
        img_cx_int = int(img_cx)

        candidates = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf)
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                if self.allowed_labels and class_name.lower() not in self.allowed_labels:
                    continue

                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                depth_value = self.get_depth_at(cx, cy)
                candidates.append(
                    (x1, y1, x2, y2, conf, class_name, cx, cy, depth_value)
                )

        locked = None
        if candidates:
            def _depth_key(c):
                depth = c[8]
                if depth == -1.0:
                    return 0.0
                if depth and depth > 0:
                    return depth
                return 1e6

            candidates.sort(key=_depth_key)
            locked = candidates[0]
            found_target = 1
            target_distance = locked[8] if locked[8] else 0.0
            delta_x = locked[6] - img_cx

        for c in candidates:
            x1, y1, x2, y2, conf, class_name, cx, cy, depth_value = c
            is_locked = locked is not None and c is locked
            depth_text = f"{depth_value:.2f}m" if depth_value and depth_value > 0 else "N/A"
            box_color = (0, 255, 0) if is_locked else (0, 165, 255)
            thickness = 3 if is_locked else 1

            cv2.rectangle(overlay, (x1, y1), (x2, y2), box_color, thickness)
            label = f"{class_name} {conf:.2f} Depth: {depth_text}"
            if is_locked:
                label += " [LOCK]"
            cv2.putText(
                overlay,
                label,
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                box_color,
                2,
            )

        if locked is not None:
            cx = locked[6]
            if abs(delta_x) <= 5:
                dx_color = (0, 255, 0)
            elif abs(delta_x) <= 20:
                dx_color = (255, 165, 0)
            else:
                dx_color = (255, 0, 255)
            cv2.circle(overlay, (cx, cy_center), 12, (0, 255, 255), 3)
            cv2.arrowedLine(
                overlay,
                (img_cx_int, cy_center),
                (cx, cy_center),
                dx_color,
                2,
                tipLength=0.05,
            )
            mid_x = (img_cx_int + cx) // 2
            cv2.putText(
                overlay,
                f"dx={delta_x:.0f}px",
                (max(0, mid_x - 50), cy_center - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                dx_color,
                2,
            )

        self.publish_target_info(found_target, target_distance, delta_x)
        return overlay

    def get_depth_at(self, x, y):
        depth_image = (
            self.latest_depth_image_raw
            if self.latest_depth_image_raw is not None
            else self.latest_depth_image_compressed
        )
        if depth_image is None:
            return -1.0
        if len(depth_image.shape) == 3:
            depth_image = depth_image[:, :, 0]
        try:
            depth_value = depth_image[y, x]
            if depth_value < 0.0001 or depth_value == 0.0:
                return -1.0
            return depth_value / 1000.0
        except IndexError:
            return -1.0

    def publish_image(self, image):
        try:
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(image)
            self.image_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Could not publish image: {e}")

    def publish_target_info(self, found, distance, delta_x):
        msg = Float32MultiArray()
        msg.data = [float(found), float(distance), float(delta_x)]
        self.target_pub.publish(msg)

    def publish_x_multi_depths(self, image):
        height, width = image.shape[:2]
        cy_center = height // 2
        segment_length = width // self.x_num_splits
        points = [(i * segment_length, cy_center) for i in range(self.x_num_splits)]
        depth_values = [self.get_depth_at(x, cy_center) for x, _ in points]
        depth_msg = Float32MultiArray()
        depth_msg.data = depth_values
        self.x_multi_depth_pub.publish(depth_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node._infer_stop.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
