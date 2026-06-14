import rclpy
from rclpy.executors import MultiThreadedExecutor
from yolo_pkg.ros_communicator import RosCommunicator
from yolo_pkg.image_processor import ImageProcessor
from yolo_pkg.yolo_depth_extractor import YoloDepthExtractor
from yolo_pkg.yolo_bounding_box import YoloBoundingBox
from yolo_pkg.boundingbox_visaulizer import BoundingBoxVisualizer
from yolo_pkg.camera_geometry import CameraGeometry
import math
import threading
import time
from std_msgs.msg import String, Float32MultiArray
from yolo_pkg.load_params import LoadParams

# Cap how often we run the detect/draw/publish pipeline. The Unity camera
# streams at ~30 fps; processing faster than that just re-decodes and
# re-publishes the SAME frame, which pegs the CPU and floods the Foxglove
# WebSocket (visible as lag). 20 fps is smooth for visualization and leaves
# CPU headroom for the ROS executor that ingests fresh camera frames.
TARGET_FPS = 20.0
MIN_PERIOD = 1.0 / TARGET_FPS



def _init_ros_node():
    """
    Initialize the ROS 2 node with MultiThreadedExecutor for efficient handling of multiple subscribers.
    """
    rclpy.init()
    node = RosCommunicator()  # Initialize the ROS node
    executor = MultiThreadedExecutor()  # Use MultiThreadedExecutor
    executor.add_node(node)  # Add the node to the executor
    thread = threading.Thread(
        target=executor.spin
    )  # Start the executor in a separate thread
    thread.start()
    return node, executor, thread  # Return the node, executor, and thread


def menu():
    print("Select mode:")
    print("1: Draw bounding boxes without screenshot.")
    print("2: Draw bounding boxes with screenshot.")
    print("3: 5 fps screenshot.")
    print("4: segmentation.")
    print("Press Ctrl+C to exit.")

    user_input = input("Enter your choice (1/4): ")
    return user_input


def main():
    """
    Main function to initialize the node and run the bounding box visualizer.
    """
    load_params = LoadParams("yolo_pkg")
    ros_communicator, executor, ros_thread = _init_ros_node()
    image_processor = ImageProcessor(ros_communicator, load_params)
    yolo_boundingbox = YoloBoundingBox(image_processor, load_params)
    yolo_depth_extractor = YoloDepthExtractor(
        yolo_boundingbox, image_processor, ros_communicator
    )
    boundingbox_visualizer = BoundingBoxVisualizer(
        image_processor, yolo_boundingbox, ros_communicator
    )
    camera_geometry = CameraGeometry(yolo_depth_extractor)

    user_input = menu()

    try:
        last_rgb_msg = None
        last_hb_time = time.time()
        frames_since_hb = 0
        HB_PERIOD = 2.0  # seconds between status heartbeats
        while True:
            loop_start = time.time()

            # Heartbeat so the node is observable in `docker logs`: report the
            # input frame rate, or warn loudly when no camera frames are arriving
            # (the usual cause of "it looks like nothing is happening").
            if loop_start - last_hb_time >= HB_PERIOD:
                if frames_since_hb > 0:
                    fps_in = frames_since_hb / (loop_start - last_hb_time)
                    print(f"[yolo] running — {fps_in:.1f} fps in, publishing /yolo/detection/compressed", flush=True)
                else:
                    print("[yolo] waiting for camera frames on /camera/image/compressed "
                          "(none yet — is Unity connected to rosbridge?)", flush=True)
                last_hb_time = loop_start
                frames_since_hb = 0

            # Skip the whole pipeline unless a NEW camera frame has arrived.
            # Each subscriber callback replaces the cached message object, so an
            # identity check ('is') tells us whether the frame is actually new.
            # Without this we re-decode + re-encode + republish the same image
            # at full CPU speed, which is the main cause of the Foxglove lag.
            current_rgb_msg = ros_communicator.get_latest_data("rgb_compress")
            if current_rgb_msg is None or current_rgb_msg is last_rgb_msg:
                time.sleep(0.005)
                continue
            last_rgb_msg = current_rgb_msg
            frames_since_hb += 1

            if user_input == "1":
                offsets_3d = camera_geometry.calculate_offset_from_crosshair_2d()
                boundingbox_visualizer.draw_bounding_boxes(
                    draw_crosshair=True,
                    screenshot=False,
                    segmentation_status=False,
                    bounding_status=True,
                    offsets_3d_json=offsets_3d,
                )

                offset_msg = String()
                offset_msg.data = offsets_3d
                ros_communicator.publish_data("object_offset", offset_msg)

                # Publish /yolo/target_info [detected, dist_m, dx_px] so that
                # auto_task.py search/align/drive can see detections.
                # get_yolo_object_depth() reuses the bounding-box cache (40 ms TTL)
                # so YOLO does not run again; only the depth lookup is repeated.
                cx = camera_geometry.camera_intrinsics.get("cx", 320.0)
                yolo_objs = yolo_depth_extractor.get_yolo_object_depth()
                if yolo_objs:
                    obj = yolo_objs[0]
                    x1, y1, x2, y2 = obj["box"]
                    dx = (x1 + x2) / 2.0 - cx   # positive = object to right of centre
                    raw_d = obj.get("depth", -1.0)
                    dist = (float(raw_d)
                            if (raw_d is not None and raw_d > 0 and not math.isnan(float(raw_d)))
                            else -1.0)
                    tgt_data = [1.0, dist, dx]
                else:
                    tgt_data = [0.0, -1.0, 0.0]
                tgt_msg = Float32MultiArray()
                tgt_msg.data = tgt_data
                ros_communicator.publish_data("yolo_target_info", tgt_msg)

            elif user_input == "2":
                boundingbox_visualizer.draw_bounding_boxes(
                    draw_crosshair=True,
                    screenshot=True,
                    segmentation_status=False,
                    bounding_status=True,
                )
            elif user_input == "3":
                boundingbox_visualizer.draw_bounding_boxes(
                    draw_crosshair=True,
                    screenshot=False,
                    segmentation_status=False,
                    bounding_status=True,
                )

                # store 5fps unity camera picture
                boundingbox_visualizer.save_fps_screenshot()

            elif user_input == "4":
                boundingbox_visualizer.draw_bounding_boxes(
                    draw_crosshair=True,
                    screenshot=False,
                    segmentation_status=True,
                    bounding_status=False,
                )
            else:
                print("Invalid input.")

            # Example action for yolo_depth_extractor (can be removed if not needed)
            # depth_data = yolo_depth_extractor.get_yolo_object_depth()
            # print(f"Object Depth: {depth_data}")

            # Hold the loop to at most TARGET_FPS so we never publish faster
            # than the camera produces frames.
            elapsed = time.time() - loop_start
            if elapsed < MIN_PERIOD:
                time.sleep(MIN_PERIOD - elapsed)

    except KeyboardInterrupt:
        print("Shutting down gracefully...")
    finally:
        # Shut down the executor and ROS
        executor.shutdown()
        rclpy.shutdown()
        ros_thread.join()


if __name__ == "__main__":
    main()
