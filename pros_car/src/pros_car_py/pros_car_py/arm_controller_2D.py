# return 角度一律 radians
import math
import tf2_ros
import tf2_geometry_msgs
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from pros_car_py.car_models import DeviceDataTypeEnum
import numpy as np
import time
import sys
from scipy.spatial.transform import Rotation as R
import pybullet as p
import numpy as np
import threading
import rclpy


class ArmController:
    def __init__(self, ros_communicator, data_processor):
        self.ros_communicator = ros_communicator
        self.data_processor = data_processor
        self.target_marker = None
        
        # Lazy TF listener — AutoTask ground-grasp never needs TF. Eagerly subscribing
        # to /tf made pros_car spam "TF_OLD_DATA" warnings when Unity's odom stamps
        # arrive slightly behind wall-clock over rosbridge (harmless but noisy).
        self.tf_buffer = None
        self.tf_listener = None
        
        # ==========================================
        # 1. 手臂基礎與機構設定 (統一管理區)
        # ==========================================
        self.base_link_name = 'arm_ik_base' # 第一個馬達的基準座標系
        
        # 動態定義所有關節 (加入 length 臂長 與 angle_offset 角度補償)
        # angle_offset: IK 算出來的數學 0 度可能不是 Unity 的 0 度，可透過這個補償
        self.joint_limits = [
            {"length": 0.08089007, "min_angle": -180, "max_angle": 0, "init": -146, "offset": 270, "dir": -1.0},  # Joint 0 (Shoulder)
            {"length": 0.11, "min_angle": -240, "max_angle": 0, "init": -70,   "offset": -120, "dir": -1.0},  # Joint 1 (Elbow)
            {"length": 0.00, "min_angle": 20, "max_angle": 90,  "init": 90,  "offset": 0.0, "dir": 1.0},  # Joint 2 (Gripper)
        ]
        
        self.joint_angles = [joint["init"] for joint in self.joint_limits]
        self.manual_step = 3.0

        # Unity HOLDS the last arm pose, and single publishes can be dropped over
        # the bridge (same failure mode as the wheel commands). A one-shot publish
        # like open_gripper can be lost, leaving the gripper stuck closed in Unity.
        # This heartbeat first retries the init pose for 5s (so rosbridge has time
        # to subscribe), then re-publishes the CURRENT joint angles every 0.3s
        # forever, so Unity always converges to and holds whatever pose was last
        # commanded. Re-reading self.joint_angles means in-progress smooth moves
        # are tracked automatically.
        self._arm_hb_stop = False
        def _arm_heartbeat():
            for _ in range(5):
                time.sleep(1.0)
                self._clamp_and_publish()
            print("[Arm] Init position published.")
            while not self._arm_hb_stop:
                self._clamp_and_publish()
                time.sleep(0.3)
        threading.Thread(target=_arm_heartbeat, daemon=True).start()
        print(f"[Arm] Controller initialized: {len(self.joint_limits)} joints managed.")

    def _ensure_tf_listener(self):
        """Create TF buffer/listener on first use (manual IK grasp only)."""
        if self.tf_buffer is not None:
            return
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=30.0)
        )
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self.ros_communicator
        )

    # ==========================================
    # 2. 手動控制邏輯 (Manual Control)
    # ==========================================
    def manual_control(self, index, key):
        """處理手動按鍵輸入，並根據 index 控制特定關節"""
        
        # 處理不依賴 index 的全域指令 ('b', 'q')
        if key == "b":
            # Reset arm to the init angles defined in __init__.
            self.joint_angles = [joint["init"] for joint in self.joint_limits]
            self._clamp_and_publish()
            self._visualize_arm_lines()
            print("[Arm] Reset to initial angles.")
            return False

        elif key == "q":
            print("[Arm] Exiting manual control.")
            return True

        # Per-joint control ('i' / 'k')
        if 0 <= index < len(self.joint_limits):
            if key == "i":
                self.joint_angles[index] += self.manual_step
            elif key == "k":
                self.joint_angles[index] -= self.manual_step
            else:
                print(f"[Arm] Key '{key}' invalid. Use 'i' (+), 'k' (-), 'b' (reset), 'q' (exit).")
                return False

            self._clamp_and_publish()
            self._visualize_arm_lines()
        else:
            print(f"[Arm] Index {index} out of range (valid 0-{len(self.joint_limits) - 1}).")
            
        return False

    def auto_control(self, key=None, mode="auto_arm_control"):
        """Automatically grasp whatever is at /yolo/target_marker."""

        # 1. Latest target position.
        target_marker = self.ros_communicator.latest_yolo_marker
        if not target_marker:
            print("[Arm] No YOLO target received yet, waiting...")
            return
        
        if key == "g":
            self.target_marker = target_marker
        elif key == "q":
            self.target_marker = None
            return
        elif key == "b":
            self.joint_angles = [joint["init"] for joint in self.joint_limits]
            self._clamp_and_publish()
            self._visualize_arm_lines()
            print("[Arm] Reset to initial angles.")
            return
        else:
            print(f"[Arm] Key '{key}' invalid. Use 'g' (grasp), 'b' (reset), 'q' (cancel).")
            return

        # 2. 建立目標的 PointStamped (原本在 map 座標系)
        target_map = PointStamped()
        target_map.header.frame_id = target_marker.header.frame_id # 通常是 'map'
        target_map.header.stamp = self.ros_communicator.get_clock().now().to_msg()
        target_map.point = target_marker.pose.position

        try:
            self._ensure_tf_listener()
            # 3. 將 map 上的網球，轉換到手臂基準座標系
            transform = self.tf_buffer.lookup_transform(
                self.base_link_name,
                target_map.header.frame_id,
                rclpy.time.Time()
            )
            target_base = tf2_geometry_msgs.do_transform_point(target_map, transform)
            
            x_target = target_base.point.x
            z_target = target_base.point.z
            
            print(f"[Arm] Target relative to base: X={x_target:.3f}, Z={z_target:.3f}")

            # 🌟 4. 開啟背景執行緒，執行「抓取與緩慢歸位」的完整排程
            # 使用 daemon=True 確保程式關閉時執行緒會自動結束
            threading.Thread(
                target=self._execute_grab_sequence, 
                args=(x_target, z_target), 
                daemon=True
            ).start()

        except Exception as e:
            print(f"[Arm] Coordinate transform / TF failed: {e}")
    
    def open_gripper(self):
        """Open the gripper to release a held object."""
        print("[Arm] Opening gripper to release...")
        open_pose = [None, None, self.joint_limits[2]["max_angle"]]
        self._smooth_move_to(open_pose, step=5.0, delay=0.1)
        print("[Arm] Gripper open.")

    def set_door_push_pose(self):
        """Position arm at door knob: Elbow=0°, Wrist=197°, Finger=90°."""
        print("[Arm] Positioning at door...")
        # shoulder=0 → Unity Elbow=0°, elbow=-197 → Unity Wrist=197°, finger=90° open
        self._smooth_move_to([0.0, -197.0, 90.0], step=3.0, delay=0.1)
        print("[Arm] Door position ready.")

    def push_door(self):
        """Close finger, then rotate shoulder to Elbow=100° to push the door open."""
        print("[Arm] Closing finger before push...")
        self._smooth_move_to([None, None, self.joint_limits[2]["min_angle"]], step=5.0, delay=0.1)
        time.sleep(0.3)
        print("[Arm] Pushing door (Elbow 0→100°)...")
        self._smooth_move_to([-100.0, None, None], step=2.0, delay=0.1)
        print("[Arm] Door pushed.")

    def execute_ground_grasp_sequence(self, shoulder_deg=-90.0, elbow_deg=-200.0, pre_close_fn=None):
        """IK-free grasp aimed at ground-level targets.

        The arm's geometry (L1=0.08m + L2=0.11m, total reach ~0.19m) can't
        produce a valid 2D IK solution for objects on the ground when the
        arm base is mounted on top of the car body (~0.30m up). The IK
        math returns out-of-limit angles, the joints clamp to their stops,
        and the gripper closes in mid-air -- never touching the bear.

        Instead, drive the arm to a fixed "reach down and forward" pose
        that places the gripper as close to ground level as the joints
        allow, then close the gripper. Unity uses a FixedJoint snap when
        the gripper closes near a graspable object, so getting the gripper
        within snap range is what matters -- exact end-effector position
        doesn't need to be IK-correct.

        Defaults are conservative starting points; tune per scene:
          shoulder_deg = -90:  swing the shoulder 90 deg from init (-180)
                                so the arm is pointing roughly forward.
          elbow_deg    = -200: fold the elbow so the forearm tips downward.
        Both clamp to joint_limits in _smooth_move_to.
        """
        print(f"[Arm] ground-grasp: shoulder={shoulder_deg} elbow={elbow_deg}")

        # Step 0: force-sync arm to init position.
        # joint_angles may already equal init in Python but Unity could be out of sync
        # (e.g. after a restart where the startup publish was lost). Directly set and
        # publish so Unity is guaranteed to be at the correct starting pose.
        print("[Arm 0/5] Syncing to start position...")
        self.joint_angles = [self.joint_limits[i]["init"] for i in range(len(self.joint_limits))]
        self._clamp_and_publish()
        time.sleep(1.0)

        # Step 1: open the gripper.
        print("[Arm 1/5] Opening gripper...")
        open_pose = [None, None, self.joint_limits[2]["max_angle"]]
        self._smooth_move_to(open_pose, step=5.0, delay=0.1)
        time.sleep(0.5)

        # Step 2: reach toward the ground.
        print("[Arm 2/4] Reaching toward ground (no IK)...")
        reach_pose = [shoulder_deg, elbow_deg, None]
        self._smooth_move_to(reach_pose, step=3.0, delay=0.1)
        time.sleep(0.8)  # let the arm settle in front of the bear

        # Step 3: optional crawl to push gripper into contact before closing
        if pre_close_fn is not None:
            print("[Arm 3/4] Crawling into bear...")
            pre_close_fn()

        # Step 4: close on the bear.
        print("[Arm 4/5] Closing gripper...")
        close_pose = [None, None, self.joint_limits[2]["min_angle"]]
        self._smooth_move_to(close_pose, step=5.0, delay=0.1)
        time.sleep(1.0)  # let Unity FixedJoint snap

        # Step 5: lift back to init so the snapped bear travels with us.
        print("[Arm 5/5] Lifting + returning home...")
        lift_pose = [None, self.joint_limits[1]["init"], None]
        self._smooth_move_to(lift_pose, step=3.0, delay=0.1)
        home_pose = [self.joint_limits[0]["init"], None, None]
        self._smooth_move_to(home_pose, step=3.0, delay=0.1)
        print("[Arm] Ground-grasp complete.")

    def _execute_grab_sequence(self, x_target, z_target):
        """Full grasp sequence executed on a background thread."""

        # Step 1: open the gripper (only Joint 2 moves; faster step=5.0)
        print("[Arm 1/4] Opening gripper...")
        target_open = [None, None, self.joint_limits[2]["max_angle"]]
        self._smooth_move_to(target_open, step=5.0, delay=0.1)
        time.sleep(0.5)

        # Step 2: move arm to target via 2D IK
        print("[Arm 2/4] Moving to target...")
        deg1, deg2 = self._calculate_2d_ik(x_target, z_target)
        self._smooth_move_to([deg1, deg2, None], step=5.0, delay=0.1)
        time.sleep(0.5)

        # Step 3: close the gripper on the target
        print("[Arm 3/4] Closing gripper...")
        target_close = [None, None, self.joint_limits[2]["min_angle"]]
        self._smooth_move_to(target_close, step=5.0, delay=0.1)
        time.sleep(1.0)  # let Unity's FixedJoint snap

        # Step 4: slowly return to home pose
        print("[Arm 4/4] Returning to home pose...")
        init_angles = [None, self.joint_limits[1]["init"], None]
        self._smooth_move_to(init_angles, step=5.0, delay=0.1)
        init_angles = [self.joint_limits[0]["init"], None, None]
        self._smooth_move_to(init_angles, step=5.0, delay=0.1)

        print("[Arm] Grasp sequence complete.")

    def _calculate_2d_ik(self, x, z):
        """2D inverse kinematics. Returns (shoulder_deg, elbow_deg)."""
        L1 = self.joint_limits[0]["length"]
        L2 = self.joint_limits[1]["length"]

        D = math.sqrt(x**2 + z**2)
        if D > (L1 + L2):
            print("[Arm] Target beyond max reach -- using fully extended pose.")
            D = L1 + L2 - 0.001
        
        cos_theta2 = (D**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2)) 
        
        # 由上往下夾 (Elbow Up)
        theta2_rad = -math.acos(cos_theta2) 
        
        alpha = math.atan2(z, x)
        beta = math.acos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
        theta1_rad = alpha + beta 
        
        # 轉換為 Degrees 並加上偏移量
        deg1 = math.degrees(theta1_rad) + self.joint_limits[0]["offset"]
        deg2 = math.degrees(theta2_rad) + self.joint_limits[1]["offset"]
        
        deg1 = self._normalize_angle(deg1, self.joint_limits[0]["min_angle"], self.joint_limits[0]["max_angle"])
        deg2 = self._normalize_angle(deg2, self.joint_limits[1]["min_angle"], self.joint_limits[1]["max_angle"])
        
        print(f"[Arm] IK solved: shoulder={deg1:.1f} deg, elbow={deg2:.1f} deg")
        return deg1, deg2
    
    def _smooth_move_to(self, target_angles, step=2.0, delay=0.05):
        """
        Smoothly move the arm to a target pose (linear joint interpolation).
        - target_angles: [j0_target, j1_target, j2_target]  (None = leave joint alone)
        - step:          max degrees per tick (smaller = smoother)
        - delay:         seconds between ticks (larger = slower)

        Safety: clamp each non-None target into the joint's physical limit
        before iterating, otherwise the loop can spin forever when IK asks
        for an unreachable angle (the joint stays clamped at the limit,
        the diff stays huge, all_reached never flips True).
        """
        clamped = list(target_angles)
        for i in range(len(self.joint_limits)):
            if clamped[i] is None:
                continue
            lo = self.joint_limits[i]["min_angle"]
            hi = self.joint_limits[i]["max_angle"]
            if clamped[i] < lo or clamped[i] > hi:
                print(
                    f"[ArmController] joint {i} target {clamped[i]:.1f} "
                    f"out of [{lo}, {hi}] -- clamping."
                )
                clamped[i] = max(lo, min(hi, clamped[i]))

        # Hard cap on iterations so we can never freeze the autotask, even
        # if some other invariant breaks.
        MAX_ITERS = 1500  # 1500 * delay = ~75s worst case at default delay
        iters = 0
        while iters < MAX_ITERS:
            iters += 1
            all_reached = True

            for i in range(len(self.joint_angles)):
                if clamped[i] is None:
                    continue
                diff = clamped[i] - self.joint_angles[i]
                if abs(diff) <= step:
                    self.joint_angles[i] = clamped[i]
                else:
                    self.joint_angles[i] += step if diff > 0 else -step
                    all_reached = False

            self._clamp_and_publish()
            self._visualize_arm_lines()

            if all_reached:
                return
            time.sleep(delay)

        print(
            f"[ArmController] _smooth_move_to hit MAX_ITERS={MAX_ITERS}; "
            f"bailing to avoid freezing the autotask."
        )

    def _normalize_angle(self, angle, min_limit, max_limit):
        """
        嘗試加減 360 度，尋找是否有多轉或少轉一圈後，
        剛好能落入 [min_limit, max_limit] 物理極限內的同界角。
        """
        # 1. 將角度正規化到 0 ~ 360 的基準
        base_angle = angle % 360.0
        
        # 2. 準備三個候選角度：少一圈、當前(0~360)、多一圈
        candidates = [base_angle - 360.0, base_angle, base_angle + 360.0]
        
        # 3. 檢查哪一個落在合法範圍內
        for cand in candidates:
            if min_limit <= cand <= max_limit:
                return cand # 找到合法的同界角，直接回傳！
                
        # 如果加減 360 度後都不在範圍內，代表這個姿勢真的超出了手臂極限。
        # 我們先回傳原始角度，後續交給 _clamp_and_publish 去強制卡在邊界防呆。
        return angle
    # ==========================================
    # 5. 視覺化手臂 (Foxglove Lines)
    # ==========================================
    def _visualize_arm_lines(self):
        """根據當前的角度和長度，算出 3 個點的座標並發布 3D 視覺化線條"""
        L1 = self.joint_limits[0]["length"]
        L2 = self.joint_limits[1]["length"]
        
        # 扣除補償值，轉回純數學弧度，方便做正向運動學(FK)
        th1 = math.radians(self.joint_angles[0] - self.joint_limits[0]["offset"])
        th2 = math.radians(self.joint_angles[1] - self.joint_limits[1]["offset"])
        
        # 基座位置 P0
        p0 = Point(x=0.0, y=0.0, z=0.0)
        # 關節1位置 P1
        p1 = Point(x=L1 * math.cos(th1), y=0.0, z=L1 * math.sin(th1))
        # 夾爪末端位置 P2
        p2 = Point(x=p1.x + L2 * math.cos(th1 + th2), y=0.0, z=p1.z + L2 * math.sin(th1 + th2))
        
        # 建立 Marker
        marker = Marker()
        marker.header.frame_id = self.base_link_name
        marker.header.stamp = self.ros_communicator.get_clock().now().to_msg()
        marker.ns = "arm_kinematics"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.scale.x = 0.02 # 線條粗細
        marker.color.a = 1.0  # 不透明度
        marker.color.r = 0.0  # 青藍色
        marker.color.g = 1.0
        marker.color.b = 1.0
        
        marker.points = [p0, p1, p2]
        
        self.ros_communicator.publish_arm_visual_lines(marker)

    def _clamp_and_publish(self):
        """確保所有數值在安全範圍內，並轉換為「弧度」後發布"""
        for i in range(len(self.joint_limits)):
            min_a = self.joint_limits[i]["min_angle"]
            max_a = self.joint_limits[i]["max_angle"]
            self.joint_angles[i] = max(min_a, min(max_a, self.joint_angles[i]))
            
        joint_pos_radians = [
            math.radians(float(self.joint_angles[i]) * self.joint_limits[i].get("dir", 1.0)) 
            for i in range(len(self.joint_angles))
        ]
        self.ros_communicator.publish_robot_arm_angle(joint_pos_radians)