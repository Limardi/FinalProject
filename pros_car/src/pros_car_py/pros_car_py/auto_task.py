"""
AutoTaskController — full competition run (T1 + T2 + T3)

Flow:
  search → align → drive → observe(5s) → grasp → return
  → bridge(ascent/descent) → search → grasp → bridge_return → return
  → door → observe(5s) → unlock → search → grasp → return → finished
"""

import math
import threading
import time

from pros_car_py.waypoints import BRIDGE_APPROACH, BRIDGE_BEFORE, BRIDGE_TOP, BRIDGE_AFTER, DOOR, DOOR_VIA

# ── Tuning ────────────────────────────────────────────────────────────────────
# Act only on NEW /yolo/target_info frames; between frames replay the last command.
YOLO_LOOP_SLEEP_S = 0.05
NAV_WAIT_S     = 1.5    # max wait for fresh AMCL pose before acting anyway
LOST_GRACE_S   = 3.0    # tolerate slow YOLO before declaring target lost
CONFIRM_FRAMES = 1      # one detection is enough when YOLO runs at ~5-10 Hz

YOLO_WAIT_PHASES = frozenset({"search", "align", "drive", "observe"})
NAV_PHASES  = frozenset({"return", "bridge", "bridge_return", "door"})

# HW4 custom_nav uses FORWARD_SLOW / rotation at 20 Hz.
FWD_CMD        = "FORWARD_SLOW"
ROT_CMD        = "CLOCKWISE_ROTATION_SLOW"
ROT_CMD_CCW    = "COUNTERCLOCKWISE_ROTATION_SLOW"
ROT_CMD_MED    = "CLOCKWISE_ROTATION_MEDIAN"
ROT_CMD_CCW_MED = "COUNTERCLOCKWISE_ROTATION_MEDIAN"

# Rotation pulse: (on_ticks, cycle_ticks). Lower RPM needs longer ON bursts.
ROT_PULSE_SEARCH       = (2, 8)   # 250 RPM, 2 on / 6 off
ROT_PULSE_ALIGN_COARSE = (2, 8)   # 275 RPM, |dx| > 25
ROT_PULSE_ALIGN_FINE   = (1, 6)   # 250 RPM, |dx| <= 25
ROT_PULSE_DRIVE        = (1, 5)   # heading fix while driving
ROT_PULSE_RETURN_CLOSE = (1, 4)   # gentle rotation during final home approach

# Centre first (±5 px), then drive forward. Grasp when close enough.
CENTER_DEADBAND = 15.0   # px — |dx| must be within this before FORWARD
STOP_DIST       = 0.3   # m  — stop and grasp when this close
DEPTH_LIMIT     = 0.7   # m  — forward path clear (camera_nav)

# Terminal output — log state at most once per interval (IGVI logs per stage, not per tick).
STATE_LOG_INTERVAL_S = 2.0

GRASP_SETTLE_S         = 6.0   # Unity arm animation lag (IGVI uses 1s + joint_states)
GRASP_CHECK_TIMEOUT_S  = 4.0   # wait for one fresh YOLO frame before deciding
GRASP_ON_GROUND_M      = 0.55  # bear still this far → on ground (missed)
MAX_GRASP_ATTEMPTS     = 2     # IGVI default max_grasp_attempts

# Unity scoring — must hold target in view this long (Task 1 & Task 3 observe)
OBSERVE_SEC            = 5.0

# AMCL waypoint navigation
HOME_DIST            = 0.4   # release inside the blue init box
HOME_ARRIVE_CONFIRM  = 2     # consecutive ticks inside HOME_DIST before releasing
HOME_CLOSE_M         = 0.6   # below this, use tighter turn deadband + pulsed rotation
RETURN_CLOSE_TURN_DB = 10.0  # deg — final approach: align before driving forward
NAV_TURN_DB    = 25.0   # deg — rotate in place above this heading error
BRIDGE_YAW_DB  = 8.0
BRIDGE_ARRIVE_DIST = 0.15

LABEL_BEAR = "Bear"
LABEL_KNOB = "knob"


class AutoTaskController:
    def __init__(self, ros_communicator, data_processor, nav_processing, arm_controller):
        self.ros = ros_communicator
        self.dp  = data_processor
        self.arm = arm_controller

        self._thread        = None
        self._stop_event    = threading.Event()
        self._running       = False
        self._phase         = "idle"
        self._phase_start   = 0.0
        self._last_seen     = 0.0
        self._last_dist     = 999.0
        self._confirm_count = 0
        self._tick          = 0
        self._grasp_started    = False
        self._home_pos         = None
        self._bridge_wp_idx    = 0
        self._bridge_return_idx = -1
        self._bridge_waypoints = [BRIDGE_BEFORE, BRIDGE_TOP, BRIDGE_AFTER]
        self._observe_start    = None
        self._observe_next     = "arrived"
        self._observe_require_yolo = True
        self._post_grasp_phase  = "return"
        self._post_return_phase = "bridge"
        self._nav_last_dist        = 999.0
        self._nav_backup_until     = 0.0
        self._stuck_time           = 0.0
        self._stuck_pos            = None
        self._door_via_done        = False
        self._bridge_approach_done = False
        self._home_confirm         = 0
        self._home_yaw             = 0.0
        self._last_nav_cmd_t      = 0.0
        self._last_nav_pose       = None
        self._last_pub_action     = None
        self._last_pub_t          = 0.0
        self._last_action         = "STOP"
        self._grasp_status        = "idle"
        self._last_state_log_t    = 0.0
        self._last_state_key      = ""
        self._acted_yolo_rx       = 0

    # ── Public API ────────────────────────────────────────────────────────────

    def status_line(self):
        info = self.dp.get_yolo_target_info()
        if info:
            return (
                f"Phase: {self._phase} | action: {self._last_action} | "
                f"yolo found={int(info[0])} dist={info[1]:.2f}m dx={info[2]:.0f}px"
            )
        return f"Phase: {self._phase} | action: {self._last_action}"

    def auto_control(self, key=None):
        if key == "q":
            self.stop()
            return True
        if not self._running:
            self.start()
        return False

    def start(self):
        if self._running:
            self.stop()
        self._stop_event.clear()
        self._tick          = 0
        self._last_seen     = 0.0
        self._last_dist     = 999.0
        self._confirm_count = 0
        self._grasp_started        = False
        self._bridge_approach_done = False
        self._bridge_wp_idx        = 0
        self._bridge_return_idx    = -1
        self._observe_start        = None
        self._observe_require_yolo = True
        self._door_via_done        = False
        self._home_confirm         = 0
        self._last_nav_cmd_t      = 0.0
        self._last_nav_pose       = None
        self._last_pub_action     = None
        self._last_pub_t          = 0.0
        self._last_action         = "STOP"
        self._grasp_status        = "idle"
        self._last_state_log_t    = 0.0
        self._last_state_key      = ""
        self._acted_yolo_rx       = 0
        self._post_grasp_phase  = "return"
        self._post_return_phase = "bridge"
        self._home_pos = [0.0, 0.0]
        print("[AutoTask] Home fixed at (0.00, 0.00)")
        # Capture home yaw from AMCL so _do_return can use (home_yaw + 180°)
        # as a fixed return heading — more reliable than bearing from drifted position.
        self._home_yaw = 0.0
        for _ in range(20):
            pose = self.dp.get_processed_amcl_pose()
            if pose is not None:
                ori = pose[1]
                self._home_yaw = math.atan2(2.0 * ori[3] * ori[2], 1.0 - 2.0 * ori[2] * ori[2])
                print(f"[AutoTask] Home yaw captured: {math.degrees(self._home_yaw):.1f}°")
                break
            time.sleep(0.05)
        else:
            print("[AutoTask] AMCL not ready — home yaw defaulting to 0.0°")
        self._set_label(LABEL_BEAR)
        self._goto("search")
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self._running = True
        print("[AutoTask] started.")

    def stop(self):
        if not self._running:
            return
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self._running = False
        for _ in range(5):
            self._pub("STOP")
            time.sleep(0.05)
        print("[AutoTask] stopped. (Back in menu — select Exit to return to shell.)")

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        handlers = {
            "search":        self._do_search,
            "align":         self._do_align,
            "drive":         self._do_drive,
            "observe":       self._do_observe,
            "arrived":       self._do_arrived,
            "return":        self._do_return,
            "bridge":        self._do_bridge,
            "bridge_return": self._do_bridge_return,
            "door":          self._do_door,
            "door_push":     self._do_door_push,
            "finished":      self._do_finished,
        }
        _last_yolo_rx = 0
        _last_pose = None

        while not self._stop_event.is_set():
            phase = self._phase
            if phase in YOLO_WAIT_PHASES:
                time.sleep(YOLO_LOOP_SLEEP_S)
            elif phase in NAV_PHASES:
                deadline = time.time() + NAV_WAIT_S
                while not self._stop_event.is_set() and time.time() < deadline:
                    cur = self.dp.get_processed_amcl_pose()
                    if cur is not _last_pose:
                        _last_pose = cur
                        break
                    time.sleep(0.05)
            else:
                time.sleep(0.1)

            # ── Act on the fresh observation ──────────────────────────────────
            self._tick += 1
            handler = handlers.get(self._phase)
            if handler:
                try:
                    handler()
                except Exception as e:
                    import traceback
                    print(f"[AutoTask] ERROR in phase '{self._phase}': {e}")
                    traceback.print_exc()
                    self._pub("STOP")
            self._print_state()

        self._pub("STOP")

    # ── Phases ────────────────────────────────────────────────────────────────

    def _yolo_is_new(self):
        """True when a fresh /yolo/target_info arrived since the last acted frame."""
        rx = self.ros.yolo_target_info_rx_count
        if rx == self._acted_yolo_rx:
            return False
        self._acted_yolo_rx = rx
        return True

    def _replay_yolo_cmd(self):
        """Keep the last wheel command alive between YOLO frames."""
        if self._last_action and self._last_action != "STOP":
            self._pub(self._last_action)

    def _do_search(self):
        """Rotate until YOLO sees target."""
        if not self._yolo_is_new():
            self._replay_yolo_cmd()
            return
        info = self.dp.get_yolo_target_info()
        if info and info[0] == 1:
            self._confirm_count += 1
            if self._confirm_count >= CONFIRM_FRAMES:
                self._confirm_count = 0
                self._last_seen = time.time()
                if info[1] > 0:
                    self._last_dist = info[1]
                print(f"[search] target found  dist={info[1]:.2f}m  dx={info[2]:.0f}px → align")
                self._pub("STOP")
                self._goto("align")
            else:
                self._pub_rotate_pulse(ROT_CMD, *ROT_PULSE_SEARCH)
        else:
            self._confirm_count = 0
            self._pub_rotate_pulse(ROT_CMD, *ROT_PULSE_SEARCH)

    def _do_align(self):
        """Rotate in place until |dx| <= CENTER_DEADBAND, then hand off to drive."""
        if not self._yolo_is_new():
            self._replay_yolo_cmd()
            return

        info = self.dp.get_yolo_target_info()
        if not info or info[0] != 1:
            if time.time() - self._last_seen > LOST_GRACE_S:
                print("[align] target lost → search")
                self._goto("search")
            else:
                self._pub("STOP")
            return

        self._last_seen = time.time()
        if info[1] > 0:
            self._last_dist = info[1]

        dx = info[2]

        if self._approach_ready(info):
            print(f"[align] approach ready (dist={info[1]:.2f}m  dx={dx:.0f}px) → observe")
            self._pub("STOP")
            self._begin_observe("arrived")
            return

        if abs(dx) <= CENTER_DEADBAND:
            print(f"[align] centred (dx={dx:.0f}px) → drive")
            self._pub("STOP")
            self._goto("drive")
            return

        if abs(dx) > 25:
            rotate = ROT_CMD_MED if dx > 0 else ROT_CMD_CCW_MED
            self._pub_rotate_pulse(rotate, *ROT_PULSE_ALIGN_COARSE)
        else:
            rotate = ROT_CMD if dx > 0 else ROT_CMD_CCW
            self._pub_rotate_pulse(rotate, *ROT_PULSE_ALIGN_FINE)

    def _do_drive(self):
        """Approach target using camera_nav-style discrete actions."""
        if not self._yolo_is_new():
            self._replay_yolo_cmd()
            return

        info = self.dp.get_yolo_target_info()
        action = self._camera_nav_action()

        if info and info[0] == 1:
            self._last_seen = time.time()
            if info[1] > 0:
                self._last_dist = info[1]

            if self._approach_ready(info):
                print(f"[drive] approach ready (dist={info[1]:.2f}m  dx={info[2]:.0f}px) → observe")
                self._pub("STOP")
                self._begin_observe("arrived")
                return

        if action == "STOP":
            if info and info[0] == 1 and (info[1] == -1.0 or (0.0 < info[1] <= STOP_DIST)):
                print(f"[drive] arrived at dist={info[1]:.2f}m → observe")
                self._begin_observe("arrived")
            elif self._last_dist < 1.0 and (not info or info[0] != 1):
                print(f"[drive] target left FOV (last={self._last_dist:.2f}m) → observe")
                self._begin_observe("arrived")
            elif time.time() - self._last_seen > LOST_GRACE_S:
                print("[drive] target lost → search")
                self._last_dist = 999.0
                self._goto("search")
            else:
                self._pub("STOP")
            return

        if action in (ROT_CMD, ROT_CMD_CCW):
            self._pub_rotate_pulse(action, *ROT_PULSE_DRIVE)
            return

        self._pub(action)

    def _begin_observe(self, next_phase, require_yolo=True):
        """Start 5 s scoring observe before grasp or door unlock."""
        self._observe_next         = next_phase
        self._observe_require_yolo = require_yolo
        self._observe_start        = None
        self._goto("observe")

    def _do_observe(self):
        """Task 1/3 scoring: hold for OBSERVE_SEC (YOLO target or fixed pose at door)."""
        self._pub("STOP")
        info = self.dp.get_yolo_target_info()

        if self._observe_require_yolo:
            if not info or info[0] != 1:
                if self._observe_start and time.time() - self._observe_start > LOST_GRACE_S:
                    print("[observe] target lost → search")
                    self._observe_start = None
                    self._goto("search")
                return
        elif self._observe_start is None:
            print(f"[observe] at task pose — hold {OBSERVE_SEC:.0f}s for scoring")

        if self._observe_start is None:
            self._observe_start = time.time()
            if self._observe_require_yolo:
                print(f"[observe] target in view — hold {OBSERVE_SEC:.0f}s for scoring")

        elapsed = time.time() - self._observe_start
        if elapsed >= OBSERVE_SEC:
            print(f"[observe] done ({elapsed:.1f}s) → {self._observe_next}")
            self._observe_start = None
            self._goto(self._observe_next)
        elif self._tick % 40 == 0:
            print(f"[observe] {elapsed:.1f}/{OBSERVE_SEC:.0f}s")

    def _do_arrived(self):
        """Car stopped near bear — run grasp sequence once on a background thread."""
        self._pub("STOP")
        if not self._grasp_started:
            self._grasp_started = True
            threading.Thread(target=self._run_grasp, daemon=True).start()

    def _run_grasp(self):
        """IGVI-style: arm sequence → settle → one vision check → retry if needed."""
        grasped = False
        for attempt in range(1, MAX_GRASP_ATTEMPTS + 1):
            self._grasp_status = f"attempt {attempt}"
            print(f"[grasp] attempt {attempt}/{MAX_GRASP_ATTEMPTS}: running arm sequence")
            try:
                self._pub("STOP")
                self.arm.execute_ground_grasp_sequence(
                    shoulder_deg=-145.0,
                    elbow_deg=-68.0,
                    pre_close_fn=None,
                )
            except Exception as e:
                print(f"[grasp] arm error: {e}")
                break

            self._pub("STOP")
            self._grasp_status = "settle"
            print(f"[grasp] waiting {GRASP_SETTLE_S:.0f}s for Unity animation...")
            time.sleep(GRASP_SETTLE_S)

            self._grasp_status = "check"
            grasped, detail = self._detect_grasp()
            print(f"[grasp] check_grasp: {'OK' if grasped else 'FAIL'} — {detail}")
            if grasped:
                break
            if attempt < MAX_GRASP_ATTEMPTS:
                print("[grasp] not held — backing up for retry")
                self._pub("BACKWARD_SLOW")
                time.sleep(1.0)
                self._pub("STOP")
                time.sleep(0.3)

        self._grasp_status = "idle"
        self._grasp_started = False
        if grasped:
            print(f"[AutoTask] Grasp confirmed → {self._post_grasp_phase}")
            self._goto(self._post_grasp_phase)
        else:
            print("[AutoTask] Grasp failed → search")
            self._last_dist = 999.0
            self._goto("search")

    def _detect_grasp(self):
        """IGVI grab_object_server.detect_grasp pattern adapted for Unity.

        IGVI reads /joint_states gripper_joint: grasped when the finger closed
        but did NOT reach the full close target (object blocking). Unity does not
        publish joint feedback to pros_car, so we use one fresh YOLO frame:
          - bear still on ground (> GRASP_ON_GROUND_M) → missed
          - bear gone or within arm range → held
        """
        rx_at = self.ros.yolo_target_info_rx_count
        deadline = time.time() + GRASP_CHECK_TIMEOUT_S
        while time.time() < deadline and not self._stop_event.is_set():
            if self.ros.yolo_target_info_rx_count != rx_at:
                info = self.dp.get_yolo_target_info()
                if info is None or info[0] != 1:
                    return True, "bear not visible (likely held)"
                dist = info[1]
                if dist > GRASP_ON_GROUND_M:
                    return False, f"bear still on ground at {dist:.2f}m"
                return True, f"bear at {dist:.2f}m (held/closer)"
            time.sleep(0.1)
        return False, "no fresh YOLO frame for grasp check"

    def _do_return(self):
        """Drive to home using bearing-to-target AMCL nav (no early-release fallback)."""
        if self._home_pos is None:
            print("[return] No home recorded → finished")
            self._pub("STOP")
            self._goto("finished")
            return

        if self._grasp_started:
            self._pub("STOP")
            return

        pose = self.dp.get_processed_amcl_pose()
        if pose is None:
            self._pub("STOP")
            return

        pos, ori = pose
        cur_x, cur_y = pos[0], pos[1]
        dist = math.hypot(self._home_pos[0] - cur_x, self._home_pos[1] - cur_y)

        if dist < HOME_DIST:
            self._pub("STOP")
            self._home_confirm += 1
            if self._home_confirm >= HOME_ARRIVE_CONFIRM:
                self._grasp_started = True
                print(f"[return] Home reached (dist={dist:.2f}m) → releasing bear")
                self._door_via_done = False
                self._stuck_pos     = None
                threading.Thread(
                    target=self._push_and_release,
                    args=(self._post_return_phase,),
                    daemon=True,
                ).start()
            else:
                print(
                    f"[return] at home dist={dist:.2f}m"
                    f" — confirm {self._home_confirm}/{HOME_ARRIVE_CONFIRM}"
                )
            return

        self._home_confirm = 0
        self._amcl_nav_step(
            self._home_pos[0], self._home_pos[1], HOME_DIST,
            label="return",
            close_dist=HOME_CLOSE_M,
            close_turn_db=RETURN_CLOSE_TURN_DB,
        )

    def _do_bridge(self):
        """Task 2: approach → ascent (TOP) → descent (AFTER) → search bridge bear."""
        if not self._bridge_approach_done:
            arrived = self._amcl_nav_step(
                BRIDGE_APPROACH["x"], BRIDGE_APPROACH["y"],
                BRIDGE_ARRIVE_DIST, label="bridge_approach",
            )
            if arrived:
                print("[bridge] Approach point reached → ascending ramp")
                self._bridge_approach_done = True
            return

        if self._bridge_wp_idx < len(self._bridge_waypoints):
            wp = self._bridge_waypoints[self._bridge_wp_idx]
            label = ("ascent" if self._bridge_wp_idx == 0 else
                     "top" if self._bridge_wp_idx == 1 else "descent")
            arrived = self._amcl_nav_step(
                wp["x"], wp["y"], BRIDGE_ARRIVE_DIST, label=f"bridge_{label}",
            )
            if not arrived:
                return
            if "yaw" in wp and not self._align_yaw(wp["yaw"], BRIDGE_YAW_DB, label=f"bridge_{label}"):
                return
            self._bridge_wp_idx += 1
            name = wp.get("name", label)
            print(f"[bridge] waypoint {self._bridge_wp_idx}/{len(self._bridge_waypoints)} reached ({name})")
            return

        print("[bridge] Ascent + descent done → searching for bridge bear")
        self._pub("STOP")
        self._bridge_approach_done = False
        self._bridge_wp_idx = 0
        self._post_grasp_phase  = "bridge_return"
        self._post_return_phase = "door"
        self._last_dist = 999.0
        self._confirm_count = 0
        self._set_label(LABEL_BEAR)
        self._goto("search")

    def _do_bridge_return(self):
        """Task 2 recovery: reverse bridge waypoints then return home with bear."""
        if self._bridge_return_idx < 0:
            self._bridge_return_idx = len(self._bridge_waypoints) - 1

        if self._bridge_return_idx >= 0:
            wp = self._bridge_waypoints[self._bridge_return_idx]
            arrived = self._amcl_nav_step(
                wp["x"], wp["y"], BRIDGE_ARRIVE_DIST,
                label="bridge_return", allow_reverse=True,
            )
            if not arrived:
                return
            if "yaw" in wp and not self._align_yaw(wp["yaw"], BRIDGE_YAW_DB, label="bridge_return"):
                return
            print(f"[bridge_return] back at waypoint {self._bridge_return_idx + 1}")
            self._bridge_return_idx -= 1
            return

        print("[bridge_return] Off bridge → heading home")
        self._bridge_return_idx = -1
        self._goto("return")

    def _do_door(self):
        self._set_label(LABEL_KNOB)

        pose = self.dp.get_processed_amcl_pose()
        if pose is None:
            return
        cur_x, cur_y = pose[0][0], pose[0][1]

        now = time.time()
        if self._stuck_pos is None:
            self._stuck_pos  = (cur_x, cur_y)
            self._stuck_time = now
        else:
            moved = math.sqrt((cur_x - self._stuck_pos[0])**2 + (cur_y - self._stuck_pos[1])**2)
            if moved > 0.1:
                self._stuck_pos  = (cur_x, cur_y)
                self._stuck_time = now
            elif now - self._stuck_time > 4.0:
                sectors = self.dp.get_lidar_obstacle_sectors()
                print("[door] Stuck! Spinning to recover...")
                if sectors and sectors.get("left") is not None and sectors["left"] < 0.5:
                    self._pub(ROT_CMD_CCW)
                else:
                    self._pub(ROT_CMD)
                time.sleep(1.0)
                self._stuck_pos  = None
                self._stuck_time = now
                return

        if not self._door_via_done:
            arrived = self._amcl_nav_step(
                DOOR_VIA["x"], DOOR_VIA["y"], BRIDGE_ARRIVE_DIST, label="door_via",
            )
            if arrived:
                print("[door] Via-point cleared → heading to door")
                self._door_via_done = True
            return

        arrived = self._amcl_nav_step(
            DOOR["x"], DOOR["y"], BRIDGE_ARRIVE_DIST, label="door",
        )
        if arrived:
            print("[door] Door reached → observe before unlock")
            self._pub("STOP")
            self._begin_observe("door_push", require_yolo=False)

    def _do_door_push(self):
        self._pub("STOP")
        if not self._grasp_started:
            self._grasp_started = True
            threading.Thread(target=self._run_door_push, daemon=True).start()

    def _run_door_push(self):
        self.arm.set_door_push_pose()
        time.sleep(0.3)
        self.arm.push_door()
        self._pub("STOP")
        self._grasp_started = False
        # Task 3 Clear: find and retrieve the bear near the door
        print("[door] Unlocked → Task 3 clear (search door bear)")
        self._set_label(LABEL_BEAR)
        self._post_grasp_phase  = "return"
        self._post_return_phase = "finished"
        self._last_dist = 999.0
        self._confirm_count = 0
        self._goto("search")

    def _push_and_release(self, next_phase):
        self._pub("STOP")
        time.sleep(0.2)
        self.arm.open_gripper()
        self._grasp_started = False
        if next_phase == "door":
            self._set_label(LABEL_KNOB)
        self._goto(next_phase)

    def _do_finished(self):
        self._pub("STOP")

    # ── Navigation helpers ─────────────────────────────────────────────────────

    def _amcl_nav_step(
        self,
        target_x,
        target_y,
        arrive_dist,
        label="nav",
        allow_reverse=False,
        close_dist=None,
        close_turn_db=RETURN_CLOSE_TURN_DB,
    ):
        """AMCL waypoint step — rotate toward target, then drive forward."""
        pose = self.dp.get_processed_amcl_pose()
        if pose is None:
            return False

        pos, ori = pose
        cur_x, cur_y = pos[0], pos[1]
        cur_yaw = math.atan2(2.0 * ori[3] * ori[2], 1.0 - 2.0 * ori[2] * ori[2])

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Arrival check BEFORE the throttle: a stopped car (pose barely changes)
        # must still be able to confirm it is inside arrive_dist across ticks.
        if dist < arrive_dist:
            return True

        now = time.time()
        if self._last_nav_pose is not None:
            lx, ly, lyaw = self._last_nav_pose
            moved = math.sqrt((cur_x - lx) ** 2 + (cur_y - ly) ** 2)
            yaw_delta = abs(math.degrees(cur_yaw - lyaw))
            yaw_delta = min(yaw_delta, 360.0 - yaw_delta)
            if moved < 0.03 and yaw_delta < 3.0 and now - self._last_nav_cmd_t < 0.4:
                return False

        bearing   = math.atan2(dy, dx)
        angle_err = math.degrees(bearing - cur_yaw)
        angle_err = (angle_err + 180) % 360 - 180

        if self._tick % 10 == 0:
            print(f"[{label}] dist={dist:.2f}m  angle_err={angle_err:.1f}°")

        turn_db = close_turn_db if close_dist is not None and dist < close_dist else NAV_TURN_DB
        rotate = ROT_CMD if angle_err < 0 else ROT_CMD_CCW

        if allow_reverse and abs(angle_err) > 90.0:
            self._pub("BACKWARD_SLOW")
        elif abs(angle_err) > turn_db:
            if close_dist is not None and dist < close_dist and abs(angle_err) < 30.0:
                self._pub_rotate_pulse(rotate, *ROT_PULSE_RETURN_CLOSE)
            else:
                self._pub(rotate)
        else:
            self._pub(FWD_CMD)

        self._last_nav_cmd_t = now
        self._last_nav_pose = (cur_x, cur_y, cur_yaw)
        return False

    def _align_yaw(self, target_yaw, deadband_deg, label="nav"):
        """Rotate in place until within deadband of target_yaw (radians). Returns True when aligned."""
        pose = self.dp.get_processed_amcl_pose()
        if pose is None:
            return False
        _, ori = pose
        cur_yaw = math.atan2(2.0 * ori[3] * ori[2], 1.0 - 2.0 * ori[2] * ori[2])
        yaw_err = math.degrees(target_yaw - cur_yaw)
        yaw_err = (yaw_err + 180) % 360 - 180
        if abs(yaw_err) <= deadband_deg:
            return True
        if self._tick % 10 == 0:
            print(f"[{label}] yaw align  err={yaw_err:.1f}°")
        self._pub(ROT_CMD if yaw_err < 0 else ROT_CMD_CCW)
        return False

    # ── Sensor helpers (nav_processing.camera_nav pattern) ─────────────────────

    @staticmethod
    def _filter_negative_one(depth_list):
        return [depth for depth in depth_list if depth != -1.0]

    def _depth_sectors(self):
        """20-point depth scan split like camera_nav_unity: left / forward / right."""
        camera_multi_depth = self.dp.get_camera_x_multi_depth()
        if camera_multi_depth is None:
            return None, None, None

        camera_multi_depth = list(camera_multi_depth)
        forward = self._filter_negative_one(camera_multi_depth[7:13])
        left    = self._filter_negative_one(camera_multi_depth[0:7])
        right   = self._filter_negative_one(camera_multi_depth[13:20])
        return forward, left, right

    def _camera_nav_action(self, search_rotate=None):
        """Same decision tree as nav_processing.camera_nav() — slow actions only."""
        if search_rotate is None:
            search_rotate = ROT_CMD

        yolo_target_info = self.dp.get_yolo_target_info()
        camera_forward_depth, camera_left_depth, camera_right_depth = self._depth_sectors()

        if yolo_target_info is None:
            return "STOP"

        if camera_forward_depth:
            path_clear = all(depth > DEPTH_LIMIT for depth in camera_forward_depth)
            if not path_clear:
                if camera_left_depth and any(depth < DEPTH_LIMIT for depth in camera_left_depth):
                    return ROT_CMD
                if camera_right_depth and any(depth < DEPTH_LIMIT for depth in camera_right_depth):
                    return ROT_CMD_CCW
                return "STOP"

        if yolo_target_info[0] == 1:
            dist = yolo_target_info[1]
            dx   = yolo_target_info[2]

            # Close to bear: stop for grasp — do NOT spin to fix dx (causes errors)
            if dist == -1.0 or (0.0 < dist <= STOP_DIST):
                return "STOP"

            if 0.0 < dist <= STOP_DIST + 0.1 and abs(dx) <= CENTER_DEADBAND:
                return "STOP"

            if dx > CENTER_DEADBAND:
                return ROT_CMD
            if dx < -CENTER_DEADBAND:
                return ROT_CMD_CCW
            return FWD_CMD

        return search_rotate

    # ── Utilities ─────────────────────────────────────────────────────────────

    def _approach_ready(self, info):
        """True when close enough to grasp — distance matters more than perfect dx."""
        if not info or info[0] != 1:
            return False
        dist, dx = info[1], info[2]
        if dist == -1.0:
            return True
        if 0.0 < dist <= STOP_DIST:
            return True
        if 0.0 < dist <= STOP_DIST + 0.1 and abs(dx) <= CENTER_DEADBAND:
            return True
        return False

    def _print_state(self, force=False):
        """Short status line, throttled to STATE_LOG_INTERVAL_S."""
        phase = self._phase
        action = self._last_action
        key = (phase, action, self._grasp_status)
        now = time.time()
        if (
            not force
            and key == self._last_state_key
            and now - self._last_state_log_t < STATE_LOG_INTERVAL_S
        ):
            return
        self._last_state_log_t = now
        self._last_state_key = key

        if phase in YOLO_WAIT_PHASES:
            info = self.dp.get_yolo_target_info()
            if phase == "observe":
                elapsed = (time.time() - self._observe_start) if self._observe_start else 0.0
                print(f"[state] observe | STOP | {elapsed:.0f}/{OBSERVE_SEC:.0f}s")
            elif info and info[0] == 1:
                print(f"[state] {phase} | {action} | dist={info[1]:.2f}m dx={info[2]:.0f}px")
            else:
                print(f"[state] {phase} | {action} | no target")
        elif phase == "arrived":
            print(f"[state] arrived | {action} | grasp={self._grasp_status}")
        elif phase in NAV_PHASES:
            pose = self.dp.get_processed_amcl_pose()
            if pose is None:
                print(f"[state] {phase} | {action}")
                return
            pos, ori = pose
            x, y = pos[0], pos[1]
            yaw = math.degrees(
                math.atan2(2.0 * ori[3] * ori[2], 1.0 - 2.0 * ori[2] * ori[2])
            )
            extra = ""
            if phase == "return" and self._home_pos is not None:
                dist = math.hypot(self._home_pos[0] - x, self._home_pos[1] - y)
                extra = f" home={dist:.2f}m"
            print(f"[state] {phase} | {action} | ({x:.1f},{y:.1f}) yaw={yaw:.0f}°{extra}")
        else:
            print(f"[state] {phase} | {action}")

    def _goto(self, phase):
        self._phase          = phase
        self._phase_start    = time.time()
        self._nav_last_dist  = 999.0
        self._nav_backup_until = 0.0
        self._last_nav_cmd_t  = 0.0
        self._last_nav_pose   = None
        if phase == "return":
            self._home_confirm = 0
        if phase == "bridge":
            self._bridge_approach_done = False
            self._bridge_wp_idx = 0
        if phase == "bridge_return":
            self._bridge_return_idx = len(self._bridge_waypoints) - 1
        if phase == "observe":
            self._observe_start = None
        if phase in YOLO_WAIT_PHASES:
            self._acted_yolo_rx = 0
        print(f"[AutoTask] ── {phase} ──")
        self._print_state(force=True)

    def _pub_rotate_pulse(self, rotate_cmd, on_ticks, cycle_ticks):
        """Send rotate only part of the time — pairs with 250–275 RPM slow commands."""
        action = rotate_cmd if (self._tick % cycle_ticks) < on_ticks else "STOP"
        self._pub(action)

    def _pub(self, action):
        self._last_action = action
        now = time.time()
        # Deduplicate: only resend the same command every 0.3 s (heartbeat so
        # Unity keeps the last velocity), but send a NEW command immediately.
        # This prevents rosbridge queue build-up from causing stale commands to
        # execute after the state machine has already moved on.
        if action == self._last_pub_action and now - self._last_pub_t < 0.3:
            return
        self._last_pub_action = action
        self._last_pub_t = now
        self.ros.publish_car_control(action, publish_rear=True, publish_front=True)

    def _set_label(self, label):
        try:
            self.ros.publish_target_label(label)
        except Exception as e:
            print(f"[AutoTask] label error: {e}")
