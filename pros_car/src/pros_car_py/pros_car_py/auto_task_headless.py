"""
Headless launcher for AutoTaskController.

Boots only the ROS plumbing needed by AutoTaskController (ros_communicator,
data_processor, nav2_processing, arm_controller) and runs the state machine
without the urwid TUI. Intended for running the full T1 -> T2 -> T3 routine
non-interactively (e.g. from `docker exec`).

Usage (inside the pros_car docker, with install/setup.bash sourced):
    ros2 run pros_car_py auto_task_headless
"""
import signal
import sys
import threading
import time

import rclpy

from pros_car_py.arm_controller_2D import ArmController
from pros_car_py.auto_task import AutoTaskController
from pros_car_py.data_processor import DataProcessor
from pros_car_py.nav_processing import Nav2Processing
from pros_car_py.ros_communicator import RosCommunicator


def main():
    rclpy.init()
    try:
        rclpy.logging.set_logger_level(
            "tf2_ros", rclpy.logging.LoggingSeverity.ERROR
        )
    except Exception:
        pass
    node = RosCommunicator()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    data_processor = DataProcessor(node)
    nav_processing = Nav2Processing(node, data_processor)
    arm_controller = ArmController(node, data_processor)
    controller = AutoTaskController(
        node, data_processor, nav_processing, arm_controller
    )

    # Give subscribers ~3s to receive initial messages (amcl_pose, yolo, etc.)
    print("[headless] waiting 3s for ROS topics to populate...", flush=True)
    time.sleep(3.0)

    print("[headless] starting AutoTask state machine.", flush=True)
    controller.start()

    # Graceful shutdown on SIGINT/SIGTERM
    stop_flag = {"v": False}

    def _handle_sig(signum, _frame):
        print(f"[headless] received signal {signum}, stopping...", flush=True)
        stop_flag["v"] = True

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    try:
        # Block while the worker thread runs. AutoTaskController.stop() sets
        # the stop event when DONE/ABORTED, after which _running flips off.
        while not stop_flag["v"] and controller._running:
            time.sleep(0.5)
    finally:
        controller.stop()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print("[headless] exited.", flush=True)


if __name__ == "__main__":
    main()
