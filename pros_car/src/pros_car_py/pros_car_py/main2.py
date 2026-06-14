import urwid
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
import time
import signal
from pros_car_py.joint_config import JOINT_UPDATES_POSITIVE, JOINT_UPDATES_NEGATIVE
from pros_car_py.car_controller import CarController
from pros_car_py.arm_controller_2D import ArmController
from pros_car_py.data_processor import DataProcessor
from pros_car_py.nav_processing import Nav2Processing
from pros_car_py.ros_communicator import RosCommunicator
from pros_car_py.crane_controller import CraneController
from pros_car_py.custom_control import CustomControl
from pros_car_py.ik_solver import PybulletRobotController
from pros_car_py.mode_app import ModeApp
from pros_car_py.auto_task import AutoTaskController


def init_ros_node():
    rclpy.init()
    # Unity odom/AMCL TF can arrive with slightly old stamps — harmless.
    try:
        rclpy.logging.set_logger_level(
            "tf2_ros", rclpy.logging.LoggingSeverity.ERROR
        )
    except Exception:
        pass
    node = RosCommunicator()
    # MultiThreadedExecutor so a slow callback (lidar / camera image) can't block
    # the lightweight amcl_pose and yolo_target_info callbacks the control loop
    # depends on. Single-threaded spin serialised them, causing stale pose/detection.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return node, executor, thread


def _restore_terminal(loop):
    """Urwid leaves stdin in raw mode if the process exits abruptly (e.g. Ctrl+C)."""
    try:
        if loop is not None and getattr(loop, "screen", None) is not None:
            loop.screen.reset_default_terminal()
    except Exception:
        pass


def _cleanup_ros(node, executor, thread):
    try:
        executor.shutdown(timeout_sec=2.0)
    except Exception:
        pass
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass
    if thread.is_alive():
        thread.join(timeout=3.0)


def main():
    ros_communicator, executor, ros_thread = init_ros_node()
    data_processor = DataProcessor(ros_communicator)
    nav2_processing = Nav2Processing(ros_communicator, data_processor)
    ik_solver = PybulletRobotController(end_eff_index=5)
    car_controller = CarController(ros_communicator, nav2_processing)
    arm_controller = ArmController(ros_communicator, data_processor)
    crane_controller = CraneController(
        ros_communicator, data_processor, ik_solver, num_joints=7
    )
    custom_control = CustomControl(car_controller, arm_controller)
    auto_task_controller = AutoTaskController(
        ros_communicator, data_processor, nav2_processing, arm_controller
    )
    app = ModeApp(
        car_controller,
        arm_controller,
        custom_control,
        crane_controller,
        auto_task_controller,
    )

    def _shutdown_stop():
        """Stop the car before exiting. Unity holds the last velocity, so without
        this the car keeps moving after the node/container is stopped."""
        try:
            auto_task_controller.stop()
        except Exception:
            pass
        try:
            ros_communicator.emergency_stop()
        except Exception:
            pass
        # Sleep so STOP messages flush through rosbridge before the publisher
        # is destroyed by rclpy.shutdown(). Without this, Ctrl+C kills the socket
        # before Unity receives the zero-velocity command and the car keeps moving.
        time.sleep(0.5)

    def _on_signal(signum, frame):
        """Exit urwid cleanly so the terminal is restored before ROS teardown."""
        try:
            app.loop.exit()
        except Exception:
            raise KeyboardInterrupt

    # Catch both Ctrl+C (SIGINT) and docker stop (SIGTERM).
    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    try:
        app.main()
    except KeyboardInterrupt:
        pass
    finally:
        _restore_terminal(app.loop)
        _shutdown_stop()
        _cleanup_ros(ros_communicator, executor, ros_thread)
        print("robot_control stopped — shell is ready.")


if __name__ == "__main__":
    main()
