import math
import threading
import time

from nav_msgs.msg import Odometry
from PyQt5.QtCore import QThread, pyqtSignal
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Int32, String


def format_position(position) -> str:
    if position is None:
        return "--"
    return f"x {position[0]:.1f}  y {position[1]:.1f}  z {position[2]:.1f} m"


def format_battery(percentage, power_w) -> str:
    if percentage is None or not math.isfinite(percentage):
        return "--"
    power = "--" if power_w is None else f"{power_w:.0f} W"
    return f"{percentage * 100.0:.1f}%  |  {power}"


def format_safety(blocked, minimum_distance) -> str:
    if blocked is None:
        return "--"
    state = "阻塞" if blocked else "安全"
    if minimum_distance is None or not math.isfinite(minimum_distance):
        return state
    return f"{state}  |  最近 {minimum_distance:.2f} m"


class RosTelemetryThread(QThread):
    telemetry_received = pyqtSignal(dict)
    connection_changed = pyqtSignal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._stop_event = threading.Event()
        self._state = {
            "ugv_position": None,
            "uav_position": None,
            "battery_percentage": None,
            "battery_power_w": None,
            "ugv_drive_battery_percentage": None,
            "ugv_drive_power_w": None,
            "ugv_charging_battery_percentage": None,
            "ugv_charging_power_w": None,
            "cooperative_status": "",
            "uav_status": "",
            "docked": None,
            "safety_blocked": None,
            "minimum_distance": None,
            "dynamic_obstacles": 0,
            "uav_replans": 0,
        }
        self._ground_truth_at = 0.0

    @staticmethod
    def _position(message: Odometry):
        point = message.pose.pose.position
        return (float(point.x), float(point.y), float(point.z))

    def _set(self, key, value) -> None:
        self._state[key] = value

    def _ground_truth_callback(self, message: Odometry) -> None:
        self._ground_truth_at = time.monotonic()
        self._set("ugv_position", self._position(message))

    def _ugv_odom_callback(self, message: Odometry) -> None:
        if time.monotonic() - self._ground_truth_at > 1.0:
            self._set("ugv_position", self._position(message))

    def _uav_odom_callback(self, message: Odometry) -> None:
        self._set("uav_position", self._position(message))

    def _create_subscriptions(self, node: Node) -> None:
        node.create_subscription(
            Odometry, "/ground_truth/odom", self._ground_truth_callback, 10
        )
        node.create_subscription(Odometry, "/odom", self._ugv_odom_callback, 10)
        node.create_subscription(
            Odometry, "/uav/odom", self._uav_odom_callback, 10
        )
        node.create_subscription(
            BatteryState,
            "/uav/battery_state",
            lambda message: self._set(
                "battery_percentage", float(message.percentage)
            ),
            10,
        )
        node.create_subscription(
            Float32,
            "/uav/battery_power_w",
            lambda message: self._set("battery_power_w", float(message.data)),
            10,
        )
        node.create_subscription(
            BatteryState,
            "/ugv/drive_battery_state",
            lambda message: self._set(
                "ugv_drive_battery_percentage", float(message.percentage)
            ),
            10,
        )
        node.create_subscription(
            Float32,
            "/ugv/drive_power_w",
            lambda message: self._set(
                "ugv_drive_power_w", float(message.data)
            ),
            10,
        )
        node.create_subscription(
            BatteryState,
            "/ugv/charging_battery_state",
            lambda message: self._set(
                "ugv_charging_battery_percentage", float(message.percentage)
            ),
            10,
        )
        node.create_subscription(
            Float32,
            "/ugv/charging_source_power_w",
            lambda message: self._set(
                "ugv_charging_power_w", float(message.data)
            ),
            10,
        )
        node.create_subscription(
            String,
            "/cooperative_delivery/mission_status",
            lambda message: self._set("cooperative_status", str(message.data)),
            10,
        )
        node.create_subscription(
            String,
            "/uav/mission_status",
            lambda message: self._set("uav_status", str(message.data)),
            10,
        )
        node.create_subscription(
            Bool,
            "/uav/docked",
            lambda message: self._set("docked", bool(message.data)),
            10,
        )
        node.create_subscription(
            Bool,
            "/uav/safety/blocked",
            lambda message: self._set("safety_blocked", bool(message.data)),
            10,
        )
        node.create_subscription(
            Float32,
            "/uav/safety/min_distance",
            lambda message: self._set("minimum_distance", float(message.data)),
            10,
        )
        node.create_subscription(
            Int32,
            "/dynamic_obstacles/count",
            lambda message: self._set("dynamic_obstacles", int(message.data)),
            10,
        )
        node.create_subscription(
            Int32,
            "/uav/replan_count",
            lambda message: self._set("uav_replans", int(message.data)),
            10,
        )

    def run(self) -> None:
        context = Context()
        node = None
        executor = None
        try:
            rclpy.init(
                context=context,
                signal_handler_options=SignalHandlerOptions.NO,
            )
            node = Node("simulation_dashboard_monitor", context=context)
            self._create_subscriptions(node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self.connection_changed.emit("ROS 监控已连接")
            last_emit = 0.0
            while not self._stop_event.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.1)
                now = time.monotonic()
                if now - last_emit >= 0.2:
                    self.telemetry_received.emit(dict(self._state))
                    last_emit = now
        except Exception as error:
            self.connection_changed.emit(f"ROS 监控不可用：{error}")
        finally:
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
            if node is not None:
                node.destroy_node()
            if context.ok():
                context.shutdown()

    def stop(self) -> None:
        self._stop_event.set()
        self.wait(2000)
