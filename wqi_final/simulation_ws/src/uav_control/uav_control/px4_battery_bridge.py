"""Forward the project battery model to PX4's external MAVLink battery input."""

import math
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String


def battery_sample_fresh(received_sim_ns, received_wall, now_sim_ns, now_wall, timeout):
    if received_sim_ns is None or received_wall is None:
        return False
    sim_age = (now_sim_ns - received_sim_ns) / 1e9
    wall_age = now_wall - received_wall
    # Sensors publish on simulation time. A separate generous wall watchdog
    # still detects a paused/crashed simulator, without penalizing low RTF.
    return (0 <= sim_age <= timeout
            and 0 <= wall_age <= max(30.0, timeout * 10.0))


def battery_mavlink_fields(voltage, current, soc, capacity_ah, charge_ah):
    values = (voltage, current, soc, capacity_ah, charge_ah)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Battery inputs must be finite")
    if not (0 < voltage < 65.535 and 0 <= soc <= 1 and capacity_ah > 0
            and 0 <= charge_ah <= capacity_ah and abs(current) <= 327.67):
        raise ValueError("Battery inputs exceed MAVLink bounds")
    # MAVLink permits total pack voltage in the first cell slot when individual
    # cell voltages are unavailable. Do not invent per-cell measurements.
    return dict(
        current_consumed=round((capacity_ah - charge_ah) * 1000),
        energy_consumed=-1,
        voltages=[round(voltage * 1000)] + [65535] * 9,
        # ROS BatteryState is negative on discharge; MAVLink is positive.
        current_battery=round(-current * 100),
        battery_remaining=round(soc * 100),
    )


class Px4BatteryBridge(Node):
    def __init__(self):
        super().__init__("px4_battery_bridge")
        self.declare_parameter("mavlink_endpoint", "udpout:127.0.0.1:14580")
        self.declare_parameter("battery_timeout", 2.0)
        self.declare_parameter("publish_rate", 5.0)
        try:
            from pymavlink import mavutil
        except ImportError:
            site = os.environ.get("PX4_PYMAVLINK_SITE", "")
            if not site:
                root = Path(os.environ.get("PX4_AUTOPILOT_DIR", ".")).parent
                sites = [p for name in ("virtualenv", "venv")
                         for p in (root / name / "lib").glob("python*/site-packages")
                         if (p / "pymavlink").is_dir()]
                site = str(sites[0]) if sites else ""
            if site:
                sys.path.append(site)
            try:
                from pymavlink import mavutil
            except ImportError as error:
                raise RuntimeError("Missing pymavlink: run bash ./setup_px4_backend.sh") from error
        from px4_msgs.msg import BatteryStatus

        self.mavutil = mavutil
        self.connection = mavutil.mavlink_connection(
            str(self.get_parameter("mavlink_endpoint").value),
            source_system=1, source_component=191,
        )
        self.message = None
        self.received_at = None
        self.received_sim_ns = None
        self.confirmed_at = None
        self.confirmed_sim_ns = None
        self.timeout = float(self.get_parameter("battery_timeout").value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "px4/battery_bridge_status", qos)
        self.create_subscription(BatteryState, "battery_state", self._battery, qos)
        self.create_subscription(
            BatteryStatus, "/fmu/out/battery_status", self._confirmation,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value),
                          self._publish)
        self.get_logger().info("Paper-model battery -> PX4 MAVLink bridge ready")

    def _battery(self, message):
        try:
            battery_mavlink_fields(message.voltage, message.current,
                                  message.percentage, message.capacity, message.charge)
        except ValueError as error:
            self.message = None
            self.get_logger().error(str(error))
            return
        self.message = message
        self.received_at = time.monotonic()
        self.received_sim_ns = self.get_clock().now().nanoseconds

    def _confirmation(self, message):
        if self.message is not None and message.connected:
            if (abs(message.remaining - self.message.percentage) <= 0.011
                    and abs(message.voltage_v - self.message.voltage) <= 0.06):
                self.confirmed_at = time.monotonic()
                self.confirmed_sim_ns = self.get_clock().now().nanoseconds

    def _publish(self):
        now = time.monotonic()
        sim_now = self.get_clock().now().nanoseconds
        if self.message is None or not battery_sample_fresh(
            self.received_sim_ns, self.received_at, sim_now, now, self.timeout
        ):
            self.status_pub.publish(String(data="STALE:project_battery_unavailable"))
            return
        m = self.message
        fields = battery_mavlink_fields(m.voltage, m.current, m.percentage,
                                       m.capacity, m.charge)
        self.connection.mav.heartbeat_send(
            self.mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            self.mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0,
        )
        self.connection.mav.battery_status_send(
            id=0, battery_function=1, type=1, temperature=32767, **fields,
        )
        synchronized = battery_sample_fresh(
            self.confirmed_sim_ns, self.confirmed_at, sim_now, now, self.timeout)
        self.status_pub.publish(String(data=(
            "SYNCED:paper_energy_model" if synchronized else "WAITING:PX4_confirmation"
        )))


def main(args=None):
    rclpy.init(args=args)
    node = Px4BatteryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.connection.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
