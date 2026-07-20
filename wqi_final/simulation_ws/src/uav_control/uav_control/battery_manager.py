import math
import threading

from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Int8, String
from visualization_msgs.msg import Marker

from uav_control.battery_model import BatteryModel, BatteryParameters


LANDED = 0
FLYING = 1


class BatteryManager(Node):
    def __init__(self) -> None:
        super().__init__("battery_manager")
        defaults = {
            "capacity_wh": 100.0,
            "nominal_voltage": 22.2,
            "empty_voltage": 19.8,
            "full_voltage": 25.2,
            "initial_percentage": 0.8,
            "discharge_efficiency": 0.92,
            "reserve_percentage": 0.2,
            "prediction_margin_factor": 1.25,
            "warning_percentage": 0.25,
            "critical_percentage": 0.1,
            "airframe_mass_kg": 1.20,
            "sensor_mass_kg": 0.277,
            "maximum_payload_mass_kg": 1.0,
            "default_payload_mass_kg": 0.30,
            "rotor_count": 4,
            "rotor_radius_m": 0.12,
            "air_density_kgpm3": 1.225,
            "gravity_mps2": 9.80665,
            "blade_profile_power_w": 105.0,
            "induced_power_correction": 0.10,
            "rotor_tip_speed_mps": 120.0,
            "fuselage_drag_ratio": 0.60,
            "rotor_solidity": 0.05,
            "horizontal_flat_plate_area_m2": 0.030,
            "vertical_flat_plate_area_m2": 0.080,
            "computer_power_w": 12.0,
            "lidar_power_w": 8.0,
            "camera_power_w": 3.0,
            "communication_power_w": 2.0,
            "landed_idle_power_w": 8.0,
            "docked_idle_power_w": 5.0,
            "charge_power_w": 180.0,
            "charge_efficiency": 0.9,
            "estimated_horizontal_speed_mps": 1.2,
            "estimated_horizontal_acceleration_mps2": 0.8,
            "estimated_vertical_speed_mps": 0.6,
            "estimated_vertical_acceleration_mps2": 0.5,
            "model_integration_step_s": 0.10,
            "hover_speed_threshold_mps": 0.12,
            "vertical_motion_threshold_mps": 0.08,
            "motion_acceleration_threshold_mps2": 0.20,
            "acceleration_filter_alpha": 0.25,
            "update_rate": 10.0,
            "publish_rate": 2.0,
            "maximum_update_step_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {
            name: self.get_parameter(name).value
            for name in defaults
        }
        self.parameters = BatteryParameters.from_mapping(values)
        self.model = BatteryModel(self.parameters)

        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.phase = "IDLE"
        self.vehicle_state = LANDED
        self.docked = False
        self.velocity_mps = (0.0, 0.0, 0.0)
        self.previous_velocity_mps = None
        self.filtered_acceleration_mps2 = [0.0, 0.0, 0.0]
        self.payload_mass_kg = 0.0
        self.last_update_ns = None
        self.last_reported_mode = ""
        self.last_level = "NORMAL"

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            "mission_status",
            self._phase_callback,
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Int8,
            "state",
            self._state_callback,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            "odom",
            self._odom_callback,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            "docked",
            self._docked_callback,
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            "payload_mass",
            self._payload_callback,
            transient_qos,
            callback_group=self.callback_group,
        )
        self.battery_pub = self.create_publisher(
            BatteryState, "battery_state", transient_qos
        )
        self.percentage_pub = self.create_publisher(
            Float32, "battery_percentage", transient_qos
        )
        self.status_pub = self.create_publisher(
            String, "battery_status", transient_qos
        )
        self.power_pub = self.create_publisher(
            Float32, "battery_power_w", transient_qos
        )
        self.power_consumption_pub = self.create_publisher(
            Float32, "power_consumption", transient_qos
        )
        self.propulsion_power_pub = self.create_publisher(
            Float32, "propulsion_power_w", transient_qos
        )
        self.auxiliary_power_pub = self.create_publisher(
            Float32, "auxiliary_power_w", transient_qos
        )
        self.remaining_energy_pub = self.create_publisher(
            Float32, "remaining_energy", transient_qos
        )
        self.total_mass_pub = self.create_publisher(
            Float32, "total_mass_kg", transient_qos
        )
        self.consumed_pub = self.create_publisher(
            Float32, "battery_consumed_wh", transient_qos
        )
        self.charged_pub = self.create_publisher(
            Float32, "battery_charged_wh", transient_qos
        )
        self.marker_pub = self.create_publisher(
            Marker, "battery_marker", transient_qos
        )

        self.create_timer(
            1.0 / self.parameters.update_rate,
            self._update,
            callback_group=self.callback_group,
        )
        self.create_timer(
            1.0 / self.parameters.publish_rate,
            self._publish,
            callback_group=self.callback_group,
        )
        self._publish()
        self.get_logger().info(
            "UAV battery manager ready: "
            f"{self.parameters.capacity_wh:.1f} Wh, "
            f"initial {self.model.soc * 100.0:.1f}%, "
            f"reserve {self.parameters.reserve_percentage * 100.0:.1f}%"
        )

    def _phase_callback(self, message: String) -> None:
        with self.lock:
            self.phase = message.data

    def _state_callback(self, message: Int8) -> None:
        with self.lock:
            self.vehicle_state = int(message.data)

    def _odom_callback(self, message: Odometry) -> None:
        velocity = message.twist.twist.linear
        with self.lock:
            self.velocity_mps = (
                float(velocity.x),
                float(velocity.y),
                float(velocity.z),
            )

    def _docked_callback(self, message: Bool) -> None:
        with self.lock:
            self.docked = bool(message.data)

    def _payload_callback(self, message: Float32) -> None:
        payload = float(message.data)
        if (
            not math.isfinite(payload)
            or payload < 0.0
            or payload > self.parameters.maximum_payload_mass_kg
        ):
            self.get_logger().error(
                f"Ignoring invalid UAV payload mass {payload!r} kg"
            )
            return
        with self.lock:
            self.payload_mass_kg = payload

    def _operating_state(self):
        with self.lock:
            return (
                self.phase,
                self.vehicle_state == FLYING,
                self.docked,
                self.velocity_mps,
                self.payload_mass_kg,
            )

    def _update(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return
        if self.last_update_ns is None or now_ns <= self.last_update_ns:
            self.last_update_ns = now_ns
            return
        remaining = (now_ns - self.last_update_ns) / 1e9
        self.last_update_ns = now_ns
        phase, flying, docked, velocity, payload = self._operating_state()
        acceleration = [0.0, 0.0, 0.0]
        if self.previous_velocity_mps is not None and remaining > 1e-9:
            alpha = self.parameters.acceleration_filter_alpha
            for index in range(3):
                raw_acceleration = (
                    velocity[index] - self.previous_velocity_mps[index]
                ) / remaining
                self.filtered_acceleration_mps2[index] += alpha * (
                    raw_acceleration
                    - self.filtered_acceleration_mps2[index]
                )
                acceleration[index] = self.filtered_acceleration_mps2[index]
        self.previous_velocity_mps = velocity
        horizontal_speed = math.hypot(velocity[0], velocity[1])
        horizontal_acceleration = math.hypot(
            acceleration[0], acceleration[1]
        )
        acceleration_velocity_dot = (
            acceleration[0] * velocity[0]
            + acceleration[1] * velocity[1]
        )
        while remaining > 1e-9:
            step = min(remaining, self.parameters.maximum_update_step_s)
            self.model.step(
                step,
                phase,
                flying,
                docked,
                horizontal_speed_mps=horizontal_speed,
                vertical_speed_mps=velocity[2],
                horizontal_acceleration_mps2=horizontal_acceleration,
                vertical_acceleration_mps2=acceleration[2],
                horizontal_acceleration_velocity_dot_m2ps3=(
                    acceleration_velocity_dot
                ),
                payload_mass_kg=payload,
            )
            remaining -= step
        self._log_state_changes()

    def _battery_level(self) -> str:
        if self.model.soc <= self.parameters.critical_percentage:
            return "CRITICAL"
        if self.model.soc <= self.parameters.warning_percentage:
            return "LOW"
        return "NORMAL"

    def _log_state_changes(self) -> None:
        snapshot = self.model.snapshot()
        level = self._battery_level()
        if snapshot.mode != self.last_reported_mode:
            self.get_logger().info(
                f"Battery mode: {snapshot.mode} ({snapshot.soc * 100.0:.1f}%)"
            )
            self.last_reported_mode = snapshot.mode
        if level != self.last_level:
            log = self.get_logger().error if level == "CRITICAL" else self.get_logger().warn
            if level == "NORMAL":
                log = self.get_logger().info
            log(f"Battery level: {level} ({snapshot.soc * 100.0:.1f}%)")
            self.last_level = level

    def _publish(self) -> None:
        snapshot = self.model.snapshot()
        now = self.get_clock().now().to_msg()
        message = BatteryState()
        message.header.stamp = now
        message.header.frame_id = "uav/base_link"
        message.voltage = float(snapshot.voltage)
        message.temperature = math.nan
        message.current = float(snapshot.current_a)
        capacity_ah = self.parameters.capacity_wh / self.parameters.nominal_voltage
        message.charge = float(capacity_ah * snapshot.soc)
        message.capacity = float(capacity_ah)
        message.design_capacity = float(capacity_ah)
        message.percentage = float(snapshot.soc)
        if snapshot.mode == "CHARGING":
            message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif snapshot.mode == "FULL":
            message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        message.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_DEAD
            if snapshot.soc <= 0.0
            else BatteryState.POWER_SUPPLY_HEALTH_GOOD
        )
        message.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        message.present = True
        message.location = "campus_uav"
        message.serial_number = "SIM-UAV-BATTERY-001"
        self.battery_pub.publish(message)
        self.percentage_pub.publish(Float32(data=float(snapshot.soc * 100.0)))
        self.power_pub.publish(
            Float32(data=float(snapshot.battery_power_w))
        )
        self.power_consumption_pub.publish(
            Float32(data=float(snapshot.battery_power_w))
        )
        self.propulsion_power_pub.publish(
            Float32(data=float(snapshot.propulsion_power_w))
        )
        self.auxiliary_power_pub.publish(
            Float32(data=float(snapshot.auxiliary_power_w))
        )
        self.remaining_energy_pub.publish(
            Float32(data=float(snapshot.energy_wh))
        )
        self.total_mass_pub.publish(
            Float32(data=float(snapshot.total_mass_kg))
        )
        self.consumed_pub.publish(
            Float32(data=float(snapshot.consumed_energy_wh))
        )
        self.charged_pub.publish(
            Float32(data=float(snapshot.charged_energy_wh))
        )

        status = (
            f"{snapshot.mode}:{snapshot.soc * 100.0:.1f}%:"
            f"{snapshot.energy_wh:.2f}Wh:{snapshot.battery_power_w:+.1f}W:"
            f"payload={snapshot.payload_mass_kg:.2f}kg"
        )
        self.status_pub.publish(String(data=status))
        self.marker_pub.publish(self._marker(status, now))

    def _marker(self, status: str, stamp) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "uav/base_link"
        marker.ns = "uav_battery"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.85
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.32
        marker.color.a = 1.0
        level = self._battery_level()
        if level == "CRITICAL":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.1, 0.1
        elif level == "LOW":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.65, 0.05
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.1, 1.0, 0.35
        mode, percentage, *_ = status.split(":")
        marker.text = f"UAV Battery {percentage} | {mode}"
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = BatteryManager()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
