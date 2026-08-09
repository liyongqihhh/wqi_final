import math
import threading

from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String

from cooperative_delivery.ugv_energy_model import (
    UgvEnergyModel,
    UgvEnergyParameters,
)


class UgvEnergyManager(Node):
    def __init__(self) -> None:
        super().__init__("ugv_energy_manager")
        defaults = UgvEnergyParameters.defaults()
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        self.parameters = UgvEnergyParameters.from_mapping(values)
        self.model = UgvEnergyModel(self.parameters)

        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.linear_speed_mps = 0.0
        self.angular_speed_radps = 0.0
        self.filtered_acceleration_mps2 = 0.0
        self.previous_linear_speed_mps = None
        self.cargo_mass_kg = 0.0
        self.uav_docked = False
        self.uav_battery_power_w = 0.0
        self.last_update_ns = None
        self.last_charger_available = None

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._odom_callback,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            "/ugv/cargo_mass_kg",
            self._cargo_callback,
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            "/uav/docked",
            self._docked_callback,
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            "/uav/battery_power_w",
            self._uav_power_callback,
            transient_qos,
            callback_group=self.callback_group,
        )

        self.drive_battery_pub = self.create_publisher(
            BatteryState, "drive_battery_state", transient_qos
        )
        self.charging_battery_pub = self.create_publisher(
            BatteryState, "charging_battery_state", transient_qos
        )
        self.drive_power_pub = self.create_publisher(
            Float32, "drive_power_w", transient_qos
        )
        self.charging_power_pub = self.create_publisher(
            Float32, "charging_source_power_w", transient_qos
        )
        self.charging_output_pub = self.create_publisher(
            Float32, "uav_charging_output_power_w", transient_qos
        )
        self.drive_remaining_pub = self.create_publisher(
            Float32, "drive_remaining_wh", transient_qos
        )
        self.charging_remaining_pub = self.create_publisher(
            Float32, "charging_remaining_wh", transient_qos
        )
        self.drive_consumed_pub = self.create_publisher(
            Float32, "drive_consumed_wh", transient_qos
        )
        self.charging_consumed_pub = self.create_publisher(
            Float32, "charging_consumed_wh", transient_qos
        )
        self.carried_mass_pub = self.create_publisher(
            Float32, "total_carried_mass_kg", transient_qos
        )
        self.charger_available_pub = self.create_publisher(
            Bool, "charger_available", transient_qos
        )
        self.status_pub = self.create_publisher(
            String, "energy_status", transient_qos
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
            "UGV dual-battery manager ready: "
            f"drive {self.parameters.drive_capacity_wh:.1f} Wh at "
            f"{self.model.snapshot().drive_soc * 100.0:.1f}%, charger "
            f"{self.parameters.charging_capacity_wh:.1f} Wh at "
            f"{self.model.snapshot().charging_soc * 100.0:.1f}%"
        )

    def _odom_callback(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        with self.lock:
            self.linear_speed_mps = math.hypot(float(linear.x), float(linear.y))
            self.angular_speed_radps = float(angular.z)

    def _cargo_callback(self, message: Float32) -> None:
        mass = float(message.data)
        if not math.isfinite(mass) or mass < 0.0:
            self.get_logger().error(f"Ignoring invalid UGV cargo mass {mass!r}")
            return
        with self.lock:
            self.cargo_mass_kg = mass

    def _docked_callback(self, message: Bool) -> None:
        with self.lock:
            self.uav_docked = bool(message.data)

    def _uav_power_callback(self, message: Float32) -> None:
        power = float(message.data)
        if math.isfinite(power):
            with self.lock:
                self.uav_battery_power_w = power

    def _operating_state(self):
        with self.lock:
            return (
                self.linear_speed_mps,
                self.angular_speed_radps,
                self.cargo_mass_kg,
                self.uav_docked,
                self.uav_battery_power_w,
            )

    def _update(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= 0:
            return
        if self.last_update_ns is None or now_ns <= self.last_update_ns:
            self.last_update_ns = now_ns
            return
        remaining = (now_ns - self.last_update_ns) / 1.0e9
        self.last_update_ns = now_ns
        linear, angular, cargo, docked, uav_power = self._operating_state()
        if self.previous_linear_speed_mps is None:
            acceleration = 0.0
        else:
            raw = (linear - self.previous_linear_speed_mps) / max(remaining, 1e-9)
            alpha = self.parameters.acceleration_filter_alpha
            self.filtered_acceleration_mps2 += alpha * (
                raw - self.filtered_acceleration_mps2
            )
            acceleration = self.filtered_acceleration_mps2
        self.previous_linear_speed_mps = linear
        while remaining > 1e-9:
            step = min(remaining, self.parameters.maximum_update_step_s)
            self.model.step(
                step,
                linear,
                angular,
                acceleration,
                cargo,
                docked,
                uav_power,
            )
            remaining -= step
        available = self.model.snapshot().charger_available
        if available != self.last_charger_available:
            level = "available" if available else "empty"
            self.get_logger().info(f"UGV UAV-charging battery is {level}")
            self.last_charger_available = available

    @staticmethod
    def _battery_message(
        now,
        frame_id: str,
        location: str,
        serial: str,
        energy_wh: float,
        capacity_wh: float,
        voltage: float,
        power_w: float,
    ) -> BatteryState:
        soc = max(0.0, min(1.0, energy_wh / capacity_wh))
        capacity_ah = capacity_wh / voltage
        message = BatteryState()
        message.header.stamp = now
        message.header.frame_id = frame_id
        message.voltage = float(voltage)
        message.temperature = math.nan
        message.current = float(power_w / voltage)
        message.charge = float(capacity_ah * soc)
        message.capacity = float(capacity_ah)
        message.design_capacity = float(capacity_ah)
        message.percentage = float(soc)
        if soc <= 0.0:
            message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
            message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
        elif soc >= 1.0 and power_w <= 0.0:
            message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        else:
            message.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        message.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        message.present = True
        message.location = location
        message.serial_number = serial
        return message

    def _publish(self) -> None:
        snapshot = self.model.snapshot()
        now = self.get_clock().now().to_msg()
        self.drive_battery_pub.publish(self._battery_message(
            now,
            "base_link",
            "ugv_drive_pack",
            "SIM-UGV-DRIVE-001",
            snapshot.drive_energy_wh,
            self.parameters.drive_capacity_wh,
            self.parameters.drive_nominal_voltage,
            snapshot.drive_power_w,
        ))
        self.charging_battery_pub.publish(self._battery_message(
            now,
            "base_link",
            "ugv_uav_charging_pack",
            "SIM-UGV-CHARGER-001",
            snapshot.charging_energy_wh,
            self.parameters.charging_capacity_wh,
            self.parameters.charging_nominal_voltage,
            snapshot.charging_source_power_w,
        ))
        self.drive_power_pub.publish(Float32(data=float(snapshot.drive_power_w)))
        self.charging_power_pub.publish(
            Float32(data=float(snapshot.charging_source_power_w))
        )
        self.charging_output_pub.publish(
            Float32(data=float(snapshot.charging_output_power_w))
        )
        self.drive_remaining_pub.publish(
            Float32(data=float(snapshot.drive_energy_wh))
        )
        self.charging_remaining_pub.publish(
            Float32(data=float(snapshot.charging_energy_wh))
        )
        self.drive_consumed_pub.publish(
            Float32(data=float(snapshot.drive_consumed_wh))
        )
        self.charging_consumed_pub.publish(
            Float32(data=float(snapshot.charging_consumed_wh))
        )
        self.carried_mass_pub.publish(
            Float32(data=float(snapshot.total_carried_mass_kg))
        )
        self.charger_available_pub.publish(
            Bool(data=bool(snapshot.charger_available))
        )
        self.status_pub.publish(String(data=(
            f"DRIVE:{snapshot.drive_soc * 100.0:.1f}%:"
            f"{snapshot.drive_power_w:.1f}W | "
            f"UAV_CHARGER:{snapshot.charging_soc * 100.0:.1f}%:"
            f"{snapshot.charging_source_power_w:.1f}W | "
            f"cargo={snapshot.cargo_mass_kg:.2f}kg:"
            f"carried={snapshot.total_carried_mass_kg:.2f}kg"
        )))


def main(args=None):
    rclpy.init(args=args)
    node = UgvEnergyManager()
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
