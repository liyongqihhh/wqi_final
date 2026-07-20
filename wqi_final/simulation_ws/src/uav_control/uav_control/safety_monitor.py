import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan, PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker

from uav_control.safety_geometry import (
    is_diagonal_ground_return,
    minimum_valid_scan_range,
    minimum_obstacle_distances,
)


class SafetyMonitor(Node):
    """Fuse the 3D lidar and short-range sensors into a collision envelope."""

    RANGE_SENSORS = (
        "down",
        "front_down",
        "rear_down",
        "left_down",
        "right_down",
    )
    BLIND_SPOT_SENSORS = RANGE_SENSORS[1:]

    def __init__(self) -> None:
        super().__init__("safety_monitor")
        self.declare_parameter("safety_radius", 1.8)
        self.declare_parameter("safety_center_height", 0.18)
        self.declare_parameter("self_filter_radius", 0.58)
        self.declare_parameter("lidar_height", 0.45)
        self.declare_parameter("platform_protected_min_height", 1.2)
        self.declare_parameter("down_sensor_height", 0.09)
        self.declare_parameter("diagonal_sensor_height", 0.11)
        self.declare_parameter("diagonal_downward_angle", 0.78539816339)
        self.declare_parameter("diagonal_ground_tolerance", 0.08)
        self.declare_parameter("blind_spot_threshold", 1.8)
        self.declare_parameter("sensor_timeout", 1.0)
        self.declare_parameter("ground_return_tolerance", 0.18)
        self.declare_parameter("publish_rate", 10.0)

        self.safety_radius = float(self.get_parameter("safety_radius").value)
        self.center_height = float(
            self.get_parameter("safety_center_height").value
        )
        self.self_filter_radius = float(
            self.get_parameter("self_filter_radius").value
        )
        self.lidar_height = float(self.get_parameter("lidar_height").value)
        self.platform_protected_min_height = float(
            self.get_parameter("platform_protected_min_height").value
        )
        self.down_sensor_height = float(
            self.get_parameter("down_sensor_height").value
        )
        self.diagonal_sensor_height = float(
            self.get_parameter("diagonal_sensor_height").value
        )
        self.diagonal_downward_angle = float(
            self.get_parameter("diagonal_downward_angle").value
        )
        self.diagonal_ground_tolerance = float(
            self.get_parameter("diagonal_ground_tolerance").value
        )
        self.lidar_to_down_sensor = self.lidar_height - self.down_sensor_height
        self.blind_spot_threshold = float(
            self.get_parameter("blind_spot_threshold").value
        )
        self.sensor_timeout = float(self.get_parameter("sensor_timeout").value)
        self.ground_tolerance = float(
            self.get_parameter("ground_return_tolerance").value
        )
        publish_rate = float(self.get_parameter("publish_rate").value)

        self.lock = threading.Lock()
        self.lidar_updated_at = None
        self.lidar_min_distance = math.inf
        self.lidar_platform_min_distance = math.inf
        self.range_values = {name: math.inf for name in self.RANGE_SENSORS}
        self.range_updated_at = {name: None for name in self.RANGE_SENSORS}

        self.create_subscription(
            PointCloud2,
            "lidar/points",
            self._point_cloud_callback,
            qos_profile_sensor_data,
        )
        self.range_publishers = {}
        for name in self.RANGE_SENSORS:
            self.range_publishers[name] = self.create_publisher(
                Range,
                f"range/{name}",
                qos_profile_sensor_data,
            )
            self.create_subscription(
                LaserScan,
                f"range_raw/{name}",
                lambda message, sensor=name: self._range_callback(sensor, message),
                qos_profile_sensor_data,
            )

        self.blocked_pub = self.create_publisher(Bool, "safety/blocked", 10)
        self.status_pub = self.create_publisher(String, "safety/status", 10)
        self.platform_blocked_pub = self.create_publisher(
            Bool, "safety/platform_blocked", 10
        )
        self.platform_status_pub = self.create_publisher(
            String, "safety/platform_status", 10
        )
        self.minimum_pub = self.create_publisher(
            Float32, "safety/min_distance", 10
        )
        self.clearance_pub = self.create_publisher(
            Float32, "safety/ground_clearance", 10
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            Marker, "safety_sphere", marker_qos
        )
        self.timer = self.create_timer(
            1.0 / max(publish_rate, 1.0), self._publish_safety_state
        )
        self.get_logger().info(
            f"UAV safety monitor is ready (radius={self.safety_radius:.2f} m)"
        )

    def _range_callback(self, sensor: str, message: LaserScan) -> None:
        value = minimum_valid_scan_range(
            message.ranges,
            float(message.range_min),
            float(message.range_max),
        )
        with self.lock:
            self.range_values[sensor] = value
            self.range_updated_at[sensor] = time.monotonic()
        public_message = Range()
        public_message.header = message.header
        public_message.radiation_type = Range.INFRARED
        public_message.field_of_view = max(
            float(message.angle_max - message.angle_min),
            0.01,
        )
        public_message.min_range = float(message.range_min)
        public_message.max_range = float(message.range_max)
        public_message.range = value
        self.range_publishers[sensor].publish(public_message)

    def _point_cloud_callback(self, message: PointCloud2) -> None:
        with self.lock:
            ground_clearance = self.range_values["down"]
        try:
            points = point_cloud2.read_points(
                message,
                field_names=("x", "y", "z"),
                skip_nans=True,
            )
            minimum, platform_minimum = minimum_obstacle_distances(
                points,
                self.lidar_height,
                self.center_height,
                ground_clearance,
                self.lidar_to_down_sensor,
                self.ground_tolerance,
                self.self_filter_radius,
                self.platform_protected_min_height,
            )
        except (AssertionError, KeyError, TypeError, ValueError) as error:
            self.get_logger().error(f"Cannot read UAV lidar point cloud: {error}")
            return
        with self.lock:
            self.lidar_min_distance = minimum
            self.lidar_platform_min_distance = platform_minimum
            self.lidar_updated_at = time.monotonic()

    def _snapshot(self):
        with self.lock:
            return (
                self.lidar_updated_at,
                self.lidar_min_distance,
                self.lidar_platform_min_distance,
                dict(self.range_values),
                dict(self.range_updated_at),
            )

    def _publish_marker(self, blocked: bool) -> None:
        marker = Marker()
        marker.header.frame_id = "uav/base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "uav_safety_envelope"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.z = self.center_height
        marker.pose.orientation.w = 1.0
        diameter = 2.0 * self.safety_radius
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        if blocked:
            marker.color.r = 0.95
            marker.color.g = 0.08
            marker.color.b = 0.05
        else:
            marker.color.r = 0.05
            marker.color.g = 0.85
            marker.color.b = 0.25
        marker.color.a = 0.18
        self.marker_pub.publish(marker)

    def _publish_safety_state(self) -> None:
        now = time.monotonic()
        (
            lidar_time,
            lidar_minimum,
            platform_lidar_minimum,
            ranges,
            range_times,
        ) = self._snapshot()
        stale = []
        if lidar_time is None or now - lidar_time > self.sensor_timeout:
            stale.append("top_3d_lidar")
        for name, updated_at in range_times.items():
            if updated_at is None or now - updated_at > self.sensor_timeout:
                stale.append(name)

        blocked_sensors = []
        obstacle_distances = [lidar_minimum]
        if lidar_minimum <= self.safety_radius:
            blocked_sensors.append("top_3d_lidar")
        platform_blocked_sensors = []
        if platform_lidar_minimum <= self.safety_radius:
            platform_blocked_sensors.append("top_3d_lidar")
        for name in self.BLIND_SPOT_SENSORS:
            distance = ranges[name]
            sees_ground = is_diagonal_ground_return(
                distance,
                ranges["down"],
                self.down_sensor_height,
                self.diagonal_sensor_height,
                self.diagonal_downward_angle,
                self.diagonal_ground_tolerance,
            )
            if (
                math.isfinite(distance)
                and not sees_ground
            ):
                obstacle_distances.append(distance)
                if distance < self.blind_spot_threshold:
                    blocked_sensors.append(name)

        blocked = bool(stale or blocked_sensors)
        platform_blocked = bool(stale or platform_blocked_sensors)
        finite_distances = [
            value
            for value in obstacle_distances
            if math.isfinite(value)
        ]
        minimum = min(finite_distances, default=math.inf)
        if stale:
            status = "SENSOR_STALE:" + ",".join(stale)
        elif blocked_sensors:
            status = "BLOCKED:" + ",".join(blocked_sensors)
        else:
            status = "CLEAR"
        if stale:
            platform_status = "SENSOR_STALE:" + ",".join(stale)
        elif platform_blocked_sensors:
            platform_status = "BLOCKED:" + ",".join(platform_blocked_sensors)
        else:
            platform_status = "CLEAR"

        self.blocked_pub.publish(Bool(data=blocked))
        self.status_pub.publish(String(data=status))
        self.platform_blocked_pub.publish(Bool(data=platform_blocked))
        self.platform_status_pub.publish(String(data=platform_status))
        self.minimum_pub.publish(Float32(data=float(minimum)))
        self.clearance_pub.publish(Float32(data=float(ranges["down"])))
        self._publish_marker(blocked)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
