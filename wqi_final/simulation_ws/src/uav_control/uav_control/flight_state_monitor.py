import math

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8, String


STATE_NAMES = {
    0: "LANDED",
    1: "FLYING",
    2: "TAKING_OFF",
    3: "LANDING",
}


class FlightStateMonitor(Node):
    def __init__(self) -> None:
        super().__init__("flight_state_monitor")
        self.declare_parameter("path_publish_rate", 2.0)
        self.declare_parameter("path_min_distance", 0.1)
        self.declare_parameter("path_max_poses", 3000)
        self.path_min_distance = float(self.get_parameter("path_min_distance").value)
        self.path_max_poses = int(self.get_parameter("path_max_poses").value)
        publish_rate = float(self.get_parameter("path_publish_rate").value)

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path = Path()
        self.path.header.frame_id = "map"
        self.last_position = None
        self.last_state = None
        self.path_pub = self.create_publisher(Path, "path", 10)
        self.state_pub = self.create_publisher(String, "flight_state", transient_qos)
        self.create_subscription(Odometry, "odom", self._odom_callback, 30)
        self.create_subscription(Int8, "state", self._state_callback, 10)
        self.create_timer(1.0 / max(publish_rate, 0.2), self._publish_path)

    def _odom_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        if self.last_position is not None:
            distance = math.sqrt(
                (position.x - self.last_position[0]) ** 2
                + (position.y - self.last_position[1]) ** 2
                + (position.z - self.last_position[2]) ** 2
            )
            if distance < self.path_min_distance:
                return
        pose = message.pose
        from geometry_msgs.msg import PoseStamped

        stamped = PoseStamped()
        stamped.header = message.header
        stamped.header.frame_id = "map"
        stamped.pose = pose.pose
        self.path.poses.append(stamped)
        if len(self.path.poses) > self.path_max_poses:
            self.path.poses = self.path.poses[-self.path_max_poses:]
        self.last_position = (position.x, position.y, position.z)

    def _state_callback(self, message: Int8) -> None:
        state_name = STATE_NAMES.get(int(message.data), f"UNKNOWN_{message.data}")
        if state_name == self.last_state:
            return
        self.last_state = state_name
        self.state_pub.publish(String(data=state_name))
        self.get_logger().info(f"Flight state: {state_name}")

    def _publish_path(self) -> None:
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = FlightStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
