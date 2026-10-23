"""Align only B-car odometry; the original A-car localizer is unchanged."""

import math
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class RelayLocalizer(Node):
    def __init__(self):
        super().__init__("ground_truth_localizer")
        self.wheel = None
        self.broadcaster = TransformBroadcaster(self)
        self.last_command = None
        self.last_command_ns = None
        self.command_pub = self.create_publisher(Twist, "drive_cmd", 10)
        self.create_subscription(Twist, "cmd_vel_safe", self._command, 10)
        self.create_timer(.05, self._watchdog)
        self.create_subscription(Odometry, "odom", self._wheel, qos_profile_sensor_data)
        self.create_subscription(Odometry, "ground_truth/odom", self._truth, qos_profile_sensor_data)

    def _command(self, message):
        if not all(math.isfinite(v) for v in (message.linear.x, message.angular.z)):
            return
        command = Twist()
        command.linear.x = max(-.3, min(.4, message.linear.x))
        command.angular.z = max(-.65, min(.65, message.angular.z))
        self.last_command, self.last_command_ns = command, self.get_clock().now().nanoseconds
        self.command_pub.publish(command)

    def _watchdog(self):
        now = self.get_clock().now().nanoseconds
        valid = self.last_command_ns is not None and 0 <= now - self.last_command_ns <= 500_000_000
        self.command_pub.publish(self.last_command if valid else Twist())

    def _wheel(self, message):
        self.wheel = message

    def _truth(self, message):
        if self.wheel is None:
            return
        p, w = message.pose.pose, self.wheel.pose.pose
        angle = yaw(p.orientation) - yaw(w.orientation)
        c, s = math.cos(angle), math.sin(angle)
        transform = TransformStamped()
        transform.header.stamp = (self.get_clock().now() + Duration(seconds=.2)).to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "ugv_b/odom"
        transform.transform.translation.x = p.position.x - (c*w.position.x - s*w.position.y)
        transform.transform.translation.y = p.position.y - (s*w.position.x + c*w.position.y)
        transform.transform.rotation.z = math.sin(angle / 2)
        transform.transform.rotation.w = math.cos(angle / 2)
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = RelayLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
