#!/usr/bin/env python3
"""Publish map->odom correction from Gazebo ground truth and wheel odometry."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(quaternion) -> float:
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class GazeboGroundTruthLocalizer(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_ground_truth_localizer")
        self._wheel_odom = None
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            "/odom",
            self._on_wheel_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self._on_ground_truth,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Waiting for /odom and /ground_truth/odom to align map with Gazebo"
        )

    def _on_wheel_odom(self, message: Odometry) -> None:
        self._wheel_odom = message

    def _on_ground_truth(self, ground_truth: Odometry) -> None:
        if self._wheel_odom is None:
            return

        wheel_pose = self._wheel_odom.pose.pose
        ground_pose = ground_truth.pose.pose
        wheel_yaw = yaw_from_quaternion(wheel_pose.orientation)
        ground_yaw = yaw_from_quaternion(ground_pose.orientation)
        map_to_odom_yaw = ground_yaw - wheel_yaw

        cos_yaw = math.cos(map_to_odom_yaw)
        sin_yaw = math.sin(map_to_odom_yaw)
        rotated_wheel_x = (
            cos_yaw * wheel_pose.position.x
            - sin_yaw * wheel_pose.position.y
        )
        rotated_wheel_y = (
            sin_yaw * wheel_pose.position.x
            + cos_yaw * wheel_pose.position.y
        )

        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now() + Duration(seconds=0.2)
        ).to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = (
            ground_pose.position.x - rotated_wheel_x
        )
        transform.transform.translation.y = (
            ground_pose.position.y - rotated_wheel_y
        )
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(map_to_odom_yaw / 2.0)
        transform.transform.rotation.w = math.cos(map_to_odom_yaw / 2.0)
        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = GazeboGroundTruthLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
