#!/usr/bin/env python3
"""Wait for real simulated UGV telemetry before starting Nav2."""

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def conflicting_nodes(nodes):
    reserved = {
        "robot_state_publisher", "gazebo", "gazebo_ros2_control", "controller_manager",
        "controller_server", "planner_server", "bt_navigator", "map_server",
        "global_filter_mask_server", "local_filter_mask_server",
    }
    return sorted({name for name, namespace in nodes if namespace == "/" and name in reserved})


class UgvStartupProbe(Node):
    def __init__(self):
        super().__init__("ugv_startup_probe")
        self.declare_parameter("timeout", 120.0)
        self.declare_parameter("preflight_only", False)
        self.received = set()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, "/ground_truth/odom", self.on_ground_truth, qos_profile_sensor_data
        )
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_odom(self, message):
        self.received.add("odom")

    def on_ground_truth(self, message):
        self.received.add("ground_truth")

    def on_scan(self, message):
        self.received.add("scan")

    def ready(self):
        return self.received == {"odom", "ground_truth", "scan"} and (
            self.buffer.can_transform("odom", "base_link", rclpy.time.Time())
        )


def check_existing_simulation(node):
    deadline = time.monotonic() + 2.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    conflicts = conflicting_nodes(node.get_node_names_and_namespaces())
    if conflicts:
        node.get_logger().error(
            "Previous simulation/navigation nodes are still running: " + ", ".join(conflicts)
            + ". Stop the old launch terminals with Ctrl+C before starting campus_ugv."
        )
        return False
    return rclpy.ok()


def main():
    rclpy.init()
    node = UgvStartupProbe()
    success = False
    try:
        if node.get_parameter("preflight_only").value:
            success = check_existing_simulation(node)
            return 0 if success else 1
        deadline = time.monotonic() + node.get_parameter("timeout").value
        node.get_logger().info("Waiting for UGV odom, ground truth, lidar and TF...")
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                node.get_logger().info("UGV telemetry ready; starting navigation.")
                success = True
                break
        if not success and rclpy.ok():
            node.get_logger().error("UGV startup timed out; inspect Gazebo and controllers.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    raise SystemExit(main())
