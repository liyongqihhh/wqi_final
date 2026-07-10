#!/usr/bin/env python3
"""校园配送任务管理器 — 从物流中心出发，按最优顺序访问目标建筑，每站停留 N 秒。"""

import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

DEFAULT_WAIT_DURATION = 10.0
DEFAULT_START_POINT = "logistics_center"


def load_waypoints() -> dict:
    pkg_share = Path(__file__).resolve().parents[2] / "src" / "ugvcar_description"
    yaml_path = pkg_share / "config" / "delivery_waypoints.yaml"
    if not yaml_path.exists():
        raise FileNotFoundError(f"waypoints config not found: {yaml_path}")
    with open(yaml_path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def distance(a: dict, b: dict) -> float:
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def optimize_order(waypoints: dict, targets: list[str], start: str) -> list[str]:
    """贪心最近邻排序：从起点出发，每次选最近的下一个目标。"""
    remaining = set(targets)
    ordered = []
    current = start
    while remaining:
        best = min(remaining, key=lambda t: distance(waypoints[current], waypoints[t]))
        ordered.append(best)
        remaining.remove(best)
        current = best
    return ordered


def waypoint_to_pose(wp: dict, navigator: BasicNavigator) -> PoseStamped:
    from tf_transformations import quaternion_from_euler

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = wp["x"]
    pose.pose.position.y = wp["y"]
    pose.pose.position.z = 0.0
    q = quaternion_from_euler(0, 0, wp["yaw"])
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]
    return pose


def main():
    rclpy.init()
    navigator = BasicNavigator()

    navigator.declare_parameter("delivery_targets", [""])
    navigator.declare_parameter("start_point", DEFAULT_START_POINT)
    navigator.declare_parameter("wait_duration", DEFAULT_WAIT_DURATION)

    targets_raw = navigator.get_parameter("delivery_targets").get_parameter_value().string_array_value
    start = navigator.get_parameter("start_point").get_parameter_value().string_value
    wait_duration = navigator.get_parameter("wait_duration").get_parameter_value().double_value

    waypoints = load_waypoints()

    if start not in waypoints:
        navigator.get_logger().error(f"unknown start point: {start}")
        rclpy.shutdown()
        return

    invalid = [t for t in targets_raw if t not in waypoints]
    if invalid:
        navigator.get_logger().error(f"unknown targets: {invalid}")
        rclpy.shutdown()
        return

    targets = [t.strip() for t in targets_raw if t.strip() and t.strip() in waypoints]
    if not targets:
        navigator.get_logger().error("no valid delivery targets")
        rclpy.shutdown()
        return

    ordered = optimize_order(waypoints, targets, start)
    navigator.get_logger().info(f"delivery route: {start} -> {' -> '.join(ordered)} -> {start}")

    init_pose = waypoint_to_pose(waypoints[start], navigator)
    navigator.setInitialPose(init_pose)
    navigator.waitUntilNav2Active()

    total_success = 0
    total_failed = 0
    start_time = time.time()

    for name in ordered:
        navigator.get_logger().info(f"navigating to {name}")
        pose = waypoint_to_pose(waypoints[name], navigator)
        navigator.goToPose(pose)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                navigator.get_logger().info(
                    f"  eta: {feedback.estimated_time_remaining:.1f}s"
                    f"  dist: {feedback.distance_remaining:.2f}m"
                )

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(f"arrived at {name}")
            total_success += 1
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn(f"navigation to {name} canceled")
            total_failed += 1
        else:
            navigator.get_logger().error(f"navigation to {name} failed")
            total_failed += 1
            continue

        navigator.get_logger().info(f"waiting {wait_duration:.0f}s at {name}")
        time.sleep(wait_duration)

    # 返回物流中心
    navigator.get_logger().info(f"returning to {start}")
    home_pose = waypoint_to_pose(waypoints[start], navigator)
    navigator.goToPose(home_pose)
    while not navigator.isTaskComplete():
        pass
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f"returned to {start}")
        total_success += 1
    else:
        navigator.get_logger().error("return trip failed")

    elapsed = time.time() - start_time
    navigator.get_logger().info(
        f"delivery complete: {total_success} succeeded, {total_failed} failed, {elapsed:.1f}s total"
    )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
