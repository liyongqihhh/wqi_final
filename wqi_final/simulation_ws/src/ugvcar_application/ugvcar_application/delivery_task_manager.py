#!/usr/bin/env python3
# Copyright 2026 liyongqihhh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Run campus deliveries in an exactly optimized visit order."""

import math
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
from uav_navigation.road_network import RoadNetwork, RoadNetworkError
from uav_navigation.route_optimizer import optimize_visit_order

DEFAULT_WAIT_DURATION = 10.0
DEFAULT_START_POINT = "logistics_center"
FEEDBACK_LOG_INTERVAL = 1.0


def load_waypoints() -> dict:
    from ament_index_python.packages import get_package_share_directory
    yaml_path = (
        Path(get_package_share_directory("ugvcar_description"))
        / "config"
        / "delivery_waypoints.yaml"
    )
    if not yaml_path.exists():
        raise FileNotFoundError(f"waypoints config not found: {yaml_path}")
    with open(yaml_path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_road_network() -> RoadNetwork:
    from ament_index_python.packages import get_package_share_directory
    layout_path = (
        Path(get_package_share_directory("ugvcar_description"))
        / "config"
        / "campus_layout.yaml"
    )
    return RoadNetwork.from_yaml(layout_path)


def distance(a: dict, b: dict) -> float:
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def optimize_order(
    waypoints: dict,
    targets: list[str],
    start: str,
    route_distance=None,
) -> list[str]:
    """Find the exact shortest visit order that returns to the start."""
    unique_targets = list(dict.fromkeys(targets))
    metric = route_distance or (
        lambda first, second: distance(waypoints[first], waypoints[second])
    )
    plan = optimize_visit_order(
        len(unique_targets),
        lambda index: metric(start, unique_targets[index]),
        lambda origin, destination: metric(
            unique_targets[origin],
            unique_targets[destination],
        ),
        lambda index: metric(unique_targets[index], start),
    )
    return [unique_targets[index] for index in plan.order]


def waypoint_to_pose(wp: dict, navigator: BasicNavigator) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = wp["x"]
    pose.pose.position.y = wp["y"]
    pose.pose.position.z = 0.0
    half_yaw = wp["yaw"] / 2.0
    pose.pose.orientation.z = math.sin(half_yaw)
    pose.pose.orientation.w = math.cos(half_yaw)
    return pose


def wait_for_navigation(navigator: BasicNavigator) -> None:
    next_log_time = 0.0
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        now = time.monotonic()
        if feedback and now >= next_log_time:
            eta_seconds = Duration.from_msg(
                feedback.estimated_time_remaining
            ).nanoseconds / 1e9
            navigator.get_logger().info(
                f"  eta: {eta_seconds:.1f}s"
                f"  dist: {feedback.distance_remaining:.2f}m"
            )
            next_log_time = now + FEEDBACK_LOG_INTERVAL


def main():
    rclpy.init()
    navigator = BasicNavigator()

    navigator.declare_parameter("delivery_targets", ["teaching_building"])
    navigator.declare_parameter("start_point", DEFAULT_START_POINT)
    navigator.declare_parameter("wait_duration", DEFAULT_WAIT_DURATION)

    targets_raw = (
        navigator.get_parameter("delivery_targets")
        .get_parameter_value()
        .string_array_value
    )
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

    navigator.waitUntilNav2Active(localizer="map_server")

    try:
        road_network = load_road_network()

        def road_distance(first, second):
            first_waypoint = waypoints[first]
            second_waypoint = waypoints[second]
            return road_network.distance(
                (first_waypoint["x"], first_waypoint["y"]),
                (second_waypoint["x"], second_waypoint["y"]),
            )

        ordered = optimize_order(
            waypoints, targets, start, route_distance=road_distance
        )
        route_source = "CAMPUS_ROAD_GRAPH"
    except RoadNetworkError as error:
        navigator.get_logger().warning(
            f"road-graph optimization unavailable: {error}"
        )
        ordered = optimize_order(waypoints, targets, start)
        route_source = "EUCLIDEAN_FALLBACK"
    navigator.get_logger().info(
        f"delivery route [{route_source}]: {start} -> "
        f"{' -> '.join(ordered)} -> {start}"
    )

    total_success = 0
    total_failed = 0
    start_time = time.time()

    for name in ordered:
        navigator.get_logger().info(f"navigating to {name}")
        pose = waypoint_to_pose(waypoints[name], navigator)
        navigator.goToPose(pose)
        wait_for_navigation(navigator)

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
    wait_for_navigation(navigator)
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info(f"returned to {start}")
        total_success += 1
    else:
        navigator.get_logger().error("return trip failed")

    elapsed = time.time() - start_time
    navigator.get_logger().info(
        f"delivery complete: {total_success} succeeded, "
        f"{total_failed} failed, {elapsed:.1f}s total"
    )

    rclpy.shutdown()


if __name__ == "__main__":
    main()
