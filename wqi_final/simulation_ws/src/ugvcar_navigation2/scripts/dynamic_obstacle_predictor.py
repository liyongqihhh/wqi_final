#!/usr/bin/env python3
import math

from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int32, String
from ugvcar_navigation2_interfaces.msg import (
    DynamicObstacle,
    DynamicObstacleArray,
)

from dynamic_obstacle_prediction import (
    CollisionRiskLatch,
    DynamicObstacleTracker,
    PoseHistory,
    body_to_world,
    calculate_collision_risk,
    choose_drivable_collision_risk,
    choose_open_avoidance_side,
    compose_pose,
    effective_prediction_radius,
    forward_escape_guard_points,
    plan_goal_changed,
    prediction_cloud_points,
    path_tangent,
    scan_clusters,
    world_to_body,
)


def _yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


class DynamicObstaclePredictor(Node):
    """Publish velocity-aware collision points for Nav2 replanning."""

    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_predictor")
        self.declare_parameter("tracking_range", 15.0)
        self.declare_parameter("cluster_gap", 0.45)
        self.declare_parameter("minimum_cluster_samples", 3)
        self.declare_parameter("maximum_cluster_diameter", 1.8)
        self.declare_parameter("association_distance", 0.75)
        self.declare_parameter("velocity_alpha", 0.35)
        self.declare_parameter("track_timeout", 0.8)
        self.declare_parameter("maximum_track_speed", 1.8)
        self.declare_parameter("association_position_gate", 0.18)
        self.declare_parameter("diameter_tolerance", 0.55)
        self.declare_parameter("track_history_window", 1.4)
        self.declare_parameter("minimum_motion_duration", 0.35)
        self.declare_parameter("minimum_motion_displacement", 0.12)
        self.declare_parameter("minimum_direction_consistency", 0.65)
        self.declare_parameter("maximum_fit_residual", 0.14)
        self.declare_parameter("maximum_prediction_age", 0.25)
        self.declare_parameter("minimum_dynamic_speed", 0.12)
        self.declare_parameter("maximum_dynamic_speed", 1.5)
        self.declare_parameter("minimum_observations", 5)
        self.declare_parameter("robot_radius", 0.22)
        self.declare_parameter("default_obstacle_radius", 0.45)
        self.declare_parameter("safety_margin", 0.30)
        self.declare_parameter("response_time", 5.00)
        self.declare_parameter("braking_deceleration", 0.60)
        self.declare_parameter("lateral_maneuver_acceleration", 0.50)
        self.declare_parameter("collision_horizon", 15.0)
        self.declare_parameter("planning_buffer", 1.50)
        self.declare_parameter("minimum_closing_speed", 0.05)
        self.declare_parameter("nominal_robot_speed", 0.40)
        self.declare_parameter("collision_corridor_margin", 0.50)
        self.declare_parameter("proximity_guard_distance", 6.00)
        # A 16 m spatial cap reaches the predicted meeting point of a 0.65 m/s
        # obstacle catching the 0.40 m/s UGV from 6 m behind. The wider 0.75 m
        # sampling interval keeps the point-cloud cost bounded in VirtualBox.
        self.declare_parameter("guard_sample_spacing", 0.75)
        self.declare_parameter("maximum_guard_length", 16.00)
        # The guard closes the obstacle side of a 5.5 m road without forcing
        # the combined UGV-UAV footprint onto the opposite road edge.
        self.declare_parameter("guard_side_offset", 1.60)
        self.declare_parameter("guard_tail_length", 3.00)
        # The lethal prediction footprint includes the physical obstacle and
        # the requested clearance. Non-lethal inflation alone does not force
        # SmacPlanner to preserve that clearance at a moving encounter.
        self.declare_parameter("risk_point_radius", 0.65)
        self.declare_parameter("risk_disc_samples", 12)
        self.declare_parameter("risk_clear_confirmations", 5)
        self.declare_parameter("risk_lost_confirmations", 30)
        self.declare_parameter("risk_release_distance", 3.50)
        self.declare_parameter("risk_reassociation_distance", 1.20)
        self.declare_parameter("pose_history_duration", 3.0)
        self.declare_parameter("maximum_pose_extrapolation", 0.12)
        self.declare_parameter("sensor_offset_x", 0.0)
        self.declare_parameter("sensor_offset_y", 0.0)
        self.declare_parameter("sensor_offset_yaw", 0.0)
        self.declare_parameter("reversal_minimum_speed", 0.08)
        self.declare_parameter("reversal_cosine_threshold", -0.25)
        # Near-field occupancy comes from the real scan. Excluding predicted
        # samples around the current footprint prevents a future crossing from
        # making the planner's current start cell lethal.
        self.declare_parameter("robot_exclusion_radius", 1.20)
        self.declare_parameter("enable_near_escape_gate", False)
        self.declare_parameter("escape_gate_minimum_forward_distance", 2.40)
        self.declare_parameter("escape_gate_activation_margin", 0.50)
        self.declare_parameter("escape_gate_side_offset", 1.80)
        self.declare_parameter("escape_gate_safe_side_intrusion", 0.35)
        self.declare_parameter("road_side_lateral_offset", 1.20)
        self.declare_parameter("road_side_blocked_cost", 65.0)
        self.declare_parameter("road_side_preference_margin", 15.0)

        self.tracking_range = float(self.get_parameter("tracking_range").value)
        self.cluster_gap = float(self.get_parameter("cluster_gap").value)
        self.minimum_cluster_samples = int(
            self.get_parameter("minimum_cluster_samples").value
        )
        self.maximum_cluster_diameter = float(
            self.get_parameter("maximum_cluster_diameter").value
        )
        self.minimum_dynamic_speed = float(
            self.get_parameter("minimum_dynamic_speed").value
        )
        self.maximum_dynamic_speed = float(
            self.get_parameter("maximum_dynamic_speed").value
        )
        self.minimum_observations = int(
            self.get_parameter("minimum_observations").value
        )
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.default_obstacle_radius = float(
            self.get_parameter("default_obstacle_radius").value
        )
        self.safety_margin = float(
            self.get_parameter("safety_margin").value
        )
        self.response_time = float(
            self.get_parameter("response_time").value
        )
        self.braking_deceleration = float(
            self.get_parameter("braking_deceleration").value
        )
        self.lateral_maneuver_acceleration = float(
            self.get_parameter("lateral_maneuver_acceleration").value
        )
        self.collision_horizon = float(
            self.get_parameter("collision_horizon").value
        )
        self.planning_buffer = float(
            self.get_parameter("planning_buffer").value
        )
        self.minimum_closing_speed = float(
            self.get_parameter("minimum_closing_speed").value
        )
        self.nominal_robot_speed = float(
            self.get_parameter("nominal_robot_speed").value
        )
        self.collision_corridor_margin = float(
            self.get_parameter("collision_corridor_margin").value
        )
        self.proximity_guard_distance = float(
            self.get_parameter("proximity_guard_distance").value
        )
        self.guard_sample_spacing = float(
            self.get_parameter("guard_sample_spacing").value
        )
        self.maximum_guard_length = float(
            self.get_parameter("maximum_guard_length").value
        )
        self.guard_side_offset = float(
            self.get_parameter("guard_side_offset").value
        )
        self.guard_tail_length = float(
            self.get_parameter("guard_tail_length").value
        )
        requested_risk_point_radius = float(
            self.get_parameter("risk_point_radius").value
        )
        self.risk_point_radius = effective_prediction_radius(
            requested_risk_point_radius,
            self.default_obstacle_radius,
            self.safety_margin,
        )
        self.risk_disc_samples = int(
            self.get_parameter("risk_disc_samples").value
        )
        self.robot_exclusion_radius = float(
            self.get_parameter("robot_exclusion_radius").value
        )
        self.enable_near_escape_gate = bool(
            self.get_parameter("enable_near_escape_gate").value
        )
        self.escape_gate_minimum_forward_distance = float(
            self.get_parameter(
                "escape_gate_minimum_forward_distance"
            ).value
        )
        self.escape_gate_activation_margin = float(
            self.get_parameter("escape_gate_activation_margin").value
        )
        self.escape_gate_side_offset = float(
            self.get_parameter("escape_gate_side_offset").value
        )
        self.escape_gate_safe_side_intrusion = float(
            self.get_parameter("escape_gate_safe_side_intrusion").value
        )
        self.sensor_offset = (
            float(self.get_parameter("sensor_offset_x").value),
            float(self.get_parameter("sensor_offset_y").value),
            float(self.get_parameter("sensor_offset_yaw").value),
        )
        self.road_side_lateral_offset = float(
            self.get_parameter("road_side_lateral_offset").value
        )
        self.road_side_blocked_cost = float(
            self.get_parameter("road_side_blocked_cost").value
        )
        self.road_side_preference_margin = float(
            self.get_parameter("road_side_preference_margin").value
        )
        self.tracker = DynamicObstacleTracker(
            association_distance=float(
                self.get_parameter("association_distance").value
            ),
            velocity_alpha=float(self.get_parameter("velocity_alpha").value),
            track_timeout=float(self.get_parameter("track_timeout").value),
            maximum_track_speed=float(
                self.get_parameter("maximum_track_speed").value
            ),
            position_gate=float(
                self.get_parameter("association_position_gate").value
            ),
            diameter_tolerance=float(
                self.get_parameter("diameter_tolerance").value
            ),
            history_window=float(
                self.get_parameter("track_history_window").value
            ),
            minimum_motion_duration=float(
                self.get_parameter("minimum_motion_duration").value
            ),
            minimum_displacement=float(
                self.get_parameter("minimum_motion_displacement").value
            ),
            minimum_direction_consistency=float(
                self.get_parameter("minimum_direction_consistency").value
            ),
            maximum_fit_residual=float(
                self.get_parameter("maximum_fit_residual").value
            ),
            maximum_prediction_age=float(
                self.get_parameter("maximum_prediction_age").value
            ),
            reversal_minimum_speed=float(
                self.get_parameter("reversal_minimum_speed").value
            ),
            reversal_cosine_threshold=float(
                self.get_parameter("reversal_cosine_threshold").value
            ),
        )
        self.pose_history = PoseHistory(
            history_duration=float(
                self.get_parameter("pose_history_duration").value
            ),
            maximum_extrapolation=float(
                self.get_parameter("maximum_pose_extrapolation").value
            ),
        )
        self.latest_plan = ()
        self.road_mask = None
        self.risk_latch = CollisionRiskLatch(
            clear_confirmations=int(
                self.get_parameter("risk_clear_confirmations").value
            ),
            lost_confirmations=int(
                self.get_parameter("risk_lost_confirmations").value
            ),
            release_distance=float(
                self.get_parameter("risk_release_distance").value
            ),
            reassociation_distance=float(
                self.get_parameter("risk_reassociation_distance").value
            ),
        )

        self.prediction_pub = self.create_publisher(
            LaserScan, "/scan_dynamic_predictions", qos_profile_sensor_data
        )
        self.prediction_points_pub = self.create_publisher(
            PointCloud2,
            "/points_dynamic_predictions",
            qos_profile_sensor_data,
        )
        self.status_pub = self.create_publisher(
            String, "/ugv/dynamic_replanning/status", 10
        )
        self.track_count_pub = self.create_publisher(
            Int32, "/ugv/dynamic_replanning/tracked_obstacles", 10
        )
        self.dynamic_tracks_pub = self.create_publisher(
            DynamicObstacleArray,
            "/ugv/tracked_dynamic_obstacles",
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/ground_truth/odom", self._ground_truth_callback, 20
        )
        self.create_subscription(Path, "/plan", self._plan_callback, 10)
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            "/local_keepout_filter_mask",
            self._road_mask_callback,
            transient_qos,
        )
        self.get_logger().info(
            "UGV 360-degree velocity-aware collision prediction is ready"
        )

    @staticmethod
    def _pose(message: Odometry) -> tuple[float, float, float]:
        position = message.pose.pose.position
        return (
            float(position.x),
            float(position.y),
            _yaw_from_odometry(message),
        )

    def _ground_truth_callback(self, message: Odometry) -> None:
        timestamp = self._message_time(message)
        self.pose_history.add(timestamp, self._pose(message))

    def _plan_callback(self, message: Path) -> None:
        next_plan = tuple(
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
            )
            for pose in message.poses
        )
        # Planner recovery can publish a transient empty Path before the
        # replacement arrives.  Treating that as a new mission resets the
        # latched passing side and makes the UGV cross a moving obstacle's
        # trajectory on the next plan.
        if not next_plan:
            return
        # A risk may be detected while the robot is still parked and has no
        # route tangent. Do not carry that provisional passing side into the
        # newly commanded mission. Periodic replans to the same goal retain the
        # existing world-frame guard, so the path remains stable.
        if plan_goal_changed(self.latest_plan, next_plan):
            self.risk_latch.reset()
        self.latest_plan = next_plan

    def _road_mask_callback(self, message: OccupancyGrid) -> None:
        self.road_mask = message

    def _road_mask_cost(self, x: float, y: float) -> float:
        message = self.road_mask
        if message is None or message.info.resolution <= 0.0:
            return 100.0
        origin = message.info.origin
        yaw = math.atan2(
            2.0 * origin.orientation.w * origin.orientation.z,
            1.0 - 2.0 * origin.orientation.z * origin.orientation.z,
        )
        dx = float(x) - float(origin.position.x)
        dy = float(y) - float(origin.position.y)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        column = int(math.floor(local_x / message.info.resolution))
        row = int(math.floor(local_y / message.info.resolution))
        if (
            column < 0
            or row < 0
            or column >= message.info.width
            or row >= message.info.height
        ):
            return 100.0
        value = int(message.data[row * message.info.width + column])
        return 100.0 if value < 0 else float(value)

    def _road_side_preference(self, base_pose, planning_direction):
        if self.road_mask is None or planning_direction is None:
            return None
        direction = planning_direction
        length = math.hypot(*direction)
        if length <= 1.0e-6:
            return None
        forward = (direction[0] / length, direction[1] / length)
        left = (-forward[1], forward[0])
        side_costs = {"left": [], "right": []}
        for side, sign in (("left", 1.0), ("right", -1.0)):
            for ahead in (0.5, 1.5, 2.5, 3.5):
                for lateral_delta in (-0.40, 0.0, 0.40):
                    lateral = (
                        self.road_side_lateral_offset + lateral_delta
                    )
                    side_costs[side].append(self._road_mask_cost(
                        base_pose[0]
                        + ahead * forward[0]
                        + sign * lateral * left[0],
                        base_pose[1]
                        + ahead * forward[1]
                        + sign * lateral * left[1],
                    ))
        return choose_open_avoidance_side(
            side_costs["left"],
            side_costs["right"],
            self.road_side_blocked_cost,
            self.road_side_preference_margin,
        )

    def _calculate_risk(
        self,
        track,
        base_pose,
        robot_velocity,
        planning_direction,
        preferred_avoidance_side,
    ):
        return calculate_collision_risk(
            track=track,
            robot_pose=base_pose,
            robot_velocity=robot_velocity,
            robot_radius=self.robot_radius,
            default_obstacle_radius=self.default_obstacle_radius,
            safety_margin=self.safety_margin,
            response_time=self.response_time,
            braking_deceleration=self.braking_deceleration,
            lateral_maneuver_acceleration=(
                self.lateral_maneuver_acceleration
            ),
            collision_horizon=self.collision_horizon,
            planning_buffer=self.planning_buffer,
            minimum_closing_speed=self.minimum_closing_speed,
            nominal_robot_speed=self.nominal_robot_speed,
            collision_corridor_margin=self.collision_corridor_margin,
            proximity_guard_distance=self.proximity_guard_distance,
            guard_sample_spacing=self.guard_sample_spacing,
            maximum_guard_length=self.maximum_guard_length,
            guard_side_offset=self.guard_side_offset,
            guard_tail_length=self.guard_tail_length,
            planning_direction=planning_direction,
            preferred_avoidance_side=preferred_avoidance_side,
        )

    def _select_drivable_risk(
        self,
        track,
        risk,
        base_pose,
        robot_velocity,
        planning_direction,
        locked_side,
    ):
        if self.road_mask is None or locked_side is not None:
            return risk
        alternative_side = (
            "left" if risk.avoidance_side == "right" else "right"
        )
        alternative = self._calculate_risk(
            track,
            base_pose,
            robot_velocity,
            planning_direction,
            alternative_side,
        )
        return choose_drivable_collision_risk(
            risk,
            alternative,
            self._road_mask_cost(*risk.safe_point),
            self._road_mask_cost(*alternative.safe_point),
            self.road_side_blocked_cost,
            self.road_side_preference_margin,
        )

    def _message_time(self, message) -> float:
        timestamp = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        if timestamp <= 0.0:
            return self.get_clock().now().nanoseconds * 1.0e-9
        return timestamp

    def _publish_prediction(self, source: LaserScan, points) -> None:
        prediction = LaserScan()
        prediction.header = source.header
        prediction.angle_min = source.angle_min
        prediction.angle_max = source.angle_max
        prediction.angle_increment = source.angle_increment
        prediction.time_increment = source.time_increment
        prediction.scan_time = source.scan_time
        prediction.range_min = source.range_min
        prediction.range_max = min(
            float(source.range_max), self.tracking_range
        )
        # This source clears the previous dynamic layer in every direction.
        # PointCloud2 below performs marking without LaserScan occlusion.
        prediction.ranges = [math.inf] * len(source.ranges)
        self.prediction_pub.publish(prediction)
        self.prediction_points_pub.publish(
            point_cloud2.create_cloud_xyz32(source.header, points)
        )

    def _publish_tracks(self, source: LaserScan, tracks) -> None:
        message = DynamicObstacleArray()
        message.header.stamp = source.header.stamp
        message.header.frame_id = "map"
        for track in tracks:
            obstacle = DynamicObstacle()
            obstacle.id = int(track.identifier)
            obstacle.position.x = float(track.x)
            obstacle.position.y = float(track.y)
            obstacle.velocity.x = float(track.vx)
            obstacle.velocity.y = float(track.vy)
            obstacle.radius = max(
                self.default_obstacle_radius,
                0.5 * max(0.0, float(track.diameter)),
            )
            message.obstacles.append(obstacle)
        self.dynamic_tracks_pub.publish(message)

    def _scan_callback(self, message: LaserScan) -> None:
        timestamp = self._message_time(message)
        base_pose = self.pose_history.pose_at(timestamp)
        pose_gap = self.pose_history.nearest_time_gap(timestamp)
        pose = (
            compose_pose(base_pose, self.sensor_offset)
            if base_pose is not None
            else None
        )
        if pose is None:
            self.tracker.reset()
            self.risk_latch.reset()
            self._publish_prediction(
                message, ()
            )
            self._publish_tracks(message, ())
            self.track_count_pub.publish(Int32(data=0))
            self.status_pub.publish(String(
                data=(
                    "waiting_for_synchronized_map_pose;"
                    f"nearest_pose_gap={pose_gap:.3f}s"
                )
            ))
            return
        tracking_range = min(float(message.range_max), self.tracking_range)
        clusters = scan_clusters(
            message.ranges,
            message.angle_min,
            message.angle_increment,
            message.range_min,
            tracking_range,
            self.cluster_gap,
            self.minimum_cluster_samples,
            self.maximum_cluster_diameter,
        )
        detections = [
            (*body_to_world((cluster.x, cluster.y), pose), cluster.diameter)
            for cluster in clusters
        ]
        self.tracker.update(detections, timestamp)
        dynamic_tracks = self.tracker.confirmed_tracks(
            self.maximum_dynamic_speed,
            self.minimum_observations,
        )
        self._publish_tracks(message, dynamic_tracks)
        robot_velocity = self.pose_history.velocity_at(timestamp) or (0.0, 0.0)
        planning_direction = path_tangent(
            self.latest_plan,
            (base_pose[0], base_pose[1]),
        )
        road_side_preference = self._road_side_preference(
            base_pose, planning_direction
        )
        continuity_side = self.risk_latch.preferred_avoidance_side
        if planning_direction is None:
            # Before Nav2 publishes a route there is no meaningful route-left
            # or route-right side. Publishing a default guard while parked can
            # bend the first plan to the wrong shoulder and latch that choice
            # for the whole encounter.
            self.risk_latch.reset()
            risks = []
        else:
            risks = []
            for track in dynamic_tracks:
                risk = self._calculate_risk(
                    track,
                    base_pose,
                    robot_velocity,
                    planning_direction,
                    continuity_side or road_side_preference,
                )
                risks.append(self._select_drivable_risk(
                    track,
                    risk,
                    base_pose,
                    robot_velocity,
                    planning_direction,
                    continuity_side,
                ))
        active_risks = self.risk_latch.update(risks)
        nearest_risk = min(
            active_risks,
            key=lambda risk: (
                risk.time_to_closest,
                risk.current_distance,
                risk.identifier,
            ),
            default=None,
        )
        risk_body = [
            world_to_body(point, pose)
            for risk in active_risks
            for point in risk.guard_points
        ]
        escape_gate_points = (
            [
                point
                for risk in active_risks
                for point in forward_escape_guard_points(
                    risk,
                    robot_pose=pose,
                    exclusion_radius=self.robot_exclusion_radius,
                    risk_radius=self.risk_point_radius,
                    minimum_forward_distance=(
                        self.escape_gate_minimum_forward_distance
                    ),
                    guard_side_offset=self.escape_gate_side_offset,
                    activation_margin=self.escape_gate_activation_margin,
                    safe_side_intrusion=(
                        self.escape_gate_safe_side_intrusion
                    ),
                )
            ]
            if self.enable_near_escape_gate
            else []
        )
        risk_body.extend(escape_gate_points)
        predicted_points = prediction_cloud_points(
            risk_body,
            self.risk_point_radius,
            self.robot_exclusion_radius,
            self.risk_disc_samples,
        )

        self._publish_prediction(message, predicted_points)
        count = len(dynamic_tracks)
        self.track_count_pub.publish(Int32(data=count))
        risk_status = (
            (
                f"zone={nearest_risk.zone};"
                f"avoidance_side={nearest_risk.avoidance_side};"
                "robot_velocity=("
                f"{nearest_risk.robot_velocity[0]:.2f},"
                f"{nearest_risk.robot_velocity[1]:.2f})mps;"
                "obstacle_velocity=("
                f"{nearest_risk.obstacle_velocity[0]:.2f},"
                f"{nearest_risk.obstacle_velocity[1]:.2f})mps;"
                f"obstacle_speed={nearest_risk.obstacle_speed:.2f}mps;"
                f"closing_speed={nearest_risk.closing_speed:.2f}mps;"
                f"distance={nearest_risk.current_distance:.2f}m;"
                f"safe_distance={nearest_risk.safe_distance:.2f}m;"
                f"maneuver_time={nearest_risk.maneuver_time:.2f}s;"
                f"collision_ttc={nearest_risk.time_to_collision:.2f}s;"
                f"ttc={nearest_risk.time_to_closest:.2f}s;"
                f"closest={nearest_risk.closest_distance:.2f}m;"
                "collision_point=("
                f"{nearest_risk.collision_point[0]:.2f},"
                f"{nearest_risk.collision_point[1]:.2f});"
                "safe_point=("
                f"{nearest_risk.safe_point[0]:.2f},"
                f"{nearest_risk.safe_point[1]:.2f})"
            )
            if nearest_risk is not None
            else "zone=clear"
        )
        self.status_pub.publish(String(
            data=(
                f"candidates={len(clusters)};"
                f"stable_dynamic_tracks={count};"
                f"active_threats={len(active_risks)};"
                f"risk_points={len(risk_body)};"
                f"escape_gate_points={len(escape_gate_points)};"
                f"robot_speed={math.hypot(*robot_velocity):.2f}mps;"
                f"road_preference={road_side_preference or 'auto'};"
                f"side_lock={continuity_side or 'auto'};"
                f"{risk_status};"
                f"pose_gap={pose_gap:.3f}s"
            )
        ))


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclePredictor()
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
