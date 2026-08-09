import math
from dataclasses import replace
from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from dynamic_obstacle_prediction import (  # noqa: E402
    CollisionRiskLatch,
    DynamicObstacleTracker,
    PoseHistory,
    TrackedObstacle,
    body_to_world,
    calculate_collision_risk,
    choose_drivable_collision_risk,
    choose_open_avoidance_side,
    compose_pose,
    effective_prediction_radius,
    forward_escape_guard_points,
    predicted_swept_points,
    prediction_cloud_points,
    path_tangent,
    plan_goal_changed,
    project_discs_to_scan,
    scan_clusters,
    time_to_collision_boundary,
    world_to_body,
)


def test_scan_clustering_covers_front_and_rear_obstacles():
    ranges = [math.inf] * 360
    for index in (178, 179, 180, 181, 182):
        ranges[index] = 3.0
    for index in (0, 1, 2, 357, 358, 359):
        ranges[index] = 2.0
    clusters = scan_clusters(
        ranges,
        -math.pi,
        2.0 * math.pi / 360.0,
        0.1,
        9.0,
        0.45,
        3,
        1.8,
    )
    assert len(clusters) == 2
    assert any(cluster.x > 1.5 for cluster in clusters)
    assert any(cluster.x < -1.5 for cluster in clusters)


def test_prediction_radius_preserves_physical_clearance_margin():
    assert effective_prediction_radius(0.45, 0.45, 0.20) == pytest.approx(
        0.65
    )
    assert effective_prediction_radius(0.80, 0.45, 0.20) == pytest.approx(
        0.80
    )


@pytest.mark.parametrize(
    "configured,obstacle,margin",
    ((0.0, 0.45, 0.20), (0.65, -0.45, 0.20), (0.65, 0.45, -0.20)),
)
def test_prediction_radius_rejects_invalid_geometry(
    configured, obstacle, margin
):
    with pytest.raises(ValueError):
        effective_prediction_radius(configured, obstacle, margin)


def test_road_cost_policy_selects_only_a_clearly_open_side():
    assert choose_open_avoidance_side(
        (12.0, 25.0, 35.0), (35.0, 100.0, 100.0)
    ) == "left"
    assert choose_open_avoidance_side(
        (100.0, 100.0), (12.0, 25.0)
    ) == "right"
    assert choose_open_avoidance_side(
        (12.0, 25.0), (12.0, 25.0)
    ) is None


def test_collision_risk_uses_only_a_drivable_alternate_safe_point():
    primary = _risk(_track(40, 2.0, 0.0, -0.50, 0.0))
    alternative = replace(
        primary,
        avoidance_side="left",
        safe_point=(primary.safe_point[0], -primary.safe_point[1]),
    )
    assert choose_drivable_collision_risk(
        primary,
        alternative,
        primary_cost=100.0,
        alternative_cost=12.0,
        blocked_cost=65.0,
        preference_margin=15.0,
    ) is alternative
    assert choose_drivable_collision_risk(
        primary,
        alternative,
        primary_cost=35.0,
        alternative_cost=100.0,
        blocked_cost=65.0,
        preference_margin=15.0,
    ) is primary


def test_plan_goal_change_ignores_periodic_replans_to_same_goal():
    previous = ((0.0, 0.0), (10.0, 5.0))
    assert plan_goal_changed((), previous) is True
    assert plan_goal_changed(previous, ((1.0, 0.0), (10.2, 5.1))) is False
    assert plan_goal_changed(previous, ((1.0, 0.0), (12.0, 5.0))) is True


def test_body_world_transforms_are_inverse():
    pose = (4.0, -2.0, math.pi / 3.0)
    world = body_to_world((1.2, -0.4), pose)
    assert world_to_body(world, pose) == pytest.approx((1.2, -0.4))


def test_pose_history_interpolates_sensor_time_and_wrapped_yaw():
    history = PoseHistory(history_duration=2.0, maximum_extrapolation=0.12)
    history.add(1.0, (0.0, 0.0, math.radians(179.0)))
    history.add(1.1, (0.1, 0.0, math.radians(-179.0)))
    pose = history.pose_at(1.05)
    assert pose[:2] == pytest.approx((0.05, 0.0))
    assert abs(abs(pose[2]) - math.pi) < math.radians(0.2)
    assert history.pose_at(1.3) is None


def test_pose_history_returns_map_velocity_at_scan_time():
    history = PoseHistory(history_duration=2.0, maximum_extrapolation=0.12)
    history.add(1.0, (2.0, -1.0, 0.0))
    history.add(1.1, (2.02, -0.97, 0.0))
    assert history.velocity_at(1.05) == pytest.approx((0.2, 0.3))
    assert history.velocity_at(1.3) is None


def test_sensor_pose_composes_fixed_offset_in_base_heading():
    sensor = compose_pose(
        (4.0, -2.0, math.pi / 2.0),
        (0.2, 0.0, 0.1),
    )
    assert sensor == pytest.approx((4.0, -1.8, math.pi / 2.0 + 0.1))


def test_tracker_estimates_moving_obstacle_velocity():
    tracker = DynamicObstacleTracker(
        association_distance=1.0,
        velocity_alpha=1.0,
        track_timeout=1.0,
    )
    tracker.update([(0.0, -3.0)], 0.0)
    tracker.update([(0.0, -2.6)], 1.0)
    tracks = tracker.dynamic_tracks(0.1, 1.0, 2)
    assert len(tracks) == 1
    assert tracks[0].vx == pytest.approx(0.0)
    assert tracks[0].vy == pytest.approx(0.4)


def test_tracker_does_not_classify_stationary_cluster_jitter_as_motion():
    tracker = DynamicObstacleTracker(
        association_distance=0.75,
        velocity_alpha=1.0,
        track_timeout=1.0,
    )
    samples = (0.00, 0.03, -0.02, 0.02, -0.01, 0.01, 0.00)
    for index, x in enumerate(samples):
        tracker.update([(x, 2.0, 0.5)], index * 0.1)
    assert tracker.dynamic_tracks(0.05, 1.5, 5) == ()
    assert tracker.confirmed_tracks(1.5, 5) == ()


def test_tracker_keeps_confirmed_target_when_it_decelerates_at_endpoint():
    tracker = DynamicObstacleTracker(
        association_distance=1.0,
        velocity_alpha=1.0,
        track_timeout=1.0,
        history_window=0.6,
    )
    for index, y in enumerate((0.00, 0.05, 0.10, 0.15, 0.20)):
        tracker.update([(0.0, y, 0.9)], index * 0.1)
    assert tracker.dynamic_tracks(0.1, 1.0, 3)[0].motion_confirmed

    for index in range(5, 14):
        tracker.update([(0.0, 0.20, 0.9)], index * 0.1)

    assert tracker.dynamic_tracks(0.1, 1.0, 3) == ()
    held = tracker.confirmed_tracks(1.0, 3)
    assert len(held) == 1
    assert held[0].motion_confirmed is True
    assert held[0].speed == pytest.approx(0.0)


def test_tracker_rejects_impossible_jump_instead_of_moving_the_track():
    tracker = DynamicObstacleTracker(
        association_distance=1.2,
        velocity_alpha=1.0,
        track_timeout=1.0,
        maximum_track_speed=1.0,
        position_gate=0.10,
    )
    tracker.update([(0.0, 2.0, 0.5)], 0.0)
    tracker.update([(0.05, 2.0, 0.5)], 0.1)
    tracker.update([(1.0, 2.0, 0.5)], 0.2)
    assert len(tracker.tracks) == 2
    assert tracker.tracks[1].x == pytest.approx(0.05)
    assert tracker.tracks[2].x == pytest.approx(1.0)


def test_tracker_updates_confirmed_velocity_immediately_after_reversal():
    tracker = DynamicObstacleTracker(
        association_distance=1.0,
        velocity_alpha=1.0,
        track_timeout=1.0,
    )
    for index, y in enumerate((0.0, 0.05, 0.10, 0.15, 0.20)):
        tracker.update([(0.0, y, 0.9)], index * 0.1)
        tracker.dynamic_tracks(0.1, 1.0, 3)

    track = tracker.dynamic_tracks(0.1, 1.0, 3)[0]
    assert track.motion_confirmed is True
    assert track.vy > 0.0

    tracker.update([(0.0, 0.15, 0.9)], 0.5)
    reversed_track = tracker.dynamic_tracks(0.1, 1.0, 3)[0]
    assert reversed_track.identifier == track.identifier
    assert reversed_track.vy == pytest.approx(-0.5)


def test_rear_approach_prediction_stays_attached_to_obstacle():
    tracker = DynamicObstacleTracker(
        association_distance=1.0,
        velocity_alpha=1.0,
        track_timeout=1.0,
    )
    tracker.update([(0.0, -3.0)], 0.0)
    tracker.update([(0.0, -2.5)], 1.0)
    track = tracker.dynamic_tracks(0.1, 1.0, 2)[0]
    points = predicted_swept_points(
        (track,),
        horizon=8.0,
        time_step=1.0,
        maximum_distance=0.35,
    )
    assert points[0] == pytest.approx((0.0, -2.5))
    assert points[-1][1] > points[0][1]
    assert math.dist(points[0], points[-1]) == pytest.approx(0.35)


def test_fast_obstacle_prediction_is_capped_by_distance_not_horizon():
    tracker = DynamicObstacleTracker(
        association_distance=2.0,
        velocity_alpha=1.0,
        track_timeout=1.0,
    )
    tracker.update([(0.0, 0.0)], 0.0)
    tracker.update([(1.5, 0.0)], 1.0)
    track = tracker.dynamic_tracks(0.1, 2.0, 2)[0]
    points = predicted_swept_points(
        (track,),
        horizon=3.5,
        time_step=0.2,
        maximum_distance=0.35,
    )
    assert points[0] == pytest.approx((1.5, 0.0))
    assert max(math.dist(points[0], point) for point in points) <= (
        0.35 + 1.0e-9
    )


def _risk(track, robot_velocity=(0.22, 0.0)):
    return calculate_collision_risk(
        track=track,
        robot_pose=(0.0, 0.0, 0.0),
        robot_velocity=robot_velocity,
        robot_radius=0.22,
        default_obstacle_radius=0.45,
        safety_margin=0.20,
        response_time=3.00,
        braking_deceleration=0.60,
        lateral_maneuver_acceleration=0.50,
        collision_horizon=15.0,
        planning_buffer=1.50,
        minimum_closing_speed=0.05,
        nominal_robot_speed=0.22,
        collision_corridor_margin=0.50,
        proximity_guard_distance=6.00,
        guard_sample_spacing=0.50,
        maximum_guard_length=5.00,
        guard_side_offset=1.20,
    )


def _track(identifier, x, y, vx, vy):
    return TrackedObstacle(
        identifier=identifier,
        x=x,
        y=y,
        vx=vx,
        vy=vy,
        observations=8,
        updated_at=1.0,
        diameter=0.90,
    )


def _guard_centers(risk, side_offset=1.20, spacing=0.50):
    row_width = int(math.ceil(side_offset / spacing)) + 1
    return risk.guard_points[::row_width]


def test_head_on_speed_creates_safe_distance_and_front_risk_point():
    risk = _risk(_track(1, 5.0, 0.0, -0.50, 0.0))
    assert risk.threatening is True
    assert risk.zone == "front"
    assert risk.closing_speed == pytest.approx(0.72)
    assert risk.safe_distance > 3.5
    assert risk.maneuver_time > 2.0
    assert risk.time_to_collision == pytest.approx(5.73611111)
    assert risk.closest_distance == pytest.approx(0.0)
    assert risk.collision_point[0] == pytest.approx(1.52777778)
    assert risk.safe_point == pytest.approx((1.52777778, -1.20))
    assert risk.avoidance_side == "right"
    assert risk.guard_points[0] == pytest.approx((5.0, 0.0))
    assert _guard_centers(risk)[-1][0] < risk.collision_point[0]
    assert risk.guard_points[1][1] > risk.guard_points[0][1]


def test_same_road_collision_strip_rejects_lateral_velocity_spike():
    risk = _risk(_track(41, 5.0, 0.0, -0.50, 0.25))
    centers = _guard_centers(risk)
    assert risk.collision_point[1] == pytest.approx(0.0)
    assert all(point[1] == pytest.approx(0.0) for point in centers)


def test_rear_diagonal_convergence_crosses_existing_route():
    risk = _risk(_track(42, -4.0, 1.30, 0.50, -0.10))
    centers = _guard_centers(risk)
    assert risk.threatening is True
    assert risk.zone == "rear_left"
    # The 0.65 m prediction disc around this endpoint intersects y=0, so the
    # old center-line path becomes invalid even with the short test guard.
    assert risk.collision_point[1] < 0.65
    assert max(point[1] for point in centers) >= 1.30
    assert min(point[1] for point in centers) < 0.65
    assert any(
        point[0] > 0.0 and abs(point[1]) < 0.65
        for point in centers
    )


def test_rear_catch_beyond_time_horizon_projects_meeting_point_ahead():
    risk = calculate_collision_risk(
        track=_track(43, -5.80, 1.00, 0.63, -0.08),
        robot_pose=(0.0, 0.0, 0.0),
        robot_velocity=(0.40, 0.0),
        robot_radius=0.60,
        default_obstacle_radius=0.45,
        safety_margin=0.20,
        response_time=5.00,
        braking_deceleration=0.60,
        lateral_maneuver_acceleration=0.50,
        collision_horizon=15.0,
        planning_buffer=1.50,
        minimum_closing_speed=0.05,
        nominal_robot_speed=0.40,
        collision_corridor_margin=0.50,
        proximity_guard_distance=6.00,
        guard_sample_spacing=0.75,
        maximum_guard_length=16.00,
        guard_side_offset=1.80,
    )

    assert risk.threatening is True
    assert risk.zone.startswith("rear")
    assert risk.time_to_closest > 15.0
    assert risk.collision_point[0] > 8.0
    assert risk.collision_point[1] < -0.50
    assert risk.avoidance_side == "left"
    assert risk.safe_point[0] == pytest.approx(risk.collision_point[0])
    assert risk.safe_point[1] > risk.collision_point[1]


def test_path_tangent_uses_forward_path_order_near_robot():
    direction = path_tangent(
        ((0.0, -4.0), (0.0, -1.0), (0.0, 1.5), (1.0, 4.0)),
        (0.1, -0.8),
        lookahead_distance=2.0,
    )
    assert direction[0] == pytest.approx(0.0)
    assert direction[1] == pytest.approx(1.0)


def test_planning_tangent_keeps_road_side_when_body_faces_backward():
    risk = calculate_collision_risk(
        track=_track(31, 0.0, 3.0, 0.0, -0.45),
        robot_pose=(0.0, 0.0, -math.pi / 2.0),
        robot_velocity=(0.0, -0.10),
        robot_radius=0.60,
        default_obstacle_radius=0.45,
        safety_margin=0.20,
        response_time=3.00,
        braking_deceleration=0.60,
        lateral_maneuver_acceleration=0.50,
        collision_horizon=15.0,
        planning_buffer=1.50,
        minimum_closing_speed=0.05,
        nominal_robot_speed=0.22,
        collision_corridor_margin=0.50,
        proximity_guard_distance=6.00,
        guard_sample_spacing=0.50,
        maximum_guard_length=8.00,
        guard_side_offset=2.40,
        planning_direction=(0.0, 1.0),
    )
    assert risk.avoidance_side == "right"
    assert risk.guard_points[1][0] < risk.guard_points[0][0]


def test_collision_guard_is_a_connected_obstacle_swept_corridor():
    risk = _risk(_track(19, 2.0, 0.0, -0.50, 0.0))
    centers = _guard_centers(risk)
    row_width = int(math.ceil(1.20 / 0.50)) + 1
    sides = tuple(
        point
        for index, point in enumerate(risk.guard_points)
        if index % row_width
    )
    assert centers[0] == pytest.approx((2.0, 0.0))
    assert centers[-1][0] < risk.collision_point[0]
    assert centers[-1] == pytest.approx((-2.38888889, 0.0))
    assert all(point[1] == pytest.approx(0.0) for point in centers)
    assert all(point[1] > 0.0 for point in sides)
    first_row = risk.guard_points[:row_width]
    assert first_row[-1] == pytest.approx((2.0, 1.20))
    assert all(
        0.0 < math.dist(first, second) <= 0.50 + 1.0e-9
        for first, second in zip(first_row, first_row[1:])
    )
    assert all(
        0.0 < math.dist(first, second) <= 0.50 + 1.0e-9
        for first, second in zip(
            centers, centers[1:]
        )
    )


def test_faster_head_on_obstacle_increases_required_safe_distance():
    slow = _risk(_track(1, 2.0, 0.0, -0.20, 0.0))
    fast = _risk(_track(2, 2.0, 0.0, -0.80, 0.0))
    assert fast.obstacle_speed > slow.obstacle_speed
    assert fast.closing_speed > slow.closing_speed
    assert fast.safe_distance > slow.safe_distance


def test_relative_motion_collision_time_covers_front_and_rear_approach():
    head_on = time_to_collision_boundary(
        (5.0, 0.0), (-0.72, 0.0), 0.87, 15.0
    )
    rear_catch = time_to_collision_boundary(
        (-3.0, 0.0), (0.28, 0.0), 0.87, 15.0
    )
    assert head_on == pytest.approx((5.0 - 0.87) / 0.72)
    assert rear_catch == pytest.approx((3.0 - 0.87) / 0.28)


def test_relative_motion_collision_time_rejects_safe_or_diverging_paths():
    crossing_clear = time_to_collision_boundary(
        (3.0, 2.0), (-0.5, 0.0), 0.87, 15.0
    )
    diverging = time_to_collision_boundary(
        (2.0, 0.0), (0.5, 0.0), 0.87, 15.0
    )
    assert math.isinf(crossing_clear)
    assert math.isinf(diverging)


def test_rear_left_catch_up_is_classified_as_collision_threat():
    risk = _risk(_track(3, -1.5, 0.55, 0.50, -0.10))
    assert risk.threatening is True
    assert risk.zone == "rear_left"
    assert risk.avoidance_side == "right"
    assert risk.closing_speed > 0.25
    assert risk.time_to_closest < 15.0
    assert math.isfinite(risk.time_to_collision)
    assert risk.closest_distance < 0.20
    assert risk.guard_points[0] == pytest.approx((-1.5, 0.55))
    assert math.dist(
        max(
            risk.guard_points,
            key=lambda point: math.dist(point, risk.obstacle_position),
        ),
        risk.obstacle_position,
    ) > math.dist(risk.collision_point, risk.obstacle_position)
    assert risk.guard_points[1][1] > risk.guard_points[0][1]


def test_rear_right_crossing_catch_uses_predicted_center_detour():
    risk = _risk(_track(7, -1.5, -0.55, 0.50, 0.10))
    assert risk.threatening is True
    assert risk.zone == "rear_right"
    assert risk.avoidance_side == "right"
    assert risk.guard_points[0] == pytest.approx((-1.5, -0.55))
    assert risk.guard_points[1][1] > risk.guard_points[0][1]


def test_rear_right_parallel_catch_blocks_right_and_requests_left_detour():
    risk = _risk(_track(8, -1.5, -0.55, 0.50, 0.0))
    assert risk.threatening is True
    assert risk.zone == "rear_right"
    assert risk.avoidance_side == "left"
    assert risk.guard_points[1][1] < risk.guard_points[0][1]


def test_near_head_on_noise_keeps_deterministic_right_hand_detour():
    risk = calculate_collision_risk(
        track=_track(24, 2.0, -0.70, -0.45, 0.0),
        robot_pose=(0.0, 0.0, 0.0),
        robot_velocity=(0.22, 0.0),
        robot_radius=0.60,
        default_obstacle_radius=0.45,
        safety_margin=0.20,
        response_time=3.00,
        braking_deceleration=0.60,
        lateral_maneuver_acceleration=0.50,
        collision_horizon=15.0,
        planning_buffer=1.50,
        minimum_closing_speed=0.05,
        nominal_robot_speed=0.22,
        collision_corridor_margin=0.50,
        proximity_guard_distance=6.00,
        guard_sample_spacing=0.50,
        maximum_guard_length=5.00,
        guard_side_offset=1.20,
    )
    assert risk.zone == "front_right"
    assert risk.avoidance_side == "right"
    assert risk.guard_points[1][1] > risk.guard_points[0][1]


def test_maneuver_time_expands_rear_detection_beyond_old_horizon():
    risk = _risk(_track(8, -2.35, 0.0, 0.50, 0.0))
    assert 8.0 < risk.time_to_closest < 12.0
    assert risk.safe_distance + 1.50 >= risk.current_distance
    assert risk.threatening is True


def test_rear_conflict_is_announced_early_enough_for_a_lane_change():
    risk = _risk(_track(18, -3.30, 0.0, 0.50, 0.0))
    assert risk.zone == "rear"
    assert risk.time_to_closest > 10.0
    assert risk.safe_distance + 1.50 >= risk.current_distance
    assert risk.threatening is True


def test_static_road_preference_can_override_default_passing_side():
    kwargs = dict(
        track=_track(26, -3.30, 0.0, 0.50, 0.0),
        robot_pose=(0.0, 0.0, 0.0),
        robot_velocity=(0.22, 0.0),
        robot_radius=0.60,
        default_obstacle_radius=0.45,
        safety_margin=0.20,
        response_time=3.00,
        braking_deceleration=0.60,
        lateral_maneuver_acceleration=0.50,
        collision_horizon=15.0,
        planning_buffer=1.50,
        minimum_closing_speed=0.05,
        nominal_robot_speed=0.22,
        collision_corridor_margin=0.50,
        proximity_guard_distance=6.00,
        guard_sample_spacing=0.50,
        maximum_guard_length=8.00,
        guard_side_offset=1.50,
        planning_direction=(1.0, 0.0),
    )
    default = calculate_collision_risk(**kwargs)
    left = calculate_collision_risk(
        **kwargs, preferred_avoidance_side="left"
    )
    assert default.avoidance_side == "right"
    assert left.avoidance_side == "left"
    assert left.guard_points[1][1] < left.guard_points[0][1]


def test_static_road_preference_cannot_cross_lateral_obstacle_trajectory():
    kwargs = dict(
        track=_track(29, -2.5, 1.25, 0.50, 0.0),
        robot_pose=(0.0, 0.0, 0.0),
        robot_velocity=(0.22, 0.0),
        robot_radius=0.60,
        default_obstacle_radius=0.45,
        safety_margin=0.20,
        response_time=3.00,
        braking_deceleration=0.60,
        lateral_maneuver_acceleration=0.50,
        collision_horizon=15.0,
        planning_buffer=1.50,
        minimum_closing_speed=0.05,
        nominal_robot_speed=0.22,
        collision_corridor_margin=0.50,
        proximity_guard_distance=6.00,
        guard_sample_spacing=0.50,
        maximum_guard_length=8.00,
        guard_side_offset=1.50,
        planning_direction=(1.0, 0.0),
    )
    geometric = calculate_collision_risk(**kwargs)
    conflicting_preference = calculate_collision_risk(
        **kwargs,
        preferred_avoidance_side="left",
    )
    assert geometric.avoidance_side == "right"
    assert conflicting_preference.avoidance_side == "right"
    assert conflicting_preference.guard_points == geometric.guard_points


def test_obstacle_moving_away_does_not_create_virtual_risk():
    risk = _risk(_track(4, 1.5, 0.0, 0.60, 0.0))
    assert risk.closing_speed == 0.0
    assert math.isinf(risk.time_to_collision)
    assert risk.threatening is False


def test_close_same_lane_guard_is_attached_to_measured_obstacle():
    risk = _risk(_track(22, 2.5, 0.0, 0.22, 0.0))
    assert risk.closing_speed == 0.0
    assert risk.threatening is True
    assert risk.collision_point == pytest.approx((2.5, 0.0))
    assert risk.guard_points[0] == pytest.approx((2.5, 0.0))


def test_confirmed_endpoint_obstacle_behind_stays_a_close_route_threat():
    risk = _risk(
        _track(27, -1.5, 0.0, 0.0, 0.0),
        robot_velocity=(0.0, 0.0),
    )
    assert risk.zone == "rear"
    assert risk.closing_speed == 0.0
    assert risk.threatening is True


def test_observed_crossing_builds_connected_guard_to_conflict_area():
    risk = calculate_collision_risk(
        track=_track(23, 0.0, -28.027, 0.0, -0.45),
        robot_pose=(0.346, -31.245, 0.22),
        robot_velocity=(0.215, 0.048),
        robot_radius=0.60,
        default_obstacle_radius=0.45,
        safety_margin=0.20,
        response_time=3.00,
        braking_deceleration=0.60,
        lateral_maneuver_acceleration=0.50,
        collision_horizon=15.0,
        planning_buffer=1.50,
        minimum_closing_speed=0.05,
        nominal_robot_speed=0.22,
        collision_corridor_margin=0.50,
        proximity_guard_distance=6.00,
        guard_sample_spacing=0.50,
        maximum_guard_length=5.00,
        guard_side_offset=1.20,
    )
    assert risk.closest_distance > 1.50
    assert risk.threatening is True
    assert risk.guard_points[0] == pytest.approx((0.0, -28.027))
    centers = _guard_centers(risk)
    assert centers[-1][1] < -30.30
    assert all(
        second[1] <= first[1]
        for first, second in zip(
            centers, centers[1:]
        )
    )


def test_diagonal_trajectory_with_safe_closest_approach_is_not_blocked():
    risk = _risk(_track(5, 1.5, 1.5, -0.30, 0.30))
    assert risk.closest_distance > 1.0
    assert risk.threatening is False


def test_collision_risk_latch_requires_repeated_clear_observations():
    threat = _risk(_track(6, 2.0, 0.0, -0.50, 0.0))
    latch = CollisionRiskLatch(clear_confirmations=3)
    assert latch.update((threat,)) == (threat,)
    assert latch.update(()) == (threat,)
    assert latch.update(()) == (threat,)
    assert latch.update(()) == ()


def test_collision_risk_latch_retains_passing_side_until_mission_reset():
    threat = _risk(_track(34, 2.0, 0.0, -0.50, 0.0))
    latch = CollisionRiskLatch(clear_confirmations=2)
    latch.update((threat,))
    assert latch.preferred_avoidance_side == threat.avoidance_side
    latch.update(())
    assert latch.update(()) == ()
    assert latch.preferred_avoidance_side == threat.avoidance_side
    latch.reset()
    assert latch.preferred_avoidance_side is None


def test_collision_risk_latch_does_not_release_while_obstacle_is_close():
    threat = _risk(_track(30, 2.0, 0.0, -0.50, 0.0))
    close_but_separating = replace(
        threat,
        threatening=False,
        obstacle_position=(2.5, 0.0),
        current_distance=2.5,
        time_to_closest=-1.0,
        closest_distance=2.5,
    )
    latch = CollisionRiskLatch(
        clear_confirmations=2,
        release_distance=3.5,
    )

    latch.update((threat,))
    for _ in range(4):
        active = latch.update((close_but_separating,))
        assert len(active) == 1
        assert active[0].guard_points == threat.guard_points


def test_collision_risk_latch_releases_after_confirmed_safe_separation():
    threat = _risk(_track(31, 2.0, 0.0, -0.50, 0.0))
    safely_separated = replace(
        threat,
        threatening=False,
        obstacle_position=(4.0, 0.0),
        current_distance=4.0,
        time_to_closest=-1.0,
        closest_distance=4.0,
    )
    latch = CollisionRiskLatch(
        clear_confirmations=2,
        release_distance=3.5,
    )

    latch.update((threat,))
    assert len(latch.update((safely_separated,))) == 1
    assert latch.update((safely_separated,)) == ()


def test_collision_risk_latch_uses_longer_timeout_for_lost_track():
    threat = _risk(_track(32, 2.0, 0.0, -0.50, 0.0))
    latch = CollisionRiskLatch(
        clear_confirmations=2,
        lost_confirmations=4,
        release_distance=3.5,
    )

    latch.update((threat,))
    assert len(latch.update(())) == 1
    assert len(latch.update(())) == 1
    assert len(latch.update(())) == 1
    assert latch.update(()) == ()


def test_collision_risk_latch_keeps_future_close_approach_active():
    threat = _risk(_track(33, 2.0, 0.0, -0.50, 0.0))
    future_conflict = replace(
        threat,
        threatening=False,
        obstacle_position=(4.0, 0.0),
        current_distance=4.0,
        time_to_closest=2.0,
        closest_distance=0.8,
    )
    latch = CollisionRiskLatch(
        clear_confirmations=2,
        release_distance=3.5,
    )

    latch.update((threat,))
    assert len(latch.update((future_conflict,))) == 1
    assert len(latch.update((future_conflict,))) == 1


def test_collision_risk_latch_does_not_switch_detour_side_mid_conflict():
    rear_left = _risk(_track(9, -1.5, 0.55, 0.50, 0.0))
    rear_right = _risk(_track(9, -1.5, -0.55, 0.50, 0.0))
    latch = CollisionRiskLatch(clear_confirmations=3)
    assert rear_left.avoidance_side == "right"
    assert rear_right.avoidance_side == "left"
    latch.update((rear_left,))
    held = latch.update((rear_right,))[0]
    assert held.avoidance_side == "right"
    assert held.guard_points == rear_left.guard_points


def test_collision_episode_locks_nearby_reassociated_track_to_one_side():
    rear_left = _risk(_track(10, -1.5, 0.55, 0.50, 0.0))
    rear_right = _risk(_track(11, -1.5, -0.55, 0.50, 0.0))
    latch = CollisionRiskLatch(clear_confirmations=2)
    latch.update((rear_left,))
    nearby_reassociation = replace(
        rear_right,
        obstacle_position=(-1.45, 0.50),
    )
    active = latch.update((nearby_reassociation,))
    assert len(active) == 1
    assert active[0].avoidance_side == "right"
    latch.update(())
    assert latch.update(()) == ()
    active = latch.update((rear_right,))
    assert active[0].avoidance_side == "left"


def test_spatially_separate_threats_keep_independent_risk_events():
    front = _risk(_track(20, 2.0, 0.0, -0.50, 0.0))
    rear_right = _risk(_track(21, -1.5, -0.55, 0.50, 0.10))
    latch = CollisionRiskLatch(
        clear_confirmations=2,
        reassociation_distance=0.90,
    )

    active = latch.update((front, rear_right))

    assert len(active) == 2
    assert {risk.identifier for risk in active} == {20, 21}
    assert {risk.obstacle_position for risk in active} == {
        (2.0, 0.0),
        (-1.5, -0.55),
    }


def test_collision_episode_keeps_first_world_anchor_until_fully_clear():
    initial = _risk(_track(12, -1.5, 0.55, 0.50, 0.0))
    reassociated = replace(
        initial,
        identifier=13,
        collision_point=(9.0, 8.0),
        guard_points=((9.0, 8.0), (9.0, 8.65)),
    )
    latch = CollisionRiskLatch(clear_confirmations=2)

    first_active = latch.update((initial,))
    anchor = first_active[0].collision_point
    guards = first_active[0].guard_points
    reassociated_active = latch.update((reassociated,))
    assert {
        risk.collision_point for risk in reassociated_active
    } == {anchor}
    assert {
        risk.guard_points for risk in reassociated_active
    } == {guards}

    latch.update(())
    assert latch.update(()) == ()
    next_active = latch.update((reassociated,))
    assert next_active[0].collision_point == (9.0, 8.0)
    assert next_active[0].guard_points == ((9.0, 8.0), (9.0, 8.65))


def test_prediction_scan_marks_both_rear_and_forward_sweep():
    ranges = project_discs_to_scan(
        centers=((-2.0, 0.0), (2.0, 0.0)),
        radius=0.5,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / 360.0,
        sample_count=360,
        range_min=0.1,
        range_max=9.0,
        exclusion_radius=0.8,
    )
    assert math.isfinite(ranges[180])
    assert math.isfinite(ranges[0])


def test_prediction_scan_rejects_disc_overlapping_robot_exclusion_edge():
    ranges = project_discs_to_scan(
        centers=((0.9, 0.0),),
        radius=0.3,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / 360.0,
        sample_count=360,
        range_min=0.1,
        range_max=9.0,
        exclusion_radius=1.1,
    )
    assert all(math.isinf(distance) for distance in ranges)


def test_prediction_scan_keeps_far_corridor_when_near_disc_is_rejected():
    ranges = project_discs_to_scan(
        centers=((1.4, 0.0), (2.2, 0.0)),
        radius=0.3,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / 360.0,
        sample_count=360,
        range_min=0.1,
        range_max=9.0,
        exclusion_radius=1.1,
    )
    assert ranges[180] == pytest.approx(1.9)


def test_near_obstacle_creates_forward_right_passing_gate():
    risk = replace(
        _risk(_track(24, 2.0, 0.0, -0.5, 0.0)),
        collision_point=(0.8, 0.25),
        avoidance_side="right",
    )
    points = forward_escape_guard_points(
        risk,
        robot_pose=(0.0, 0.0, 0.0),
        exclusion_radius=1.2,
        risk_radius=0.65,
        minimum_forward_distance=2.2,
        guard_side_offset=1.2,
        activation_margin=0.2,
    )
    assert points[0] == pytest.approx((2.2, 0.0))
    assert points[1] == pytest.approx((2.2, 1.2))


def test_rear_catch_up_does_not_create_a_moving_forward_gate():
    risk = _risk(_track(42, -1.5, 0.0, 0.50, 0.0))
    assert risk.zone == "rear"
    assert forward_escape_guard_points(
        risk,
        robot_pose=(0.0, 0.0, 0.0),
        exclusion_radius=1.2,
        risk_radius=0.65,
        minimum_forward_distance=2.2,
        guard_side_offset=1.2,
        activation_margin=0.2,
    ) == ()


def test_forward_gate_uses_current_obstacle_and_intrudes_safe_side():
    risk = replace(
        _risk(_track(28, 0.8, 0.25, -0.5, 0.0)),
        collision_point=(8.0, 0.25),
        obstacle_position=(0.8, 0.25),
        avoidance_side="right",
    )
    points = forward_escape_guard_points(
        risk,
        robot_pose=(0.0, 0.0, 0.0),
        exclusion_radius=1.2,
        risk_radius=0.65,
        minimum_forward_distance=2.4,
        guard_side_offset=1.8,
        activation_margin=0.5,
        safe_side_intrusion=0.35,
    )
    assert len(points) == 3
    assert points[0] == pytest.approx((2.4, 0.25))
    assert points[1] == pytest.approx((2.4, 2.05))
    assert points[2] == pytest.approx((2.4, -0.10))


def test_far_collision_point_does_not_create_moving_escape_gate():
    risk = replace(
        _risk(_track(25, 4.0, 0.0, -0.5, 0.0)),
        collision_point=(4.0, 0.0),
    )
    assert forward_escape_guard_points(
        risk,
        robot_pose=(0.0, 0.0, 0.0),
        exclusion_radius=1.2,
        risk_radius=0.65,
        minimum_forward_distance=2.2,
        guard_side_offset=1.2,
        activation_margin=0.2,
    ) == ()


def test_near_future_collision_does_not_move_gate_before_obstacle_arrives():
    risk = replace(
        _risk(_track(30, 4.0, 0.0, -0.5, 0.0)),
        collision_point=(0.8, 0.0),
        obstacle_position=(4.0, 0.0),
    )
    assert forward_escape_guard_points(
        risk,
        robot_pose=(0.0, 0.0, 0.0),
        exclusion_radius=1.2,
        risk_radius=0.65,
        minimum_forward_distance=2.2,
        guard_side_offset=1.2,
        activation_margin=0.2,
    ) == ()


def test_prediction_cloud_preserves_all_non_occluded_corridor_discs():
    points = prediction_cloud_points(
        centers=((2.0, 0.0), (3.0, 0.0)),
        radius=0.4,
        exclusion_radius=1.2,
        perimeter_samples=8,
    )
    assert len(points) == 18
    assert (2.0, 0.0, 0.2) in points
    assert (3.0, 0.0, 0.2) in points


def test_prediction_cloud_rejects_near_disc_without_dropping_far_disc():
    points = prediction_cloud_points(
        centers=((1.4, 0.0), (2.2, 0.0)),
        radius=0.3,
        exclusion_radius=1.1,
        perimeter_samples=8,
    )
    assert len(points) == 9
    assert (2.2, 0.0, 0.2) in points


def test_prediction_scan_ignores_disc_fully_inside_robot_exclusion():
    ranges = project_discs_to_scan(
        centers=((0.6, 0.0),),
        radius=0.3,
        angle_min=-math.pi,
        angle_increment=2.0 * math.pi / 360.0,
        sample_count=360,
        range_min=0.1,
        range_max=9.0,
        exclusion_radius=1.1,
    )
    assert all(math.isinf(distance) for distance in ranges)
