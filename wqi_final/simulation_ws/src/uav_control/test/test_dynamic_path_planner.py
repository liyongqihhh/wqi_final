import math

import pytest

from uav_control.dynamic_path_planner import (
    path_lookahead_point,
    plan_dynamic_path,
    vector_in_map_frame,
)


COMMON = {
    "orientation": (0.0, 0.0, 0.0, 1.0),
    "clearance": 2.5,
    "warning_distance": 6.0,
    "rear_warning_distance": 4.0,
    "target_clearance": 1.5,
    "minimum_altitude": 0.8,
    "maximum_altitude": 20.0,
    "spacing": 0.5,
}


def test_body_vector_rotates_into_map_frame():
    half_yaw = math.pi / 4.0
    result = vector_in_map_frame(
        (1.0, 0.0, 0.0),
        (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)),
    )
    assert result == pytest.approx((0.0, 1.0, 0.0), abs=1.0e-9)


def test_head_on_obstacle_generates_continuous_detour_path():
    plan = plan_dynamic_path(
        current=(0.0, 0.0, 15.0),
        target=(20.0, 0.0, 15.0),
        obstacle_vector_body=(4.0, 0.0, 0.0),
        **COMMON,
    )
    assert plan.avoiding is True
    assert len(plan.points) > 10
    assert plan.points[0] == pytest.approx((0.0, 0.0, 15.0))
    assert plan.points[-1] == pytest.approx((20.0, 0.0, 15.0))
    assert max(abs(point[1]) for point in plan.points) > 1.0
    assert all(
        later[0] + 1.0e-9 >= earlier[0]
        for earlier, later in zip(plan.points, plan.points[1:])
    )


def test_obstacle_on_left_selects_right_side():
    plan = plan_dynamic_path(
        current=(0.0, 0.0, 15.0),
        target=(20.0, 0.0, 15.0),
        obstacle_vector_body=(4.0, 1.0, 0.0),
        **COMMON,
    )
    assert plan.avoiding is True
    assert min(point[1] for point in plan.points) < -1.0


def test_close_rear_obstacle_changes_forward_path_without_reversing():
    plan = plan_dynamic_path(
        current=(0.0, 0.0, 15.0),
        target=(20.0, 0.0, 15.0),
        obstacle_vector_body=(-3.0, 0.0, 0.0),
        **COMMON,
    )
    assert plan.avoiding is True
    assert plan.reason == "rear_dynamic_obstacle"
    assert min(point[0] for point in plan.points) >= -1.0e-9
    assert max(abs(point[1]) for point in plan.points) > 1.0


def test_rear_return_does_not_pull_uav_away_from_a_nearby_target():
    plan = plan_dynamic_path(
        current=(0.25, -0.10, 8.05),
        target=(0.0, 0.0, 8.0),
        obstacle_vector_body=(-1.0, 0.0, -0.5),
        **COMMON,
    )
    assert plan.avoiding is False
    assert plan.reason == "target_within_clearance"
    assert plan.points[-1] == pytest.approx((0.0, 0.0, 8.0))


def test_far_rear_obstacle_does_not_change_route():
    plan = plan_dynamic_path(
        current=(0.0, 0.0, 15.0),
        target=(20.0, 0.0, 15.0),
        obstacle_vector_body=(-8.0, 0.0, 0.0),
        **COMMON,
    )
    assert plan.avoiding is False
    assert all(abs(point[1]) < 1.0e-9 for point in plan.points)


def test_facade_at_target_is_not_treated_as_dynamic_blockage():
    plan = plan_dynamic_path(
        current=(0.0, 0.0, 15.0),
        target=(5.0, 0.0, 15.0),
        obstacle_vector_body=(5.5, 0.0, 0.0),
        **COMMON,
    )
    assert plan.avoiding is False


def test_vertical_detour_respects_altitude_limits():
    plan = plan_dynamic_path(
        current=(0.0, 0.0, 19.0),
        target=(10.0, 0.0, 19.0),
        obstacle_vector_body=(4.0, 0.0, -1.0),
        **COMMON,
    )
    assert plan.avoiding is True
    assert all(0.8 <= point[2] <= 20.0 for point in plan.points)


def test_lookahead_selects_forward_path_point():
    point = path_lookahead_point(
        current=(1.2, 0.0, 15.0),
        points=((0.0, 0.0, 15.0), (1.0, 0.0, 15.0), (2.0, 0.2, 15.0),
                (3.0, 0.5, 15.0)),
        lookahead_distance=1.0,
    )
    assert point[0] >= 2.0
