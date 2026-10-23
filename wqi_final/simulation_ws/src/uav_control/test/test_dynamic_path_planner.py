import math

import pytest

from uav_control.dynamic_path_planner import (
    ObstacleClosingSpeed,
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
        rear_closing_speed=1.0,
        **COMMON,
    )
    assert plan.avoiding is True
    assert plan.reason == "rear_dynamic_obstacle"
    assert min(point[0] for point in plan.points) >= -1.0e-9
    assert max(abs(point[1]) for point in plan.points) > 1.0


def test_stationary_facade_behind_departure_does_not_trigger_detour():
    plan = plan_dynamic_path(
        current=(42.0, 8.5, 33.6), target=(30.0, 11.0, 33.6),
        obstacle_vector_body=(0.0, -3.5, 0.0), rear_closing_speed=-0.2,
        **COMMON,
    )
    assert not plan.avoiding
    assert all(point[2] == pytest.approx(33.6) for point in plan.points)


@pytest.mark.parametrize("start_z,end_z", [(15.0, 33.6), (33.6, 15.0)])
def test_clear_vertical_column_next_to_facade_stays_straight(start_z, end_z):
    settings = dict(COMMON, maximum_altitude=45.0)
    plan = plan_dynamic_path(
        current=(42.0, 8.5, start_z), target=(42.0, 8.5, end_z),
        obstacle_vector_body=(0.0, -3.5, -0.5), **settings,
    )
    assert not plan.avoiding
    assert all(point[:2] == pytest.approx((42.0, 8.5)) for point in plan.points)


@pytest.mark.parametrize("start_z,end_z", [(3.0, 15.0), (15.0, 3.0)])
@pytest.mark.parametrize("wall_distance", [2.35, 2.45, 2.50, 2.65])
def test_logistics_wall_does_not_interrupt_vertical_column(
    start_z, end_z, wall_distance,
):
    settings = dict(COMMON, clearance=2.2)
    plan = plan_dynamic_path(
        current=(0.08, -43.42, start_z), target=(0.0, -43.5, end_z),
        obstacle_vector_body=(0.1, -wall_distance, 0.15), **settings,
    )
    assert not plan.avoiding
    assert all(abs(point[0]) <= 0.08 for point in plan.points)
    assert all(-43.5 <= point[1] <= -43.42 for point in plan.points)


@pytest.mark.parametrize("obstacle", [(0.0, 0.0, 4.0), (0.0, -1.7, 0.0)])
def test_vertical_column_keeps_overhead_and_close_side_obstacle_protection(obstacle):
    plan = plan_dynamic_path(
        current=(0.0, -43.5, 3.0), target=(0.0, -43.5, 15.0),
        obstacle_vector_body=obstacle, **dict(COMMON, clearance=2.2),
    )
    assert plan.avoiding


def test_horizontal_cruise_retains_wider_wall_buffer():
    plan = plan_dynamic_path(
        current=(0.0, -43.5, 3.0), target=(15.0, -43.5, 3.0),
        obstacle_vector_body=(4.0, -2.4, 0.0), **COMMON,
    )
    assert plan.avoiding


def test_obstacle_inside_clearance_is_not_ignored_when_retreating():
    plan = plan_dynamic_path(
        current=(0.0, 0.0, 15.0), target=(20.0, 0.0, 15.0),
        obstacle_vector_body=(-1.0, 0.0, 0.0), **COMMON,
    )
    assert plan.avoiding


def test_closing_speed_distinguishes_approaching_and_receding_returns():
    tracker = ObstacleClosingSpeed()
    assert tracker.update((-4.0, 0.0, 0.0), 1.0) == 0.0
    assert tracker.update((-3.0, 0.0, 0.0), 1.5) > 0.2
    tracker = ObstacleClosingSpeed()
    tracker.update((-3.0, 0.0, 0.0), 1.0)
    assert tracker.update((-4.0, 0.0, 0.0), 1.5) < 0.0
    assert tracker.update((-4.0, 0.0, 0.0), 1.5) < 0.0


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
