import math

import pytest

from uav_control.safety_geometry import (
    StableObstacleVectorFilter,
    distance_from_safety_center,
    is_diagonal_ground_return,
    is_ground_return,
    minimum_valid_scan_range,
    minimum_obstacle_distance,
    minimum_obstacle_distances,
    nearest_obstacle_vector,
    obstacle_surface_vector,
)


def test_distance_uses_body_center_offset():
    distance = distance_from_safety_center(
        x=1.0,
        y=0.0,
        z=-0.27,
        lidar_height=0.45,
        center_height=0.18,
    )
    assert distance == 1.0


def test_lidar_ground_ring_is_filtered():
    assert is_ground_return(
        lidar_z=-0.45,
        ground_clearance=0.09,
        lidar_to_down_sensor=0.36,
        tolerance=0.02,
    )


def test_diagonal_sensor_distinguishes_ground_from_obstacle():
    expected_ground = 0.11 / math.sin(math.pi / 4.0)
    assert is_diagonal_ground_return(
        expected_ground,
        ground_clearance=0.09,
        down_sensor_height=0.09,
        diagonal_sensor_height=0.11,
        downward_angle=math.pi / 4.0,
        tolerance=0.02,
    )
    assert not is_diagonal_ground_return(
        0.45,
        ground_clearance=1.0,
        down_sensor_height=0.09,
        diagonal_sensor_height=0.11,
        downward_angle=math.pi / 4.0,
        tolerance=0.02,
    )


def test_minimum_distance_ignores_ground_and_keeps_obstacle():
    points = [
        (0.8, 0.0, -0.45),
        (0.6, 0.0, -0.27),
    ]
    minimum = minimum_obstacle_distance(
        points,
        lidar_height=0.45,
        center_height=0.18,
        ground_clearance=0.09,
        lidar_to_down_sensor=0.36,
        ground_tolerance=0.02,
    )
    assert minimum == 0.6


def test_minimum_distance_filters_the_uav_body():
    points = [
        (0.2, 0.0, -0.27),
        (0.9, 0.0, -0.27),
    ]
    minimum = minimum_obstacle_distance(
        points,
        lidar_height=0.45,
        center_height=0.18,
        ground_clearance=math.inf,
        lidar_to_down_sensor=0.36,
        ground_tolerance=0.02,
        self_filter_radius=0.58,
    )
    assert minimum == 0.9


def test_platform_distance_ignores_points_below_protected_height():
    minimum, platform_minimum = minimum_obstacle_distances(
        [
            (0.8, 0.0, -0.45),
            (0.9, 0.0, 0.15),
            (1.2, 0.0, 0.80),
        ],
        lidar_height=0.45,
        center_height=0.18,
        ground_clearance=math.inf,
        lidar_to_down_sensor=0.36,
        ground_tolerance=0.02,
        platform_protected_min_height=1.2,
    )
    assert math.isclose(minimum, math.hypot(0.8, 0.18))
    assert math.isclose(platform_minimum, math.hypot(1.2, 1.07))


def test_platform_distance_is_clear_when_all_points_are_below():
    _, platform_minimum = minimum_obstacle_distances(
        [(0.8, 0.0, -0.45)],
        lidar_height=0.45,
        center_height=0.18,
        ground_clearance=math.inf,
        lidar_to_down_sensor=0.36,
        ground_tolerance=0.02,
        platform_protected_min_height=1.2,
    )
    assert math.isinf(platform_minimum)


def test_scan_range_clamps_no_return_to_sensor_maximum():
    assert minimum_valid_scan_range(
        [math.inf, math.nan, 1.0e40],
        minimum_range=0.05,
        maximum_range=4.0,
    ) == 4.0


def test_scan_range_selects_nearest_valid_ray():
    assert minimum_valid_scan_range(
        [4.0, 1.25, 2.0, 0.01],
        minimum_range=0.05,
        maximum_range=4.0,
    ) == 1.25


def test_nearest_obstacle_vector_filters_ground_and_self_returns():
    minimum, vector = nearest_obstacle_vector(
        [
            (0.8, 0.0, -0.45),
            (0.2, 0.0, -0.27),
            (0.9, -0.3, -0.27),
        ],
        lidar_height=0.45,
        center_height=0.18,
        ground_clearance=0.09,
        lidar_to_down_sensor=0.36,
        ground_tolerance=0.02,
        self_filter_radius=0.58,
    )
    assert math.isclose(minimum, math.hypot(0.9, 0.3))
    assert vector == (0.9, -0.3, 0.0)


def test_obstacle_surface_vector_uses_one_local_surface_patch():
    minimum, vector = obstacle_surface_vector(
        [
            (2.00, -0.10, -0.27),
            (2.05, 0.00, -0.27),
            (2.10, 0.10, -0.27),
            (1.95, 0.80, -0.27),
        ],
        lidar_height=0.45,
        center_height=0.18,
        ground_clearance=math.inf,
        lidar_to_down_sensor=0.36,
        ground_tolerance=0.02,
        self_filter_radius=0.58,
        surface_depth=0.35,
        patch_radius=0.75,
    )
    assert minimum == pytest.approx(math.hypot(2.0, 0.1))
    assert vector == pytest.approx((2.05, 0.0, 0.0))


def test_stable_vector_filter_rejects_single_frame_target_jump():
    tracker = StableObstacleVectorFilter(
        alpha=0.5,
        association_distance=0.5,
        switch_confirmations=2,
        maximum_hold_updates=2,
    )
    assert tracker.update((2.0, 0.0, 0.0)) is None
    assert tracker.update((2.1, 0.0, 0.0)) == pytest.approx(
        (2.05, 0.0, 0.0)
    )
    assert tracker.update((0.0, 3.0, 0.0)) == pytest.approx(
        (2.05, 0.0, 0.0)
    )
    assert tracker.update((2.0, 0.1, 0.0)) == pytest.approx(
        (2.025, 0.05, 0.0)
    )


def test_stable_vector_filter_switches_after_repeated_observations():
    tracker = StableObstacleVectorFilter(
        alpha=0.5,
        association_distance=0.5,
        switch_confirmations=2,
        maximum_hold_updates=2,
    )
    tracker.update((2.0, 0.0, 0.0))
    tracker.update((2.0, 0.0, 0.0))
    tracker.update((0.0, 3.0, 0.0))
    switched = tracker.update((0.1, 3.0, 0.0))
    assert switched == pytest.approx((0.05, 3.0, 0.0))
