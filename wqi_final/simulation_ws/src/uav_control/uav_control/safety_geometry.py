import math
from typing import Iterable


def distance_from_safety_center(
    x: float,
    y: float,
    z: float,
    lidar_height: float,
    center_height: float,
) -> float:
    """Return a lidar point's distance from the body-centered safety sphere."""
    relative_z = z + lidar_height - center_height
    return math.sqrt(x * x + y * y + relative_z * relative_z)


def is_ground_return(
    lidar_z: float,
    ground_clearance: float,
    lidar_to_down_sensor: float,
    tolerance: float,
) -> bool:
    """Identify the flat-ground ring seen by downward lidar channels."""
    if not math.isfinite(ground_clearance):
        return False
    expected_ground_z = -(ground_clearance + lidar_to_down_sensor)
    return abs(lidar_z - expected_ground_z) <= tolerance


def is_diagonal_ground_return(
    measured_range: float,
    ground_clearance: float,
    down_sensor_height: float,
    diagonal_sensor_height: float,
    downward_angle: float,
    tolerance: float,
) -> bool:
    """Return true when a diagonal ray agrees with the measured ground plane."""
    if not all(math.isfinite(value) for value in (measured_range, ground_clearance)):
        return False
    vertical_clearance = (
        ground_clearance + diagonal_sensor_height - down_sensor_height
    )
    vertical_component = math.sin(downward_angle)
    if vertical_clearance <= 0.0 or vertical_component <= 0.0:
        return False
    expected_range = vertical_clearance / vertical_component
    return abs(measured_range - expected_range) <= tolerance


def minimum_valid_scan_range(
    ranges: Iterable,
    minimum_range: float,
    maximum_range: float,
) -> float:
    """Reduce a ray cone to one bounded range value."""
    valid = [
        float(value)
        for value in ranges
        if math.isfinite(value) and minimum_range <= value <= maximum_range
    ]
    return min(valid, default=maximum_range)


def minimum_obstacle_distance(
    points: Iterable,
    lidar_height: float,
    center_height: float,
    ground_clearance: float,
    lidar_to_down_sensor: float,
    ground_tolerance: float,
    self_filter_radius: float = 0.0,
) -> float:
    minimum, _ = minimum_obstacle_distances(
        points,
        lidar_height,
        center_height,
        ground_clearance,
        lidar_to_down_sensor,
        ground_tolerance,
        self_filter_radius,
    )
    return minimum


def minimum_obstacle_distances(
    points: Iterable,
    lidar_height: float,
    center_height: float,
    ground_clearance: float,
    lidar_to_down_sensor: float,
    ground_tolerance: float,
    self_filter_radius: float = 0.0,
    platform_protected_min_height: float = None,
):
    """Return full-sphere and platform-mode obstacle distances."""
    protected_height = (
        center_height
        if platform_protected_min_height is None
        else float(platform_protected_min_height)
    )
    minimum = math.inf
    protected_minimum = math.inf
    for point in points:
        x, y, z = (float(point[0]), float(point[1]), float(point[2]))
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        if is_ground_return(
            z,
            ground_clearance,
            lidar_to_down_sensor,
            ground_tolerance,
        ):
            continue
        distance = distance_from_safety_center(
            x,
            y,
            z,
            lidar_height,
            center_height,
        )
        if distance <= self_filter_radius:
            continue
        minimum = min(minimum, distance)
        if z + lidar_height >= protected_height:
            protected_minimum = min(protected_minimum, distance)
    return minimum, protected_minimum
