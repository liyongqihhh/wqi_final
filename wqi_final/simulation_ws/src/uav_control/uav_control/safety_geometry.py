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


def nearest_obstacle_vector(
    points: Iterable,
    lidar_height: float,
    center_height: float,
    ground_clearance: float,
    lidar_to_down_sensor: float,
    ground_tolerance: float,
    self_filter_radius: float = 0.0,
):
    """Return distance and body-frame vector to the nearest valid obstacle."""
    minimum = math.inf
    nearest = None
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
        relative_z = z + lidar_height - center_height
        distance = math.sqrt(x * x + y * y + relative_z * relative_z)
        if distance <= self_filter_radius:
            continue
        if distance < minimum:
            minimum = distance
            nearest = (x, y, relative_z)
    return minimum, nearest


def obstacle_surface_vector(
    points: Iterable,
    lidar_height: float,
    center_height: float,
    ground_clearance: float,
    lidar_to_down_sensor: float,
    ground_tolerance: float,
    self_filter_radius: float = 0.0,
    surface_depth: float = 0.35,
    patch_radius: float = 0.75,
):
    """Return a robust vector to the nearest obstacle surface patch."""
    candidates = []
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
        relative_z = z + lidar_height - center_height
        vector = (x, y, relative_z)
        distance = math.sqrt(x * x + y * y + relative_z * relative_z)
        if distance <= self_filter_radius:
            continue
        candidates.append((distance, vector))
    if not candidates:
        return math.inf, None

    minimum, nearest = min(candidates, key=lambda item: item[0])
    patch = [
        vector
        for distance, vector in candidates
        if (
            distance <= minimum + float(surface_depth)
            and math.dist(vector, nearest) <= float(patch_radius)
        )
    ]
    patch.sort()
    middle = len(patch) // 2

    def median(index: int) -> float:
        ordered = sorted(vector[index] for vector in patch)
        if len(ordered) % 2:
            return ordered[middle]
        return 0.5 * (ordered[middle - 1] + ordered[middle])

    return minimum, (median(0), median(1), median(2))


class StableObstacleVectorFilter:
    """Keep a lidar obstacle vector attached to one observed surface."""

    def __init__(
        self,
        alpha: float = 0.35,
        association_distance: float = 0.9,
        switch_confirmations: int = 3,
        maximum_hold_updates: int = 3,
    ) -> None:
        self.alpha = float(alpha)
        self.association_distance = float(association_distance)
        self.switch_confirmations = max(1, int(switch_confirmations))
        self.maximum_hold_updates = max(0, int(maximum_hold_updates))
        self.current = None
        self.pending = None
        self.pending_count = 0
        self.unmatched_count = 0

    def reset(self) -> None:
        self.current = None
        self.pending = None
        self.pending_count = 0
        self.unmatched_count = 0

    def _update_pending(self, candidate):
        if (
            self.pending is None
            or math.dist(self.pending, candidate) > self.association_distance
        ):
            self.pending = candidate
            self.pending_count = 1
        else:
            self.pending = tuple(
                0.5 * previous + 0.5 * value
                for previous, value in zip(self.pending, candidate)
            )
            self.pending_count += 1
        if self.pending_count < self.switch_confirmations:
            return None
        accepted = self.pending
        self.pending = None
        self.pending_count = 0
        return accepted

    def update(self, vector):
        candidate = tuple(float(value) for value in vector)
        if len(candidate) != 3 or not all(
            math.isfinite(value) for value in candidate
        ):
            raise ValueError("Obstacle vector must contain three finite values")
        if self.current is None:
            accepted = self._update_pending(candidate)
            if accepted is not None:
                self.current = accepted
                self.unmatched_count = 0
            return self.current
        if math.dist(self.current, candidate) <= self.association_distance:
            alpha = max(0.0, min(1.0, self.alpha))
            self.current = tuple(
                alpha * value + (1.0 - alpha) * previous
                for previous, value in zip(self.current, candidate)
            )
            self.pending = None
            self.pending_count = 0
            self.unmatched_count = 0
            return self.current

        self.unmatched_count += 1
        accepted = self._update_pending(candidate)
        if accepted is not None:
            self.current = accepted
            self.unmatched_count = 0
        elif self.unmatched_count > self.maximum_hold_updates:
            self.current = None
        return self.current
