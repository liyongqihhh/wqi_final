from dataclasses import dataclass
import math


Point3 = tuple[float, float, float]


@dataclass(frozen=True)
class DynamicPathPlan:
    points: tuple[Point3, ...]
    avoiding: bool
    preferred_side: int
    reason: str


def _vector_length(vector) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def _normalize(vector) -> Point3:
    length = _vector_length(vector)
    if length <= 1.0e-9:
        raise ValueError("Cannot normalize a zero-length vector")
    return tuple(float(value) / length for value in vector)


def vector_in_map_frame(vector, orientation) -> Point3:
    """Rotate a body-frame vector into the map frame."""
    x, y, z = (float(value) for value in vector)
    qx, qy, qz, qw = (float(value) for value in orientation)
    values = (x, y, z, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Obstacle vector transform inputs must be finite")
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1.0e-9:
        raise ValueError("UAV orientation quaternion is invalid")
    qx, qy, qz, qw = (
        qx / norm,
        qy / norm,
        qz / norm,
        qw / norm,
    )
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + qy * tz - qz * ty,
        y + qw * ty + qz * tx - qx * tz,
        z + qw * tz + qx * ty - qy * tx,
    )


def _route_point(start: Point3, direction: Point3, distance: float) -> Point3:
    return tuple(
        start[index] + direction[index] * float(distance)
        for index in range(3)
    )


def _offset_point(point: Point3, direction: Point3, distance: float) -> Point3:
    return tuple(
        point[index] + direction[index] * float(distance)
        for index in range(3)
    )


def _clamp_altitude(point: Point3, minimum: float, maximum: float) -> Point3:
    return (
        point[0],
        point[1],
        max(float(minimum), min(float(maximum), point[2])),
    )


def _chaikin(points, iterations: int = 2) -> tuple[Point3, ...]:
    result = tuple(points)
    for _ in range(max(0, int(iterations))):
        if len(result) <= 2:
            break
        refined = [result[0]]
        for start, end in zip(result, result[1:]):
            refined.append(tuple(
                0.75 * start[index] + 0.25 * end[index]
                for index in range(3)
            ))
            refined.append(tuple(
                0.25 * start[index] + 0.75 * end[index]
                for index in range(3)
            ))
        refined.append(result[-1])
        result = tuple(refined)
    return result


def resample_polyline(points, spacing: float) -> tuple[Point3, ...]:
    """Resample a three-dimensional polyline at approximately fixed spacing."""
    spacing = float(spacing)
    if spacing <= 0.0 or not math.isfinite(spacing):
        raise ValueError("Path spacing must be positive and finite")
    source = tuple(
        tuple(float(value) for value in point)
        for point in points
    )
    if not source:
        return ()
    output = [source[0]]
    for start, end in zip(source, source[1:]):
        distance = math.dist(start, end)
        samples = max(1, int(math.ceil(distance / spacing)))
        for sample in range(1, samples + 1):
            ratio = sample / samples
            point = tuple(
                start[index] + ratio * (end[index] - start[index])
                for index in range(3)
            )
            if math.dist(output[-1], point) > 1.0e-6:
                output.append(point)
    return tuple(output)


def _horizontal_normal(route_direction: Point3, preferred_side: int) -> Point3:
    horizontal = math.hypot(route_direction[0], route_direction[1])
    side = 1 if int(preferred_side) >= 0 else -1
    if horizontal <= 1.0e-6:
        return (float(side), 0.0, 0.0)
    return (
        -route_direction[1] / horizontal * side,
        route_direction[0] / horizontal * side,
        0.0,
    )


def plan_dynamic_path(
    current,
    target,
    orientation,
    obstacle_vector_body,
    clearance: float,
    warning_distance: float,
    rear_warning_distance: float,
    target_clearance: float,
    minimum_altitude: float,
    maximum_altitude: float,
    spacing: float,
    preferred_side: int = 1,
) -> DynamicPathPlan:
    """Generate a smooth forward-progressing path from live 3D obstacle data."""
    start = tuple(float(value) for value in current)
    destination = tuple(float(value) for value in target)
    values = (
        *start,
        *destination,
        float(clearance),
        float(warning_distance),
        float(rear_warning_distance),
        float(target_clearance),
        float(minimum_altitude),
        float(maximum_altitude),
        float(spacing),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Dynamic path inputs must be finite")
    if (
        clearance <= 0.0
        or warning_distance <= 0.0
        or rear_warning_distance <= 0.0
        or target_clearance < 0.0
        or minimum_altitude >= maximum_altitude
    ):
        raise ValueError("Dynamic path limits are invalid")

    route = tuple(
        destination[index] - start[index] for index in range(3)
    )
    route_length = _vector_length(route)
    if route_length <= 1.0e-6:
        return DynamicPathPlan(
            points=(start, destination),
            avoiding=False,
            preferred_side=preferred_side,
            reason="target_reached",
        )
    if route_length <= float(target_clearance):
        return DynamicPathPlan(
            points=resample_polyline((start, destination), spacing),
            avoiding=False,
            preferred_side=preferred_side,
            reason="target_within_clearance",
        )
    route_direction = _normalize(route)
    obstacle = vector_in_map_frame(obstacle_vector_body, orientation)
    obstacle_distance = _vector_length(obstacle)
    if obstacle_distance <= 1.0e-6:
        raise ValueError("Obstacle vector is too small")

    projection = sum(
        obstacle[index] * route_direction[index] for index in range(3)
    )
    closest_projection = max(0.0, min(route_length, projection))
    lateral = tuple(
        obstacle[index] - route_direction[index] * closest_projection
        for index in range(3)
    )
    lateral_distance = _vector_length(lateral)
    obstacle_near_target = (
        projection >= route_length - float(target_clearance)
        and obstacle_distance >= route_length - float(target_clearance)
    )
    ahead_conflict = (
        not obstacle_near_target
        and obstacle_distance <= float(warning_distance)
        and 0.0 <= projection <= route_length
        and lateral_distance <= float(clearance)
    )
    rear_conflict = (
        projection < 0.0
        and obstacle_distance <= float(rear_warning_distance)
    )
    close_conflict = (
        obstacle_distance <= float(clearance)
        and not obstacle_near_target
    )
    if not (ahead_conflict or rear_conflict or close_conflict):
        return DynamicPathPlan(
            points=resample_polyline((start, destination), spacing),
            avoiding=False,
            preferred_side=preferred_side,
            reason="clear",
        )

    side = 1 if int(preferred_side) >= 0 else -1
    if rear_conflict:
        bypass_direction = _horizontal_normal(route_direction, side)
    elif lateral_distance > 0.20:
        bypass_direction = _normalize(tuple(-value for value in lateral))
        route_normal = _horizontal_normal(route_direction, 1)
        signed_side = sum(
            bypass_direction[index] * route_normal[index]
            for index in range(3)
        )
        if abs(signed_side) > 0.1:
            side = 1 if signed_side > 0.0 else -1
    else:
        bypass_direction = _horizontal_normal(route_direction, side)

    center_distance = max(0.0, min(route_length, projection))
    transition = max(2.0, float(clearance) * 1.4)
    if rear_conflict:
        center_distance = min(route_length, transition)
    entry_distance = max(0.0, center_distance - transition)
    exit_distance = min(route_length, center_distance + transition)
    if exit_distance <= entry_distance + 0.5:
        exit_distance = min(route_length, entry_distance + transition)

    offset = float(clearance)
    entry = _offset_point(
        _route_point(start, route_direction, entry_distance),
        bypass_direction,
        0.35 * offset,
    )
    apex = _offset_point(
        _route_point(start, route_direction, center_distance),
        bypass_direction,
        offset,
    )
    exit_point = _offset_point(
        _route_point(start, route_direction, exit_distance),
        bypass_direction,
        0.35 * offset,
    )
    anchors = tuple(
        _clamp_altitude(point, minimum_altitude, maximum_altitude)
        for point in (start, entry, apex, exit_point, destination)
    )
    smoothed = _chaikin(anchors, iterations=2)
    path = resample_polyline(smoothed, spacing)
    reason = "rear_dynamic_obstacle" if rear_conflict else "route_obstacle"
    return DynamicPathPlan(
        points=path,
        avoiding=True,
        preferred_side=side,
        reason=reason,
    )


def path_lookahead_point(
    current,
    points,
    lookahead_distance: float,
) -> Point3:
    """Select a forward point on a replanned path without reversing."""
    position = tuple(float(value) for value in current)
    path = tuple(tuple(float(value) for value in point) for point in points)
    if not path:
        raise ValueError("Cannot follow an empty path")
    nearest = min(
        range(len(path)),
        key=lambda index: math.dist(position, path[index]),
    )
    travelled = math.dist(position, path[nearest])
    previous = path[nearest]
    for point in path[nearest + 1:]:
        travelled += math.dist(previous, point)
        if travelled >= float(lookahead_distance):
            return point
        previous = point
    return path[-1]
