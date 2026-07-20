import math
from collections.abc import Iterable


def _point3(values: Iterable[float], name: str) -> tuple[float, float, float]:
    point = tuple(float(value) for value in values)
    if len(point) != 3:
        raise ValueError(f"{name} must contain exactly three coordinates")
    if not all(math.isfinite(value) for value in point):
        raise ValueError(f"{name} coordinates must be finite")
    return point


def limited_position_setpoint(
    current: Iterable[float],
    target: Iterable[float],
    maximum_step: float,
) -> tuple[float, float, float]:
    """Return a setpoint no farther than maximum_step from current."""
    current_point = _point3(current, "current")
    target_point = _point3(target, "target")
    maximum_step = float(maximum_step)
    if not math.isfinite(maximum_step) or maximum_step <= 0.0:
        raise ValueError("maximum_step must be finite and greater than zero")

    distance = math.dist(current_point, target_point)
    if distance == 0.0 or distance <= maximum_step:
        return target_point

    scale = maximum_step / distance
    return tuple(
        current_value + (target_value - current_value) * scale
        for current_value, target_value in zip(current_point, target_point)
    )


def adaptive_position_setpoint(
    current: Iterable[float],
    target: Iterable[float],
    cruise_step: float,
    approach_step: float,
    slowdown_distance: float,
) -> tuple[tuple[float, float, float], bool]:
    """Limit position error and use a shorter lookahead near the destination."""
    current_point = _point3(current, "current")
    target_point = _point3(target, "target")
    cruise_step = float(cruise_step)
    approach_step = float(approach_step)
    slowdown_distance = float(slowdown_distance)

    parameters = (cruise_step, approach_step, slowdown_distance)
    if not all(math.isfinite(value) and value > 0.0 for value in parameters):
        raise ValueError("setpoint distances must be finite and greater than zero")
    if approach_step > cruise_step:
        raise ValueError("approach_step must not exceed cruise_step")

    distance = math.dist(current_point, target_point)
    approaching = distance <= slowdown_distance
    maximum_step = approach_step if approaching else cruise_step
    return (
        limited_position_setpoint(current_point, target_point, maximum_step),
        approaching,
    )
