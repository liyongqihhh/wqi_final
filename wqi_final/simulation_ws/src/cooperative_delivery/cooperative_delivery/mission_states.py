from enum import Enum
import math


class CooperativePhase(str, Enum):
    IDLE = "IDLE"
    PREPARING = "PREPARING"
    UGV_TRANSIT = "UGV_TRANSIT"
    UGV_SETTLING = "UGV_SETTLING"
    UAV_DETACHING = "UAV_DETACHING"
    UAV_DELIVERING = "UAV_DELIVERING"
    UAV_DOCKING = "UAV_DOCKING"
    RETURNING_HOME = "RETURNING_HOME"
    COMPLETED = "COMPLETED"
    CANCELED = "CANCELED"
    FAILED = "FAILED"


TERMINAL_PHASES = {
    CooperativePhase.COMPLETED,
    CooperativePhase.CANCELED,
    CooperativePhase.FAILED,
}


def is_terminal(phase: CooperativePhase) -> bool:
    return phase in TERMINAL_PHASES


def is_vehicle_settled(speed: float, maximum_speed: float) -> bool:
    return 0.0 <= speed <= maximum_speed


class NavigationProgressTracker:
    """Track real vehicle motion without assuming the goal gets closer."""

    def __init__(
        self,
        minimum_translation: float,
        minimum_rotation: float,
    ) -> None:
        thresholds = (minimum_translation, minimum_rotation)
        if not all(math.isfinite(value) and value > 0.0 for value in thresholds):
            raise ValueError("Navigation progress thresholds must be positive")
        self.minimum_translation = minimum_translation
        self.minimum_rotation = minimum_rotation
        self.anchor = None

    @staticmethod
    def _angle_difference(first: float, second: float) -> float:
        return abs(math.atan2(
            math.sin(second - first),
            math.cos(second - first),
        ))

    def update(self, x: float, y: float, yaw: float) -> bool:
        pose = (float(x), float(y), float(yaw))
        if not all(math.isfinite(value) for value in pose):
            return False
        if self.anchor is None:
            self.anchor = pose
            return False

        translation = math.hypot(
            pose[0] - self.anchor[0],
            pose[1] - self.anchor[1],
        )
        rotation = self._angle_difference(self.anchor[2], pose[2])
        if (
            translation < self.minimum_translation
            and rotation < self.minimum_rotation
        ):
            return False

        self.anchor = pose
        return True


def navigation_timeout_for_distance(
    distance: float,
    minimum: float,
    seconds_per_meter: float,
    maximum: float,
) -> float:
    values = (distance, minimum, seconds_per_meter, maximum)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Navigation timeout inputs must be finite")
    if minimum <= 0.0 or seconds_per_meter <= 0.0 or maximum < minimum:
        raise ValueError("Navigation timeout limits are invalid")
    estimated = minimum + max(0.0, distance) * seconds_per_meter
    return min(maximum, max(minimum, estimated))
