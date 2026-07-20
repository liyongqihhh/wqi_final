from enum import Enum


class MissionPhase(str, Enum):
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER"
    CRUISE = "CRUISE"
    APPROACH = "APPROACH"
    DELIVERING = "DELIVERING"
    RETURNING = "RETURNING"
    LANDING = "LANDING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


TERMINAL_PHASES = {MissionPhase.COMPLETED, MissionPhase.FAILED}


def is_terminal(phase: MissionPhase) -> bool:
    return phase in TERMINAL_PHASES


def uses_local_delivery_profile(requested_home_name: str) -> bool:
    """A custom home is supplied by the UGV-UAV cooperative manager."""
    return bool(requested_home_name.strip())


def is_settled_on_ground(
    altitude: float,
    linear_speed: float,
    max_altitude: float = 0.25,
    max_speed: float = 0.1,
) -> bool:
    return altitude <= max_altitude and linear_speed <= max_speed


def is_settled_at_altitude(
    altitude: float,
    linear_speed: float,
    target_altitude: float,
    altitude_tolerance: float = 0.25,
    max_speed: float = 0.1,
) -> bool:
    return (
        altitude <= target_altitude + altitude_tolerance
        and linear_speed <= max_speed
    )
