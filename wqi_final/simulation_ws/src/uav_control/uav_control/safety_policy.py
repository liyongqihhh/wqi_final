BLIND_SPOT_SENSORS = frozenset(
    {"front_down", "rear_down", "left_down", "right_down"}
)


def safety_issue(
    blocked: bool,
    status: str,
    allow_platform_proximity: bool,
) -> str:
    """Return the active safety issue, allowing only deck-sensor proximity."""
    if not blocked:
        return ""

    status = status or "Collision envelope blocked"
    if allow_platform_proximity and status.startswith("BLOCKED:"):
        sensors = {
            name.strip()
            for name in status.removeprefix("BLOCKED:").split(",")
            if name.strip()
        }
        if sensors and sensors.issubset(BLIND_SPOT_SENSORS):
            return ""
    return status
