from uav_application.mission_states import (
    MissionPhase,
    is_settled_at_altitude,
    is_settled_on_ground,
    is_terminal,
    uses_local_delivery_profile,
)


def test_required_mission_phases_exist():
    required = {
        "IDLE",
        "TAKEOFF",
        "HOVER",
        "CRUISE",
        "DELIVERING",
        "RETURNING",
        "LANDING",
        "COMPLETED",
        "FAILED",
    }
    assert required.issubset({phase.value for phase in MissionPhase})


def test_only_completion_and_failure_are_terminal():
    assert is_terminal(MissionPhase.COMPLETED)
    assert is_terminal(MissionPhase.FAILED)
    assert not is_terminal(MissionPhase.CRUISE)


def test_ground_contact_requires_low_altitude_and_low_speed():
    assert is_settled_on_ground(0.02, 0.01)
    assert not is_settled_on_ground(0.30, 0.01)
    assert not is_settled_on_ground(0.02, 0.20)


def test_vehicle_deck_contact_uses_requested_landing_height():
    assert is_settled_at_altitude(0.23, 0.01, 0.23)
    assert is_settled_at_altitude(0.40, 0.01, 0.23)
    assert not is_settled_at_altitude(0.60, 0.01, 0.23)
    assert not is_settled_at_altitude(0.23, 0.20, 0.23)


def test_custom_home_selects_cooperative_local_delivery_profile():
    assert uses_local_delivery_profile("dormitory")
    assert uses_local_delivery_profile("teaching_building")
    assert not uses_local_delivery_profile("")
    assert not uses_local_delivery_profile("   ")
