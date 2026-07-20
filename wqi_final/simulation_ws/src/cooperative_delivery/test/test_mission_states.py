import pytest

from cooperative_delivery.mission_states import (
    CooperativePhase,
    NavigationProgressTracker,
    is_terminal,
    is_vehicle_settled,
    navigation_timeout_for_distance,
)


def test_cooperative_state_machine_has_required_vehicle_handoffs():
    phases = {phase.value for phase in CooperativePhase}
    assert {
        "UGV_TRANSIT",
        "UGV_SETTLING",
        "UAV_DETACHING",
        "UAV_DELIVERING",
        "UAV_DOCKING",
        "RETURNING_HOME",
    }.issubset(phases)


def test_success_failure_and_cancellation_are_terminal():
    assert is_terminal(CooperativePhase.COMPLETED)
    assert is_terminal(CooperativePhase.CANCELED)
    assert is_terminal(CooperativePhase.FAILED)
    assert not is_terminal(CooperativePhase.UAV_DELIVERING)


def test_vehicle_settle_threshold_is_inclusive():
    assert is_vehicle_settled(0.0, 0.05)
    assert is_vehicle_settled(0.05, 0.05)
    assert not is_vehicle_settled(0.051, 0.05)


def test_navigation_timeout_scales_and_is_bounded():
    assert navigation_timeout_for_distance(0.0, 180.0, 6.0, 900.0) == 180.0
    assert navigation_timeout_for_distance(112.0, 180.0, 6.0, 900.0) == 852.0
    assert navigation_timeout_for_distance(1000.0, 180.0, 6.0, 900.0) == 900.0


def test_navigation_timeout_rejects_invalid_limits():
    with pytest.raises(ValueError):
        navigation_timeout_for_distance(10.0, 180.0, 0.0, 900.0)
    with pytest.raises(ValueError):
        navigation_timeout_for_distance(10.0, 900.0, 6.0, 180.0)


def test_navigation_progress_accepts_detours_and_turns():
    tracker = NavigationProgressTracker(0.05, 0.10)

    assert not tracker.update(62.0, 19.5, 1.57)
    assert tracker.update(62.0, 19.5, 1.40)
    assert tracker.update(62.10, 19.5, 1.40)
    assert tracker.update(62.20, 19.5, 1.40)


def test_navigation_progress_rejects_stationary_noise():
    tracker = NavigationProgressTracker(0.05, 0.10)

    assert not tracker.update(10.0, 20.0, 0.0)
    assert not tracker.update(10.01, 20.01, 0.02)
    assert not tracker.update(float("nan"), 20.0, 0.0)


def test_navigation_progress_handles_wrapped_yaw():
    tracker = NavigationProgressTracker(0.05, 0.10)

    assert not tracker.update(0.0, 0.0, 3.13)
    assert not tracker.update(0.0, 0.0, -3.13)
    assert tracker.update(0.0, 0.0, -2.9)
