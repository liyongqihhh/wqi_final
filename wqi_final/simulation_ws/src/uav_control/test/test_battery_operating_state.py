import threading
from types import SimpleNamespace

import pytest

from uav_control.battery_manager import BatteryManager
from rclpy.logging import get_logger


@pytest.mark.parametrize("state,airborne", [(0, False), (1, True), (2, True), (3, True)])
def test_takeoff_and_landing_do_not_use_ground_idle_power(state, airborne):
    node = SimpleNamespace(lock=threading.Lock(), model=SimpleNamespace(soc=.8),
        charging_target_soc=.9, charging_active=False, phase="IDLE",
        vehicle_state=state, docked=False, charger_available=True,
        velocity_mps=(0, 0, 0), payload_mass_kg=0)
    assert BatteryManager._operating_state(node)[1] is airborne


def test_charge_target_hysteresis_prevents_sample_by_sample_switching():
    node = SimpleNamespace(lock=threading.Lock(), model=SimpleNamespace(soc=.6),
        charging_target_soc=.6, charging_active=True, phase="IDLE",
        vehicle_state=0, docked=True, charger_available=True,
        velocity_mps=(0, 0, 0), payload_mass_kg=0)
    assert BatteryManager._operating_state(node)[2] is False
    node.model.soc = .5995
    assert BatteryManager._operating_state(node)[2] is False
    node.model.soc = .597
    assert BatteryManager._operating_state(node)[2] is True


def test_locked_to_deck_does_not_charge_while_motors_are_taking_off():
    node = SimpleNamespace(lock=threading.Lock(), model=SimpleNamespace(soc=.5),
        charging_target_soc=.6, charging_active=True, phase="TAKEOFF",
        vehicle_state=2, docked=True, charger_available=True,
        velocity_mps=(0, 0, 0), payload_mass_kg=0)
    assert BatteryManager._operating_state(node)[2] is False


def test_level_changes_do_not_change_severity_at_one_ros_logging_callsite():
    logger = get_logger("battery_level_transition_regression")
    model = SimpleNamespace(soc=.8)
    model.snapshot = lambda: SimpleNamespace(mode="HOVER", soc=model.soc)
    node = SimpleNamespace(model=model, last_reported_mode="HOVER", last_level=None,
        parameters=SimpleNamespace(critical_percentage=.10, warning_percentage=.25),
        get_logger=lambda: logger)
    node._battery_level = lambda: BatteryManager._battery_level(node)
    for soc, expected in ((.8, "NORMAL"), (.24, "LOW"), (.09, "CRITICAL"),
                          (0, "CRITICAL"), (.5, "NORMAL"), (.24, "LOW")):
        model.soc = soc
        BatteryManager._log_state_changes(node)
        assert node.last_level == expected
