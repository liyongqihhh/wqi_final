from dataclasses import replace
from pathlib import Path

import pytest

from cooperative_delivery.relay_energy import DriveLeg, FlightLeg, evaluate_relay_energy
from cooperative_delivery.ugv_energy_model import UgvEnergyParameters
from uav_control.battery_model import BatteryParameters


def parameters():
    root = Path(__file__).parents[2]
    return (UgvEnergyParameters.from_mapping({}), BatteryParameters.from_yaml(
        root / "uav_control/config/battery_model.yaml").for_backend("px4"))


def initial():
    return {"a_drive": 240., "a_charge": 200., "b_drive": 240., "b_charge": 200., "uav": 80.}


def test_other_car_also_consumes_idle_energy_during_transit():
    p, b = parameters()
    result = evaluate_relay_energy([DriveLeg("a", 40., .3, "a")], p, b, initial())
    assert result.feasible
    assert result.duration_s == pytest.approx(120.)
    assert result.consumed_wh["b_drive"] == pytest.approx(p.idle_power_w * 120 / 3600)
    assert result.consumed_wh["a_drive"] > result.consumed_wh["b_drive"]
    assert result.consumed_wh["a_charge"] > 0
    assert result.consumed_wh["b_charge"] == 0
    assert result.consumed_wh["uav"] == pytest.approx(
        max(b.docked_idle_power_w, b.landed_idle_power_w) * 120 / (3600 * b.discharge_efficiency))


def test_charging_draws_only_actual_departure_pack_and_conserves_conversion_losses():
    p, b = parameters()
    energy = dict(initial(), uav=10.)
    result = evaluate_relay_energy([FlightLeg("b", "a", 15., 60., 40.)], p, b, energy)
    seconds = 30 * 3600 / b.net_charge_power_w
    source_wh = (30 + b.docked_idle_power_w * seconds / 3600) / (
        b.charge_efficiency * p.charger_transfer_efficiency)
    assert result.feasible
    assert result.charging_wait_s == pytest.approx(seconds)
    assert result.remaining_wh["uav"] == pytest.approx(25.)
    assert result.consumed_wh["a_charge"] == 0
    assert result.consumed_wh["b_charge"] == pytest.approx(
        source_wh + p.charger_idle_power_w * seconds / 3600)
    assert result.consumed_wh["b_charge"] > 30
    assert result.consumed_wh["a_drive"] == pytest.approx(
        p.idle_power_w * (seconds + 60) / 3600)


@pytest.mark.parametrize("pack", ["a_drive", "a_charge", "b_drive", "b_charge"])
def test_each_ground_pack_is_checked_before_movement(pack):
    p, b = parameters()
    energy = initial()
    energy[pack] = 0.
    result = evaluate_relay_energy([DriveLeg("a", 40, .3, "a")], p, b, energy)
    assert not result.feasible and pack in result.reason
    assert result.duration_s == 0
    assert energy[pack] == 0  # Input snapshots are never mutated.


def test_finite_b_charging_pack_rejects_a_later_sortie():
    p, b = parameters()
    energy = dict(initial(), b_charge=p.charging_reserve_wh + 1)
    result = evaluate_relay_energy([
        FlightLeg("a", "b", 50., 30., 80.),
        FlightLeg("b", "b", 30., 30., 80.)], p, b, energy)
    assert not result.feasible and "b_charge" in result.reason


def test_capacity_and_return_reserve_are_not_bypassed():
    p, b = parameters()
    result = evaluate_relay_energy([FlightLeg("a", "b", 50., 30., 110.)], p, b, initial())
    assert not result.feasible and "capacity" in result.reason
    with pytest.raises(ValueError, match="reserve"):
        evaluate_relay_energy([FlightLeg("a", "b", 70., 30., 80.)], p, b, initial())


def test_empty_zero_distance_route_does_not_invent_consumption():
    p, b = parameters()
    result = evaluate_relay_energy([], p, b, initial())
    assert result.feasible and result.remaining_wh == initial()


@pytest.mark.parametrize("value", [None, float("nan"), -1, 400])
def test_invalid_energy_telemetry_fails_closed(value):
    p, b = parameters()
    with pytest.raises(ValueError, match="five batteries"):
        evaluate_relay_energy([], p, b, dict(initial(), a_drive=value))


def test_heavier_cargo_costs_more_and_ground_wait_can_exhaust_drive_reserve():
    p, b = parameters()
    low = evaluate_relay_energy([DriveLeg("a", 40, 0, "a")], p, b, initial())
    high = evaluate_relay_energy([DriveLeg("a", 40, 1, "a")], p, b, initial())
    assert high.consumed_wh["a_drive"] > low.consumed_wh["a_drive"]
    p = replace(p, idle_power_w=10000)
    assert not evaluate_relay_energy([FlightLeg("a", "b", 10., 3600., 80.)], p, b, initial()).feasible
