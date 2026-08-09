from pathlib import Path

import pytest

from cooperative_delivery.ugv_energy_model import (
    UgvEnergyModel,
    UgvEnergyParameters,
    estimate_ugv_drive_energy,
)


CONFIG = Path(__file__).parents[1] / "config" / "ugv_energy_model.yaml"


def parameters():
    return UgvEnergyParameters.from_yaml(CONFIG)


def test_yaml_defines_two_independent_batteries():
    p = parameters()
    assert p.drive_capacity_wh == 300.0
    assert p.charging_capacity_wh == 250.0
    assert p.drive_reserve_wh == 60.0
    assert p.charging_reserve_wh == 25.0


def test_drive_power_increases_with_cargo_and_docked_uav_mass():
    model = UgvEnergyModel(parameters())
    empty = model.drive_power_w(0.22, 0.0, 0.0, 0.0, False)
    cargo = model.drive_power_w(0.22, 0.0, 0.0, 1.0, False)
    docked = model.drive_power_w(0.22, 0.0, 0.0, 1.0, True)
    assert cargo > empty
    assert docked > cargo


def test_acceleration_and_turning_increase_drive_power():
    model = UgvEnergyModel(parameters())
    steady = model.drive_power_w(0.22, 0.0, 0.0, 0.5, True)
    accelerating = model.drive_power_w(0.22, 0.0, 0.5, 0.5, True)
    turning = model.drive_power_w(0.22, 0.4, 0.0, 0.5, True)
    assert accelerating > steady
    assert turning > steady


def test_uav_charging_uses_only_the_charging_pack():
    model = UgvEnergyModel(parameters(), 0.80, 0.80)
    before = model.snapshot()
    after = model.step(60.0, 0.0, 0.0, 0.0, 0.3, True, -157.0)
    assert after.drive_energy_wh < before.drive_energy_wh
    assert after.charging_energy_wh < before.charging_energy_wh
    assert after.charging_output_power_w == pytest.approx(157.0)
    assert after.charging_consumed_wh > 0.0


def test_detached_uav_does_not_draw_from_charging_pack():
    model = UgvEnergyModel(parameters(), 0.80, 0.80)
    before = model.snapshot()
    after = model.step(60.0, 0.0, 0.0, 0.0, 0.0, False, -157.0)
    assert after.charging_energy_wh == before.charging_energy_wh
    assert after.charging_output_power_w == 0.0


def test_uav_charging_stops_at_separate_pack_reserve():
    p = parameters()
    reserve_soc = p.charging_reserve_percentage
    model = UgvEnergyModel(p, 0.80, reserve_soc)
    before = model.snapshot()
    after = model.step(60.0, 0.0, 0.0, 0.0, 0.0, True, -157.0)
    assert after.charging_energy_wh == before.charging_energy_wh
    assert after.charging_output_power_w == 0.0
    assert not after.charger_available


def test_drive_preflight_accounts_for_payload_reduction_and_return():
    p = parameters()
    estimate = estimate_ugv_drive_energy(
        p,
        available_energy_wh=240.0,
        leg_distances_m=[60.0, 30.0],
        payload_masses_kg=[0.4, 0.3],
        return_distance_m=70.0,
    )
    assert estimate.raw_energy_wh > 0.0
    assert estimate.predicted_energy_wh > estimate.raw_energy_wh
    assert estimate.required_energy_wh == pytest.approx(
        estimate.predicted_energy_wh + p.drive_reserve_wh
    )
    assert estimate.feasible


def test_drive_preflight_rejects_energy_below_reserve():
    estimate = estimate_ugv_drive_energy(
        parameters(),
        available_energy_wh=10.0,
        leg_distances_m=[1.0],
        payload_masses_kg=[0.2],
    )
    assert not estimate.feasible


def test_invalid_initial_soc_is_rejected():
    with pytest.raises(ValueError):
        UgvEnergyModel(parameters(), 1.1, 0.5)
