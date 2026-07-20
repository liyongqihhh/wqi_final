import pytest

from cooperative_delivery.energy_planner import (
    EnergySortie,
    plan_cooperative_energy,
)


def sortie(name, x, energy):
    return EnergySortie(name, x, 0.0, energy)


def test_sequence_accounts_for_docked_ugv_transit_charging():
    plan = plan_cooperative_energy(
        initial_energy_wh=30.0,
        battery_capacity_wh=100.0,
        reserve_energy_wh=20.0,
        net_charge_power_w=157.0,
        ugv_planning_speed_mps=0.22,
        initial_x=0.0,
        initial_y=0.0,
        sorties=[sortie("first", 55.0, 20.0)],
    )
    assert plan.feasible
    assert plan.steps[0].minimum_charge_wh == pytest.approx(
        157.0 * 250.0 / 3600.0
    )
    assert plan.steps[0].takeoff_energy_wh > 30.0


def test_sequence_rejects_first_sortie_that_breaks_reserve():
    plan = plan_cooperative_energy(
        initial_energy_wh=45.0,
        battery_capacity_wh=100.0,
        reserve_energy_wh=20.0,
        net_charge_power_w=0.0,
        ugv_planning_speed_mps=0.22,
        initial_x=0.0,
        initial_y=0.0,
        sorties=[
            sortie("first", 0.0, 20.0),
            sortie("second", 0.0, 10.0),
        ],
    )
    assert not plan.feasible
    assert "second" in plan.message
    assert len(plan.steps) == 1


def test_sequence_caps_charge_at_battery_capacity():
    plan = plan_cooperative_energy(
        initial_energy_wh=99.0,
        battery_capacity_wh=100.0,
        reserve_energy_wh=20.0,
        net_charge_power_w=157.0,
        ugv_planning_speed_mps=0.22,
        initial_x=0.0,
        initial_y=0.0,
        sorties=[sortie("target", 100.0, 10.0)],
    )
    assert plan.feasible
    assert plan.steps[0].takeoff_energy_wh == 100.0
    assert plan.final_energy_wh == 90.0


def test_invalid_empty_sequence_is_rejected():
    with pytest.raises(ValueError):
        plan_cooperative_energy(
            80.0, 100.0, 20.0, 157.0, 0.22, 0.0, 0.0, []
        )
