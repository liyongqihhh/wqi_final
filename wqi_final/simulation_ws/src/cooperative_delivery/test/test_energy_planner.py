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


def test_sequence_caps_charge_by_separate_ugv_charging_battery():
    plan = plan_cooperative_energy(
        initial_energy_wh=30.0,
        battery_capacity_wh=100.0,
        reserve_energy_wh=20.0,
        net_charge_power_w=180.0,
        ugv_planning_speed_mps=1.0,
        initial_x=0.0,
        initial_y=0.0,
        sorties=[sortie("target", 100.0, 15.0)],
        charger_available_energy_wh=4.0,
        charger_transfer_efficiency=0.5,
    )
    assert not plan.feasible
    assert plan.final_energy_wh == pytest.approx(32.0)
    assert plan.remaining_charger_energy_wh == pytest.approx(0.0)


def test_sequence_reports_charging_pack_energy_used():
    plan = plan_cooperative_energy(
        initial_energy_wh=50.0,
        battery_capacity_wh=100.0,
        reserve_energy_wh=10.0,
        net_charge_power_w=100.0,
        ugv_planning_speed_mps=1.0,
        initial_x=0.0,
        initial_y=0.0,
        sorties=[sortie("target", 36.0, 5.0)],
        charger_available_energy_wh=20.0,
        charger_transfer_efficiency=0.8,
    )
    step = plan.steps[0]
    assert step.minimum_charge_wh == pytest.approx(1.0)
    assert step.charging_source_energy_wh == pytest.approx(1.25)
    assert plan.remaining_charger_energy_wh == pytest.approx(18.75)

def test_sequence_uses_planned_road_distance_for_charging():
    plan = plan_cooperative_energy(
        initial_energy_wh=50.0,
        battery_capacity_wh=100.0,
        reserve_energy_wh=10.0,
        net_charge_power_w=100.0,
        ugv_planning_speed_mps=1.0,
        initial_x=0.0,
        initial_y=0.0,
        sorties=[EnergySortie(
            target_name="road_target",
            launch_x=3.0,
            launch_y=4.0,
            mission_energy_wh=5.0,
            ugv_distance_m=20.0,
        )],
    )

    step = plan.steps[0]
    assert step.ugv_distance_m == 20.0
    assert step.minimum_charge_wh == pytest.approx(100.0 * 20.0 / 3600.0)



def test_invalid_empty_sequence_is_rejected():
    with pytest.raises(ValueError):
        plan_cooperative_energy(
            80.0, 100.0, 20.0, 157.0, 0.22, 0.0, 0.0, []
        )
