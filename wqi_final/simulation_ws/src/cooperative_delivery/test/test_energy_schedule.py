from dataclasses import replace
from pathlib import Path

import pytest

from cooperative_delivery.energy_schedule import DeliveryEnergy, evaluate_schedule, optimize_schedule
from cooperative_delivery.ugv_energy_model import UgvEnergyParameters
from uav_control.battery_model import BatteryParameters


def inputs():
    ugv = UgvEnergyParameters.from_mapping(UgvEnergyParameters.defaults())
    battery = BatteryParameters.from_yaml(
        Path(__file__).parents[2] / "uav_control/config/battery_model.yaml")
    return ugv, battery


def test_as_needed_reduces_wait_and_losses_relative_to_full_charge():
    u, b = inputs()
    items = [DeliveryEnergy(.3, 5, 90)]
    matrix = [[0, 2], [2, 0]]
    args = ((0,), items, matrix, u, b, 30, 250, 200)
    full = evaluate_schedule(*args, charge_policy="full")
    needed = evaluate_schedule(*args, charge_policy="as_needed")
    assert full.feasible and needed.feasible
    assert needed.charge_wait_s < full.charge_wait_s
    assert needed.energy_wh < full.energy_wh


def test_insufficient_source_or_drive_pack_makes_schedule_infeasible():
    u, b = inputs()
    args = ((0,), [DeliveryEnergy(.3, 15, 90)], [[0, 2], [2, 0]], u, b)
    assert not evaluate_schedule(*args, 21, 250, 0).feasible
    assert not evaluate_schedule(*args, 80, 1, 200).feasible


def test_heavy_package_delivery_order_can_differ_from_distance_order():
    u, b = inputs()
    u = replace(u, rolling_resistance_coefficient=1.0, idle_power_w=0,
                linear_loss_power_w_per_mps=0, prediction_margin_factor=1)
    items = [DeliveryEnergy(.01, 1, 1), DeliveryEnergy(1.0, 1, 1)]
    matrix = [[0, 10, 10], [10, 0, 10], [10, 10, 0]]
    plan = optimize_schedule(items, matrix, u, b, 80, 250, 200,
                             seed_order=(0, 1), time_weight=0)
    assert plan.feasible
    assert plan.order == (1, 0)
    assert plan.method == "EXACT_PERMUTATIONS"


def test_large_schedule_is_bounded_and_preserves_duplicate_items():
    u, b = inputs()
    items = [DeliveryEnergy(.1, .1, 1)] * 7
    matrix = [[0 if i == j else 1 for j in range(8)] for i in range(8)]
    plan = optimize_schedule(items, matrix, u, b, 80, 250, 200, seed_order=range(7))
    assert sorted(plan.order) == list(range(7))
    assert plan.method == "BOUNDED_2OPT"


def test_invalid_weights_and_matrix_are_rejected():
    u, b = inputs()
    with pytest.raises(ValueError):
        evaluate_schedule((0,), [DeliveryEnergy(.1, 1, 1)], [[0]], u, b, 80, 250, 200)
    with pytest.raises(ValueError):
        evaluate_schedule((0,), [DeliveryEnergy(.1, 1, 1)], [[0, 1], [1, 0]],
                          u, b, 80, 250, 200, time_weight=0, energy_weight=0)


def test_full_battery_cannot_make_oversized_sortie_feasible():
    u, b = inputs()
    plan = evaluate_schedule((0,), [DeliveryEnergy(.1, b.capacity_wh, 30)],
                             [[0, 1], [1, 0]], u, b, b.capacity_wh, 250, 200)
    assert not plan.feasible


def test_charge_wait_is_added_when_transit_is_zero():
    u, b = inputs()
    plan = evaluate_schedule((0,), [DeliveryEnergy(.1, 5, 30)],
                             [[0, 0], [0, 0]], u, b, 20, 250, 200)
    assert plan.feasible
    assert plan.charge_wait_s > 0
    assert plan.duration_s == pytest.approx(30 + plan.charge_wait_s)
