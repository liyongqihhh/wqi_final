from types import SimpleNamespace

import pytest

from delivery_evaluation.energy_estimator import (
    UgvEnergyIntegrator,
    UgvEnergyParameters,
)
from delivery_evaluation.metrics_collector import MissionMetricsCollector
from delivery_evaluation.models import PoseTarget, RunRecord


def collector(ugv_radius_m=0.22, uav_radius_m=0.56):
    parameters = UgvEnergyParameters(
        idle_power_w=40.0,
        linear_power_w_per_mps=100.0,
        angular_power_w_per_radps=20.0,
        payload_power_w_per_kg=5.0,
        drivetrain_efficiency=1.0,
        maximum_update_step_s=1.0,
        maximum_position_jump_m=2.0,
    )
    return MissionMetricsCollector(
        UgvEnergyIntegrator(parameters),
        ugv_collision_radius_m=ugv_radius_m,
        uav_collision_radius_m=uav_radius_m,
    )


def odometry(x, y, z=0.0, stamp_ns=0):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=int(stamp_ns // 1_000_000_000),
                nanosec=int(stamp_ns % 1_000_000_000),
            )
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                angular=SimpleNamespace(z=0.0),
            )
        ),
    )


def record():
    return RunRecord(
        run_id="test-run",
        mode="cooperative",
        scenario="test",
        repetition=1,
        obstacle_density="low",
        random_seed=42,
        targets=["building"],
    )


def test_endpoint_is_captured_once_at_delivery_event():
    metrics = collector()
    metrics.latest_uav = ((10.0, 20.0, 5.0), 0.0)
    metrics.start(record(), 0)
    target = PoseTarget(10.0, 20.0, 5.0)
    metrics.mark_uav_endpoint(target, "building")

    metrics.latest_uav = ((0.0, 0.0, 0.0), 0.0)
    metrics.mark_uav_endpoint(target, "building")
    result = metrics.finish(1_000_000_000)

    assert result.uav_endpoint_error_m == pytest.approx(0.0)


def test_replan_counter_keeps_run_maximum():
    metrics = collector()
    metrics.start(record(), 0)
    metrics.update_replans(2)
    metrics.update_replans(1)
    result = metrics.finish(1_000_000_000)

    assert result.uav_replan_count == 2


def test_safety_metrics_exclude_takeoff_and_landing_ground_clearance():
    metrics = collector()
    metrics.start(record(), 0)
    metrics.set_phase("TAKEOFF", 0)
    metrics.update_clearance(0.2)
    metrics.update_safety(True, 100_000_000)
    metrics.set_phase("CRUISE", 1_000_000_000)
    metrics.update_clearance(2.4)
    metrics.update_safety(True, 1_500_000_000)
    metrics.update_safety(False, 2_500_000_000)
    metrics.set_phase("LANDING", 3_000_000_000)
    metrics.update_clearance(0.1)
    metrics.update_safety(True, 3_500_000_000)
    result = metrics.finish(4_000_000_000)

    assert result.minimum_uav_clearance_m == pytest.approx(2.4)
    assert result.uav_safety_hold_count == 1
    assert result.uav_safety_hold_s == pytest.approx(1.0)


def test_leaving_airborne_phase_closes_active_safety_hold():
    metrics = collector()
    metrics.start(record(), 0)
    metrics.set_phase("RETURNING", 0)
    metrics.update_safety(True, 500_000_000)
    metrics.set_phase("LANDING", 2_000_000_000)
    result = metrics.finish(3_000_000_000)

    assert result.uav_safety_hold_count == 1
    assert result.uav_safety_hold_s == pytest.approx(1.5)


def test_live_dual_ugv_batteries_replace_offline_drive_estimate():
    metrics = collector()
    metrics.update_ugv_drive_battery(0.8)
    metrics.update_ugv_charging_battery(0.7)
    metrics.update_ugv_drive_consumed(10.0)
    metrics.update_ugv_charging_consumed(4.0)
    metrics.update_consumed(8.0)
    metrics.update_charged(3.0)
    metrics.start(record(), 0)

    metrics.update_ugv_drive_battery(0.75)
    metrics.update_ugv_charging_battery(0.68)
    metrics.update_ugv_drive_consumed(12.5)
    metrics.update_ugv_charging_consumed(5.5)
    metrics.update_consumed(14.0)
    metrics.update_charged(5.0)
    result = metrics.finish(1_000_000_000)

    assert result.ugv_drive_energy_wh == pytest.approx(2.5)
    assert result.ugv_charging_energy_wh == pytest.approx(1.5)
    assert result.ugv_energy_wh == pytest.approx(4.0)
    assert result.uav_energy_wh == pytest.approx(6.0)
    assert result.uav_charged_wh == pytest.approx(2.0)
    assert result.total_energy_wh == pytest.approx(8.0)
    assert result.initial_ugv_drive_soc == pytest.approx(0.8)
    assert result.final_ugv_charging_soc == pytest.approx(0.68)


def test_missing_uav_energy_telemetry_does_not_create_false_consumption():
    metrics = collector()
    metrics.start(record(), 0)
    result = metrics.finish(1_000_000_000)

    assert result.uav_energy_wh == pytest.approx(0.0)
    assert result.uav_charged_wh == pytest.approx(0.0)


def test_ground_truth_clearance_counts_distinct_ugv_collision_episodes():
    metrics = collector(ugv_radius_m=0.5)
    metrics.update_ugv(odometry(-2.0, 0.0))
    metrics.update_dynamic_obstacle(
        "pedestrian",
        "ground",
        0.5,
        odometry(0.0, 0.0, 0.8),
    )
    metrics.start(record(), 0)

    metrics.update_ugv(odometry(-0.8, 0.0, stamp_ns=1_000_000_000))
    metrics.update_dynamic_obstacle(
        "pedestrian",
        "ground",
        0.5,
        odometry(0.0, 0.0, 0.8),
    )
    metrics.update_ugv(odometry(-2.0, 0.0, stamp_ns=2_000_000_000))
    metrics.update_ugv(odometry(-0.8, 0.0, stamp_ns=3_000_000_000))
    result = metrics.finish(4_000_000_000)

    assert result.minimum_ugv_obstacle_clearance_m == pytest.approx(-0.2)
    assert result.ugv_collision_count == 2
    assert result.uav_collision_count == 0
    assert result.collision_free is False


def test_air_ground_truth_uses_three_dimensional_uav_clearance():
    metrics = collector(uav_radius_m=0.5)
    metrics.update_uav(odometry(0.0, 0.0, 12.0))
    metrics.update_dynamic_obstacle(
        "aircraft",
        "air",
        0.5,
        odometry(0.0, 0.0, 10.0),
    )
    metrics.start(record(), 0)
    metrics.update_uav(
        odometry(0.0, 0.0, 10.8, stamp_ns=1_000_000_000)
    )
    result = metrics.finish(2_000_000_000)

    assert result.minimum_uav_dynamic_clearance_m == pytest.approx(-0.2)
    assert result.uav_collision_count == 1
    assert result.ugv_collision_count == 0
    assert result.collision_free is False
