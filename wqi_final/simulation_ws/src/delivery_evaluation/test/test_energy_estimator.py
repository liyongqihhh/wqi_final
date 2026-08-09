import pytest

from delivery_evaluation.energy_estimator import (
    UgvEnergyIntegrator,
    UgvEnergyParameters,
)


def parameters():
    return UgvEnergyParameters(
        idle_power_w=40.0,
        linear_power_w_per_mps=100.0,
        angular_power_w_per_radps=20.0,
        payload_power_w_per_kg=5.0,
        drivetrain_efficiency=1.0,
        maximum_update_step_s=1.0,
        maximum_position_jump_m=2.0,
    )


def test_power_accounts_for_motion_and_payload():
    model = UgvEnergyIntegrator(parameters())
    assert model.power_w(1.0, 0.5, 2.0) == pytest.approx(160.0)


def test_energy_uses_trapezoidal_integration():
    model = UgvEnergyIntegrator(parameters())
    assert model.update(0, 0.0, 0.0)
    assert model.update(1_000_000_000, 1.0, 0.0)
    assert model.energy_wh == pytest.approx(90.0 / 3600.0)


def test_large_time_gap_is_not_integrated():
    model = UgvEnergyIntegrator(parameters())
    model.update(0, 0.0, 0.0)
    assert not model.update(2_000_000_000, 1.0, 0.0)
    assert model.energy_wh == 0.0
