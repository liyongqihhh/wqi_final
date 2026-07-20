from pathlib import Path

import pytest

from uav_control.battery_model import (
    BatteryConfigurationError,
    BatteryModel,
    BatteryParameters,
    MissionEnergySegment,
    MissionPowerProfile,
)


CONFIG = Path(__file__).parents[1] / "config" / "battery_model.yaml"


def parameters():
    return BatteryParameters.from_yaml(CONFIG)


def test_configuration_matches_uav_geometry_and_mass():
    config = parameters()
    assert config.rotor_count == 4
    assert config.base_mass_kg == pytest.approx(1.477)
    assert config.total_rotor_area_m2 == pytest.approx(
        4.0 * 3.141592653589793 * 0.12 ** 2
    )


def test_payload_induced_power_follows_mass_to_three_halves():
    model = BatteryModel(parameters())
    unloaded = model.hover_induced_power_w(0.0)
    loaded = model.hover_induced_power_w(0.5)
    expected_ratio = (
        (parameters().base_mass_kg + 0.5) / parameters().base_mass_kg
    ) ** 1.5
    assert loaded / unloaded == pytest.approx(expected_ratio)
    assert loaded > unloaded


def test_zeng_horizontal_model_reduces_to_hover_at_zero_speed():
    model = BatteryModel(parameters())
    horizontal, kappa = model.horizontal_propulsion_power_w(0.0)
    assert kappa == pytest.approx(1.0)
    assert horizontal == pytest.approx(
        model.hover_propulsion_power_w(0.0)
    )


def test_zeng_speed_curve_eventually_rises_with_parasite_power():
    model = BatteryModel(parameters())
    moderate, _ = model.horizontal_propulsion_power_w(5.0)
    fast, _ = model.horizontal_propulsion_power_w(40.0)
    assert fast > moderate


def test_dai_acceleration_and_turning_raise_same_speed_power():
    model = BatteryModel(parameters())
    steady = model.flight_power(
        "CRUISE", True, horizontal_speed_mps=1.2
    )
    accelerating = model.flight_power(
        "CRUISE",
        True,
        horizontal_speed_mps=1.2,
        horizontal_acceleration_mps2=0.8,
        horizontal_acceleration_velocity_dot_m2ps3=0.96,
    )
    turning = model.flight_power(
        "CRUISE",
        True,
        horizontal_speed_mps=1.2,
        horizontal_acceleration_mps2=0.8,
        horizontal_acceleration_velocity_dot_m2ps3=0.0,
    )
    assert accelerating.battery_power_w > steady.battery_power_w
    assert turning.battery_power_w > steady.battery_power_w
    assert accelerating.thrust_to_weight_ratio > 1.0


def test_gong_vertical_ascent_hover_and_descent_ordering():
    model = BatteryModel(parameters())
    ascent = model.vertical_propulsion_power_w(0.6, 0.0)
    hover = model.vertical_propulsion_power_w(0.0, 0.0)
    descent = model.vertical_propulsion_power_w(-0.6, 0.0)
    assert ascent > hover > descent


def test_vertical_acceleration_and_landing_braking_raise_power():
    model = BatteryModel(parameters())
    climb = model.flight_power(
        "TAKEOFF", True, vertical_speed_mps=0.4
    )
    accelerating_climb = model.flight_power(
        "TAKEOFF",
        True,
        vertical_speed_mps=0.4,
        vertical_acceleration_mps2=0.5,
    )
    descent = model.flight_power(
        "LANDING", True, vertical_speed_mps=-0.4
    )
    landing_brake = model.flight_power(
        "LANDING",
        True,
        vertical_speed_mps=-0.4,
        vertical_acceleration_mps2=0.5,
    )
    assert accelerating_climb.battery_power_w > climb.battery_power_w
    assert landing_brake.battery_power_w > descent.battery_power_w
    assert accelerating_climb.mode == "ASCENT_ACCELERATION"
    assert landing_brake.mode == "DESCENT_DECELERATION"


def test_runtime_modes_use_signed_vector_acceleration():
    model = BatteryModel(parameters())
    acceleration = model.flight_power(
        "CRUISE",
        True,
        horizontal_speed_mps=0.8,
        horizontal_acceleration_mps2=0.4,
        horizontal_acceleration_velocity_dot_m2ps3=0.32,
    )
    deceleration = model.flight_power(
        "CRUISE",
        True,
        horizontal_speed_mps=0.8,
        horizontal_acceleration_mps2=0.4,
        horizontal_acceleration_velocity_dot_m2ps3=-0.32,
    )
    maneuver = model.flight_power(
        "CRUISE",
        True,
        horizontal_speed_mps=0.8,
        horizontal_acceleration_mps2=0.4,
        horizontal_acceleration_velocity_dot_m2ps3=0.0,
    )
    assert acceleration.mode == "HORIZONTAL_ACCELERATION"
    assert deceleration.mode == "HORIZONTAL_DECELERATION"
    assert maneuver.mode == "MANEUVER"


def test_landing_context_rejects_false_ascent_classification():
    model = BatteryModel(parameters())
    breakdown = model.flight_power(
        "LANDING",
        True,
        vertical_speed_mps=0.1,
        vertical_acceleration_mps2=0.4,
    )
    assert breakdown.mode == "DESCENT_DECELERATION"


def test_landed_state_ignores_stale_motion():
    model = BatteryModel(parameters())
    breakdown = model.flight_power(
        "COMPLETED",
        False,
        horizontal_speed_mps=0.2,
        vertical_speed_mps=-0.2,
        vertical_acceleration_mps2=0.3,
    )
    assert breakdown.mode == "LANDED_IDLE"
    assert breakdown.battery_power_w == parameters().landed_idle_power_w


def test_docked_vehicle_charges_and_flight_discharges():
    model = BatteryModel(parameters(), initial_percentage=0.5)
    initial = model.energy_wh
    charging = model.step(60.0, "IDLE", False, True)
    assert charging.mode == "CHARGING"
    assert charging.energy_wh > initial
    assert charging.charged_energy_wh > 0.0
    assert charging.consumed_energy_wh == 0.0

    flying = model.step(
        60.0,
        "CRUISE",
        True,
        False,
        horizontal_speed_mps=1.0,
        payload_mass_kg=0.3,
    )
    assert flying.mode == "CRUISE"
    assert flying.energy_wh < charging.energy_wh
    assert flying.consumed_energy_wh > 0.0
    assert flying.payload_mass_kg == pytest.approx(0.3)


def test_segment_integration_contains_propulsion_and_auxiliary_energy():
    model = BatteryModel(parameters(), initial_percentage=0.8)
    profile = MissionPowerProfile(
        segments=(
            MissionEnergySegment(
                phase="horizontal_acceleration",
                duration_seconds=2.0,
                horizontal_speed_start_mps=0.0,
                horizontal_speed_end_mps=1.2,
                payload_mass_kg=0.3,
            ),
            MissionEnergySegment(
                phase="cruise",
                duration_seconds=60.0,
                horizontal_speed_start_mps=1.2,
                horizontal_speed_end_mps=1.2,
                payload_mass_kg=0.3,
            ),
        ),
        initial_payload_mass_kg=0.3,
    )
    breakdown = model.energy_breakdown_wh(profile)
    assessment = model.estimate(profile)
    assert breakdown.propulsion_energy_wh > 0.0
    assert breakdown.auxiliary_energy_wh > 0.0
    assert breakdown.payload_energy_penalty_wh > 0.0
    assert assessment.estimated_mission_energy_wh > (
        assessment.raw_mission_energy_wh
    )
    assert assessment.required_energy_wh == pytest.approx(
        assessment.estimated_mission_energy_wh
        + assessment.safety_reserve_wh
    )


def test_loaded_mission_costs_more_than_unloaded_mission():
    model = BatteryModel(parameters(), initial_percentage=1.0)
    unloaded = MissionPowerProfile(segments=(
        MissionEnergySegment(
            phase="hover",
            duration_seconds=30.0,
            payload_mass_kg=0.0,
        ),
    ))
    loaded = MissionPowerProfile(
        segments=(
            MissionEnergySegment(
                phase="hover",
                duration_seconds=30.0,
                payload_mass_kg=0.5,
            ),
        ),
        initial_payload_mass_kg=0.5,
    )
    assert (
        model.estimate(loaded).estimated_mission_energy_wh
        > model.estimate(unloaded).estimated_mission_energy_wh
    )


def test_low_battery_fails_preflight():
    model = BatteryModel(parameters(), initial_percentage=0.1)
    assessment = model.estimate(MissionPowerProfile(segments=(
        MissionEnergySegment(
            phase="hover",
            duration_seconds=1.0,
        ),
    )))
    assert not assessment.feasible
    assert assessment.required_energy_wh > assessment.current_energy_wh


def test_payload_above_limit_is_rejected():
    model = BatteryModel(parameters())
    with pytest.raises(ValueError):
        model.flight_power("HOVER", True, payload_mass_kg=1.1)


def test_invalid_percentage_configuration_is_rejected():
    values = {
        field: getattr(parameters(), field)
        for field in BatteryParameters.__dataclass_fields__
    }
    values["reserve_percentage"] = 1.2
    with pytest.raises(BatteryConfigurationError):
        BatteryParameters.from_mapping(values)


def test_empty_initial_battery_configuration_is_allowed():
    values = {
        field: getattr(parameters(), field)
        for field in BatteryParameters.__dataclass_fields__
    }
    values["initial_percentage"] = 0.0
    parsed = BatteryParameters.from_mapping(values)
    assert parsed.initial_percentage == 0.0
