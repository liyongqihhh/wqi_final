from pathlib import Path

import pytest

from uav_application.mission_energy import MissionEnergyPlanner
from uav_control.battery_model import BatteryModel, BatteryParameters
from uav_navigation.waypoint_navigator import WaypointMap


SOURCE_ROOT = Path(__file__).parents[2]
WAYPOINT_CONFIG = (
    SOURCE_ROOT
    / "uav_navigation"
    / "config"
    / "uav_delivery_waypoints.yaml"
)
BATTERY_CONFIG = (
    SOURCE_ROOT / "uav_control" / "config" / "battery_model.yaml"
)


def planner():
    waypoint_map = WaypointMap(WAYPOINT_CONFIG)
    parameters = BatteryParameters.from_yaml(BATTERY_CONFIG)
    return MissionEnergyPlanner(waypoint_map, parameters), parameters


def test_standalone_profile_contains_all_power_phases():
    energy_planner, _ = planner()
    profile = energy_planner.plan(
        ["teaching_building"], "", 0.0, True
    )
    assert profile.ascent_acceleration_seconds > 0.0
    assert profile.climb_seconds > 0.0
    assert profile.ascent_deceleration_seconds > 0.0
    assert profile.horizontal_acceleration_seconds > 0.0
    assert profile.cruise_seconds > 0.0
    assert profile.horizontal_deceleration_seconds > 0.0
    assert profile.hover_seconds >= 10.0
    assert profile.descent_acceleration_seconds > 0.0
    assert profile.descent_seconds > 0.0
    assert profile.descent_deceleration_seconds > 0.0
    assert profile.horizontal_distance_m > 0.0
    assert profile.initial_payload_mass_kg == pytest.approx(0.30)
    assert profile.segments


def test_long_leg_uses_trapezoidal_motion_profile():
    motion = MissionEnergyPlanner.motion_durations(10.0, 2.0, 1.0)
    assert motion.acceleration_seconds == 2.0
    assert motion.cruise_seconds == 3.0
    assert motion.deceleration_seconds == 2.0
    assert motion.peak_speed_mps == 2.0


def test_short_leg_uses_triangular_motion_profile():
    motion = MissionEnergyPlanner.motion_durations(1.0, 2.0, 1.0)
    assert motion.acceleration_seconds == 1.0
    assert motion.cruise_seconds == 0.0
    assert motion.deceleration_seconds == 1.0
    assert motion.peak_speed_mps == 1.0


def test_safe_return_is_reserved_even_when_return_home_is_false():
    energy_planner, _ = planner()
    with_return = energy_planner.plan(
        ["cafeteria"], "", 0.0, True
    )
    without_requested_return = energy_planner.plan(
        ["cafeteria"], "", 0.0, False
    )
    assert without_requested_return == with_return


def test_cooperative_local_sortie_requires_less_energy_than_standalone():
    energy_planner, parameters = planner()
    standalone = energy_planner.plan(
        ["teaching_building"], "", 0.0, True
    )
    local = energy_planner.plan(
        ["teaching_building"], "teaching_building", 0.42, True
    )
    standalone_energy = BatteryModel(parameters, 1.0).estimate(standalone)
    local_energy = BatteryModel(parameters, 1.0).estimate(local)
    assert local_energy.required_energy_wh < standalone_energy.required_energy_wh


def test_multi_target_profile_is_longer_than_single_target():
    energy_planner, _ = planner()
    single = energy_planner.plan(["laboratory"], "", 0.0, True)
    multiple = energy_planner.plan(
        ["laboratory", "library", "dormitory_2"], "", 0.0, True
    )
    assert multiple.horizontal_distance_m > single.horizontal_distance_m
    assert (
        multiple.horizontal_acceleration_seconds
        > single.horizontal_acceleration_seconds
    )
    assert (
        multiple.horizontal_deceleration_seconds
        > single.horizontal_deceleration_seconds
    )
    assert multiple.hover_seconds > single.hover_seconds


def test_payload_is_removed_after_each_delivery():
    energy_planner, _ = planner()
    profile = energy_planner.plan(
        ["laboratory", "library"], "", 0.0, True
    )
    payloads = [segment.payload_mass_kg for segment in profile.segments]
    assert profile.initial_payload_mass_kg == pytest.approx(0.60)
    assert any(value == pytest.approx(0.60) for value in payloads)
    assert any(value == pytest.approx(0.25) for value in payloads)
    assert payloads[-1] == pytest.approx(0.0)


def test_explicit_payload_override_changes_energy():
    energy_planner, parameters = planner()
    light = energy_planner.plan(
        ["teaching_building"],
        "",
        0.0,
        True,
        payload_masses_kg=[0.1],
    )
    heavy = energy_planner.plan(
        ["teaching_building"],
        "",
        0.0,
        True,
        payload_masses_kg=[0.8],
    )
    model = BatteryModel(parameters, 1.0)
    assert (
        model.estimate(heavy).estimated_mission_energy_wh
        > model.estimate(light).estimated_mission_energy_wh
    )


def test_total_payload_above_limit_is_rejected():
    energy_planner, _ = planner()
    with pytest.raises(ValueError):
        energy_planner.plan(
            ["teaching_building", "laboratory"],
            "",
            0.0,
            True,
            payload_masses_kg=[0.6, 0.6],
        )


def test_higher_delivery_floor_requires_more_energy():
    energy_planner, parameters = planner()
    low = energy_planner.plan(
        ["teaching_building"],
        "teaching_building",
        0.42,
        True,
        payload_masses_kg=[0.3],
        target_floors=[2],
    )
    high = energy_planner.plan(
        ["teaching_building"],
        "teaching_building",
        0.42,
        True,
        payload_masses_kg=[0.3],
        target_floors=[8],
    )
    model = BatteryModel(parameters, 1.0)
    assert (
        model.estimate(high).estimated_mission_energy_wh
        > model.estimate(low).estimated_mission_energy_wh
    )
