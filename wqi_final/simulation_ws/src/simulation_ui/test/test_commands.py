from pathlib import Path

import pytest
import yaml

from simulation_ui.config import (
    BUILDING_BY_ID,
    SIMULATION_MODES,
    UAV_BATTERY_RESERVE_PERCENT,
    CommandBuilder,
    DeliveryItem,
    ViewerMode,
    battery_admission_notice,
)


def test_all_five_modes_have_launch_commands():
    builder = CommandBuilder()
    for mode in SIMULATION_MODES:
        commands = builder.simulation_commands(
            mode.key, ViewerMode.RVIZ, 80, False
        )
        assert commands
        assert all("ros2 launch" in spec.command for spec in commands)


def test_viewer_selection_controls_both_frontends():
    builder = CommandBuilder()
    rviz_only = builder.simulation_commands(
        "campus_uav", ViewerMode.RVIZ, 80, False
    )[0].command
    both = builder.simulation_commands(
        "campus_uav", ViewerMode.BOTH, 80, False
    )[0].command
    assert "gui:=false rviz:=true" in rviz_only
    assert "gui:=true rviz:=true" in both


def test_energy_mode_uses_selected_battery_but_stage_four_uses_full_charge():
    builder = CommandBuilder()
    stage_four = builder.simulation_commands(
        "cooperative", ViewerMode.GAZEBO, 30, False
    )[0].command
    stage_five = builder.simulation_commands(
        "cooperative_energy", ViewerMode.GAZEBO, 30, False
    )[0].command
    assert "initial_battery_percentage:=1.00" in stage_four
    assert "initial_battery_percentage:=0.30" in stage_five


def test_low_battery_notice_explains_energy_admission_risk():
    notice = battery_admission_notice("cooperative_energy", 18)
    assert notice.severity == "critical"
    assert notice.requires_confirmation
    assert "18%" in notice.message
    assert "安全储备" in notice.message


def test_normal_battery_does_not_require_confirmation():
    notice = battery_admission_notice("cooperative_energy", 80)
    assert notice.severity == "normal"
    assert not notice.requires_confirmation


def test_ui_reserve_matches_battery_model_configuration():
    battery_file = (
        Path(__file__).parents[2]
        / "uav_control"
        / "config"
        / "battery_model.yaml"
    )
    battery_parameters = yaml.safe_load(
        battery_file.read_text(encoding="utf-8")
    )["/uav/battery_manager"]["ros__parameters"]
    assert UAV_BATTERY_RESERVE_PERCENT == pytest.approx(
        100.0 * battery_parameters["reserve_percentage"]
    )


def test_uav_goal_contains_floor_payload_and_return_setting():
    builder = CommandBuilder()
    task = builder.task_command(
        "campus_uav", "teaching_building", 6, 0.45, True
    )
    assert task is not None
    assert "target_floors: [6]" in task.command
    assert "payload_masses_kg: [0.450]" in task.command
    assert "return_home: true" in task.command


def test_cooperative_goal_uses_cooperative_action():
    task = CommandBuilder().task_command(
        "cooperative_energy", "dormitory_4", 13, 0.20, True
    )
    assert task is not None
    assert "/cooperative_delivery/execute_mission" in task.command
    assert "ExecuteCooperativeDelivery" in task.command


def test_multi_item_goal_keeps_targets_floors_and_payloads_aligned():
    task = CommandBuilder().delivery_task_command(
        "cooperative_energy",
        [
            DeliveryItem("library", 5, 0.15),
            DeliveryItem("teaching_building", 2, 0.45),
            DeliveryItem("dormitory_4", 11, 0.25),
        ],
        True,
    )
    assert task is not None
    assert "targets: [library, teaching_building, dormitory_4]" in task.command
    assert "target_floors: [5, 2, 11]" in task.command
    assert "payload_masses_kg: [0.150, 0.450, 0.250]" in task.command


def test_multi_target_ugv_launch_parameter_contains_all_destinations():
    task = CommandBuilder().delivery_task_command(
        "campus_ugv",
        [
            DeliveryItem("laboratory", 1, 0.0),
            DeliveryItem("library", 1, 0.0),
        ],
        True,
    )
    assert task is not None
    assert 'delivery_targets:="[laboratory, library]"' in task.command


def test_standalone_uav_rejects_total_payload_over_one_flight_limit():
    with pytest.raises(ValueError, match="total payload"):
        CommandBuilder().delivery_task_command(
            "campus_uav",
            [
                DeliveryItem("laboratory", 3, 0.60),
                DeliveryItem("library", 3, 0.50),
            ],
            True,
        )


def test_floor_altitude_matches_campus_floor_height():
    teaching = BUILDING_BY_ID["teaching_building"]
    assert teaching.altitude_for_floor(1) == 1.6
    assert teaching.altitude_for_floor(3) == 8.0
    assert teaching.altitude_for_floor(8) == 24.0


def test_ui_building_metadata_matches_uav_waypoint_configuration():
    waypoint_file = (
        Path(__file__).parents[2]
        / "uav_navigation"
        / "config"
        / "uav_delivery_waypoints.yaml"
    )
    waypoints = yaml.safe_load(
        waypoint_file.read_text(encoding="utf-8")
    )["waypoints"]
    for target_id, building in BUILDING_BY_ID.items():
        configured = waypoints[target_id]
        assert building.maximum_floor == configured["maximum_floor"]
        assert building.default_floor == configured["delivery_floor"]
        assert building.default_payload_kg == pytest.approx(
            configured["payload_mass_kg"]
        )
