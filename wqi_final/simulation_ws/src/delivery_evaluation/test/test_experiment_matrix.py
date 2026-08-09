from pathlib import Path

from delivery_evaluation.experiment_matrix import (
    csv_values,
    launch_command,
)


def test_csv_values_ignores_whitespace_and_empty_items():
    assert csv_values("ugv_only, cooperative, ") == [
        "ugv_only", "cooperative"
    ]


def test_launch_command_is_headless_and_reproducible():
    command = launch_command(
        mode="cooperative",
        scenario="teaching_building",
        density="medium",
        repetitions=3,
        seed=43,
        output_directory=Path("/tmp/results"),
        initial_battery_percentage=0.8,
        continue_on_failure=True,
        initial_ugv_drive_battery_percentage=0.7,
        initial_ugv_charging_battery_percentage=0.6,
    )

    assert "mode:=cooperative" in command
    assert "obstacle_density:=medium" in command
    assert "repetitions:=3" in command
    assert "random_seed:=43" in command
    assert "initial_battery_percentage:=0.800" in command
    assert "initial_ugv_drive_battery_percentage:=0.700" in command
    assert "initial_ugv_charging_battery_percentage:=0.600" in command
    assert "gui:=false" in command
    assert "rviz:=false" in command
