from pathlib import Path

import yaml


RVIZ_CONFIG = Path(__file__).parents[1] / "rviz" / "uav.rviz"


def test_robot_model_prefix_does_not_duplicate_tf_separator():
    config = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    robot_model = next(
        display
        for display in displays
        if display.get("Class") == "rviz_default_plugins/RobotModel"
    )

    assert robot_model["TF Prefix"] == "uav"
    assert not robot_model["TF Prefix"].endswith("/")


def test_battery_marker_is_visible():
    config = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    battery = next(
        display
        for display in displays
        if display.get("Name") == "UAV Battery Status"
    )

    assert battery["Enabled"] is True
    assert battery["Topic"]["Value"] == "/uav/battery_marker"
