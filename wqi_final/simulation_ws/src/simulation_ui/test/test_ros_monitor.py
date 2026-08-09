from simulation_ui.ros_monitor import (
    format_battery,
    format_position,
    format_safety,
)


def test_monitor_formatters_handle_missing_data():
    assert format_position(None) == "--"
    assert format_battery(None, None) == "--"
    assert format_safety(None, None) == "--"


def test_monitor_formatters_render_runtime_values():
    assert format_position((1.24, -2.05, 3.06)) == "x 1.2  y -2.0  z 3.1 m"
    assert format_battery(0.756, 189.7) == "75.6%  |  190 W"
    assert format_safety(False, 2.345) == "安全  |  最近 2.35 m"
