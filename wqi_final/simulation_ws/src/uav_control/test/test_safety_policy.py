from uav_control.safety_policy import safety_issue


def test_platform_mode_allows_only_diagonal_sensor_returns():
    assert safety_issue(
        True,
        "BLOCKED:front_down,rear_down,left_down,right_down",
        True,
    ) == ""


def test_normal_flight_keeps_diagonal_sensor_protection():
    status = "BLOCKED:front_down"
    assert safety_issue(True, status, False) == status


def test_platform_mode_never_ignores_lidar_or_stale_data():
    assert safety_issue(True, "BLOCKED:top_3d_lidar,front_down", True)
    assert safety_issue(True, "SENSOR_STALE:top_3d_lidar", True)


def test_clear_state_has_no_issue():
    assert safety_issue(False, "CLEAR", False) == ""
