from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro
import yaml
import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time
from cooperative_delivery.relay_localizer import RelayLocalizer

from cooperative_delivery.relay_vehicle import (
    second_navigation_parameters, second_vehicle_model,
    with_peer_footprint,
    second_vehicle_remappings,
    relay_runtime_parameters,
)

SRC = Path(__file__).parents[2]


def test_b_vehicle_prefixes_links_joints_sensors_and_plugins():
    share = Path(get_package_share_directory("ugvcar_description"))
    original = xacro.process_file(str(share / "urdf/ugvcar/ugvcar.urdf.xacro")).toxml()
    root = ET.fromstring(second_vehicle_model(original))
    assert all(link.get("name").startswith("ugv_b/") for link in root.findall("link"))
    assert all(joint.get("name").startswith("ugv_b/") for joint in root.findall("joint"))
    assert root.find("link[@name='ugv_b/base_link']/collision[@name='px4_carrier_deck_collision']") is not None
    for plugin in root.findall(".//plugin[@filename]"):
        assert plugin.findtext("ros/namespace") == "/ugv_b"
    assert root.find(".//plugin[@filename='libgazebo_ros2_control.so']") is None
    assert root.findtext(".//plugin[@filename='libgazebo_ros_p3d.so']/body_name") == "ugv_b/base_footprint"
    assert ET.fromstring(original).find("link[@name='base_link']") is not None


def test_b_native_drive_uses_encoder_odom_and_does_not_duplicate_joint_tf():
    share = Path(get_package_share_directory("ugvcar_description"))
    original = xacro.process_file(str(share / "urdf/ugvcar/ugvcar.urdf.xacro")).toxml()
    root = ET.fromstring(second_vehicle_model(original))
    drive = root.find(".//plugin[@filename='libgazebo_ros_diff_drive.so']")
    assert drive.findtext("robot_base_frame") == "ugv_b/base_footprint"
    assert drive.findtext("left_joint") == "ugv_b/left_wheel_joint"
    assert drive.findtext("odometry_source") == "0"
    assert drive.findtext("publish_wheel_tf") == "false"
    assert root.find("ros2_control") is None


def test_b_nav_topics_are_isolated_but_map_and_filter_are_shared():
    share = Path(get_package_share_directory("ugvcar_navigation2"))
    original = yaml.safe_load((share / "config/nav2_params.yaml").read_text())
    for dynamic in (False, True):
        data = second_navigation_parameters(original, share, dynamic)["ugv_b"]
        local = data["local_costmap"]["local_costmap"]["ros__parameters"]
        assert local["robot_base_frame"] == "ugv_b/base_link"
        assert local["global_frame"] == "ugv_b/odom"
        assert local["obstacle_layer"]["scan"]["topic"] == "/ugv_b/scan"
        assert local["keepout_filter"]["filter_info_topic"] == "/local_costmap_filter_info"
        assert data["global_costmap"]["global_costmap"]["ros__parameters"]["global_frame"] == "map"
        assert Path(data["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"]).exists()
        assert data["collision_monitor"]["ros__parameters"]["cmd_vel_out_topic"] == "/ugv_b/cmd_vel_safe"
        assert local["peer_vehicle"]["pose_topic"] == "/ground_truth/odom"
        assert local["plugins"].index("peer_vehicle") < local["plugins"].index("inflation_layer")


def test_peer_layer_does_not_modify_existing_single_car_parameters():
    original = {name: {name: {"ros__parameters": {"plugins": ["obstacle_layer", "inflation_layer"]}}}
        for name in ("local_costmap", "global_costmap")}
    updated = with_peer_footprint(original, "/ugv_b/ground_truth/odom")
    assert "peer_vehicle" not in original["local_costmap"]["local_costmap"]["ros__parameters"]
    assert updated["global_costmap"]["global_costmap"]["ros__parameters"]["peer_vehicle"]["radius"] == .60


def test_b_navigation_resolves_peer_to_a_but_predictor_resolves_ego_to_b():
    rclpy.init()
    try:
        for predictor in (False, True):
            args = ["--ros-args"]
            for source, destination in second_vehicle_remappings(predictor):
                args += ["-r", f"{source}:={destination}"]
            node = rclpy.create_node("remapping_test", namespace="ugv_b", cli_args=args,
                use_global_arguments=False)
            expected = "/ugv_b/ground_truth/odom" if predictor else "/ground_truth/odom"
            assert node.resolve_topic_name("/ground_truth/odom") == expected
            assert node.resolve_topic_name("/scan") == "/ugv_b/scan"
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_b_localizer_provides_controller_transform_tolerance():
    now = Time(seconds=10)
    node = SimpleNamespace(wheel=Odometry(), broadcaster=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: now))
    truth = Odometry()
    truth.pose.pose.position.x = 3.0
    RelayLocalizer._truth(node, truth)
    transform = node.broadcaster.sendTransform.call_args.args[0]
    assert transform.header.stamp.sec == 10
    assert transform.header.stamp.nanosec == 200_000_000
    assert transform.child_frame_id == "ugv_b/odom"
    assert transform.transform.translation.x == 3.0


def test_b_drive_watchdog_stops_stale_commands_and_clock_reset():
    now = [1_000_000_000]
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now[0]))
    node = SimpleNamespace(last_command=None, last_command_ns=None,
        command_pub=Mock(), get_clock=lambda: clock)
    command = Twist()
    command.linear.x, command.angular.z = 9.0, 9.0
    RelayLocalizer._command(node, command)
    sent = node.command_pub.publish.call_args.args[0]
    assert sent.linear.x == .4 and sent.angular.z == .65
    now[0] += 600_000_000
    RelayLocalizer._watchdog(node)
    assert node.command_pub.publish.call_args.args[0].linear.x == 0
    now[0] = 0
    RelayLocalizer._watchdog(node)
    assert node.command_pub.publish.call_args.args[0].angular.z == 0


def test_b_drive_rejects_nonfinite_commands():
    node = SimpleNamespace(command_pub=Mock())
    command = Twist()
    command.linear.x = float("nan")
    RelayLocalizer._command(node, command)
    node.command_pub.publish.assert_not_called()


def test_relay_runtime_profile_changes_rates_not_navigation_or_speed():
    share = Path(get_package_share_directory("ugvcar_navigation2"))
    original = yaml.safe_load((share / "config/nav2_params.yaml").read_text())
    import copy
    snapshot = copy.deepcopy(original)
    profiled = relay_runtime_parameters(original)
    assert profiled["controller_server"]["ros__parameters"]["controller_frequency"] == 10.0
    assert profiled["bt_navigator"]["ros__parameters"]["bt_loop_duration"] == 100
    assert original == snapshot
    profiled["controller_server"]["ros__parameters"]["controller_frequency"] = (
        original["controller_server"]["ros__parameters"]["controller_frequency"])
    profiled["bt_navigator"]["ros__parameters"]["bt_loop_duration"] = (
        original["bt_navigator"]["ros__parameters"]["bt_loop_duration"])
    assert profiled == original


@pytest.mark.parametrize("frequency", [float("nan"), float("inf"), 0, -1, 4.9, 20.1])
def test_relay_runtime_profile_rejects_unsafe_rates(frequency):
    with pytest.raises(ValueError):
        relay_runtime_parameters({}, frequency)


def test_b_runtime_profile_matches_a_static_and_keeps_dynamic_at_ten_hz():
    share = Path(get_package_share_directory("ugvcar_navigation2"))
    original = yaml.safe_load((share / "config/nav2_params.yaml").read_text())
    static = second_navigation_parameters(original, share, False, 15)["ugv_b"]
    assert static["controller_server"]["ros__parameters"]["controller_frequency"] == 15.0
    dynamic = second_navigation_parameters(original, share, True, 15)["ugv_b"]
    assert dynamic["controller_server"]["ros__parameters"]["controller_frequency"] == 10.0


def test_relay_shutdown_profile_is_scoped_to_relay_launch():
    launch = (SRC / "cooperative_delivery/launch/relay_delivery.launch.py").read_text()
    assert 'filter_info_executable="managed_filter_info_server"' in launch
    assert 'SetLaunchConfiguration("sigterm_timeout", "20.0")' in launch
    campus = (SRC / "ugvcar_navigation2/launch/campus_navigation.launch.py").read_text()
    assert '"filter_info_executable", default_value="costmap_filter_info_server"' in campus
