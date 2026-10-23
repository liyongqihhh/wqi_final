"""One Gazebo, two independent ground vehicles, one PX4 UAV."""

from copy import deepcopy
import os
from pathlib import Path
import socket
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import RegisterEventHandler, Shutdown, TimerAction
from launch.actions import SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def verify_px4_ports_free():
    for kind, port in ((socket.SOCK_STREAM, 4560), (socket.SOCK_DGRAM, 8888)):
        with socket.socket(socket.AF_INET, kind) as probe:
            if kind == socket.SOCK_STREAM:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                probe.bind(("0.0.0.0", port))
            except OSError as error:
                raise RuntimeError(
                    f"PX4 port {port} is occupied. Stop the previous UAV simulation "
                    "before launching the two-car relay; do not share a flight controller."
                ) from error


def prepare(context):
    verify_px4_ports_free()
    from cooperative_delivery.relay_plan import RelayConfig
    from cooperative_delivery.relay_vehicle import (
        second_navigation_parameters, second_vehicle_model, with_peer_footprint,
        second_vehicle_remappings,
        relay_runtime_parameters,
    )
    from uav_navigation.waypoint_navigator import WaypointMap

    def share(package):
        return Path(get_package_share_directory(package))

    def value(name):
        return LaunchConfiguration(name).perform(context)

    def truth(name):
        return value(name).lower() in ("true", "1", "yes")

    coop, ugv, nav = share("cooperative_delivery"), share("ugvcar_description"), share("ugvcar_navigation2")
    waypoint_map = WaypointMap(share("uav_navigation") / "config/uav_delivery_waypoints.yaml")
    config = RelayConfig.load(value("relay_config"), waypoint_map)
    show_rviz = truth("rviz")
    build_root = Path(os.environ.get("WQI_BUILD_ROOT", "/tmp")) / "generated_models"
    build_root.mkdir(parents=True, exist_ok=True)
    generated = Path(tempfile.mkdtemp(prefix="two_car_relay_", dir=build_root))
    robot = xacro.process_file(str(ugv / "urdf/ugvcar/ugvcar.urdf.xacro"),
        mappings={"visualize_sensor_rays": value("visualize_sensor_rays")}).toxml()
    robot = second_vehicle_model(robot)
    original_params = relay_runtime_parameters(
        yaml.safe_load((nav / "config/nav2_params.yaml").read_text()),
        float(value("controller_frequency")))
    a_navigation_file = generated / "ugv_a_nav2.yaml"
    a_navigation_file.write_text(yaml.safe_dump(with_peer_footprint(
        original_params, "/ugv_b/ground_truth/odom")), encoding="utf-8")
    navigation_file = generated / "ugv_b_nav2.yaml"
    navigation_file.write_text(yaml.safe_dump(second_navigation_parameters(
        original_params, nav, truth("enable_dynamic_obstacles"), float(value("controller_frequency")))),
        encoding="utf-8")
    rviz = yaml.safe_load((coop / "rviz/cooperative.rviz").read_text())
    rviz["Visualization Manager"]["Global Options"]["Frame Rate"] = 10
    displays = rviz["Visualization Manager"]["Displays"]
    for display in list(displays):
        if display.get("Name") in ("Campus UGV", "UGV Nav2 Plan", "UGV Local Costmap", "UGV Laser"):
            clone = deepcopy(display)
            clone["Name"] = "B " + display["Name"]
            for field in ("Description Topic", "Topic", "Update Topic"):
                if field in clone:
                    clone[field]["Value"] = "/ugv_b" + clone[field]["Value"]
            if "Links" in clone:
                clone["Links"] = {"All Links Enabled": True}
            displays.append(clone)
    displays.append({"Class": "rviz_default_plugins/MarkerArray", "Name": "Relay Platforms",
        "Enabled": True, "Value": True, "Topic": {"Value": "/relay_delivery/markers",
        "Depth": 1, "Durability Policy": "Transient Local", "Reliability Policy": "Reliable",
        "History Policy": "Keep Last"}})
    rviz_file = generated / "relay.rviz"
    rviz_file.write_text(yaml.safe_dump(rviz), encoding="utf-8")
    b = config.platform_b.stop
    spawn = Node(package="gazebo_ros", executable="spawn_entity.py", name="spawn_ugv_b",
        arguments=["-topic", "/ugv_b/robot_description", "-entity", "ugv_b", "-timeout", "120",
                   "-x", str(b.x), "-y", str(b.y), "-z", "0.005", "-Y", str(b.yaw)], output="screen")
    def spawned(event, _context):
        return [] if event.returncode == 0 else [Shutdown(reason="B car failed to spawn")]
    waiter = Node(package="cooperative_delivery", executable="wait_for_carrier_control",
        name="wait_for_a_before_b", output="screen", parameters=[{"use_sim_time": True}])
    def ready_for_b(event, _context):
        return [spawn] if event.returncode == 0 else [Shutdown(reason="A-car control initialization failed")]

    dynamic_remaps = second_vehicle_remappings()
    b_nodes = []
    for package, executable in (("nav2_controller", "controller_server"),
            ("nav2_smoother", "smoother_server"), ("nav2_planner", "planner_server"),
            ("nav2_behaviors", "behavior_server"), ("nav2_bt_navigator", "bt_navigator"),
            ("nav2_waypoint_follower", "waypoint_follower"),
            ("nav2_velocity_smoother", "velocity_smoother"),
            ("nav2_collision_monitor", "collision_monitor")):
        remaps = list(dynamic_remaps)
        if executable in ("controller_server", "behavior_server"):
            remaps += [("cmd_vel", "cmd_vel_nav")]
        if executable == "velocity_smoother":
            remaps += [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]
        b_nodes.append(Node(package=package, executable=executable, name=executable,
            namespace="ugv_b", parameters=[str(navigation_file)], remappings=remaps, output="screen"))
    lifecycle_nodes = ["controller_server", "smoother_server", "planner_server", "behavior_server",
                       "bt_navigator", "waypoint_follower", "velocity_smoother", "collision_monitor"]
    b_nodes += [Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager_navigation", namespace="ugv_b", output="screen",
        parameters=[{"use_sim_time": True, "autostart": True, "node_names": lifecycle_nodes}]),
        Node(package="cooperative_delivery", executable="relay_localizer", namespace="ugv_b",
             output="screen", parameters=[{"use_sim_time": True}]),
        Node(package="ugvcar_navigation2", executable="dynamic_obstacle_predictor.py",
             namespace="ugv_b", name="dynamic_obstacle_predictor", output="screen",
             parameters=[{"use_sim_time": True, "robot_radius": .60}],
             remappings=second_vehicle_remappings(predictor=True),
             condition=IfCondition(LaunchConfiguration("enable_dynamic_obstacles")))]
    energy_params = yaml.safe_load((coop / "config/ugv_energy_model.yaml").read_text())[
        "/ugv/energy_manager"]["ros__parameters"]
    energy_nodes = []
    for role in ("a", "b"):
        ns = f"/ugv_{role}"
        energy_nodes.append(Node(package="cooperative_delivery", executable="ugv_energy_manager",
            namespace=ns, name="energy_manager", output="screen",
            condition=IfCondition(LaunchConfiguration("enable_energy_constraints")),
            parameters=[energy_params, {"use_sim_time": True,
                "initial_drive_percentage": float(value("initial_ugv_drive_battery_percentage")),
                "initial_charging_percentage": float(value("initial_ugv_charging_battery_percentage"))}],
            remappings=[("/odom", "/odom" if role == "a" else "/ugv_b/odom"),
                        ("/ugv/cargo_mass_kg", ns + "/cargo_mass_kg"),
                        ("/uav/docked", ns + "/uav_docked")]))
    common = {name: value(name) for name in ("gui", "enable_energy_constraints",
        "enable_dynamic_obstacles", "obstacle_density", "random_seed", "visualize_sensor_rays",
        "initial_battery_percentage", "initial_ugv_drive_battery_percentage",
        "initial_ugv_charging_battery_percentage", "px4_autopilot_dir")}
    common.update(rviz="false", start_mission_manager="false", start_ugv_energy_manager="false",
                  relay_docking="true", nav2_params_file=str(a_navigation_file),
                  static_controller_frequency=value("controller_frequency"),
                  bt_loop_duration=str(original_params["bt_navigator"]["ros__parameters"]["bt_loop_duration"]),
                  filter_info_package="ugvcar_navigation2",
                  filter_info_executable="managed_filter_info_server")
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(coop / "launch/cooperative_delivery.launch.py")), launch_arguments=common.items()),
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace="ugv_b",
             name="robot_state_publisher", output="screen", parameters=[{"use_sim_time": True,
             "robot_description": ParameterValue(robot, value_type=str)}]),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawned)),
        RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=ready_for_b)),
        TimerAction(period=6.0, actions=[waiter]),
        TimerAction(period=24.0, actions=b_nodes),
        TimerAction(period=40.0, actions=energy_nodes),
        TimerAction(period=55.0, actions=[Node(package="cooperative_delivery",
            executable="relay_mission_manager", namespace="relay_delivery", name="mission_manager",
            output="screen", parameters=[{"use_sim_time": True, "relay_config": value("relay_config"),
                "energy_constraints_enabled": truth("enable_energy_constraints"),
                "flight_backend": "px4", "route_policy": "distance"}])]),
        TimerAction(period=60.0, actions=[Node(package="rviz2", executable="rviz2", name="relay_rviz",
            arguments=["-d", str(rviz_file)], parameters=[{"use_sim_time": True}], output="screen",
            additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"},
            condition=IfCondition(str(show_rviz)))])]


def generate_launch_description():
    share = Path(get_package_share_directory("cooperative_delivery"))
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("relay_config", default_value=str(share / "config/relay_delivery.yaml")),
        DeclareLaunchArgument("enable_energy_constraints", default_value="true"),
        DeclareLaunchArgument("enable_dynamic_obstacles", default_value="false"),
        DeclareLaunchArgument("obstacle_density", default_value="none", choices=["none", "low", "medium", "high"]),
        DeclareLaunchArgument("random_seed", default_value="42"),
        DeclareLaunchArgument("visualize_sensor_rays", default_value="false"),
        DeclareLaunchArgument("software_rendering", default_value="true"),
        DeclareLaunchArgument("controller_frequency", default_value="10.0"),
        DeclareLaunchArgument("initial_battery_percentage", default_value="0.80"),
        DeclareLaunchArgument("initial_ugv_drive_battery_percentage", default_value="0.80"),
        DeclareLaunchArgument("initial_ugv_charging_battery_percentage", default_value="0.80"),
        DeclareLaunchArgument("px4_autopilot_dir", default_value=EnvironmentVariable("PX4_AUTOPILOT_DIR",
            default_value="/home/wqi/design_final_artifacts/px4/PX4-Autopilot")),
        SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1",
            condition=IfCondition(LaunchConfiguration("software_rendering"))),
        # Both Nav2 stacks retain heartbeat protection, allowing slow VM startup.
        SetParameter(name="bond_timeout", value=30.0),
        SetLaunchConfiguration("sigterm_timeout", "20.0"),
        SetLaunchConfiguration("sigkill_timeout", "10.0"),
        OpaqueFunction(function=prepare),
    ])
