#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("uav_description")
    xacro_path = os.path.join(package_share, "urdf", "uav.urdf.xacro")
    params_path = os.path.join(package_share, "config", "uav_model.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    spawn_x = LaunchConfiguration("x")
    spawn_y = LaunchConfiguration("y")
    spawn_z = LaunchConfiguration("z")
    spawn_yaw = LaunchConfiguration("yaw")
    enable_docking = LaunchConfiguration("enable_docking")
    visualize_sensor_rays = LaunchConfiguration("visualize_sensor_rays")
    flight_backend = LaunchConfiguration("flight_backend")
    px4_autopilot_dir = LaunchConfiguration("px4_autopilot_dir")
    px4_condition = IfCondition(PythonExpression([
        "'", flight_backend, "' == 'px4'"
    ]))
    px4_input = [
        px4_autopilot_dir,
        "/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf",
    ]
    px4_output = "/tmp/wqi_final_px4_uav.sdf"
    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_path,
            " params_path:=",
            params_path,
            " enable_docking:=",
            enable_docking,
            " visualize_sensor_rays:=",
            visualize_sensor_rays,
        ]),
        value_type=str,
    )
    prepare_px4_model = ExecuteProcess(
        cmd=[
            "python3",
            os.path.join(package_share, "scripts", "prepare_px4_model.py"),
            "--input", px4_input,
            "--output", px4_output,
            "--docking", enable_docking,
            "--visualize", visualize_sensor_rays,
            "--relay-docking", LaunchConfiguration("relay_docking"),
            "--imu-plugin", [px4_autopilot_dir,
                             "/build/px4_sitl_default/build_gazebo-classic/libgazebo_imu_plugin.so"],
        ],
        name="prepare_px4_uav_model",
        output="screen",
        condition=px4_condition,
    )

    def spawn_prepared_px4(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="PX4 UAV model preparation failed"))]
        return [Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_campus_px4_uav",
            output="screen",
            arguments=[
                "-entity", "campus_uav", "-file", px4_output,
                "-robot_namespace", "/uav",
                "-x", spawn_x, "-y", spawn_y,
                "-z", PythonExpression([spawn_z, " + 0.12"]),
                "-Y", spawn_yaw,
            ],
        )]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="-43.5"),
        DeclareLaunchArgument("z", default_value="0.03"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("enable_docking", default_value="false"),
        DeclareLaunchArgument("relay_docking", default_value="false"),
        DeclareLaunchArgument(
            "flight_backend", default_value="px4", choices=["px4"]
        ),
        DeclareLaunchArgument(
            "px4_autopilot_dir",
            default_value=EnvironmentVariable(
                "PX4_AUTOPILOT_DIR",
                default_value="/home/wqi/design_final_artifacts/px4/PX4-Autopilot",
            ),
        ),
        DeclareLaunchArgument(
            "visualize_sensor_rays",
            default_value="false",
            description="Show Gazebo sensor rays without changing sensor topics.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace="uav",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
                "frame_prefix": "uav/",
            }],
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=prepare_px4_model,
                on_exit=spawn_prepared_px4,
            ),
            condition=px4_condition,
        ),
        prepare_px4_model,
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="uav_map_to_odom",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "map", "--child-frame-id", "uav/odom",
            ],
        ),
    ])
