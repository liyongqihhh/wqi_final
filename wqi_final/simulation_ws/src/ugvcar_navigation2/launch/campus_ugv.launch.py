"""Start standalone campus UGV physics, localization, navigation and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    description_share = get_package_share_directory("ugvcar_description")
    navigation_share = get_package_share_directory("ugvcar_navigation2")
    probe = Node(
        package="ugvcar_navigation2", executable="wait_for_ugv_ready.py", output="screen",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    preflight = Node(
        package="ugvcar_navigation2", executable="wait_for_ugv_ready.py", output="screen",
        name="ugv_launch_preflight",
        parameters=[{"preflight_only": True, "use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    physics = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            description_share, "launch", "campus_delivery_sim.launch.py"
        )),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "yaw": LaunchConfiguration("yaw"),
            "visualize_sensor_rays": LaunchConfiguration("visualize_sensor_rays"),
        }.items(),
    )])
    navigation = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            navigation_share, "launch", "campus_navigation.launch.py"
        )),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "rviz_software_rendering": LaunchConfiguration("rviz_software_rendering"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "localization_mode": "ground_truth",
            "initial_x": LaunchConfiguration("x"),
            "initial_y": LaunchConfiguration("y"),
            "initial_yaw": LaunchConfiguration("yaw"),
            "dynamic_obstacles": "false",
        }.items(),
    )])

    def after_ready(event, context):
        if event.returncode != 0:
            return [Shutdown(reason="UGV telemetry unavailable; navigation not started.")]
        return [navigation]

    def after_preflight(event, context):
        if event.returncode != 0:
            return [Shutdown(reason="Old simulation/navigation still running; startup refused.")]
        return [physics, probe]

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_software_rendering", default_value="true"),
        DeclareLaunchArgument("visualize_sensor_rays", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="-43.0"),
        DeclareLaunchArgument("yaw", default_value="1.5708"),
        RegisterEventHandler(OnProcessExit(target_action=probe, on_exit=after_ready)),
        RegisterEventHandler(OnProcessExit(target_action=preflight, on_exit=after_preflight)),
        preflight,
    ])
