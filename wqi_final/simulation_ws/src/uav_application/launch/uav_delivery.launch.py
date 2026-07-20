#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_battery_percentage = LaunchConfiguration(
        "initial_battery_percentage"
    )
    control_launch = os.path.join(
        get_package_share_directory("uav_control"),
        "launch",
        "uav_control.launch.py",
    )
    navigation_launch = os.path.join(
        get_package_share_directory("uav_navigation"),
        "launch",
        "uav_navigation.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "initial_battery_percentage", default_value="0.80"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "initial_battery_percentage": initial_battery_percentage,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
        Node(
            package="uav_application",
            executable="delivery_mission_manager",
            namespace="uav",
            name="delivery_mission_manager",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
