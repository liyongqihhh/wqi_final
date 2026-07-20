#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("uav_control"),
        "config",
        "flight_control.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_battery_percentage = LaunchConfiguration(
        "initial_battery_percentage"
    )
    battery_config = os.path.join(
        get_package_share_directory("uav_control"),
        "config",
        "battery_model.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "initial_battery_percentage",
            default_value="0.80",
            description="Initial UAV battery state of charge in the range 0 to 1.",
        ),
        Node(
            package="uav_control",
            executable="flight_controller",
            namespace="uav",
            name="flight_controller",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="uav_control",
            executable="flight_state_monitor",
            namespace="uav",
            name="flight_state_monitor",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="uav_control",
            executable="safety_monitor",
            namespace="uav",
            name="safety_monitor",
            output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="uav_control",
            executable="battery_manager",
            namespace="uav",
            name="battery_manager",
            output="screen",
            parameters=[
                battery_config,
                {
                    "use_sim_time": use_sim_time,
                    "initial_percentage": ParameterValue(
                        initial_battery_percentage, value_type=float
                    ),
                },
            ],
        ),
    ])
