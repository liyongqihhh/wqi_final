#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="uav_navigation",
            executable="waypoint_visualizer",
            namespace="uav",
            name="waypoint_visualizer",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
