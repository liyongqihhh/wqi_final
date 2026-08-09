#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("campus_dynamic_obstacles"),
        "config",
        "obstacle_routes.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("density", default_value="none"),
        DeclareLaunchArgument("random_seed", default_value="42"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        Node(
            package="campus_dynamic_obstacles",
            executable="dynamic_obstacle_spawner",
            name="dynamic_obstacle_spawner",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "density": LaunchConfiguration("density"),
                "random_seed": LaunchConfiguration("random_seed"),
                "config_file": LaunchConfiguration("config_file"),
            }],
        ),
    ])
