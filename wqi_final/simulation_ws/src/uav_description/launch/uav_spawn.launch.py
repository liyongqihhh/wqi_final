#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
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

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="-43.5"),
        DeclareLaunchArgument("z", default_value="0.03"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("enable_docking", default_value="false"),
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
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_campus_uav",
            output="screen",
            arguments=[
                "-entity", "campus_uav",
                "-topic", "/uav/robot_description",
                "-robot_namespace", "/uav",
                "-x", spawn_x,
                "-y", spawn_y,
                "-z", spawn_z,
                "-Y", spawn_yaw,
            ],
        ),
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
