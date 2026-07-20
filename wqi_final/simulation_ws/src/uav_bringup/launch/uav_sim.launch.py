#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    uav_description_share = get_package_share_directory("uav_description")
    uav_application_share = get_package_share_directory("uav_application")
    ugvcar_navigation_share = get_package_share_directory("ugvcar_navigation2")
    campus_world = os.path.join(
        get_package_share_directory("ugvcar_description"),
        "world",
        "campus_delivery.world",
    )
    rviz_config = os.path.join(uav_description_share, "rviz", "uav.rviz")
    campus_map = os.path.join(
        ugvcar_navigation_share,
        "maps",
        "campus_delivery_map.yaml",
    )

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    rviz_delay = LaunchConfiguration("rviz_delay")
    publish_map = LaunchConfiguration("publish_map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    visualize_sensor_rays = LaunchConfiguration("visualize_sensor_rays")
    initial_battery_percentage = LaunchConfiguration(
        "initial_battery_percentage"
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=campus_world),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start Gazebo GUI. Keep false when RViz is enabled in VirtualBox.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz with map, UAV, 3D lidar, safety sphere, and camera.",
        ),
        DeclareLaunchArgument(
            "rviz_delay",
            default_value="15.0",
            description="Delay RViz until Gazebo has started publishing the UAV TF.",
        ),
        DeclareLaunchArgument(
            "publish_map",
            default_value="true",
            description="Publish the campus occupancy map for RViz.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "initial_battery_percentage",
            default_value="0.80",
            description="Initial UAV battery state of charge in the range 0 to 1.",
        ),
        DeclareLaunchArgument(
            "visualize_sensor_rays",
            default_value="false",
            description="Show Gazebo lidar/range rays; sensing remains active when false.",
        ),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="-43.5"),
        DeclareLaunchArgument("spawn_z", default_value="0.03"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
            ),
            launch_arguments={"world": world, "verbose": "false"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
            ),
            condition=IfCondition(gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    uav_description_share, "launch", "uav_spawn.launch.py"
                )
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "x": LaunchConfiguration("spawn_x"),
                "y": LaunchConfiguration("spawn_y"),
                "z": LaunchConfiguration("spawn_z"),
                "yaw": LaunchConfiguration("spawn_yaw"),
                "visualize_sensor_rays": visualize_sensor_rays,
            }.items(),
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="uav_map_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "yaml_filename": campus_map,
            }],
            condition=IfCondition(publish_map),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="uav_map_lifecycle_manager",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["uav_map_server"],
            }],
            condition=IfCondition(publish_map),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    uav_application_share, "launch", "uav_delivery.launch.py"
                )
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "initial_battery_percentage": initial_battery_percentage,
            }.items(),
        ),
        TimerAction(
            period=rviz_delay,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="uav_rviz",
                    output="screen",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": use_sim_time}],
                    condition=IfCondition(rviz),
                ),
            ],
        ),
    ])
