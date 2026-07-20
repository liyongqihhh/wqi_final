#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ugv_description_share = get_package_share_directory("ugvcar_description")
    ugv_navigation_share = get_package_share_directory("ugvcar_navigation2")
    uav_description_share = get_package_share_directory("uav_description")
    uav_application_share = get_package_share_directory("uav_application")
    cooperative_share = get_package_share_directory("cooperative_delivery")

    ugv_sim_launch = os.path.join(
        ugv_description_share, "launch", "campus_delivery_sim.launch.py"
    )
    ugv_navigation_launch = os.path.join(
        ugv_navigation_share, "launch", "campus_navigation.launch.py"
    )
    uav_spawn_launch = os.path.join(
        uav_description_share, "launch", "uav_spawn.launch.py"
    )
    uav_application_launch = os.path.join(
        uav_application_share, "launch", "uav_delivery.launch.py"
    )
    rviz_config = os.path.join(cooperative_share, "rviz", "cooperative.rviz")
    campus_map = os.path.join(
        ugv_navigation_share, "maps", "campus_delivery_map.yaml"
    )
    keepout_mask = os.path.join(
        ugv_navigation_share, "maps", "campus_keepout_mask.yaml"
    )
    nav2_params = os.path.join(
        ugv_navigation_share, "config", "nav2_params.yaml"
    )

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    navigation_delay = LaunchConfiguration("navigation_delay")
    uav_spawn_delay = LaunchConfiguration("uav_spawn_delay")
    uav_application_delay = LaunchConfiguration("uav_application_delay")
    manager_delay = LaunchConfiguration("manager_delay")
    rviz_delay = LaunchConfiguration("rviz_delay")
    visualize_sensor_rays = LaunchConfiguration("visualize_sensor_rays")
    initial_battery_percentage = LaunchConfiguration(
        "initial_battery_percentage"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start Gazebo GUI; keep false when RViz is used in VirtualBox.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start the combined UGV-UAV RViz view.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "initial_battery_percentage",
            default_value="0.80",
            description="Initial UAV battery state of charge in the range 0 to 1.",
        ),
        DeclareLaunchArgument("navigation_delay", default_value="5.0"),
        DeclareLaunchArgument("uav_spawn_delay", default_value="12.0"),
        DeclareLaunchArgument("uav_application_delay", default_value="17.0"),
        DeclareLaunchArgument("manager_delay", default_value="21.0"),
        DeclareLaunchArgument("rviz_delay", default_value="8.0"),
        DeclareLaunchArgument(
            "visualize_sensor_rays",
            default_value="false",
            description="Show Gazebo UGV/UAV lidar rays; topics remain active when false.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ugv_sim_launch),
            launch_arguments={
                "gui": gui,
                "use_sim_time": use_sim_time,
                "x": "0.0",
                "y": "-43.5",
                "z": "0.005",
                "yaw": "1.5708",
                "visualize_sensor_rays": visualize_sensor_rays,
            }.items(),
        ),
        TimerAction(
            period=navigation_delay,
            actions=[
                GroupAction(
                    scoped=True,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(ugv_navigation_launch),
                            launch_arguments={
                                "rviz": "false",
                                "use_sim_time": use_sim_time,
                                "localization_mode": "ground_truth",
                                "map": campus_map,
                                "keepout_mask": keepout_mask,
                                "params_file": nav2_params,
                                "initial_x": "0.0",
                                "initial_y": "-43.5",
                                "initial_yaw": "1.5708",
                            }.items(),
                        )
                    ],
                )
            ],
        ),
        TimerAction(
            period=uav_spawn_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(uav_spawn_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "x": "0.0",
                        "y": "-43.5",
                        "z": "0.42",
                        "yaw": "1.5708",
                        "enable_docking": "true",
                        "visualize_sensor_rays": visualize_sensor_rays,
                    }.items(),
                )
            ],
        ),
        TimerAction(
            period=uav_application_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(uav_application_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "initial_battery_percentage": initial_battery_percentage,
                    }.items(),
                )
            ],
        ),
        TimerAction(
            period=manager_delay,
            actions=[
                Node(
                    package="cooperative_delivery",
                    executable="cooperative_mission_manager",
                    namespace="cooperative_delivery",
                    name="mission_manager",
                    output="screen",
                    parameters=[{"use_sim_time": use_sim_time}],
                )
            ],
        ),
        TimerAction(
            period=rviz_delay,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="cooperative_rviz",
                    output="screen",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": use_sim_time}],
                    condition=IfCondition(rviz),
                )
            ],
        ),
    ])
