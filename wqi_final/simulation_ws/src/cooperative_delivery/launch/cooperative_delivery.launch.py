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
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ugv_description_share = get_package_share_directory("ugvcar_description")
    ugv_navigation_share = get_package_share_directory("ugvcar_navigation2")
    uav_description_share = get_package_share_directory("uav_description")
    uav_application_share = get_package_share_directory("uav_application")
    cooperative_share = get_package_share_directory("cooperative_delivery")
    dynamic_obstacles_share = get_package_share_directory(
        "campus_dynamic_obstacles"
    )

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
    dynamic_obstacles_launch = os.path.join(
        dynamic_obstacles_share, "launch", "dynamic_obstacles.launch.py"
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
    ugv_energy_config = os.path.join(
        cooperative_share, "config", "ugv_energy_model.yaml"
    )
    default_mission_config = os.path.join(
        cooperative_share, "config", "cooperative_waypoints.yaml"
    )
    default_obstacle_config = os.path.join(
        dynamic_obstacles_share, "config", "obstacle_routes.yaml"
    )

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    navigation_delay = LaunchConfiguration("navigation_delay")
    uav_spawn_delay = LaunchConfiguration("uav_spawn_delay")
    uav_application_delay = LaunchConfiguration("uav_application_delay")
    manager_delay = LaunchConfiguration("manager_delay")
    dynamic_obstacle_delay = LaunchConfiguration(
        "dynamic_obstacle_delay"
    )
    rviz_delay = LaunchConfiguration("rviz_delay")
    visualize_sensor_rays = LaunchConfiguration("visualize_sensor_rays")
    initial_battery_percentage = LaunchConfiguration(
        "initial_battery_percentage"
    )
    initial_ugv_drive_battery_percentage = LaunchConfiguration(
        "initial_ugv_drive_battery_percentage"
    )
    initial_ugv_charging_battery_percentage = LaunchConfiguration(
        "initial_ugv_charging_battery_percentage"
    )
    enable_energy_constraints = LaunchConfiguration(
        "enable_energy_constraints"
    )
    enable_dynamic_obstacles = LaunchConfiguration(
        "enable_dynamic_obstacles"
    )
    combined_robot_radius = "0.60"
    obstacle_density = LaunchConfiguration("obstacle_density")
    random_seed = LaunchConfiguration("random_seed")
    obstacle_config_file = LaunchConfiguration("obstacle_config_file")
    mission_config = LaunchConfiguration("mission_config")

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
        DeclareLaunchArgument(
            "initial_ugv_drive_battery_percentage",
            default_value="0.80",
            description="Initial UGV traction battery SOC in the range 0 to 1.",
        ),
        DeclareLaunchArgument(
            "initial_ugv_charging_battery_percentage",
            default_value="0.80",
            description="Initial UGV UAV-charging battery SOC in the range 0 to 1.",
        ),
        DeclareLaunchArgument(
            "enable_energy_constraints",
            default_value="false",
            description="Enable UAV and dual-pack UGV energy constraints.",
        ),
        DeclareLaunchArgument(
            "enable_dynamic_obstacles",
            default_value="false",
            description="Start the campus dynamic obstacle generator.",
        ),
        DeclareLaunchArgument(
            "obstacle_density",
            default_value="none",
            description="Dynamic obstacle density: none, low, medium or high.",
        ),
        DeclareLaunchArgument("random_seed", default_value="42"),
        DeclareLaunchArgument(
            "obstacle_config_file",
            default_value=default_obstacle_config,
            description="Dynamic obstacle route YAML; defaults to the four-density campus set.",
        ),
        DeclareLaunchArgument(
            "mission_config",
            default_value=default_mission_config,
            description="Cooperative mission waypoint configuration.",
        ),
        DeclareLaunchArgument("navigation_delay", default_value="15.0"),
        DeclareLaunchArgument("uav_spawn_delay", default_value="28.0"),
        DeclareLaunchArgument(
            "dynamic_obstacle_delay", default_value="34.0"
        ),
        DeclareLaunchArgument("uav_application_delay", default_value="38.0"),
        DeclareLaunchArgument("manager_delay", default_value="44.0"),
        DeclareLaunchArgument("rviz_delay", default_value="48.0"),
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
                                "dynamic_obstacles": enable_dynamic_obstacles,
                                # The 0.60 m circle covers the 0.56 m docked UAV
                                # envelope while retaining enough road width for
                                # Nav2 to replan around a moving obstacle.
                                # Standalone UGV uses its 0.22 m physical radius.
                                "robot_radius": combined_robot_radius,
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
                Node(
                    package="cooperative_delivery",
                    executable="ugv_energy_manager",
                    namespace="ugv",
                    name="energy_manager",
                    output="screen",
                    condition=IfCondition(enable_energy_constraints),
                    parameters=[
                        ugv_energy_config,
                        {
                            "use_sim_time": use_sim_time,
                            "initial_drive_percentage": ParameterValue(
                                initial_ugv_drive_battery_percentage,
                                value_type=float,
                            ),
                            "initial_charging_percentage": ParameterValue(
                                initial_ugv_charging_battery_percentage,
                                value_type=float,
                            ),
                        },
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(uav_application_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "initial_battery_percentage": initial_battery_percentage,
                        "enable_energy_constraints": enable_energy_constraints,
                        "external_charger_control": "true",
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
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "energy_constraints_enabled": ParameterValue(
                            enable_energy_constraints, value_type=bool
                        ),
                        "mission_config": mission_config,
                        "ugv_energy_config": ugv_energy_config,
                    }],
                )
            ],
        ),
        TimerAction(
            period=dynamic_obstacle_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(dynamic_obstacles_launch),
                    condition=IfCondition(enable_dynamic_obstacles),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "density": obstacle_density,
                        "random_seed": random_seed,
                        "config_file": obstacle_config_file,
                    }.items(),
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
