#!/usr/bin/env python3
import os
import socket
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def check_gazebo_master(context):
    uri = context.environment.get(
        "GAZEBO_MASTER_URI", "http://localhost:11345"
    )
    address = urlparse(uri)
    try:
        connection = socket.create_connection(
            (address.hostname or "localhost", address.port or 11345),
            timeout=0.5,
        )
    except OSError:
        return []
    connection.close()
    raise RuntimeError(
        f"Gazebo master {uri} is already running. Stop the previous "
        "simulation before starting standalone UAV; in the UI use Stop all."
    )


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    uav_description_share = get_package_share_directory("uav_description")
    uav_application_share = get_package_share_directory("uav_application")
    px4_bringup_share = get_package_share_directory("uav_px4_bringup")
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
    enable_energy_constraints = LaunchConfiguration(
        "enable_energy_constraints"
    )
    flight_backend = LaunchConfiguration("flight_backend")
    px4_autopilot_dir = LaunchConfiguration("px4_autopilot_dir")
    px4_selected = IfCondition(PythonExpression([
        "'", flight_backend, "' == 'px4'"
    ]))

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=campus_world),
        DeclareLaunchArgument("verbose", default_value="true"),
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
            "initial_battery_percentage",
            default_value="0.80",
            description="Initial UAV battery state of charge in the range 0 to 1.",
        ),
        DeclareLaunchArgument(
            "enable_energy_constraints",
            default_value="false",
            description="Enable UAV battery consumption and task admission.",
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
        OpaqueFunction(function=check_gazebo_master),
        SetEnvironmentVariable(
            "GAZEBO_PLUGIN_PATH",
            [px4_autopilot_dir, "/build/px4_sitl_default/build_gazebo-classic:",
             EnvironmentVariable("GAZEBO_PLUGIN_PATH", default_value="")],
            condition=px4_selected,
        ),
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            [px4_autopilot_dir,
             "/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:",
             EnvironmentVariable("GAZEBO_MODEL_PATH", default_value="")],
            condition=IfCondition(PythonExpression([
                "'", flight_backend, "' == 'px4'"
            ])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
            ),
            launch_arguments={
                "world": world,
                "verbose": LaunchConfiguration("verbose"),
                "server_required": "true",
            }.items(),
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
                "flight_backend": flight_backend,
                "px4_autopilot_dir": px4_autopilot_dir,
            }.items(),
        ),
        TimerAction(
            period=5.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    px4_bringup_share, "launch", "px4_sitl.launch.py"
                )),
                condition=IfCondition(PythonExpression([
                    "'", flight_backend, "' == 'px4'"
                ])),
                launch_arguments={
                    "px4_autopilot_dir": px4_autopilot_dir,
                    "enable_energy_constraints": enable_energy_constraints,
                }.items(),
            )],
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
                "enable_energy_constraints": enable_energy_constraints,
                "external_charger_control": "false",
                "flight_backend": flight_backend,
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
