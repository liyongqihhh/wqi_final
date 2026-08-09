#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, filename, arguments):
    path = os.path.join(
        get_package_share_directory(package), "launch", filename
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
    )


def _launch_setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in ("ugv_only", "uav_only", "cooperative"):
        raise RuntimeError(f"Unsupported experiment mode: {mode}")
    density = LaunchConfiguration("obstacle_density").perform(context)
    if density not in ("none", "low", "medium", "high"):
        raise RuntimeError(f"Unsupported obstacle density: {density}")

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    initial_soc = LaunchConfiguration("initial_battery_percentage")
    initial_drive_soc = LaunchConfiguration(
        "initial_ugv_drive_battery_percentage"
    )
    initial_charging_soc = LaunchConfiguration(
        "initial_ugv_charging_battery_percentage"
    )
    rays = LaunchConfiguration("visualize_sensor_rays")
    actions = []
    if mode == "ugv_only":
        actions.append(_include(
            "ugvcar_description",
            "campus_delivery_sim.launch.py",
            {
                "gui": gui,
                "use_sim_time": use_sim_time,
                "x": "0.0",
                "y": "-43.5",
                "z": "0.005",
                "yaw": "1.5708",
                "visualize_sensor_rays": rays,
            },
        ))
        actions.append(TimerAction(
            period=15.0,
            actions=[_include(
                "ugvcar_navigation2",
                "campus_navigation.launch.py",
                {
                    "rviz": rviz,
                    "use_sim_time": use_sim_time,
                    "localization_mode": "ground_truth",
                    "initial_x": "0.0",
                    "initial_y": "-43.5",
                    "initial_yaw": "1.5708",
                    # Use the same RPP controller for every density; only the
                    # sensor-driven prediction and replanning layer changes.
                    "dynamic_obstacles": "true",
                },
            )],
        ))
        obstacle_delay = 25.0
        runner_delay = 35.0
    elif mode == "uav_only":
        actions.append(_include(
            "uav_bringup",
            "uav_sim.launch.py",
            {
                "gui": gui,
                "rviz": rviz,
                "use_sim_time": use_sim_time,
                "initial_battery_percentage": initial_soc,
                "enable_energy_constraints": "true",
                "visualize_sensor_rays": rays,
            },
        ))
        obstacle_delay = 20.0
        runner_delay = 40.0
    else:
        actions.append(_include(
            "cooperative_delivery",
            "cooperative_delivery.launch.py",
            {
                "gui": gui,
                "rviz": rviz,
                "use_sim_time": use_sim_time,
                "initial_battery_percentage": initial_soc,
                "initial_ugv_drive_battery_percentage": initial_drive_soc,
                "initial_ugv_charging_battery_percentage": (
                    initial_charging_soc
                ),
                "enable_energy_constraints": "true",
                "enable_dynamic_obstacles": "true",
                "obstacle_density": LaunchConfiguration("obstacle_density"),
                "random_seed": LaunchConfiguration("random_seed"),
                "visualize_sensor_rays": rays,
            },
        ))
        runner_delay = 60.0

    # The cooperative launch owns its obstacle generator so that Nav2 and the
    # selected density are enabled atomically. Standalone modes still need the
    # generator included here.
    if mode != "cooperative":
        actions.append(TimerAction(
            period=obstacle_delay,
            actions=[_include(
                "campus_dynamic_obstacles",
                "dynamic_obstacles.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "density": LaunchConfiguration("obstacle_density"),
                    "random_seed": LaunchConfiguration("random_seed"),
                },
            )],
        ))

    runner = Node(
        package="delivery_evaluation",
        executable="experiment_runner",
        name="experiment_runner",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "mode": LaunchConfiguration("mode"),
            "scenario": LaunchConfiguration("scenario"),
            "repetitions": LaunchConfiguration("repetitions"),
            "obstacle_density": LaunchConfiguration("obstacle_density"),
            "random_seed": LaunchConfiguration("random_seed"),
            "results_dir": LaunchConfiguration("results_dir"),
            "continue_on_failure": LaunchConfiguration("continue_on_failure"),
        }],
    )
    actions.append(TimerAction(period=runner_delay, actions=[runner]))
    actions.append(RegisterEventHandler(OnProcessExit(
        target_action=runner,
        on_exit=[EmitEvent(event=Shutdown(reason="Evaluation batch completed"))],
    )))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="cooperative"),
        DeclareLaunchArgument("scenario", default_value="teaching_building"),
        DeclareLaunchArgument("repetitions", default_value="1"),
        DeclareLaunchArgument("obstacle_density", default_value="none"),
        DeclareLaunchArgument("random_seed", default_value="42"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("initial_battery_percentage", default_value="0.80"),
        DeclareLaunchArgument(
            "initial_ugv_drive_battery_percentage", default_value="0.80"
        ),
        DeclareLaunchArgument(
            "initial_ugv_charging_battery_percentage", default_value="0.80"
        ),
        DeclareLaunchArgument("visualize_sensor_rays", default_value="false"),
        DeclareLaunchArgument("continue_on_failure", default_value="false"),
        DeclareLaunchArgument(
            "results_dir",
            default_value=(
                "/home/wqi/design_final/wqi_final/simulation_ws/experiment_results"
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
