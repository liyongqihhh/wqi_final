#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
    enable_energy_constraints = LaunchConfiguration(
        "enable_energy_constraints"
    )
    external_charger_control = LaunchConfiguration(
        "external_charger_control"
    )
    flight_backend = LaunchConfiguration("flight_backend")
    px4_selected = IfCondition(PythonExpression([
        "'", flight_backend, "' == 'px4'"
    ]))
    battery_config = os.path.join(
        get_package_share_directory("uav_control"),
        "config",
        "battery_model.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_sim_velocity_aiding", default_value="false"),
        DeclareLaunchArgument("validate_dynamic_paths",default_value="false"),
        DeclareLaunchArgument("dynamic_setpoint_max_step",default_value="0.8"),
        DeclareLaunchArgument("dynamic_cruise_enabled",default_value="false"),
        DeclareLaunchArgument(
            "flight_backend",
            default_value="px4",
            choices=["px4"],
            description="PX4 SITL Offboard is the only supported UAV backend.",
        ),
        DeclareLaunchArgument(
            "initial_battery_percentage",
            default_value="0.80",
            description="Initial UAV battery state of charge in the range 0 to 1.",
        ),
        DeclareLaunchArgument(
            "enable_energy_constraints",
            default_value="true",
            description="Start the paper-based UAV battery model.",
        ),
        DeclareLaunchArgument(
            "external_charger_control",
            default_value="false",
            description="Allow an external UGV charging pack to gate UAV charging.",
        ),
        DeclareLaunchArgument(
            "start_battery_manager", default_value="true",
            description="Disable only when an external finite-pack manager publishes /uav/battery_state.",
        ),
        Node(
            package="uav_control",
            executable="px4_offboard_controller",
            namespace="uav",
            name="flight_controller",
            output="screen",
            parameters=[
                config,
                os.path.join(
                    get_package_share_directory("uav_control"),
                    "config",
                    "px4_offboard.yaml",
                ),
                {"use_sim_time": use_sim_time},
                {"require_energy_bridge": ParameterValue(
                    enable_energy_constraints, value_type=bool)},
                {"enable_sim_velocity_aiding": ParameterValue(
                    LaunchConfiguration("enable_sim_velocity_aiding"), value_type=bool)},
                {"validate_dynamic_paths":ParameterValue(LaunchConfiguration("validate_dynamic_paths"),value_type=bool)},
                {"dynamic_maximum_setpoint_step":ParameterValue(LaunchConfiguration("dynamic_setpoint_max_step"),value_type=float)},
                {"dynamic_cruise_enabled":ParameterValue(LaunchConfiguration("dynamic_cruise_enabled"),value_type=bool)},
            ],
            condition=px4_selected,
        ),
        Node(
            package="uav_control", executable="px4_battery_bridge",
            namespace="uav", name="px4_battery_bridge", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(PythonExpression([
                "'", flight_backend, "' == 'px4' and '",
                enable_energy_constraints, "'.lower() == 'true'",
            ])),
        ),
        Node(
            package="uav_control",
            executable="odom_tf_publisher",
            namespace="uav",
            name="odom_tf_publisher",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(PythonExpression([
                "'", flight_backend, "' == 'px4'"
            ])),
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
            parameters=[config, {"use_sim_time": use_sim_time,
                "publish_obstacle_cloud":ParameterValue(LaunchConfiguration("validate_dynamic_paths"),value_type=bool)}],
        ),
        Node(
            package="uav_control",
            executable="battery_manager",
            namespace="uav",
            name="battery_manager",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", enable_energy_constraints, "'.lower() == 'true' and '",
                LaunchConfiguration("start_battery_manager"), "'.lower() == 'true'",
            ])),
            parameters=[
                battery_config,
                {
                    "use_sim_time": use_sim_time,
                    "initial_percentage": ParameterValue(
                        initial_battery_percentage, value_type=float
                    ),
                    "external_charger_control": ParameterValue(
                        external_charger_control, value_type=bool
                    ),
                    "flight_backend": flight_backend,
                },
            ],
        ),
    ])
