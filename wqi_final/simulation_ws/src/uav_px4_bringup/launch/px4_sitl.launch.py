#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)


def _start_px4(context):
    px4_dir = LaunchConfiguration("px4_autopilot_dir")
    build_dir = PathJoinSubstitution([px4_dir, "build", "px4_sitl_default"])
    px4_binary = PathJoinSubstitution([build_dir, "bin", "px4"])
    px4_etc = PathJoinSubstitution([build_dir, "etc"])
    rootfs = PathJoinSubstitution([build_dir, "rootfs"])
    moving_base = LaunchConfiguration("moving_base").perform(context).lower() == "true"
    environment = {
        "PX4_SIM_MODEL": "gazebo-classic_iris",
        "PX4_PARAM_COM_RCL_EXCEPT": "4",
        "PX4_PARAM_UXRCE_DDS_SYNCT": "0",
        "PX4_PARAM_MPC_LAND_SPEED": "0.3",
        "PX4_PARAM_SIM_BAT_ENABLE": PythonExpression([
            "'0' if '", LaunchConfiguration("enable_energy_constraints"),
            "'.lower() == 'true' else '1'",
        ]),
        "PX4_PARAM_EKF2_GPS_CHECK": "863" if moving_base else "1023",
        "PX4_PARAM_IMU_GYRO_CAL_EN": "0" if moving_base else "1",
        "PX4_PARAM_EKF2_EV_CTRL": "6" if moving_base else "0",
        "PX4_PARAM_EKF2_EVV_NOISE": "0.01" if moving_base else "0.1",
        "PX4_PARAM_EKF2_EVP_NOISE": "0.02" if moving_base else "0.1",
        "PX4_PARAM_EKF2_HGT_REF": "3" if moving_base else "1",
        "PX4_PARAM_EKF2_GPS_CTRL": "5" if moving_base else "7",
        # Deck support is not rotor lift. Do not relearn hover thrust from the
        # contact force; 0.71 is measured for this fixed-mass SITL Iris model.
        "PX4_PARAM_MPC_USE_HTE": "0" if moving_base else "1",
        "PX4_PARAM_MPC_THR_HOVER": "0.71" if moving_base else "0.5",
    }
    if moving_base:
        # SITL persists calibration across runs. Remove offsets learned from
        # carrier turns; the simulated IMU and EKF bias estimation stay enabled.
        for gyro in range(3):
            for axis in "XYZ":
                environment[f"PX4_PARAM_CAL_GYRO{gyro}_{axis}OFF"] = "0"
    return [ExecuteProcess(
        cmd=[px4_binary, "-d", px4_etc],
        cwd=rootfs,
        name="px4_sitl",
        output="screen",
        additional_env=environment,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "px4_autopilot_dir",
            default_value=EnvironmentVariable(
                "PX4_AUTOPILOT_DIR",
                default_value="/home/wqi/design_final_artifacts/px4/PX4-Autopilot",
            ),
        ),
        DeclareLaunchArgument("enable_energy_constraints", default_value="false"),
        DeclareLaunchArgument("moving_base", default_value="false"),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
            name="px4_dds_agent",
            output="screen",
        ),
        OpaqueFunction(function=_start_px4),
    ])
