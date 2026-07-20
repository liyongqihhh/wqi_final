import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_dir = get_package_share_directory("ugvcar_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    default_model = os.path.join(
        description_dir,
        "urdf",
        "ugvcar",
        "ugvcar.urdf.xacro",
    )
    default_world = os.path.join(description_dir, "world", "custom_room.world")

    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    visualize_sensor_rays = LaunchConfiguration("visualize_sensor_rays")

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                model,
                " visualize_sensor_rays:=",
                visualize_sensor_rays,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "verbose": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "/robot_description",
            "-entity",
            "ugvcar",
            "-timeout",
            "120.0",
        ],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "ugvcar_joint_state_broadcaster",
        ],
        output="screen",
    )
    load_diff_drive_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "ugvcar_diff_drive_controller",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value=default_model),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "visualize_sensor_rays",
                default_value="false",
            ),
            robot_state_publisher,
            gazebo,
            spawn_entity,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_diff_drive_controller],
                )
            ),
        ]
    )
