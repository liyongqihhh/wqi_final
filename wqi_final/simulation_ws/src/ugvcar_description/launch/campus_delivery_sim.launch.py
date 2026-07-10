import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ugvcar_description_dir = get_package_share_directory("ugvcar_description")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    default_model_path = os.path.join(ugvcar_description_dir, "urdf", "ugvcar", "ugvcar.urdf.xacro")
    default_world_path = os.path.join(ugvcar_description_dir, "world", "campus_delivery.world")

    model_path = LaunchConfiguration("model")
    world_path = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    spawn_x = LaunchConfiguration("x")
    spawn_y = LaunchConfiguration("y")
    spawn_z = LaunchConfiguration("z")
    spawn_yaw = LaunchConfiguration("yaw")

    robot_description = ParameterValue(Command(["xacro ", model_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        output="screen",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, "launch", "gazebo.launch.py")),
        launch_arguments={"world": world_path, "gui": gui, "verbose": "true"}.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "ugvcar",
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-Y", spawn_yaw,
        ],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "ugvcar_joint_state_broadcaster"],
        output="screen",
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "active", "ugvcar_diff_drive_controller"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("model", default_value=default_model_path, description="Absolute path to UGV xacro model"),
        DeclareLaunchArgument("world", default_value=default_world_path, description="Absolute path to campus_delivery.world"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
        DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI"),
        DeclareLaunchArgument("x", default_value="0.0", description="UGV spawn x"),
        DeclareLaunchArgument("y", default_value="-43.0", description="UGV spawn y"),
        DeclareLaunchArgument("z", default_value="0.065", description="UGV spawn z"),
        DeclareLaunchArgument("yaw", default_value="1.5708", description="UGV spawn yaw"),
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[load_joint_state_controller])),
        RegisterEventHandler(OnProcessExit(target_action=load_joint_state_controller, on_exit=[load_diff_drive_controller])),
    ])
