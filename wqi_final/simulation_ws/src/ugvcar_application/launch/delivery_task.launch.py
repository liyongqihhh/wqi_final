import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ugvcar_description")
    waypoints_file = os.path.join(pkg_share, "config", "delivery_waypoints.yaml")

    delivery_targets = LaunchConfiguration("delivery_targets")
    start_point = LaunchConfiguration("start_point")
    wait_duration = LaunchConfiguration("wait_duration")

    return LaunchDescription([
        DeclareLaunchArgument(
            "delivery_targets",
            default_value="['teaching_building']",
            description="List of building names to deliver to",
        ),
        DeclareLaunchArgument(
            "start_point",
            default_value="logistics_center",
            description="Starting point name",
        ),
        DeclareLaunchArgument(
            "wait_duration",
            default_value="10.0",
            description="Seconds to wait at each stop",
        ),
        Node(
            package="ugvcar_application",
            executable="delivery_task",
            name="delivery_task_manager",
            output="screen",
            parameters=[{
                "delivery_targets": delivery_targets,
                "start_point": start_point,
                "wait_duration": wait_duration,
            }],
        ),
    ])
