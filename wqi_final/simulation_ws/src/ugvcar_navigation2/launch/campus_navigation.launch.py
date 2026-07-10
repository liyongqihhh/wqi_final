import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ugvcar_navigation2_dir = get_package_share_directory("ugvcar_navigation2")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    default_map_path = os.path.join(ugvcar_navigation2_dir, "maps", "campus_delivery_map.yaml")
    default_keepout_mask_path = os.path.join(
        ugvcar_navigation2_dir, "maps", "campus_keepout_mask.yaml"
    )
    default_param_path = os.path.join(ugvcar_navigation2_dir, "config", "nav2_params.yaml")
    rviz_config_path = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_path = LaunchConfiguration("map")
    keepout_mask_yaml_path = LaunchConfiguration("keepout_mask")
    nav2_param_path = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
        DeclareLaunchArgument("map", default_value=default_map_path, description="Campus delivery map yaml"),
        DeclareLaunchArgument(
            "keepout_mask",
            default_value=default_keepout_mask_path,
            description="Campus road keepout mask yaml",
        ),
        DeclareLaunchArgument("params_file", default_value=default_param_path, description="Nav2 params file"),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz if true"),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="filter_mask_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "yaml_filename": keepout_mask_yaml_path,
                "topic_name": "/keepout_filter_mask",
                "frame_id": "map",
            }],
        ),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="costmap_filter_info_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "type": 0,
                "filter_info_topic": "/costmap_filter_info",
                "mask_topic": "/keepout_filter_mask",
                "base": 0.0,
                "multiplier": 1.0,
            }],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_keepout",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "filter_mask_server",
                    "costmap_filter_info_server",
                ],
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")),
            launch_arguments={
                "map": map_yaml_path,
                "use_sim_time": use_sim_time,
                "params_file": nav2_param_path,
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_path],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
