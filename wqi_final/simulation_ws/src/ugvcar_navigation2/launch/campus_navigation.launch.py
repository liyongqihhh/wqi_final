import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node


def generate_launch_description():
    ugvcar_navigation2_dir = get_package_share_directory("ugvcar_navigation2")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    default_map_path = os.path.join(ugvcar_navigation2_dir, "maps", "campus_delivery_map.yaml")
    default_keepout_mask_path = os.path.join(
        ugvcar_navigation2_dir, "maps", "campus_keepout_mask.yaml"
    )
    default_param_path = os.path.join(ugvcar_navigation2_dir, "config", "nav2_params.yaml")
    default_nav_to_pose_bt_path = os.path.join(
        ugvcar_navigation2_dir,
        "behavior_trees",
        "navigate_to_pose_if_path_invalid.xml",
    )
    rviz_config_path = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_path = LaunchConfiguration("map")
    keepout_mask_yaml_path = LaunchConfiguration("keepout_mask")
    nav2_param_path = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("rviz")
    localization_mode = LaunchConfiguration("localization_mode")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    use_amcl = IfCondition(PythonExpression(["'", localization_mode, "' == 'amcl'"]))
    use_odom = IfCondition(PythonExpression(["'", localization_mode, "' == 'odom'"]))
    use_ground_truth = IfCondition(
        PythonExpression(["'", localization_mode, "' == 'ground_truth'"])
    )
    use_sim_localization = IfCondition(
        PythonExpression(["'", localization_mode, "' != 'amcl'"])
    )
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_param_path,
        param_rewrites={
            "default_nav_to_pose_bt_xml": default_nav_to_pose_bt_path,
        },
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map_path,
            description="Campus delivery map yaml",
        ),
        DeclareLaunchArgument(
            "keepout_mask",
            default_value=default_keepout_mask_path,
            description="Campus road keepout mask yaml",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_param_path,
            description="Nav2 params file",
        ),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz if true"),
        DeclareLaunchArgument(
            "localization_mode",
            default_value="ground_truth",
            description="Use Gazebo ground truth, wheel odom fallback, or AMCL",
        ),
        DeclareLaunchArgument("initial_x", default_value="0.0", description="Map-frame spawn x"),
        DeclareLaunchArgument("initial_y", default_value="-43.0", description="Map-frame spawn y"),
        DeclareLaunchArgument(
            "initial_yaw",
            default_value="1.5708",
            description="Map-frame spawn yaw",
        ),
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
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "map": map_yaml_path,
                "use_sim_time": use_sim_time,
                "params_file": configured_nav2_params,
            }.items(),
            condition=use_amcl,
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml_path,
            }],
            condition=use_sim_localization,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server"],
            }],
            condition=use_sim_localization,
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_publisher",
            arguments=[
                "--x", initial_x,
                "--y", initial_y,
                "--z", "0.0",
                "--yaw", initial_yaw,
                "--pitch", "0.0",
                "--roll", "0.0",
                "--frame-id", "map",
                "--child-frame-id", "odom",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            condition=use_odom,
        ),
        Node(
            package="ugvcar_navigation2",
            executable="gazebo_ground_truth_localizer.py",
            name="gazebo_ground_truth_localizer",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=use_ground_truth,
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": configured_nav2_params,
                "use_composition": "False",
            }.items(),
            condition=use_sim_localization,
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
