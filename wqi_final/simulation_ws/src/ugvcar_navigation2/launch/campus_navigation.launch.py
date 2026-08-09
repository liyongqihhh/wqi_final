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
    default_local_keepout_mask_path = os.path.join(
        ugvcar_navigation2_dir,
        "maps",
        "campus_local_keepout_mask.yaml",
    )
    default_dynamic_keepout_mask_path = os.path.join(
        ugvcar_navigation2_dir,
        "maps",
        "campus_dynamic_keepout_mask.yaml",
    )
    default_param_path = os.path.join(ugvcar_navigation2_dir, "config", "nav2_params.yaml")
    default_nav_to_pose_bt_path = os.path.join(
        ugvcar_navigation2_dir,
        "behavior_trees",
        "navigate_to_pose_if_path_invalid.xml",
    )
    dynamic_nav_to_pose_bt_path = os.path.join(
        ugvcar_navigation2_dir,
        "behavior_trees",
        "navigate_to_pose_dynamic_replanning.xml",
    )
    rviz_config_path = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_path = LaunchConfiguration("map")
    keepout_mask_yaml_path = LaunchConfiguration("keepout_mask")
    local_keepout_mask_yaml_path = LaunchConfiguration("local_keepout_mask")
    dynamic_keepout_mask_yaml_path = LaunchConfiguration(
        "dynamic_keepout_mask"
    )
    nav2_param_path = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("rviz")
    localization_mode = LaunchConfiguration("localization_mode")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")
    dynamic_obstacles = LaunchConfiguration("dynamic_obstacles")
    robot_radius = LaunchConfiguration("robot_radius")
    dynamic_safety_margin = PythonExpression(["0.30"])

    dynamic_controller_frequency = PythonExpression([
        "10.0 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 20.0",
    ])
    dynamic_local_inflation_radius = PythonExpression([
        "0.65 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 0.8",
    ])
    dynamic_planner_tolerance = PythonExpression([
        "2.0 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 0.25",
    ])
    dynamic_progress_allowance = PythonExpression([
        "25.0 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 8.0",
    ])
    dynamic_failure_tolerance = PythonExpression([
        "3.0 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 1.0",
    ])
    dynamic_global_update_frequency = PythonExpression([
        "5.0 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 2.0",
    ])
    dynamic_global_publish_frequency = PythonExpression([
        "2.0 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 1.0",
    ])
    dynamic_prediction_enabled = PythonExpression([
        "True if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else False",
    ])
    dynamic_global_scan_enabled = PythonExpression([
        "False if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else True",
    ])
    dynamic_goal_yaw_tolerance = PythonExpression([
        "3.14159 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 0.25",
    ])
    dynamic_curve_minimum_speed = PythonExpression([
        "0.14 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 0.08",
    ])
    dynamic_slowdown_ratio = PythonExpression([
        "0.75 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 0.45",
    ])
    dynamic_controller_collision_horizon = PythonExpression([
        "1.2 if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else 3.0",
    ])
    dynamic_controller_collision_detection = PythonExpression([
        "False if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else True",
    ])
    dynamic_emergency_stop_enabled = PythonExpression([
        "True",
    ])
    dynamic_controller_plugin = PythonExpression([
        "'dwb_core::DWBLocalPlanner' if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else "
        "'nav2_regulated_pure_pursuit_controller::"
        "RegulatedPurePursuitController'",
    ])
    dynamic_global_planner_plugin = PythonExpression([
        "'ugvcar_navigation2/DStarLitePlanner' if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else "
        "'nav2_smac_planner/SmacPlanner2D'",
    ])
    selected_nav_to_pose_bt = PythonExpression([
        "'",
        dynamic_nav_to_pose_bt_path,
        "' if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else '",
        default_nav_to_pose_bt_path,
        "'",
    ])
    selected_global_keepout_mask = PythonExpression([
        "'",
        dynamic_keepout_mask_yaml_path,
        "' if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else '",
        keepout_mask_yaml_path,
        "'",
    ])
    selected_local_keepout_mask = PythonExpression([
        "'",
        dynamic_keepout_mask_yaml_path,
        "' if '",
        dynamic_obstacles,
        "'.lower() in ('true', '1', 'yes') else '",
        local_keepout_mask_yaml_path,
        "'",
    ])

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
            "default_nav_to_pose_bt_xml": selected_nav_to_pose_bt,
            "controller_server.ros__parameters.controller_frequency": (
                dynamic_controller_frequency
            ),
            "controller_server.ros__parameters.failure_tolerance": (
                dynamic_failure_tolerance
            ),
            (
                "controller_server.ros__parameters.progress_checker."
                "movement_time_allowance"
            ): dynamic_progress_allowance,
            (
                "local_costmap.local_costmap.ros__parameters.robot_radius"
            ): robot_radius,
            (
                "local_costmap.local_costmap.ros__parameters."
                "inflation_layer.inflation_radius"
            ): dynamic_local_inflation_radius,
            (
                "global_costmap.global_costmap.ros__parameters.robot_radius"
            ): robot_radius,
            (
                "local_costmap.local_costmap.ros__parameters."
                "keepout_filter.filter_info_topic"
            ): "/local_costmap_filter_info",
            (
                "global_costmap.global_costmap.ros__parameters."
                "keepout_filter.filter_info_topic"
            ): "/global_costmap_filter_info",
            (
                "global_costmap.global_costmap.ros__parameters."
                "update_frequency"
            ): dynamic_global_update_frequency,
            (
                "global_costmap.global_costmap.ros__parameters."
                "publish_frequency"
            ): dynamic_global_publish_frequency,
            (
                "controller_server.ros__parameters.FollowPath."
                "regulated_linear_scaling_min_speed"
            ): dynamic_curve_minimum_speed,
            (
                "controller_server.ros__parameters.FollowPath.plugin"
            ): dynamic_controller_plugin,
            (
                "planner_server.ros__parameters.GridBased.plugin"
            ): dynamic_global_planner_plugin,
            (
                "planner_server.ros__parameters.GridBased.tolerance"
            ): dynamic_planner_tolerance,
            (
                "controller_server.ros__parameters.FollowPath."
                "PredictiveCollision.robot_radius"
            ): robot_radius,
            (
                "controller_server.ros__parameters.FollowPath."
                "PredictiveCollision.safety_margin"
            ): dynamic_safety_margin,
            (
                "controller_server.ros__parameters.FollowPath."
                "max_allowed_time_to_collision_up_to_carrot"
            ): dynamic_controller_collision_horizon,
            (
                "controller_server.ros__parameters.FollowPath."
                "use_collision_detection"
            ): dynamic_controller_collision_detection,
            (
                "collision_monitor.ros__parameters.StopZone.enabled"
            ): dynamic_emergency_stop_enabled,
            (
                "collision_monitor.ros__parameters.SlowdownZone."
                "slowdown_ratio"
            ): dynamic_slowdown_ratio,
            (
                "controller_server.ros__parameters.general_goal_checker."
                "yaw_goal_tolerance"
            ): dynamic_goal_yaw_tolerance,
            (
                "global_costmap.global_costmap.ros__parameters."
                "obstacle_layer.predicted_scan.clearing"
            ): dynamic_prediction_enabled,
            (
                "global_costmap.global_costmap.ros__parameters."
                "obstacle_layer.predicted_points.marking"
            ): dynamic_prediction_enabled,
            # A moving raw scan endpoint changes every frame and repeatedly
            # invalidates an otherwise safe route. Dynamic mode uses the
            # predictor's latched map-frame guards globally; the unmodified
            # scan remains active in the local costmap and collision monitor.
            (
                "global_costmap.global_costmap.ros__parameters."
                "obstacle_layer.scan.marking"
            ): dynamic_global_scan_enabled,
            (
                "global_costmap.global_costmap.ros__parameters."
                "obstacle_layer.scan.clearing"
            ): dynamic_global_scan_enabled,
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
            description="Campus global-planning road keepout mask yaml",
        ),
        DeclareLaunchArgument(
            "local_keepout_mask",
            default_value=default_local_keepout_mask_path,
            description="Campus local-control road keepout mask yaml",
        ),
        DeclareLaunchArgument(
            "dynamic_keepout_mask",
            default_value=default_dynamic_keepout_mask_path,
            description=(
                "Global-planning mask used in dynamic mode so a combined "
                "UGV-UAV footprint can use the complete paved surface."
            ),
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
        DeclareLaunchArgument(
            "dynamic_obstacles",
            default_value="false",
            description="Enable 360-degree prediction and continuous Nav2 replanning.",
        ),
        DeclareLaunchArgument(
            "robot_radius",
            default_value="0.22",
            description="Circular Nav2 footprint radius in metres.",
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="global_filter_mask_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "yaml_filename": selected_global_keepout_mask,
                "topic_name": "/global_keepout_filter_mask",
                "frame_id": "map",
            }],
        ),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="global_costmap_filter_info_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "type": 0,
                "filter_info_topic": "/global_costmap_filter_info",
                "mask_topic": "/global_keepout_filter_mask",
                "base": 0.0,
                "multiplier": 1.0,
            }],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="local_filter_mask_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "yaml_filename": selected_local_keepout_mask,
                "topic_name": "/local_keepout_filter_mask",
                "frame_id": "map",
            }],
        ),
        Node(
            package="nav2_map_server",
            executable="costmap_filter_info_server",
            name="local_costmap_filter_info_server",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "type": 0,
                "filter_info_topic": "/local_costmap_filter_info",
                "mask_topic": "/local_keepout_filter_mask",
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
                    "global_filter_mask_server",
                    "global_costmap_filter_info_server",
                    "local_filter_mask_server",
                    "local_costmap_filter_info_server",
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
                "use_composition": "False",
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
        Node(
            package="ugvcar_navigation2",
            executable="dynamic_obstacle_predictor.py",
            name="dynamic_obstacle_predictor",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_radius": robot_radius,
                "safety_margin": dynamic_safety_margin,
            }],
            condition=IfCondition(dynamic_obstacles),
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[configured_nav2_params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_collision_monitor",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["collision_monitor"],
            }],
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
