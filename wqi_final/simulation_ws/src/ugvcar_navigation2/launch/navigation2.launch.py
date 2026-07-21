import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # 获取与拼接默认路径
    ugvcar_navigation2_dir = get_package_share_directory(
        'ugvcar_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bt_navigator_dir = get_package_share_directory(
        'nav2_bt_navigator')
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    default_nav_to_pose_bt = os.path.join(
        nav2_bt_navigator_dir,
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    )

    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(ugvcar_navigation2_dir, 'maps', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(ugvcar_navigation2_dir, 'config', 'nav2_params.yaml'))
    use_rviz = launch.substitutions.LaunchConfiguration(
        'rviz', default='true')
    room_nav2_params = RewrittenYaml(
        source_file=nav2_param_path,
        param_rewrites={
            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt,
            'amcl.ros__parameters.initial_pose.x': '0.0',
            'amcl.ros__parameters.initial_pose.y': '0.0',
            'amcl.ros__parameters.initial_pose.z': '0.0',
            'amcl.ros__parameters.initial_pose.yaw': '0.0',
        },
        convert_types=True,
    )

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),
        launch.actions.DeclareLaunchArgument('rviz', default_value=use_rviz,
                                             description='Start RViz if true'),

        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_path,
                'topic_name': '/keepout_filter_mask',
                'frame_id': 'map',
            }],
        ),
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'type': 0,
                'filter_info_topic': '/costmap_filter_info',
                'mask_topic': '/keepout_filter_mask',
                'base': 0.0,
                'multiplier': 1.0,
            }],
        ),
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_keepout',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'filter_mask_server',
                    'costmap_filter_info_server',
                ],
            }],
        ),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': room_nav2_params,
                'use_composition': 'False',
            }.items(),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            condition=IfCondition(use_rviz)),
    ])
