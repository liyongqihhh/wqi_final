#!/usr/bin/env python3
import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    SetEnvironmentVariable,
    TimerAction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def wait_before_uav(context, spawn):
    if LaunchConfiguration("relay_docking").perform(context).lower() != "true":
        return [spawn]
    waiter = Node(package="cooperative_delivery", executable="wait_for_carrier_control",
        name="wait_for_a_before_uav", output="screen", parameters=[{"use_sim_time": True}])
    def finished(event, _context):
        return [spawn] if event.returncode == 0 else [Shutdown(reason="A-car control initialization failed")]
    return [RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=finished)), waiter]


def prepare_px4_carrier(context):
    if LaunchConfiguration("flight_backend").perform(context) != "px4":
        return []
    import xacro
    from cooperative_delivery.px4_carrier_model import add_physical_deck

    model_path = LaunchConfiguration("ugv_model").perform(context)
    rays = LaunchConfiguration("visualize_sensor_rays").perform(context)
    document = xacro.process_file(model_path, mappings={"visualize_sensor_rays": rays})
    directory = Path(os.environ.get("WQI_BUILD_ROOT", "/tmp")) / "generated_models"
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", prefix="px4_carrier_", suffix=".urdf",
        dir=directory, delete=False,
    ) as stream:
        stream.write(add_physical_deck(document.toxml()))
        generated = stream.name
    return [SetLaunchConfiguration("ugv_model", generated)]


def generate_launch_description():
    ugv_description_share = get_package_share_directory("ugvcar_description")
    ugv_navigation_share = get_package_share_directory("ugvcar_navigation2")
    uav_description_share = get_package_share_directory("uav_description")
    uav_application_share = get_package_share_directory("uav_application")
    cooperative_share = get_package_share_directory("cooperative_delivery")
    dynamic_obstacles_share = get_package_share_directory(
        "campus_dynamic_obstacles"
    )
    px4_bringup_share = get_package_share_directory("uav_px4_bringup")

    ugv_sim_launch = os.path.join(
        ugv_description_share, "launch", "campus_delivery_sim.launch.py"
    )
    ugv_navigation_launch = os.path.join(
        ugv_navigation_share, "launch", "campus_navigation.launch.py"
    )
    uav_spawn_launch = os.path.join(
        uav_description_share, "launch", "uav_spawn.launch.py"
    )
    uav_application_launch = os.path.join(
        uav_application_share, "launch", "uav_delivery.launch.py"
    )
    dynamic_obstacles_launch = os.path.join(
        dynamic_obstacles_share, "launch", "dynamic_obstacles.launch.py"
    )
    rviz_config = os.path.join(cooperative_share, "rviz", "cooperative.rviz")
    campus_map = os.path.join(
        ugv_navigation_share, "maps", "campus_delivery_map.yaml"
    )
    keepout_mask = os.path.join(
        ugv_navigation_share, "maps", "campus_keepout_mask.yaml"
    )
    nav2_params = os.path.join(
        ugv_navigation_share, "config", "nav2_params.yaml"
    )
    ugv_energy_config = os.path.join(
        cooperative_share, "config", "ugv_energy_model.yaml"
    )
    default_mission_config = os.path.join(
        cooperative_share, "config", "cooperative_waypoints.yaml"
    )
    default_obstacle_config = os.path.join(
        dynamic_obstacles_share, "config", "obstacle_routes.yaml"
    )

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    navigation_delay = LaunchConfiguration("navigation_delay")
    uav_spawn_delay = LaunchConfiguration("uav_spawn_delay")
    uav_application_delay = LaunchConfiguration("uav_application_delay")
    manager_delay = LaunchConfiguration("manager_delay")
    dynamic_obstacle_delay = LaunchConfiguration(
        "dynamic_obstacle_delay"
    )
    rviz_delay = LaunchConfiguration("rviz_delay")
    visualize_sensor_rays = LaunchConfiguration("visualize_sensor_rays")
    initial_battery_percentage = LaunchConfiguration(
        "initial_battery_percentage"
    )
    initial_ugv_drive_battery_percentage = LaunchConfiguration(
        "initial_ugv_drive_battery_percentage"
    )
    initial_ugv_charging_battery_percentage = LaunchConfiguration(
        "initial_ugv_charging_battery_percentage"
    )
    enable_energy_constraints = LaunchConfiguration(
        "enable_energy_constraints"
    )
    enable_dynamic_obstacles = LaunchConfiguration(
        "enable_dynamic_obstacles"
    )
    combined_robot_radius = "0.60"
    obstacle_density = LaunchConfiguration("obstacle_density")
    random_seed = LaunchConfiguration("random_seed")
    obstacle_config_file = LaunchConfiguration("obstacle_config_file")
    mission_config = LaunchConfiguration("mission_config")
    flight_backend = LaunchConfiguration("flight_backend")
    px4_autopilot_dir = LaunchConfiguration("px4_autopilot_dir")
    px4_selected = IfCondition(PythonExpression([
        "'", flight_backend, "' == 'px4'"
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Start Gazebo GUI; keep false when RViz is used in VirtualBox.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start the combined UGV-UAV RViz view.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_mission_manager", default_value="true"),
        DeclareLaunchArgument("start_ugv_energy_manager", default_value="true"),
        DeclareLaunchArgument("relay_docking", default_value="false"),
        DeclareLaunchArgument("nav2_params_file", default_value=nav2_params),
        DeclareLaunchArgument("static_controller_frequency", default_value="20.0"),
        DeclareLaunchArgument("bt_loop_duration", default_value="50"),
        DeclareLaunchArgument("filter_info_package", default_value="nav2_map_server"),
        DeclareLaunchArgument("filter_info_executable", default_value="costmap_filter_info_server"),
        DeclareLaunchArgument("ugv_model", default_value=os.path.join(
            ugv_description_share, "urdf", "ugvcar", "ugvcar.urdf.xacro")),
        DeclareLaunchArgument(
            "flight_backend", default_value="px4", choices=["px4"]
        ),
        DeclareLaunchArgument("route_policy", default_value="energy_time",
                              choices=["input_order", "distance", "energy_time"]),
        DeclareLaunchArgument("charge_policy", default_value="as_needed",
                              choices=["full", "as_needed"]),
        DeclareLaunchArgument(
            "px4_autopilot_dir",
            default_value=EnvironmentVariable(
                "PX4_AUTOPILOT_DIR",
                default_value="/home/wqi/design_final_artifacts/px4/PX4-Autopilot",
            ),
        ),
        DeclareLaunchArgument(
            "initial_battery_percentage",
            default_value="0.80",
            description="Initial UAV battery state of charge in the range 0 to 1.",
        ),
        DeclareLaunchArgument(
            "initial_ugv_drive_battery_percentage",
            default_value="0.80",
            description="Initial UGV traction battery SOC in the range 0 to 1.",
        ),
        DeclareLaunchArgument(
            "initial_ugv_charging_battery_percentage",
            default_value="0.80",
            description="Initial UGV UAV-charging battery SOC in the range 0 to 1.",
        ),
        DeclareLaunchArgument(
            "enable_energy_constraints",
            default_value="false",
            description="Enable UAV and dual-pack UGV energy constraints.",
        ),
        DeclareLaunchArgument(
            "enable_dynamic_obstacles",
            default_value="false",
            description="Start the campus dynamic obstacle generator.",
        ),
        DeclareLaunchArgument(
            "obstacle_density",
            default_value="none",
            description="Dynamic obstacle density: none, low, medium or high.",
        ),
        DeclareLaunchArgument("random_seed", default_value="42"),
        DeclareLaunchArgument(
            "obstacle_config_file",
            default_value=default_obstacle_config,
            description="Dynamic obstacle route YAML; defaults to the four-density campus set.",
        ),
        DeclareLaunchArgument(
            "mission_config",
            default_value=default_mission_config,
            description="Cooperative mission waypoint configuration.",
        ),
        DeclareLaunchArgument("navigation_delay", default_value="15.0"),
        DeclareLaunchArgument("uav_spawn_delay", default_value="28.0"),
        DeclareLaunchArgument(
            "dynamic_obstacle_delay", default_value="34.0"
        ),
        DeclareLaunchArgument("uav_application_delay", default_value="38.0"),
        DeclareLaunchArgument("manager_delay", default_value="44.0"),
        DeclareLaunchArgument("rviz_delay", default_value="48.0"),
        DeclareLaunchArgument(
            "visualize_sensor_rays",
            default_value="false",
            description="Show Gazebo UGV/UAV lidar rays; topics remain active when false.",
        ),
        OpaqueFunction(function=prepare_px4_carrier),
        SetEnvironmentVariable(
            "GAZEBO_PLUGIN_PATH",
            [px4_autopilot_dir, "/build/px4_sitl_default/build_gazebo-classic:",
             EnvironmentVariable("GAZEBO_PLUGIN_PATH", default_value="")],
            condition=px4_selected,
        ),
        SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH",
            [px4_autopilot_dir,
             "/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:",
             EnvironmentVariable("GAZEBO_MODEL_PATH", default_value="")],
            condition=IfCondition(PythonExpression([
                "'", flight_backend, "' == 'px4'"
            ])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ugv_sim_launch),
            launch_arguments={
                "gui": gui,
                "model": LaunchConfiguration("ugv_model"),
                "use_sim_time": use_sim_time,
                "x": "0.0",
                "y": "-43.5",
                "z": "0.005",
                "yaw": "1.5708",
                "visualize_sensor_rays": visualize_sensor_rays,
            }.items(),
        ),
        TimerAction(
            period=navigation_delay,
            actions=[
                GroupAction(
                    scoped=True,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(ugv_navigation_launch),
                            launch_arguments={
                                "rviz": "false",
                                "use_sim_time": use_sim_time,
                                "localization_mode": "ground_truth",
                                "map": campus_map,
                                "keepout_mask": keepout_mask,
                                "params_file": LaunchConfiguration("nav2_params_file"),
                                "static_controller_frequency": LaunchConfiguration("static_controller_frequency"),
                                "bt_loop_duration": LaunchConfiguration("bt_loop_duration"),
                                "filter_info_package": LaunchConfiguration("filter_info_package"),
                                "filter_info_executable": LaunchConfiguration("filter_info_executable"),
                                "initial_x": "0.0",
                                "initial_y": "-43.5",
                                "initial_yaw": "1.5708",
                                "dynamic_obstacles": enable_dynamic_obstacles,
                                # The 0.60 m circle covers the 0.56 m docked UAV
                                # envelope while retaining enough road width for
                                # Nav2 to replan around a moving obstacle.
                                # Standalone UGV uses its 0.22 m physical radius.
                                "robot_radius": combined_robot_radius,
                            }.items(),
                        )
                    ],
                )
            ],
        ),
        TimerAction(
            period=uav_spawn_delay,
            actions=[
                OpaqueFunction(function=wait_before_uav, kwargs={"spawn": IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(uav_spawn_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "x": "0.0",
                        "y": "-43.5",
                        "z": "0.42",
                        "yaw": "1.5708",
                        "enable_docking": "true",
                        "relay_docking": LaunchConfiguration("relay_docking"),
                        "visualize_sensor_rays": visualize_sensor_rays,
                        "flight_backend": flight_backend,
                        "px4_autopilot_dir": px4_autopilot_dir,
                    }.items(),
                )})
            ],
        ),
        TimerAction(
            period=PythonExpression([uav_spawn_delay, " + 5.0"]),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    px4_bringup_share, "launch", "px4_sitl.launch.py"
                )),
                condition=IfCondition(PythonExpression([
                    "'", flight_backend, "' == 'px4'"
                ])),
                launch_arguments={
                    "px4_autopilot_dir": px4_autopilot_dir,
                    "enable_energy_constraints": enable_energy_constraints,
                    "moving_base": "true",
                }.items(),
            )],
        ),
        TimerAction(
            period=uav_application_delay,
            actions=[
                Node(
                    package="cooperative_delivery",
                    executable="ugv_energy_manager",
                    namespace="ugv",
                    name="energy_manager",
                    output="screen",
                    condition=IfCondition(PythonExpression([
                        "'", enable_energy_constraints, "'.lower() == 'true' and '",
                        LaunchConfiguration("start_ugv_energy_manager"), "'.lower() == 'true'",
                    ])),
                    parameters=[
                        ugv_energy_config,
                        {
                            "use_sim_time": use_sim_time,
                            "docked_uav_mass_kg": 1.535,
                            "initial_drive_percentage": ParameterValue(
                                initial_ugv_drive_battery_percentage,
                                value_type=float,
                            ),
                            "initial_charging_percentage": ParameterValue(
                                initial_ugv_charging_battery_percentage,
                                value_type=float,
                            ),
                        },
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(uav_application_launch),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "initial_battery_percentage": initial_battery_percentage,
                        "enable_energy_constraints": enable_energy_constraints,
                        "external_charger_control": "true",
                        "use_carrier_home": "true",
                        "enable_sim_velocity_aiding": "true",
                        "flight_backend": flight_backend,
                    }.items(),
                )
            ],
        ),
        TimerAction(
            period=manager_delay,
            actions=[
                Node(
                    package="cooperative_delivery",
                    executable="cooperative_mission_manager",
                    condition=IfCondition(LaunchConfiguration("start_mission_manager")),
                    namespace="cooperative_delivery",
                    name="mission_manager",
                    output="screen",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "energy_constraints_enabled": ParameterValue(
                            enable_energy_constraints, value_type=bool
                        ),
                        "mission_config": mission_config,
                        "ugv_energy_config": ugv_energy_config,
                        "flight_backend": flight_backend,
                        "route_policy": LaunchConfiguration("route_policy"),
                        "charge_policy": LaunchConfiguration("charge_policy"),
                    }],
                )
            ],
        ),
        TimerAction(
            period=dynamic_obstacle_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(dynamic_obstacles_launch),
                    condition=IfCondition(enable_dynamic_obstacles),
                    launch_arguments={
                        "use_sim_time": use_sim_time,
                        "density": obstacle_density,
                        "random_seed": random_seed,
                        "config_file": obstacle_config_file,
                    }.items(),
                )
            ],
        ),
        TimerAction(
            period=rviz_delay,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="cooperative_rviz",
                    output="screen",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": use_sim_time}],
                    condition=IfCondition(rviz),
                )
            ],
        ),
    ])
