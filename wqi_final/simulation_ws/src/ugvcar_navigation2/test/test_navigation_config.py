from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def test_behavior_tree_replans_only_when_path_is_invalid():
    path = (
        PACKAGE_ROOT
        / "behavior_trees"
        / "navigate_to_pose_if_path_invalid.xml"
    )
    root = ET.parse(path).getroot()

    rate = root.find(".//RateController")
    assert rate is not None
    assert float(rate.attrib["hz"]) >= 2.0
    assert root.find(".//PathExpiringTimer") is None
    assert root.find(".//DynamicObstacleCleared") is None
    assert root.find(".//DynamicObstacleChanged") is None
    assert root.find(".//IsPathValid") is not None

    fallback = root.find(".//Fallback[@name='FallbackComputePathToPose']")
    assert fallback is not None
    assert [child.tag for child in fallback] == [
        "ReactiveSequence", "ComputePathToPose"
    ]

    computes = root.findall(".//ComputePathToPose")
    assert len(computes) == 2
    controller_recovery = root.find(
        ".//Sequence[@name='ClearAndRecomputePath']"
    )
    assert controller_recovery is not None
    assert [child.tag for child in controller_recovery] == [
        "ClearEntireCostmap", "ComputePathToPose"
    ]


def test_dynamic_behavior_tree_refreshes_route_on_threat_state_change():
    path = (
        PACKAGE_ROOT
        / "behavior_trees"
        / "navigate_to_pose_dynamic_replanning.xml"
    )
    root = ET.parse(path).getroot()

    rate = root.find(".//RateController[@name='DynamicPathReplanner']")
    assert rate is not None
    assert float(rate.attrib["hz"]) == 1.0
    assert root.find(".//PathExpiringTimer") is None
    change_condition = root.find(".//DynamicObstacleChanged")
    assert change_condition is not None
    assert change_condition.attrib["status_topic"] == (
        "/ugv/dynamic_replanning/status"
    )
    smoothers = root.findall(".//SmoothPath")
    assert len(smoothers) == 2
    assert all(node.attrib["smoother_id"] == "simple_smoother" for node in smoothers)
    assert all(node.attrib["check_for_collisions"] == "true" for node in smoothers)

    monitor_branches = list(rate)
    assert len(monitor_branches) == 1
    conditional = monitor_branches[0]
    assert conditional.tag == "Fallback"
    assert conditional.attrib["name"] == "ComputeWhenDynamicPathChanges"
    assert conditional.find(".//ComputePathToPose") is not None
    assert conditional.find(".//IsPathValid") is not None
    assert conditional.find("./AlwaysSuccess") is not None

    computes = root.findall(".//ComputePathToPose")
    assert len(computes) == 4
    assert root.find(
        ".//Fallback[@name='SmoothOrUseRawDynamicPath']"
    ) is not None
    assert root.find(
        ".//Fallback[@name='RecoverySmoothOrUseRawDynamicPath']"
    ) is not None
    controller_recovery = root.find(
        ".//Sequence[@name='RefreshAndReplan']"
    )
    assert controller_recovery is not None
    assert [child.tag for child in controller_recovery] == [
        "ClearEntireCostmap",
        "ClearEntireCostmap",
        "Wait",
        "ComputePathToPose",
        "Fallback",
    ]
    assert root.find(".//Spin") is None
    assert root.find(".//BackUp") is None


def test_costmaps_detect_dynamic_obstacles_before_collision():
    path = PACKAGE_ROOT / "config" / "nav2_params.yaml"
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    controller = data["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    assert controller["progress_checker"]["movement_time_allowance"] <= 8.0
    assert follow_path["use_collision_detection"] is True
    assert follow_path["max_allowed_time_to_collision_up_to_carrot"] >= 3.0
    assert follow_path["regulated_linear_scaling_min_speed"] >= 0.08
    assert follow_path["inflation_cost_scaling_factor"] == 2.5
    assert follow_path["desired_linear_vel"] == 0.40
    assert follow_path["max_linear_vel"] == 0.40
    assert follow_path["min_lookahead_dist"] >= 0.40
    assert follow_path["max_lookahead_dist"] >= 1.20
    assert follow_path["lookahead_time"] >= 1.8
    assert follow_path["max_angular_accel"] <= 1.5
    assert follow_path["allow_reversing"] is False
    assert follow_path["debug_trajectory_details"] is False
    assert follow_path["sim_period"] == 0.10
    assert follow_path["vx_samples"] == 7
    assert 7 <= follow_path["vtheta_samples"] <= 11
    assert follow_path["linear_granularity"] >= 0.10
    assert follow_path["angular_granularity"] >= 0.05
    assert follow_path["sim_time"] >= 4.0
    assert follow_path["short_circuit_trajectory_evaluation"] is False
    assert "PredictiveCollision" in follow_path["critics"]
    assert "PathHeading" in follow_path["critics"]
    assert follow_path["PredictiveCollision.scale"] >= 1000000.0
    assert follow_path["PredictiveCollision.prediction_horizon"] >= 4.0
    assert follow_path["PredictiveCollision.safety_margin"] == 0.30
    assert follow_path["PredictiveCollision.preferred_clearance"] == 0.45
    assert 0.0 < follow_path["PredictiveCollision.clearance_weight"] < 0.001
    assert follow_path["PredictiveCollision.tracks_topic"] == (
        "/ugv/tracked_dynamic_obstacles"
    )
    assert follow_path["PathHeading.activation_linear_velocity"] <= 0.08
    assert follow_path["PathHeading.activation_angle"] >= 0.7

    planner = data["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["lethal_cost"] == 253
    assert planner["cost_penalty"] > 0.0
    assert planner["max_expansions"] >= 1000000

    smoother = data["velocity_smoother"]["ros__parameters"]
    assert smoother["max_velocity"][0] == 0.40
    assert smoother["max_accel"][0] <= 0.50

    collision_monitor = data["collision_monitor"]["ros__parameters"]
    assert collision_monitor["cmd_vel_in_topic"] == "/cmd_vel"
    assert collision_monitor["cmd_vel_out_topic"] == "/cmd_vel_safe"
    assert collision_monitor["polygons"] == ["StopZone", "SlowdownZone"]
    assert collision_monitor["StopZone"]["action_type"] == "stop"
    assert collision_monitor["SlowdownZone"]["action_type"] == "slowdown"
    stop_x = collision_monitor["StopZone"]["points"][::2]
    slowdown_x = collision_monitor["SlowdownZone"]["points"][::2]
    assert min(stop_x) >= -0.10
    assert min(slowdown_x) >= -0.15
    assert collision_monitor["SlowdownZone"]["slowdown_ratio"] >= 0.4
    assert collision_monitor["scan"]["topic"] == "/scan"

    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    local_inflation = local["inflation_layer"]
    local_obstacle = local["obstacle_layer"]["scan"]
    assert local["update_frequency"] >= 10.0
    assert local["width"] >= 12
    assert local["height"] >= 12
    assert "voxel_layer" not in local["plugins"]
    assert local["obstacle_layer"]["observation_sources"] == "scan"
    assert "predicted_scan" not in local["obstacle_layer"]
    assert local_inflation["inflation_radius"] == 0.8
    assert local_inflation["cost_scaling_factor"] == 2.5
    assert follow_path["cost_scaling_dist"] <= local_inflation[
        "inflation_radius"
    ]
    assert follow_path["inflation_cost_scaling_factor"] == local_inflation[
        "cost_scaling_factor"
    ]
    assert local_obstacle["obstacle_max_range"] >= 4.5
    assert local_obstacle["raytrace_max_range"] > local_obstacle[
        "obstacle_max_range"
    ]
    assert local_obstacle["clearing"] is True
    assert local_obstacle["inf_is_valid"] is True
    assert local_obstacle["observation_persistence"] == 0.0
    global_costmap = data["global_costmap"]["global_costmap"]["ros__parameters"]
    global_inflation = global_costmap["inflation_layer"]
    global_layer = global_costmap["obstacle_layer"]
    global_obstacle = global_layer["scan"]
    global_prediction = global_layer["predicted_scan"]
    global_prediction_points = global_layer["predicted_points"]
    assert global_costmap["update_frequency"] >= 2.0
    assert global_obstacle["obstacle_max_range"] >= 14.0
    assert global_obstacle["raytrace_max_range"] > global_obstacle[
        "obstacle_max_range"
    ]
    assert global_obstacle["marking"] is True
    assert global_obstacle["clearing"] is True
    assert global_obstacle["inf_is_valid"] is True
    assert global_obstacle["observation_persistence"] == 0.0
    assert global_obstacle["obstacle_min_range"] >= 1.2
    assert global_layer["observation_sources"] == (
        "scan predicted_scan predicted_points"
    )
    assert global_prediction["topic"] == "/scan_dynamic_predictions"
    assert global_prediction["marking"] is False
    assert global_prediction["clearing"] is True
    assert global_prediction["observation_persistence"] == 0.0
    assert global_prediction["obstacle_max_range"] >= 14.0
    assert global_prediction["raytrace_max_range"] > global_prediction[
        "obstacle_max_range"
    ]
    assert global_prediction_points["topic"] == (
        "/points_dynamic_predictions"
    )
    assert global_prediction_points["data_type"] == "PointCloud2"
    assert global_prediction_points["marking"] is True
    assert global_prediction_points["clearing"] is False
    assert global_prediction_points["observation_persistence"] == 0.0
    assert global_inflation["inflation_radius"] == 0.8
    assert global_inflation["cost_scaling_factor"] == 2.5
    assert local_inflation["inflation_radius"] <= global_inflation[
        "inflation_radius"
    ]


def test_local_road_shoulder_is_narrower_than_global_planning_shoulder():
    layout_path = (
        PACKAGE_ROOT.parent
        / "ugvcar_description"
        / "config"
        / "campus_layout.yaml"
    )
    with layout_path.open(encoding="utf-8") as stream:
        navigation = yaml.safe_load(stream)["navigation"]

    assert navigation["local_keepout_edge_margin"] == 0.65
    assert navigation["dynamic_keepout_edge_margin"] == 0.0
    assert navigation["dynamic_recovery_margin"] == 0.30
    assert (
        navigation["local_keepout_edge_margin"]
        < navigation["keepout_edge_margin"]
    )
    assert (
        navigation["dynamic_keepout_edge_margin"]
        < navigation["local_keepout_edge_margin"]
    )


def test_dynamic_mask_has_a_narrow_soft_corner_recovery_strip():
    mask_path = PACKAGE_ROOT / "maps" / "campus_dynamic_keepout_mask.pgm"
    raw = mask_path.read_bytes()
    magic, dimensions, maximum, pixels = raw.split(b"\n", 3)
    width, height = (int(value) for value in dimensions.split())
    assert magic == b"P5"
    assert int(maximum) == 255

    resolution = 0.1
    origin_x, origin_y = -90.0, -80.0
    world_x, world_y = 38.0, -22.9
    column = int(round((world_x - origin_x) / resolution))
    row = int(round(height - 1 - (world_y - origin_y) / resolution))
    pixel = pixels[row * width + column]

    assert 0 < pixel < 250


def test_stage_six_selects_sensor_driven_dynamic_replanning():
    launch_path = PACKAGE_ROOT / "launch" / "campus_navigation.launch.py"
    launch_text = launch_path.read_text(encoding="utf-8")

    assert 'LaunchConfiguration("dynamic_obstacles")' in launch_text
    assert 'LaunchConfiguration("robot_radius")' in launch_text
    assert 'LaunchConfiguration("local_keepout_mask")' in launch_text
    assert '"dynamic_keepout_mask"' in launch_text
    assert "dwb_core::DWBLocalPlanner" in launch_text
    assert "ugvcar_navigation2/DStarLitePlanner" in launch_text
    assert "nav2_smac_planner/SmacPlanner2D" in launch_text
    assert "RegulatedPurePursuitController" in launch_text
    assert "dynamic_controller_plugin" in launch_text
    assert "dynamic_controller_frequency" in launch_text
    assert "dynamic_local_inflation_radius" in launch_text
    assert "dynamic_planner_tolerance" in launch_text
    assert "controller_server.ros__parameters.controller_frequency" in launch_text
    assert "dynamic_global_planner_plugin" in launch_text
    assert "dynamic_failure_tolerance" in launch_text
    assert "dynamic_progress_allowance" in launch_text
    assert '"3.0 if \'"' in launch_text
    assert '"25.0 if \'"' in launch_text
    assert '"90.0 if \'"' not in launch_text
    assert '"60.0 if \'"' not in launch_text
    assert "dynamic_global_update_frequency" in launch_text
    assert "dynamic_global_publish_frequency" in launch_text
    assert "dynamic_global_scan_enabled" in launch_text
    assert "dynamic_goal_yaw_tolerance" in launch_text
    assert "dynamic_curve_minimum_speed" in launch_text
    assert "dynamic_slowdown_ratio" in launch_text
    assert "dynamic_controller_collision_horizon" in launch_text
    assert "dynamic_controller_collision_detection" in launch_text
    assert "dynamic_emergency_stop_enabled" in launch_text
    global_scan_expression = launch_text.split(
        "dynamic_global_scan_enabled", 2
    )[1].split("])", 1)[0]
    controller_collision_expression = launch_text.split(
        "dynamic_controller_collision_detection", 2
    )[1].split("])", 1)[0]
    emergency_stop_expression = launch_text.split(
        "dynamic_emergency_stop_enabled", 2
    )[1].split("])", 1)[0]
    assert '"False if \'"' in global_scan_expression
    assert '"False if \'"' in controller_collision_expression
    assert '"True"' in emergency_stop_expression
    assert "selected_nav_to_pose_bt" in launch_text
    assert "selected_global_keepout_mask" in launch_text
    assert "selected_local_keepout_mask" in launch_text
    assert "navigate_to_pose_dynamic_replanning.xml" in launch_text
    assert "dynamic_obstacle_predictor.py" in launch_text
    predictor_text = (
        PACKAGE_ROOT / "scripts" / "dynamic_obstacle_predictor.py"
    ).read_text(encoding="utf-8")
    assert "if not next_plan:" in predictor_text
    assert "Treating that as a new mission" in predictor_text
    assert 'declare_parameter("response_time", 5.00)' in predictor_text
    assert 'declare_parameter("guard_sample_spacing", 0.75)' in predictor_text
    assert 'declare_parameter("maximum_guard_length", 16.00)' in predictor_text
    assert 'declare_parameter("guard_side_offset", 1.60)' in predictor_text
    assert 'declare_parameter("safety_margin", 0.30)' in predictor_text
    assert 'declare_parameter("risk_clear_confirmations", 5)' in predictor_text
    assert 'declare_parameter("risk_lost_confirmations", 30)' in predictor_text
    assert 'declare_parameter("risk_release_distance", 3.50)' in predictor_text
    assert 'declare_parameter("enable_near_escape_gate", False)' in predictor_text
    assert "/ugv/tracked_dynamic_obstacles" in predictor_text
    assert "DynamicObstacleArray" in predictor_text
    params_text = (
        PACKAGE_ROOT / "config" / "nav2_params.yaml"
    ).read_text(encoding="utf-8")
    assert "nav2_dynamic_obstacle_cleared_condition_bt_node" in params_text
    condition_text = (
        PACKAGE_ROOT / "src" / "dynamic_obstacle_cleared_condition.cpp"
    ).read_text(encoding="utf-8")
    assert '"DynamicObstacleChanged"' in condition_text
    assert "active_threats != last_active_threats_" in condition_text
    assert "settle_ticks_remaining_" in condition_text
    assert "nav2_collision_monitor" in launch_text
    assert "lifecycle_manager_collision_monitor" in launch_text
    assert "global_obstacle_marking" not in launch_text
    assert '"obstacle_layer.predicted_scan.marking"' not in launch_text
    assert launch_text.count(
        '"obstacle_layer.predicted_points.marking"'
    ) == 1
    assert launch_text.count('"obstacle_layer.scan.marking"') == 1
    assert '"robot_radius": robot_radius' in launch_text
    assert "campus_local_keepout_mask.yaml" in launch_text
    assert "campus_dynamic_keepout_mask.yaml" in launch_text
    assert "/local_costmap_filter_info" in launch_text
    assert "/global_costmap_filter_info" in launch_text
    assert launch_text.count("ros__parameters.robot_radius") == 2
    assert '"update_frequency"' in launch_text
    assert '"publish_frequency"' in launch_text

    dynamic_bt = (
        PACKAGE_ROOT
        / "behavior_trees"
        / "navigate_to_pose_dynamic_replanning.xml"
    ).read_text(encoding="utf-8")
    assert '<RateController hz="1.0"' in dynamic_bt
    assert "<AlwaysSuccess/>" in dynamic_bt
    assert "ClearGlobalCostmap-Context" not in dynamic_bt

    cooperative_launch = (
        PACKAGE_ROOT.parent
        / "cooperative_delivery"
        / "launch"
        / "cooperative_delivery.launch.py"
    ).read_text(encoding="utf-8")
    assert "combined_robot_radius" in cooperative_launch
    assert 'combined_robot_radius = "0.60"' in cooperative_launch

    room_launch = (
        PACKAGE_ROOT / "launch" / "navigation2.launch.py"
    ).read_text(encoding="utf-8")
    assert "nav2_collision_monitor" in room_launch
    assert "lifecycle_manager_collision_monitor" in room_launch

    control_xacro = (
        PACKAGE_ROOT.parent
        / "ugvcar_description"
        / "urdf"
        / "ugvcar"
        / "ugvcar.ros2_control.xacro"
    ).read_text(encoding="utf-8")
    assert "cmd_vel_unstamped:=/cmd_vel_safe" in control_xacro
