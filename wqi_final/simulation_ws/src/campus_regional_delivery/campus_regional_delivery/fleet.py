"""Scene-local adapters around the proven second-car model and Nav2 profile."""
from copy import deepcopy
import importlib.util
import math
import re
import xml.etree.ElementTree as ET

from cooperative_delivery.relay_vehicle import second_vehicle_model, second_navigation_parameters


NAVIGATION_COMPONENTS={
    "controller_server":("nav2_controller","nav2_controller::ControllerServer"),
    "smoother_server":("nav2_smoother","nav2_smoother::SmootherServer"),
    "planner_server":("nav2_planner","nav2_planner::PlannerServer"),
    "behavior_server":("nav2_behaviors","behavior_server::BehaviorServer"),
    "bt_navigator":("nav2_bt_navigator","nav2_bt_navigator::BtNavigator"),
    "waypoint_follower":("nav2_waypoint_follower","nav2_waypoint_follower::WaypointFollower"),
    "velocity_smoother":("nav2_velocity_smoother","nav2_velocity_smoother::VelocitySmoother"),
    "collision_monitor":("nav2_collision_monitor","nav2_collision_monitor::CollisionMonitor"),
}


def select_fleet(vehicles, namespace="all", with_uav=True, start_mission_manager=True):
    if namespace == "all":
        return vehicles
    if with_uav or start_mission_manager:
        raise ValueError("Single-UGV navigation requires uav:=false and start_mission_manager:=false")
    selected = {key: vehicle for key, vehicle in vehicles.items() if vehicle["namespace"] == namespace}
    if not selected:
        raise ValueError("Unknown regional vehicle namespace: " + namespace)
    return selected


def renamed(value, namespace):
    if value == "ugv_b":
        return namespace
    if value.startswith("ugv_b/"):
        return namespace + value[len("ugv_b"):]
    if value.startswith("/ugv_b/") or value == "/ugv_b":
        return "/" + namespace + value[len("/ugv_b"):]
    return value


def fleet_model(original, namespace):
    if not re.fullmatch(r"ugv_0[0123]", namespace):
        raise ValueError("Unexpected regional vehicle namespace")
    root = ET.fromstring(second_vehicle_model(original))
    for element in root.iter():
        for name, value in list(element.attrib.items()):
            element.set(name, renamed(value, namespace))
        if element.text:
            element.text = renamed(element.text, namespace)
    return ET.tostring(root, encoding="unicode")


def fleet_parameters(original, navigation_share, namespace, peers, map_resolution=.2, dynamic=False):
    map_resolution = float(map_resolution)
    if not math.isfinite(map_resolution) or not .05 <= map_resolution <= .2:
        raise ValueError("Regional map resolution must be in [0.05, 0.2]")
    def rewrite(value):
        if isinstance(value, dict):
            return {key: rewrite(item) for key, item in value.items()}
        if isinstance(value, list):
            return [rewrite(item) for item in value]
        return renamed(value, namespace) if isinstance(value, str) else value
    data = rewrite(second_navigation_parameters(original, navigation_share,dynamic=dynamic)["ugv_b"])
    local = data["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = data["global_costmap"]["global_costmap"]["ros__parameters"]
    # Budget a coarse-cell diagonal and tracking error for Smac's centre path.
    # Keep the physical local footprint and RPP collision checks unchanged.
    padding = math.ceil((math.sqrt(2)*map_resolution+.10)/local["resolution"])*local["resolution"]
    global_["robot_radius"] = local["robot_radius"] + padding
    global_["inflation_layer"]["inflation_radius"] = max(
        global_["inflation_layer"]["inflation_radius"], global_["robot_radius"] + padding)
    if dynamic:
        # Preserve physical envelope/speed. Measured occupancy, not future
        # collision strips, belongs to the global grid; DWB owns TTC scoring.
        global_["update_frequency"],global_["publish_frequency"]=5.,2.
        global_["track_unknown_space"]=False
        for params in (local,global_):
            params["inflation_layer"]["inflate_unknown"]=False
            params["inflation_layer"]["inflate_around_unknown"]=False
        controller=data["controller_server"]["ros__parameters"]
        controller["failure_tolerance"]=3.
        controller["FollowPath"]["vtheta_samples"]=17
        # An 8 s constant-curvature rollout can reject every lane-change arc
        # on campus roads. Use receding 3 s controls; the predictive critic
        # still checks 8 s, holding the rollout endpoint for the remainder.
        # Its 0.30 m margin exceeds v^2/(2*0.65)=0.124 m braking travel.
        controller["FollowPath"]["sim_time"]=3.
        # GoalAlign projects the final goal by 0.7 m along its final yaw.
        # At a perpendicular delivery door this competes with PathHeading
        # before XY arrival and can make zero speed a local optimum. Keep
        # path/progress critics; RotateToGoal handles yaw after XY arrival.
        controller["FollowPath"]["GoalAlign.scale"]=0.
        # A bounded soft preference only among collision-free candidates
        # prevents indefinite pure rotation near moving traffic. The 1e6
        # collision term and the final collision monitor still dominate.
        controller["FollowPath"]["PredictiveCollision.motion_preference_weight"]=.00025
        controller["FollowPath"]["PathHeading.lookahead_distance"]=4.
        controller["FollowPath"]["PathHeading.activation_angle"]=.7
        controller["FollowPath"]["PathHeading.activation_linear_velocity"]=.05
        data["collision_monitor"]["ros__parameters"]["SlowdownZone"]["slowdown_ratio"]=.75
    for name in ("local_costmap", "global_costmap"):
        params = data[name][name]["ros__parameters"]
        params["plugins"].remove("peer_vehicle")
        del params["peer_vehicle"]
        for peer in peers:
            if peer == namespace:
                continue
            layer = "peer_" + peer
            params["plugins"].insert(params["plugins"].index("inflation_layer"), layer)
            params[layer] = {"plugin":"cooperative_delivery::PeerVehicleLayer",
                             "pose_topic":f"/{peer}/ground_truth/odom", "radius":.60, "timeout":1.0}
    return {namespace:data}


def prepare_uav_model(iris, imu_plugin, description_share, namespaces, output, visualize=False):
    source = description_share / "scripts/prepare_px4_model.py"
    spec = importlib.util.spec_from_file_location("project_px4_model", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.verify_contact_imu_plugin(imu_plugin)
    tree = ET.parse(iris)
    module.inject_project_plugins(tree, docking=True, visualize=visualize, relay_docking=True)
    plugin = tree.getroot().find("model/plugin[@name='ugv_uav_docking']")
    plugin.find("ugvModel").text = namespaces[0]
    plugin.find("ugvLink").text = namespaces[0] + "/base_footprint"
    # The initial aircraft is loaded on the hub vehicle, not awaiting landing.
    # Use the plugin's strict physical capture once at model insertion so an
    # idle, disarmed Iris cannot slide off the deck before an order arrives.
    ET.SubElement(plugin, "autoAttach").text = "true"
    for name in namespaces:
        carrier = ET.SubElement(plugin, "allowedCarrier")
        ET.SubElement(carrier, "model").text = name
        ET.SubElement(carrier, "link").text = name + "/base_footprint"
    ET.indent(tree, space="  ")
    tree.write(output, encoding="unicode", xml_declaration=False)


def fleet_rviz(original, regions, include_uav=True, include_energy=False, include_dynamic=False):
    data = deepcopy(original)
    displays = data["Visualization Manager"]["Displays"]
    kept = [d for d in displays if d["Name"] in (
        "Ground Grid", "Campus Map", "Campus UAV", "UAV Actual Flight Path",
        "UAV Top 3D Lidar", "UAV 3D Safety Sphere")]
    if not include_uav:
        kept = [d for d in kept if "UAV" not in d["Name"]]
    for region in regions.values():
        ns = region["namespace"]
        for original_display in displays:
            if original_display["Name"] not in ("Campus UGV", "UGV Laser", "UGV Local Costmap", "UGV Nav2 Plan"):
                continue
            clone = deepcopy(original_display)
            clone["Name"] = region["vehicle_label"] + " " + clone["Name"]
            for field in ("Description Topic", "Topic", "Update Topic"):
                if field in clone:
                    clone[field]["Value"] = "/" + ns + clone[field]["Value"]
            if "Links" in clone:
                clone["Links"] = {"All Links Enabled":True}
            kept.append(clone)
    kept.append({"Class":"rviz_default_plugins/MarkerArray", "Name":"Regional Depots and Buildings",
        "Enabled":True, "Value":True, "Topic":{"Value":"/regional_delivery/markers", "Depth":1,
        "Durability Policy":"Transient Local", "Reliability Policy":"Reliable", "History Policy":"Keep Last"}})
    kept.append({"Class":"rviz_default_plugins/Path", "Name":"UAV Regional Planned Route",
        "Enabled":True, "Value":True, "Line Style":"Lines", "Line Width":.10,
        "Color":"255; 210; 0", "Topic":{"Value":"/uav/planned_path", "Depth":1,
        "Durability Policy":"Transient Local", "Reliability Policy":"Reliable", "History Policy":"Keep Last"}})
    if not include_uav:
        kept = [d for d in kept if d["Name"] != "UAV Regional Planned Route"]
    if include_energy:
        kept.append({"Class":"rviz_default_plugins/MarkerArray", "Name":"Regional Battery States",
            "Enabled":True, "Value":True, "Topic":{"Value":"/regional_delivery/energy_markers", "Depth":1,
            "Durability Policy":"Transient Local", "Reliability Policy":"Reliable", "History Policy":"Keep Last"}})
    if include_dynamic:
        for display in kept:
            if display["Name"]=="UAV Regional Planned Route":
                display["Color"]="160; 160; 160"
        kept.append({"Class":"rviz_default_plugins/MarkerArray","Name":"Dynamic Ground and Air Traffic",
            "Enabled":True,"Value":True,"Topic":{"Value":"/regional_delivery/dynamic_markers","Depth":1,
            "Durability Policy":"Transient Local","Reliability Policy":"Reliable","History Policy":"Keep Last"}})
        if include_uav:
            kept.append({"Class":"rviz_default_plugins/Path","Name":"UAV Sensor Replanned Route",
                "Enabled":True,"Value":True,"Color":"255; 210; 0","Line Style":"Lines","Line Width":.14,
                "Topic":{"Value":"/uav/replanned_path","Depth":1,"Durability Policy":"Transient Local",
                         "Reliability Policy":"Reliable","History Policy":"Keep Last"}})
    data["Visualization Manager"]["Displays"] = kept
    data["Visualization Manager"]["Global Options"]["Frame Rate"] = 10
    return data
