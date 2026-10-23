"""Generate namespaced B-car assets without editing the original UGV assets."""

from copy import deepcopy
import math
import xml.etree.ElementTree as ET

from cooperative_delivery.px4_carrier_model import add_physical_deck


def second_vehicle_remappings(predictor=False):
    topics = ["/scan", "/received_global_plan", "/scan_dynamic_predictions",
        "/points_dynamic_predictions", "/ugv/dynamic_replanning/status",
        "/ugv/dynamic_replanning/tracked_obstacles", "/ugv/tracked_dynamic_obstacles",
        "/ugv/dstar_lite/status", "/ugv/predictive_dwa/status"]
    # Only the sensor predictor consumes ego ground truth. The fleet costmap must
    # continue receiving A's root topic, not B's own pose through a global remap.
    if predictor:
        topics.append("/ground_truth/odom")
    return [(topic, "/ugv_b" + topic) for topic in topics]


def with_peer_footprint(data, peer_topic):
    """Only relay-generated parameters include the other vehicle's physical envelope."""
    result = deepcopy(data)
    for name in ("local_costmap", "global_costmap"):
        params = result[name][name]["ros__parameters"]
        plugins = params["plugins"]
        plugins.insert(plugins.index("inflation_layer"), "peer_vehicle")
        params["peer_vehicle"] = {
            "plugin": "cooperative_delivery::PeerVehicleLayer",
            "pose_topic": peer_topic,
            "radius": .60,
            "timeout": 1.0,
        }
    return result


def second_vehicle_model(robot_xml):
    root = ET.fromstring(add_physical_deck(robot_xml))
    names = {element.get("name") for tag in ("link", "joint") for element in root.findall(tag)}
    prefix = "ugv_b/"
    for element in root.iter():
        for attribute in ("name", "link", "reference", "joint"):
            value = element.get(attribute)
            if value in names:
                element.set(attribute, prefix + value)
        if element.tag in ("frame_name", "body_name") and element.text in names:
            element.text = prefix + element.text
    root.set("name", "ugv_b")
    for control in root.findall("ros2_control"):
        root.remove(control)
    for gazebo in root.findall("gazebo"):
        for control in gazebo.findall("plugin[@filename='libgazebo_ros2_control.so']"):
            gazebo.remove(control)
    gazebo = ET.SubElement(root, "gazebo")
    gazebo.append(ET.fromstring("""
<plugin name="ugv_b_drive" filename="libgazebo_ros_diff_drive.so">
  <ros><namespace>/ugv_b</namespace><remapping>cmd_vel:=drive_cmd</remapping></ros>
  <left_joint>ugv_b/left_wheel_joint</left_joint>
  <right_joint>ugv_b/right_wheel_joint</right_joint>
  <wheel_separation>0.226</wheel_separation><wheel_diameter>0.064</wheel_diameter>
  <max_wheel_torque>0.2</max_wheel_torque><max_wheel_acceleration>0.45</max_wheel_acceleration>
  <update_rate>50</update_rate><odometry_source>0</odometry_source>
  <publish_odom>true</publish_odom><publish_odom_tf>true</publish_odom_tf>
  <publish_wheel_tf>false</publish_wheel_tf>
  <odometry_frame>ugv_b/odom</odometry_frame><robot_base_frame>ugv_b/base_footprint</robot_base_frame>
</plugin>"""))
    gazebo.append(ET.fromstring("""
<plugin name="ugv_b_joints" filename="libgazebo_ros_joint_state_publisher.so">
  <ros><namespace>/ugv_b</namespace></ros><update_rate>50</update_rate>
  <joint_name>ugv_b/left_wheel_joint</joint_name><joint_name>ugv_b/right_wheel_joint</joint_name>
</plugin>"""))
    for plugin in root.findall(".//plugin[@filename]"):
        ros = plugin.find("ros")
        if ros is None:
            ros = ET.SubElement(plugin, "ros")
        namespace = ros.find("namespace")
        if namespace is None:
            namespace = ET.SubElement(ros, "namespace")
        namespace.text = "/ugv_b"
        for remapping in ros.findall("remapping"):
            left, right = remapping.text.split(":=", 1)
            left = "/ugv_b" + left if left.startswith("/") else left
            right = "/ugv_b" + right if right.startswith("/") else right
            remapping.text = f"{left}:={right}"
    return ET.tostring(root, encoding="unicode")


def relay_runtime_parameters(data, controller_frequency=10.0):
    frequency = float(controller_frequency)
    if not math.isfinite(frequency) or not 5.0 <= frequency <= 20.0:
        raise ValueError("Relay controller frequency must be between 5 and 20 Hz")
    result = deepcopy(data)
    result["controller_server"]["ros__parameters"]["controller_frequency"] = frequency
    result["bt_navigator"]["ros__parameters"]["bt_loop_duration"] = max(50, round(1000 / frequency))
    return result


def second_navigation_parameters(data, navigation_share, dynamic=False, controller_frequency=10.0):
    def rewrite(value):
        if isinstance(value, dict):
            return {key: rewrite(item) for key, item in value.items()}
        if isinstance(value, list):
            return [rewrite(item) for item in value]
        if isinstance(value, str):
            if value in ("base_link", "base_footprint", "odom"):
                return "ugv_b/" + value
            if value.startswith("/") and not value.startswith(str(navigation_share)):
                return "/ugv_b" + value
        return value

    result = rewrite(relay_runtime_parameters(data, controller_frequency))
    bt = result["bt_navigator"]["ros__parameters"]
    filename = ("navigate_to_pose_dynamic_replanning.xml" if dynamic
                else "navigate_to_pose_if_path_invalid.xml")
    bt["default_nav_to_pose_bt_xml"] = str(navigation_share / "behavior_trees" / filename)
    local = result["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = result["global_costmap"]["global_costmap"]["ros__parameters"]
    for params, topic in ((local, "/local_costmap_filter_info"),
                          (global_, "/global_costmap_filter_info")):
        params["robot_radius"] = .60
        params["keepout_filter"]["filter_info_topic"] = topic
    controller = result["controller_server"]["ros__parameters"]
    global_["static_layer"]["map_topic"] = "/map"
    global_["obstacle_layer"]["predicted_points"]["marking"] = bool(dynamic)
    global_["obstacle_layer"]["predicted_scan"]["clearing"] = bool(dynamic)
    if dynamic:
        controller["controller_frequency"] = 10.0
        controller["progress_checker"]["movement_time_allowance"] = 25.0
        controller["FollowPath"]["plugin"] = "dwb_core::DWBLocalPlanner"
        controller["FollowPath"]["PredictiveCollision.robot_radius"] = .60
        controller["general_goal_checker"]["yaw_goal_tolerance"] = 3.14159
        result["planner_server"]["ros__parameters"]["GridBased"]["plugin"] = (
            "ugvcar_navigation2/DStarLitePlanner")
        global_["obstacle_layer"]["scan"]["marking"] = False
        global_["obstacle_layer"]["scan"]["clearing"] = False
        local["inflation_layer"]["inflation_radius"] = .65
    return {"ugv_b": with_peer_footprint(result, "/ground_truth/odom")}
