"""Opt-in throughput settings with a PX4-supported physics cadence."""
from copy import deepcopy
import xml.etree.ElementTree as ET


def fast_profile(name):
    if name not in ("stable", "fast_test"):
        raise ValueError("execution_profile must be stable or fast_test")
    return name == "fast_test"


def tune_navigation(data, profile):
    if not fast_profile(profile):
        return
    controller = data["controller_server"]["ros__parameters"]["FollowPath"]
    controller.update(desired_linear_vel=.70, max_vel_x=.70, max_speed_xy=.70,
                      acc_lim_x=.35, decel_lim_x=-.80, acc_lim_theta=.80, decel_lim_theta=-.80)
    # Braking travel plus one 0.2 s reaction window, rounded upward.
    controller["PredictiveCollision.safety_margin"] = .50
    smoother = data["velocity_smoother"]["ros__parameters"]
    smoother["max_velocity"][0] = .70
    smoother["max_accel"][0] = .35
    smoother["max_decel"][0] = -.80
    for name in ("local_costmap", "global_costmap"):
        costmap = data[name][name]["ros__parameters"]
        costmap["robot_radius"] = max(costmap["robot_radius"],1.10)
        costmap["inflation_layer"]["inflation_radius"] = max(
            costmap["inflation_layer"]["inflation_radius"],costmap["robot_radius"]+.10)
    monitor = data["collision_monitor"]["ros__parameters"]
    monitor["StopZone"]["points"] = [1.10,.55,1.10,-.55,-.60,-.55,-.60,.55]
    monitor["SlowdownZone"]["points"] = [2.20,1.,2.20,-1.,-.60,-1.,-.60,1.]


def tune_cameras(root, profile):
    if fast_profile(profile):
        for sensor in root.findall(".//sensor"):
            if sensor.get("type") in ("camera", "depth", "depth_camera"):
                rate = sensor.find("update_rate")
                if rate is not None:
                    rate.text = "2"


def model_for_profile(xml, profile):
    if not fast_profile(profile):
        return xml
    root = ET.fromstring(xml)
    tune_cameras(root, profile)
    for limit in root.findall(".//plugin[@filename='libgazebo_ros_diff_drive.so']/max_wheel_acceleration"):
        # Command acceleration remains 0.35; allow the configured 0.8 braking.
        limit.text = "0.80"
    return ET.tostring(root, encoding="unicode")


def world_for_profile(source, destination, profile, physics_rate=250):
    if not fast_profile(profile):
        return source
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    if physics_rate not in (250,500):
        raise ValueError("PX4 fast physics rate must be 250 or 500 Hz")
    # PX4's own Gazebo lockstep plugin explicitly supports both reciprocal
    # pairs: 250 Hz/0.004 s and 500 Hz/0.002 s. No clock/energy scaling.
    world.find("physics/max_step_size").text = str(1./physics_rate)
    world.find("physics/real_time_update_rate").text = str(physics_rate)
    gui = world.find("gui")
    if gui is None:
        gui = ET.SubElement(world,"gui")
    plugin = ET.SubElement(gui,"plugin",name="regional_view_rate",filename="libregional_view_rate.so")
    ET.SubElement(plugin,"fps").text = "10"
    scene = world.find("scene")
    if scene is None:
        scene = ET.SubElement(world, "scene")
    shadow = scene.find("shadows")
    if shadow is None:
        shadow = ET.SubElement(scene, "shadows")
    shadow.text = "false"
    for light in world.findall("light"):
        shadow = light.find("cast_shadows")
        if shadow is not None:
            shadow.text = "false"
    tree.write(destination, encoding="unicode")
    return destination


def energy_for_profile(data, key, value):
    result = deepcopy(data)
    changed = []
    def visit(item):
        if not isinstance(item, dict):
            return
        if key in item:
            item[key] = value
            changed.append(key)
        for child in item.values():
            visit(child)
    visit(result)
    if len(changed) != 1:
        raise ValueError("Expected exactly one energy setting: " + key)
    return result
