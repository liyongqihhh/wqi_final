import importlib.util
from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import xacro
import yaml


SCRIPT = Path(__file__).parents[1] / "scripts" / "prepare_px4_model.py"


def _module():
    spec = importlib.util.spec_from_file_location("prepare_px4_model", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_injects_project_interfaces_without_removing_px4_plugins():
    source = ET.ElementTree(ET.fromstring("""
      <sdf version="1.6"><model name="iris"><link name="base_link"/>
      <plugin name="mavlink_interface" filename="libgazebo_mavlink_interface.so"/>
      </model></sdf>"""))
    tree = _module().inject_project_plugins(source, docking=True, visualize=False)
    xml = ET.tostring(tree.getroot(), encoding="unicode")
    for token in (
        "libgazebo_mavlink_interface.so",
        "libgazebo_ros_p3d.so",
        "libugv_uav_docking_plugin.so",
        "libgazebo_ros_ray_sensor.so",
        "lidar/points",
        "range_raw/rear_down",
        "libgazebo_ros_camera.so",
    ):
        assert token in xml
    assert "<visualize>false</visualize>" in xml


def test_range_mounts_match_tf_and_rays_do_not_hit_the_px4_body():
    from scipy.spatial.transform import Rotation
    import numpy as np

    source = ET.ElementTree(ET.fromstring(
        '<sdf version="1.6"><model name="iris"><link name="base_link"/></model></sdf>'))
    model = _module().inject_project_plugins(source, docking=False).getroot().find('model')
    robot = ET.fromstring(xacro.process_file(str(SCRIPT.parents[1]/'urdf/uav.urdf.xacro')).toxml())
    half = np.array((.235, .235, .055))
    for sensor in model.findall("link[@name='base_link']/sensor"):
        if not sensor.get('name', '').endswith('_range'):
            continue
        pose = [float(v) for v in sensor.findtext('pose').split()]
        joint = robot.find(f"joint[@name='{sensor.get('name')}_joint']/origin")
        assert [float(v) for v in joint.get('xyz').split()] == pytest.approx(pose[:3])
        assert [float(v) for v in joint.get('rpy').split()] == pytest.approx(pose[3:])
        direction = Rotation.from_euler('xyz', pose[3:]).apply((1., 0., 0.))
        ray = np.array(pose[:3])+np.linspace(.05, 4., 400)[:, None]*direction
        assert not np.any(np.all(np.abs(ray) <= half, axis=1)), sensor.get('name')


def test_contact_aware_imu_is_only_enabled_for_the_carrier():
    for docking in (False, True):
        tree = ET.ElementTree(ET.fromstring('''
          <sdf version="1.6"><model name="iris"><link name="base_link"/>
          <plugin name="iris_imu" filename="libgazebo_imu_plugin.so">
            <linkName>/imu_link</linkName><imuTopic>/imu</imuTopic>
            <accelerometerNoiseDensity>0.004</accelerometerNoiseDensity>
          </plugin></model></sdf>'''))
        _module().inject_project_plugins(tree, docking=docking)
        plugin = tree.getroot().find("model/plugin")
        assert plugin.findtext("contactAwareAcceleration") == ("true" if docking else None)
        assert plugin.findtext("contactAccelerationTimeConstant") == ("0.02" if docking else None)
        assert plugin.findtext("linkName") == "/imu_link"
        assert plugin.findtext("imuTopic") == "/imu"
        assert plugin.findtext("accelerometerNoiseDensity") == "0.004"


def test_unpatched_or_missing_plugin_is_rejected(tmp_path):
    library = tmp_path / "libgazebo_imu_plugin.so"
    for contents in (None, b"native IMU without compatibility option", b"contactAwareAcceleration"):
        if contents is not None:
            library.write_bytes(contents)
        with pytest.raises(RuntimeError, match="contact-aware Gazebo IMU"):
            _module().verify_contact_imu_plugin(library)
    library.write_bytes(b"compiled plugin contactAwareAcceleration contactAccelerationTimeConstant")
    _module().verify_contact_imu_plugin(library)


def _normal(element):
    return (element.tag, sorted(element.attrib.items()), (element.text or "").strip(),
            [_normal(child) for child in element])


def _iris_fixture():
    settings = yaml.safe_load((SCRIPT.parents[1]/"config/uav_model.yaml").read_text())["visual"]
    model = ET.fromstring('''<model name="iris"><link name="base_link">
      <inertial><mass>1.5</mass><inertia><ixx>.029125</ixx><iyy>.029125</iyy><izz>.055225</izz></inertia></inertial>
      <collision name="body"><geometry><box><size>.47 .47 .11</size></box></geometry></collision>
      <visual name="detailed_body"><geometry><mesh><uri>model://iris/iris.stl</uri></mesh></geometry></visual>
      </link><plugin name="mavlink_interface" filename="libgazebo_mavlink_interface.so"/></model>''')
    for index, position in enumerate(settings["rotor_positions"]):
        link = ET.SubElement(model, "link", {"name": f"rotor_{index}"})
        ET.SubElement(link, "pose").text = " ".join(str(v) for v in (*position, 0, 0, 0))
        ET.SubElement(ET.SubElement(link, "inertial"), "mass").text = ".005"
        link.append(ET.fromstring('''<collision name="rotor_collision"><geometry><cylinder>
          <radius>.128</radius><length>.005</length></cylinder></geometry></collision>'''))
        link.append(ET.fromstring('<visual name="prop_mesh"><geometry><mesh><uri>prop.dae</uri></mesh></geometry></visual>'))
        joint = ET.SubElement(model, "joint", {"name": f"rotor_{index}_joint", "type": "revolute"})
        ET.SubElement(joint, "parent").text = "base_link"
        ET.SubElement(joint, "child").text = f"rotor_{index}"
    return model


def test_simplification_changes_only_visuals_not_px4_physics():
    model = _iris_fixture()
    before = deepcopy(model)
    _module()._simplify_visuals(model)
    assert not model.findall(".//visual/geometry/mesh")
    base = model.find("link[@name='base_link']")
    assert len(base.findall("visual/geometry/box")) == 1
    assert len([v for v in base.findall("visual") if v.get("name").startswith("simple_arm_")]) == 4
    assert len([v for v in base.findall("visual") if v.get("name").startswith("simple_leg_")]) == 4
    assert len([v for v in base.findall("visual") if v.get("name").startswith("simple_skid_")]) == 2
    assert len(model.findall(".//collision")) == 5
    for root in (model, before):
        for link in root.findall("link"):
            for visual in list(link.findall("visual")):
                link.remove(visual)
    assert _normal(model) == _normal(before)


def test_body_arms_and_landing_gear_remain_inside_existing_contact_box():
    model = _iris_fixture()
    _module()._simplify_visuals(model)
    base = model.find("link[@name='base_link']")
    minimum_z = math.inf
    for visual in base.findall("visual"):
        pose = [float(v) for v in visual.findtext("pose").split()]
        xyz, (roll, pitch, yaw) = pose[:3], pose[3:]
        assert roll == 0
        shape = list(visual.find("geometry"))[0]
        if shape.tag == "box":
            extent = [float(v)/2 for v in shape.findtext("size").split()]
        else:
            axis = [math.sin(pitch)*math.cos(yaw), math.sin(pitch)*math.sin(yaw), math.cos(pitch)]
            radius, length = float(shape.findtext("radius")), float(shape.findtext("length"))
            extent = [abs(a)*length/2+radius*math.sqrt(max(0, 1-a*a)) for a in axis]
        for center, half, bound in zip(xyz, extent, (.235, .235, .055)):
            assert center-half >= -bound-1e-9
            assert center+half <= bound+1e-9
        minimum_z = min(minimum_z, xyz[2]-extent[2])
    assert minimum_z == pytest.approx(-.055)
    for index in range(4):
        rotor = model.find(f"link[@name='rotor_{index}']")
        for name in ("radius", "length"):
            assert float(rotor.findtext("visual/geometry/cylinder/"+name)) == pytest.approx(
                float(rotor.findtext("collision/geometry/cylinder/"+name)))


def test_simple_visuals_are_idempotent():
    model = _iris_fixture()
    module = _module()
    module._simplify_visuals(model)
    before = _normal(model)
    module._simplify_visuals(model)
    assert _normal(model) == before


def test_wrong_motor_mount_cannot_be_hidden_by_the_display_model():
    model = _iris_fixture()
    model.find("link[@name='rotor_0']/pose").text = "1 2 3 0 0 0"
    with pytest.raises(ValueError, match="motor positions"):
        _module()._simplify_visuals(model)


def test_rviz_and_gazebo_share_the_same_simple_visual_dimensions_and_poses():
    root = SCRIPT.parents[1]
    robot = ET.fromstring(xacro.process_file(str(root/"urdf/uav.urdf.xacro")).toxml())
    visuals = robot.findall("link[@name='base_link']/visual")
    by_name = {visual.get("name"): visual for visual in visuals}
    assert len(by_name) == 15
    model = _iris_fixture()
    _module()._simplify_visuals(model)
    for link in model.findall("link"):
        link_pose = [float(v) for v in link.findtext("pose", "0 0 0 0 0 0").split()]
        for visual in link.findall("visual"):
            displayed = by_name[visual.get("name")]
            shape = list(visual.find("geometry"))[0]
            other = list(displayed.find("geometry"))[0]
            assert shape.tag == other.tag
            if shape.tag == "box":
                assert [float(v) for v in shape.findtext("size").split()] == pytest.approx(
                    [float(v) for v in other.get("size").split()])
            else:
                for name in ("radius", "length"):
                    assert float(shape.findtext(name)) == pytest.approx(float(other.get(name)))
            pose = [float(v) for v in visual.findtext("pose").split()]
            assert [a+b for a, b in zip(link_pose[:3], pose[:3])] == pytest.approx(
                [float(v) for v in displayed.find("origin").get("xyz").split()])
            assert pose[3:] == pytest.approx(
                [float(v) for v in displayed.find("origin").get("rpy", "0 0 0").split()])
