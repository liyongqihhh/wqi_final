import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest


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
