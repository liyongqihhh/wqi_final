"""Model-only UGV camera housing removal and internal IMU mounting checks."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import xacro


def robot_model():
    share = Path(get_package_share_directory('ugvcar_description'))
    path = share / 'urdf/ugvcar/ugvcar.urdf.xacro'
    return ET.fromstring(xacro.process_file(str(path)).toxml())


def test_camera_housing_removed_without_removing_data_frames_or_sensor():
    robot = robot_model()
    camera = robot.find("link[@name='camera_link']")
    assert camera is not None
    assert camera.find('visual') is None
    assert camera.find('collision') is None
    assert camera.find('inertial') is not None
    assert robot.find("link[@name='camera_optical_link']") is not None
    assert robot.find("joint[@name='camera_joint']") is not None
    assert robot.find("joint[@name='camera_optical_joint']") is not None
    sensor = robot.find("gazebo[@reference='camera_link']/sensor[@type='depth']")
    assert sensor is not None
    assert sensor.findtext('plugin/frame_name') == 'camera_optical_link'


def test_complete_imu_cube_is_inside_the_lower_chassis_half():
    robot = robot_model()
    cylinder = robot.find("link[@name='base_link']/collision/geometry/cylinder")
    half_height = float(cylinder.get('length')) / 2
    radius = float(cylinder.get('radius'))
    origin = robot.find("joint[@name='imu_joint']/origin")
    center = [float(v) for v in origin.get('xyz').split()]
    assert center == [0., 0., -.03]
    imu = robot.find("link[@name='imu_link']")
    for name in ('visual', 'collision'):
        part = imu.find(name)
        size = [float(v) for v in part.find('geometry/box').get('size').split()]
        offset = [float(v) for v in part.find('origin').get('xyz').split()]
        position = [c + d for c, d in zip(center, offset)]
        assert -half_height <= position[2] - size[2] / 2
        assert position[2] + size[2] / 2 <= 0
        assert math.hypot(abs(position[0]) + size[0] / 2,
                          abs(position[1]) + size[1] / 2) < radius
    sensor = robot.find("gazebo[@reference='imu_link']/sensor[@type='imu']")
    assert sensor is not None
    assert float(sensor.findtext('update_rate')) == 100.


def test_sensor_and_joint_references_still_resolve_to_existing_links():
    robot = robot_model()
    links = {link.get('name') for link in robot.findall('link')}
    for joint in robot.findall('joint'):
        assert joint.find('parent').get('link') in links
        assert joint.find('child').get('link') in links
    for gazebo in robot.findall('gazebo'):
        if gazebo.get('reference'):
            assert gazebo.get('reference') in links
