from copy import deepcopy
import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from campus_regional_delivery.fleet import fleet_parameters
from campus_regional_delivery.runtime_profile import (
    energy_for_profile,fast_profile,model_for_profile,world_for_profile)


def test_profile_is_explicit_and_unknown_values_fail():
    assert not fast_profile("stable")
    assert fast_profile("fast_test")
    with pytest.raises(ValueError):fast_profile("turbo")


def test_uav_launch_sets_the_parameter_name_actually_declared_by_controller():
    root=Path(__file__).parents[2]/"uav_control"
    tree=ast.parse((root/"launch/uav_control.launch.py").read_text())
    keys={k.value for node in ast.walk(tree) if isinstance(node,ast.Dict)
        for k in node.keys if isinstance(k,ast.Constant)}
    controller=(root/"uav_control/flight_action_server.py").read_text()
    assert 'declare_parameter("dynamic_maximum_setpoint_step"' in controller
    assert "dynamic_maximum_setpoint_step" in keys
    assert "dynamic_setpoint_max_step" not in keys


@pytest.mark.parametrize("dynamic",[False,True])
def test_fast_speed_braking_and_smoother_agree_and_original_is_preserved(dynamic):
    nav=Path(get_package_share_directory("ugvcar_navigation2"))
    original=yaml.safe_load((nav/"config/nav2_params.yaml").read_text())
    frozen=deepcopy(original)
    def parameters(profile):
        return fleet_parameters(original,nav,"ugv_01",["ugv_00","ugv_01"],
            dynamic=dynamic,execution_profile=profile)["ugv_01"]
    stable,fast=parameters("stable"),parameters("fast_test")
    assert original==frozen
    assert stable["controller_server"]["ros__parameters"]["FollowPath"]["max_vel_x"]==.40
    follow=fast["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["max_vel_x"]==.70
    assert follow["max_vel_theta"]==.65
    assert follow["PredictiveCollision.safety_margin"]>=.7**2/(2*.8)+.7*.2
    smooth=fast["velocity_smoother"]["ros__parameters"]
    assert smooth["max_velocity"][0]==follow["max_vel_x"]
    assert smooth["max_accel"][0]==follow["acc_lim_x"]
    assert smooth["max_decel"][0]==follow["decel_lim_x"]
    for name in ("local_costmap","global_costmap"):
        assert fast[name][name]["ros__parameters"]["robot_radius"]>=stable[name][name]["ros__parameters"]["robot_radius"]
        assert fast[name][name]["ros__parameters"]["resolution"]==stable[name][name]["ros__parameters"]["resolution"]
    assert fast["collision_monitor"]["ros__parameters"]["StopZone"]["points"][0]>=.6+.7**2/(2*.8)+.7*.2


def test_only_camera_rates_change_and_all_collisions_remain():
    xml='<robot><link name="base"><collision name="body"/></link><gazebo><sensor type="ray"><update_rate>10</update_rate></sensor><sensor type="imu"><update_rate>100</update_rate></sensor><sensor type="depth"><update_rate>10</update_rate></sensor></gazebo></robot>'
    assert model_for_profile(xml,"stable")==xml
    root=ET.fromstring(model_for_profile(xml,"fast_test"))
    assert [s.findtext("update_rate") for s in root.findall(".//sensor")]==["10","100","2"]
    assert root.find(".//collision[@name='body']") is not None


@pytest.mark.parametrize("rate",[250,500])
def test_fast_world_preserves_models_and_uses_px4_supported_lockstep(tmp_path,rate):
    source=Path(__file__).parents[1]/"assets/campus_regional.world"
    assert world_for_profile(source,tmp_path/"unused.world","stable")==source
    path=world_for_profile(source,tmp_path/"fast.world","fast_test",rate)
    old,new=(ET.parse(p).getroot().find("world") for p in (source,path))
    assert float(new.findtext("physics/max_step_size"))==1./rate
    assert float(new.findtext("physics/real_time_update_rate"))==rate
    assert [ET.tostring(m) for m in old.findall("model")]==[ET.tostring(m) for m in new.findall("model")]
    assert new.findtext("scene/shadows")=="false"
    assert new.findtext("gui/plugin[@name='regional_view_rate']/fps")=="10"
    with pytest.raises(ValueError):world_for_profile(source,tmp_path/"bad.world","fast_test",100)


def test_physical_drive_limit_can_deliver_configured_braking_without_changing_geometry():
    xml='<robot><link name="base"><collision name="body"/></link><gazebo><plugin filename="libgazebo_ros_diff_drive.so"><max_wheel_acceleration>0.45</max_wheel_acceleration></plugin></gazebo></robot>'
    root=ET.fromstring(model_for_profile(xml,"fast_test"))
    assert float(root.findtext(".//max_wheel_acceleration"))==.8
    assert ET.tostring(root.find("link"))==ET.tostring(ET.fromstring(xml).find("link"))


def test_planning_energy_speed_updates_unique_field_without_mutating_input():
    original={"/energy":{"ros__parameters":{"planning_speed_mps":.4,"capacity_wh":300}}}
    updated=energy_for_profile(original,"planning_speed_mps",.55)
    assert original["/energy"]["ros__parameters"]["planning_speed_mps"]==.4
    assert updated["/energy"]["ros__parameters"]=={"planning_speed_mps":.55,"capacity_wh":300}
    with pytest.raises(ValueError):energy_for_profile(original,"missing",.55)
