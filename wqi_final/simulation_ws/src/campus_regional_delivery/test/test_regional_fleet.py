from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory,get_package_prefix
from nav_msgs.msg import Odometry
import pytest
from rclpy.time import Time
import xacro
import yaml

from campus_regional_delivery.fleet import fleet_model, fleet_parameters, fleet_rviz, prepare_uav_model
from campus_regional_delivery.localizer import RegionalLocalizer


@pytest.mark.parametrize("namespace",["ugv_00","ugv_01","ugv_02","ugv_03"])
def test_native_vehicle_has_one_namespace_and_physical_deck(namespace):
    share=Path(get_package_share_directory("ugvcar_description"))
    source=xacro.process_file(str(share/"urdf/ugvcar/ugvcar.urdf.xacro")).toxml()
    robot=ET.fromstring(fleet_model(source,namespace))
    assert robot.get("name")==namespace
    assert all(l.get("name").startswith(namespace+"/") for l in robot.findall("link"))
    assert robot.find(f"link[@name='{namespace}/base_link']/collision[@name='px4_carrier_deck_collision']") is not None
    assert robot.find("ros2_control") is None
    for plugin in robot.findall(".//plugin[@filename]"):
        assert plugin.findtext("ros/namespace")=="/"+namespace
    assert "ugv_b/" not in ET.tostring(robot,encoding="unicode")


def test_initial_hub_load_is_physically_locked_without_relaxing_strict_capture(tmp_path):
    iris=tmp_path/"iris.sdf"
    iris.write_text('<sdf version="1.6"><model name="iris"><link name="base_link"/></model></sdf>')
    imu=tmp_path/"imu.so"
    imu.write_bytes(b"contactAwareAcceleration contactAccelerationTimeConstant")
    output=tmp_path/"regional.sdf"
    namespaces=[f"ugv_0{i}" for i in range(4)]
    prepare_uav_model(iris,imu,Path(get_package_share_directory("uav_description")),namespaces,output)
    plugin=ET.parse(output).find("model/plugin[@name='ugv_uav_docking']")
    assert plugin.findtext("autoAttach")=="true"
    assert plugin.findtext("strictDocking")=="true"
    assert plugin.findtext("ugvModel")=="ugv_00"
    assert plugin.findtext("ugvLink")=="ugv_00/base_footprint"
    assert [e.findtext("model") for e in plugin.findall("allowedCarrier")]==namespaces
    assert plugin.find("maximumDockingDistance") is None
    assert "autoAttach" not in iris.read_text()


@pytest.mark.parametrize("outcome",["SIGINT","SIGTERM","callback_error"])
def test_localizer_stops_callbacks_before_destroying_its_context(monkeypatch,outcome):
    import signal
    from rclpy.signals import SignalHandlerOptions
    import campus_regional_delivery.localizer as module
    order=[];handlers={}
    monkeypatch.setattr(module.signal,"signal",lambda s,h:handlers.setdefault(s,h))
    initialize=Mock()
    monkeypatch.setattr(module.rclpy,"init",initialize)
    monkeypatch.setattr(module.rclpy,"ok",lambda:True)
    monkeypatch.setattr(module.rclpy,"shutdown",lambda:order.append("context_shutdown"))
    node=SimpleNamespace(command_pub=SimpleNamespace(publish=lambda m:order.append("zero_command")),
        destroy_node=lambda:order.append("destroy_node"))
    monkeypatch.setattr(module,"RegionalLocalizer",lambda:node)
    def spin_once(**kwargs):
        order.append("callback")
        if outcome=="callback_error":raise RuntimeError("real callback error")
        handlers[getattr(signal,outcome)](None,None)
        assert "context_shutdown" not in order
    executor=SimpleNamespace(add_node=Mock(),spin_once=spin_once,
        shutdown=lambda:order.append("executor_shutdown"))
    monkeypatch.setattr(module,"SingleThreadedExecutor",lambda:executor)
    if outcome=="callback_error":
        with pytest.raises(RuntimeError,match="real callback error"):module.main()
    else:module.main()
    initialize.assert_called_once_with(signal_handler_options=SignalHandlerOptions.NO)
    assert order==["callback","executor_shutdown","zero_command","destroy_node","context_shutdown"]


@pytest.mark.parametrize("namespace",["ugv_00","ugv_01","ugv_02","ugv_03"])
def test_four_nav_stacks_share_only_map_and_filter(namespace):
    share=Path(get_package_share_directory("ugvcar_navigation2"))
    params=yaml.safe_load((share/"config/nav2_params.yaml").read_text())
    before=yaml.safe_dump(params)
    data=fleet_parameters(params,share,namespace,["ugv_00","ugv_01","ugv_02","ugv_03"])[namespace]
    local=data["local_costmap"]["local_costmap"]["ros__parameters"]
    global_=data["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["global_frame"]==namespace+"/odom"
    assert local["robot_base_frame"]==namespace+"/base_link"
    assert local["obstacle_layer"]["scan"]["topic"]=="/"+namespace+"/scan"
    assert global_["static_layer"]["map_topic"]=="/map"
    planner=data["planner_server"]["ros__parameters"]["GridBased"]
    original=params["planner_server"]["ros__parameters"]["GridBased"]
    assert planner["downsample_costmap"]==original["downsample_costmap"]
    assert planner["downsampling_factor"]==original["downsampling_factor"]
    assert yaml.safe_dump(params)==before
    assert local["keepout_filter"]["filter_info_topic"]=="/local_costmap_filter_info"
    assert "peer_vehicle" not in local["plugins"]
    peers=[local[p]["pose_topic"] for p in local["plugins"] if p.startswith("peer_")]
    assert len(peers)==3 and f"/{namespace}/ground_truth/odom" not in peers


@pytest.mark.parametrize("resolution,padding",[(.05,.20),(.1,.25),(.2,.40)])
def test_planning_margin_preserves_execution_footprint_and_controller(resolution,padding):
    share=Path(get_package_share_directory("ugvcar_navigation2"))
    original=yaml.safe_load((share/"config/nav2_params.yaml").read_text())
    before=yaml.safe_dump(original)
    data=fleet_parameters(original,share,"ugv_03",[f"ugv_0{i}" for i in range(4)],
        map_resolution=resolution)["ugv_03"]
    local=data["local_costmap"]["local_costmap"]["ros__parameters"]
    global_=data["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["robot_radius"]==pytest.approx(.60)
    assert global_["robot_radius"]==pytest.approx(.60+padding)
    assert global_["inflation_layer"]["inflation_radius"]==pytest.approx(.60+2*padding)
    assert local["inflation_layer"]==original["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]
    controller=data["controller_server"]["ros__parameters"]["FollowPath"]
    expected=dict(original["controller_server"]["ros__parameters"]["FollowPath"])
    expected["PredictiveCollision.tracks_topic"]="/ugv_03"+expected["PredictiveCollision.tracks_topic"]
    assert controller==expected
    assert controller["use_collision_detection"] is True
    assert data["planner_server"]["ros__parameters"]["GridBased"]["plugin"]=="nav2_smac_planner/SmacPlanner2D"
    assert yaml.safe_dump(original)==before


@pytest.mark.parametrize("resolution",[0.,-.1,.3,float("nan"),float("inf")])
def test_planning_margin_rejects_invalid_map_resolution(resolution):
    with pytest.raises(ValueError,match="Regional map resolution"):
        fleet_parameters({},Path("."),"ugv_03",[],map_resolution=resolution)


def test_conservative_global_footprint_keeps_all_delivery_stops_connected():
    import numpy as np
    from PIL import Image
    from scipy.ndimage import distance_transform_edt,label
    from campus_regional_delivery.layout import RegionalLayout
    folder=Path(__file__).parents[1]
    layout=RegionalLayout(folder/"config/campus_regions.yaml")
    share=Path(get_package_share_directory("ugvcar_navigation2"))
    original=yaml.safe_load((share/"config/nav2_params.yaml").read_text())
    data=fleet_parameters(original,share,"ugv_03",[f"ugv_0{i}" for i in range(4)],
        map_resolution=layout.resolution)["ugv_03"]
    radius=data["global_costmap"]["global_costmap"]["ros__parameters"]["robot_radius"]
    distance=distance_transform_edt(np.asarray(Image.open(folder/"assets/regional_map.pgm"))==255)*layout.resolution
    components,_=label(distance>radius+layout.resolution/math.sqrt(2))
    selected={int(components[layout.cell(pose[:2])]) for pose in layout.all_stops().values()}
    assert 0 not in selected and len(selected)==1


@pytest.mark.parametrize("namespace",["ugv_00","ugv_01","ugv_02","ugv_03"])
def test_localizer_never_publishes_the_other_cars_frame(namespace):
    now=Time(seconds=10)
    node=SimpleNamespace(wheel=Odometry(),broadcaster=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:now),get_namespace=lambda:"/"+namespace)
    truth=Odometry();truth.pose.pose.position.x=4.
    RegionalLocalizer._truth(node,truth)
    sent=node.broadcaster.sendTransform.call_args.args[0]
    assert sent.header.frame_id=="map"
    assert sent.child_frame_id==namespace+"/odom"


def test_fleet_rejects_arbitrary_names():
    with pytest.raises(ValueError):
        fleet_model("", "ugv_b")


@pytest.mark.parametrize("enabled",[True,False])
def test_rviz_uses_four_independent_models_without_stale_legacy_topics(enabled):
    from campus_regional_delivery.layout import RegionalLayout
    layout=RegionalLayout(Path(__file__).parents[1]/"config/campus_regions.yaml")
    original=yaml.safe_load((Path(get_package_share_directory("cooperative_delivery"))/"rviz/cooperative.rviz").read_text())
    data=fleet_rviz(original,layout.vehicles,enabled)
    displays=data["Visualization Manager"]["Displays"]
    models=[d for d in displays if d["Class"]=="rviz_default_plugins/RobotModel"]
    assert len(models)==(5 if enabled else 4)
    topics=[d["Description Topic"]["Value"] for d in models]
    assert all(f"/ugv_0{i}/robot_description" in topics for i in range(4))
    assert "/robot_description" not in topics


@pytest.mark.parametrize("scenario,ready",[("missing",False),("inactive_monitor",False),("active",True)])
def test_startup_gate_requires_real_navigation_activation(scenario,ready):
    from campus_regional_delivery.startup import navigation_states_ready,REQUIRED_NODES
    states={} if scenario=="missing" else dict.fromkeys(REQUIRED_NODES,3)
    if scenario=="inactive_monitor":
        states["collision_monitor"]=2
    assert navigation_states_ready(states) is ready


def test_installed_regional_entrypoints_are_executable():
    folder=Path(get_package_prefix("campus_regional_delivery"))/"lib/campus_regional_delivery"
    for name in ("regional_mission_manager","regional_markers","regional_localizer","wait_for_regional_navigation","regional_nav_container","regional_component_container"):
        assert os.access(folder/name,os.X_OK),name


def test_all_navigation_components_are_registered_in_this_ros_installation():
    from ament_index_python.resources import get_resource
    from campus_regional_delivery.fleet import NAVIGATION_COMPONENTS
    for package,plugin in list(NAVIGATION_COMPONENTS.values())+[
            ("nav2_lifecycle_manager","nav2_lifecycle_manager::LifecycleManager")]:
        registered,_=get_resource("rclcpp_components",package)
        assert plugin+";" in registered


def test_four_car_launch_defaults_to_validated_middleware_components_and_both_windows():
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    path=Path(__file__).parents[1]/"launch/regional_campus.launch.py"
    spec=importlib.util.spec_from_file_location("regional_launch_contract",path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context=LaunchContext()
    context.environment.pop("RMW_IMPLEMENTATION",None)
    defaults={action.name:perform_substitutions(context,action.default_value)
              for action in module.generate_launch_description().entities
              if isinstance(action,DeclareLaunchArgument)}
    assert defaults["rmw_implementation"]=="rmw_cyclonedds_cpp"
    assert defaults["use_composition"]=="true"
    assert defaults["gui"]==defaults["rviz"]=="true"
    assert defaults["rviz_render_driver"]=="softpipe"
    source=path.read_text()
    assert 'executable="regional_nav_container"' in source
    assert 'executable="component_container_isolated"' not in source


def test_vm_graphics_workaround_is_explicit_and_local_to_rviz():
    import importlib.util
    path=Path(__file__).parents[1]/"launch/regional_campus.launch.py"
    spec=importlib.util.spec_from_file_location("regional_graphics_contract",path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.rviz_environment("softpipe")=={
        "GALLIUM_DRIVER":"softpipe","DRAW_USE_LLVM":"0"}
    assert module.rviz_environment("llvmpipe")=={"GALLIUM_DRIVER":"llvmpipe"}
    with pytest.raises(ValueError):
        module.rviz_environment("unknown")
    source=path.read_text()
    assert source.count('additional_env=rviz_environment(value("rviz_render_driver"))')==1
    assert 'SetEnvironmentVariable("GALLIUM_DRIVER"' not in source


def test_gazebo_crash_stops_scene_but_normal_exit_does_not_repeat_shutdown():
    import importlib.util
    from types import SimpleNamespace
    path=Path(__file__).parents[1]/"launch/regional_campus.launch.py"
    spec=importlib.util.spec_from_file_location("regional_exit_contract",path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    event=SimpleNamespace(action=SimpleNamespace(process_details={"name":"gzserver-5"}))
    assert len(module.gazebo_exited(event,SimpleNamespace(is_shutdown=False)))==1
    assert module.gazebo_exited(event,SimpleNamespace(is_shutdown=True))==[]
    event.action.process_details["name"]="gzclient-6"
    assert module.gazebo_exited(event,SimpleNamespace(is_shutdown=False))==[]
