from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import Mock
from pathlib import Path
import xml.etree.ElementTree as ET

from test_scenario import _load_spawner_module,_route


def test_gated_actors_have_unique_plugin_nodes_and_fixed_nonreactive_routes():
    spawner=_load_spawner_module()
    route=replace(_route("air"),group="A",activation_models=("campus_uav",),
        activation_point=(0.,5.,15.),activation_distance=9.)
    plugins=[]
    for name in ("actor_a","actor_b"):
        xml=spawner.DynamicObstacleSpawner._sdf(name,route)
        root=ET.fromstring(xml)
        plugins.extend(p.get("name") for p in root.findall(".//plugin"))
        controller=root.find("model/plugin[@name='route_controller']")
        assert controller.find("activation_model").text=="campus_uav"
        assert len(controller.findall("waypoint"))==2
        assert "yield_model" not in xml and "avoidance_gain" not in xml
    # The Gazebo model plugin has no ROS node; sensor/P3D plugins do.
    ros_names=[n for n in plugins if n!="route_controller"]
    assert len(set(ros_names))==len(ros_names)


def test_activation_never_reads_robots_again_after_release():
    source=(Path(__file__).parents[1]/"src/dynamic_obstacle_plugin.cpp").read_text()
    gated=source.index("if (!active_)")
    main=source.index("const auto position = model_->WorldPose().Pos();",gated)
    assert "ModelByName" in source[gated:main]
    assert "ModelByName" not in source[main:]
    assert "SetWorldPose" not in source


def test_random_motion_sdf_has_a_volume_and_stable_individual_seed():
    from campus_dynamic_obstacles.scenario import motion_seed
    spawner=_load_spawner_module()
    route=replace(_route("air"),motion_mode="bounded_random",bounds_min=(-1.,-1.,0.),
                  bounds_max=(2.,2.,3.))
    text=spawner.DynamicObstacleSpawner._sdf("actor",route,123)
    controller=ET.fromstring(text).find("model/plugin[@name='route_controller']")
    assert controller.find("motion_mode").text=="bounded_random"
    assert controller.find("random_seed").text==str(motion_seed(123,route.name))
    assert controller.find("bounds_max").text=="2.0 2.0 3.0"


def test_extreme_spawning_keeps_only_one_service_request_in_flight():
    module=_load_spawner_module();spawner=module.DynamicObstacleSpawner
    futures=[];requests=[]
    def request(message):
        future=Mock();future.callback=None
        future.add_done_callback.side_effect=lambda cb:setattr(future,"callback",cb)
        future.result.return_value=SimpleNamespace(success=True,status_message="ok")
        futures.append(future);requests.append(message)
        return future
    node=SimpleNamespace(spawned=False,client=Mock(service_is_ready=lambda:True),
        routes=tuple(_route("air") for _ in range(33)),pending=0,failures=[],next_route_index=0,
        _sdf=spawner._sdf,_publish_result=Mock(),get_logger=Mock())
    node.client.call_async.side_effect=request
    node._spawn_next=lambda:spawner._spawn_next(node)
    node._spawn_done=lambda *args:spawner._spawn_done(node,*args)
    spawner._spawn_once(node)
    assert len(futures)==1
    for index in range(33):
        assert len(futures)==index+1
        futures[index].callback(futures[index])
    assert node.pending==0 and len(requests)==33
    assert len({r.name for r in requests})==33
    node._publish_result.assert_called_once()
