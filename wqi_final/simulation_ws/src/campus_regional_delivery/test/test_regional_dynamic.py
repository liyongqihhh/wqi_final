import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from campus_regional_delivery.dynamic_scenario import DENSITIES,traffic_config
from campus_regional_delivery.layout import RegionalLayout
from campus_dynamic_obstacles.scenario import load_obstacle_config,select_routes


ROOT=Path(__file__).parents[1]


def test_fast_encounter_release_accounts_for_faster_car_not_less_traffic():
    layout=RegionalLayout(ROOT/"config/campus_regions.yaml")
    normal,fast=(traffic_config(layout,v) for v in (.4,.7))
    assert normal["densities"]==fast["densities"]
    for name,route in fast["routes"].items():
        old=normal["routes"][name]
        assert route["waypoints"]==old["waypoints"]
        assert route["speed_mps"]==old["speed_mps"]
        if route["layer"]=="ground" and not name.endswith(("_ground_3","_ground_6")):
            assert route["activation_distance"]>old["activation_distance"]
        else:
            assert route==old
    with pytest.raises(ValueError):traffic_config(layout,float("nan"))


def scenario(tmp_path):
    layout=RegionalLayout(ROOT/"config/campus_regions.yaml")
    path=tmp_path/"traffic.yaml";path.write_text(yaml.safe_dump(traffic_config(layout)))
    return layout,*load_obstacle_config(path)


@pytest.mark.parametrize("density",["low","medium","high","very_high","extreme"])
def test_five_positive_levels_cover_ground_and_air_in_every_region(tmp_path,density):
    _,densities,routes=scenario(tmp_path)
    selected=select_routes(densities,routes,density,42)
    for layer in ("ground","air"):
        subset=[r for r in selected if r.layer==layer]
        assert len(subset)==DENSITIES[density][layer]
        assert {r.group for r in subset}=={"A","B","C"}


@pytest.mark.parametrize("seed",[1,7,42,123])
def test_density_is_nested_and_repeatable_for_same_seed(tmp_path,seed):
    _,densities,routes=scenario(tmp_path);previous=set()
    for density in DENSITIES:
        selected=select_routes(densities,routes,density,seed)
        assert selected==select_routes(densities,routes,density,seed)
        names={r.name for r in selected};assert previous<=names;previous=names
    assert len(previous)==33


def test_routes_have_no_building_collision_and_leave_bays_clear(tmp_path):
    layout,_,routes=scenario(tmp_path)
    buildings=yaml.safe_load((ROOT/"assets/building_geometry.yaml").read_text())
    for route in routes:
        for a,b in zip(route.waypoints,route.waypoints[1:]):
            for ratio in np.linspace(0,1,101):
                p=np.asarray(a)+ratio*(np.asarray(b)-a)
                for name,spec in buildings.items():
                    xmin,xmax,ymin,ymax=spec["bounds"]
                    inside=(xmin-route.radius_m<p[0]<xmax+route.radius_m and
                            ymin-route.radius_m<p[1]<ymax+route.radius_m and
                            p[2]-route.radius_m<spec["height"])
                    assert not inside,(route.name,name,p)
                for name,pose in layout.all_stops().items():
                    if route.layer=="ground":
                        assert math.dist(p[:2],pose[:2])>route.radius_m+1.8+.5+.15,(route.name,name)


def test_air_routes_intersect_actual_cruise_edges_including_oblique_crossings(tmp_path):
    layout,_,routes=scenario(tmp_path)
    diagonal=0
    for route in (r for r in routes if r.layer=="air"):
        midpoint=np.mean(route.waypoints,axis=0)
        start=np.array(layout.hub["uav_pad"][:2]);end=np.array(layout.regions[route.group]["depot_pose"][:2])
        t=np.dot(midpoint[:2]-start,end-start)/np.dot(end-start,end-start)
        assert .1<t<.95
        assert np.linalg.norm(start+t*(end-start)-midpoint[:2])<1e-8
        assert midpoint[2]==15.
        diagonal+=route.waypoints[0][2]!=route.waypoints[-1][2]
        assert route.activation_models==("campus_uav",)
        assert math.dist(route.waypoints[0],midpoint)>4.5
    assert diagonal>=6


def test_ground_crossings_release_near_a_nominal_encounter_not_a_robot_stop(tmp_path):
    _,_,routes=scenario(tmp_path)
    for route in (r for r in routes if r.layer=="ground"):
        if route.name.endswith(("_ground_3","_ground_6")):
            continue
        actor_time=math.dist(route.waypoints[0],route.activation_point)/route.speed_mps
        robot_time=route.activation_distance/.4
        assert abs(actor_time-robot_time)<1.5
        assert "ugv_00" in route.activation_models


def test_longitudinal_traffic_releases_before_parked_actor_can_block_its_trigger(tmp_path):
    _,_,routes=scenario(tmp_path)
    selected=[r for r in routes if r.name.endswith(("_ground_3","_ground_6"))]
    assert len(selected)==6
    for route in selected:
        assert route.activation_point[:2]==route.waypoints[0][:2]
        assert route.activation_distance==8.
        assert "ugv_00" in route.activation_models


def test_core_crossings_use_wide_road_centers_not_narrow_depot_connectors(tmp_path):
    layout,densities,routes=scenario(tmp_path)
    selected=select_routes(densities,routes,"low",42)
    assert any(r.layer=="ground" and r.group=="A" and -31<r.activation_point[0]<-28
               and r.activation_point[1]==22 for r in selected)
    from campus_regional_delivery.layout import segment_distance
    for route in (r for r in selected if r.layer=="ground"):
        x,y=route.activation_point[:2]
        assert any(road["width"]>=6 and any(float(segment_distance(np.array(x),np.array(y),a,b))<.05
                   for a,b in zip(road["points"],road["points"][1:])) for road in layout.data["roads"])


def test_fleet_bt_sets_namespace_on_truncation_not_only_topic_remaps():
    source=(ROOT/"launch/regional_campus.launch.py").read_text()
    assert 'truncation.set("robot_frame",f"{ns}/base_link")' in source


def test_dynamic_fleet_uses_independent_predictive_topics_without_changing_static_mode():
    from campus_regional_delivery.fleet import fleet_parameters
    from ament_index_python.packages import get_package_share_directory
    nav=Path(get_package_share_directory("ugvcar_navigation2"))
    original=yaml.safe_load((nav/"config/nav2_params.yaml").read_text())
    for ns in ("ugv_00","ugv_01","ugv_02","ugv_03"):
        data=fleet_parameters(original,nav,ns,["ugv_00","ugv_01","ugv_02","ugv_03"],dynamic=True)[ns]
        controller=data["controller_server"]["ros__parameters"]["FollowPath"]
        assert controller["plugin"]=="dwb_core::DWBLocalPlanner"
        assert controller["PredictiveCollision.tracks_topic"]==f"/{ns}/ugv/tracked_dynamic_obstacles"
        assert controller["max_vel_x"]==.4
        assert controller["GoalAlign.scale"]==0.
        assert "ApproachPathAlign" in controller["critics"]
        assert controller["ApproachPathAlign.forward_point_distance"]==.70
        assert controller["ApproachPathAlign.disable_distance"]==1.2
        assert controller["PredictiveCollision.motion_preference_weight"]==.00025
        assert controller["PathHeading.lookahead_distance"]==4.
        assert controller["PathHeading.activation_angle"]==.7
        assert controller["sim_time"]==3.
        assert data["local_costmap"]["local_costmap"]["ros__parameters"]["robot_radius"]==data["global_costmap"]["global_costmap"]["ros__parameters"]["robot_radius"]
        assert controller["PredictiveCollision.prediction_horizon"]==8.
        assert controller["PredictiveCollision.safety_margin"]>=controller["max_vel_x"]**2/(2*abs(controller["decel_lim_x"]))
        assert data["planner_server"]["ros__parameters"]["GridBased"]["plugin"]=="ugvcar_navigation2/DStarLitePlanner"
        assert data["global_costmap"]["global_costmap"]["ros__parameters"]["obstacle_layer"]["scan"]["marking"] is False


def test_optional_rviz_paths_show_sensor_route_separately_from_nominal_route():
    from campus_regional_delivery.fleet import fleet_rviz
    from ament_index_python.packages import get_package_share_directory
    original=yaml.safe_load((Path(get_package_share_directory("cooperative_delivery"))/"rviz/cooperative.rviz").read_text())
    for air in (False,True):
        result=fleet_rviz(original,{},air,True,True)
        displays={d["Name"]:d for d in result["Visualization Manager"]["Displays"]}
        assert "Dynamic Ground and Air Traffic" in displays
        assert ("UAV Sensor Replanned Route" in displays)==air
