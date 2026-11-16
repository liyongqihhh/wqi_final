import itertools
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from campus_regional_delivery.layout import RegionalLayout, segment_distance
from campus_regional_delivery.dynamic_scenario import traffic_config
from campus_dynamic_obstacles.scenario import load_obstacle_config, motion_seed

ROOT=Path(__file__).parents[1]


def test_random_volumes_are_paved_clear_of_buildings_and_protected_bays(tmp_path):
    layout=RegionalLayout(ROOT/"config/campus_regions.yaml")
    config=traffic_config(layout,motion_mode="random")
    path=tmp_path/"random.yaml";path.write_text(yaml.safe_dump(config))
    _,routes=load_obstacle_config(path)
    geometry=yaml.safe_load((ROOT/"assets/building_geometry.yaml").read_text())
    for route in routes:
        lo,hi=route.bounds_min,route.bounds_max
        assert route.motion_mode=="bounded_random"
        assert route.activation_models==()
        for p in itertools.product(*(np.linspace(a,b,9) for a,b in zip(lo,hi))):
            for name,spec in geometry.items():
                x0,x1,y0,y1=spec["bounds"]
                d=math.hypot(max(x0-p[0],0,p[0]-x1),max(y0-p[1],0,p[1]-y1))
                assert d>route.radius_m+.15 or p[2]-route.radius_m>spec["height"]+.15,(route.name,name,p)
            if route.layer=="ground":
                assert any(float(segment_distance(np.array(p[0]),np.array(p[1]),a,b))
                           <=road["width"]/2-route.radius_m-.15
                           for road in layout.roads for a,b in zip(road["sampled_points"],road["sampled_points"][1:])),(route.name,p)
        for name,pose in layout.all_stops().items():
            gap=math.hypot(max(lo[0]-pose[0],0,pose[0]-hi[0]),max(lo[1]-pose[1],0,pose[1]-hi[1]))
            assert gap>route.radius_m+1.8+.5+.15,(route.name,name,gap)
    for region in layout.regions:
        air=[r for r in routes if r.layer=="air" and r.group==region]
        assert len({r.waypoints[0][2] for r in air})==5
        points=np.array([r.waypoints[0] for r in air])
        assert np.linalg.matrix_rank(points[1:]-points[0])==3
        assert all(r.bounds_max[2]-r.bounds_min[2]>=4 for r in air)
    for a,b in itertools.combinations(routes,2):
        # Separate random volumes so actors do not collide with one another
        # and introduce impulsive/non-random velocity changes.
        gap=math.sqrt(sum(max(x-y2,x2-y,0.)**2 for x,y,x2,y2 in
            zip(a.bounds_min,a.bounds_max,b.bounds_min,b.bounds_max)))
        assert gap>a.radius_m+b.radius_m+.1,(a.name,b.name,gap)


def test_motion_seed_is_stable_per_name_and_independent_of_selection_order():
    assert motion_seed(42,"a")==motion_seed(42,"a")
    assert motion_seed(42,"a")!=motion_seed(42,"b")!=motion_seed(43,"b")


def test_global_costmaps_are_available_for_all_cars_without_four_overlaid_grids():
    from campus_regional_delivery.fleet import fleet_rviz
    original=yaml.safe_load((ROOT.parent/"cooperative_delivery/rviz/cooperative.rviz").read_text())
    layout=RegionalLayout(ROOT/"config/campus_regions.yaml")
    displays=fleet_rviz(original,layout.vehicles)["Visualization Manager"]["Displays"]
    maps=[d for d in displays if d["Name"].endswith("UGV Global Costmap")]
    assert len(maps)==4
    assert [d["Name"] for d in maps if d["Enabled"]]==["UGV-01 UGV Global Costmap"]
    assert {d["Topic"]["Value"] for d in maps}=={f"/ugv_0{i}/global_costmap/costmap" for i in range(4)}
    for d in maps:
        assert d["Update Topic"]["Value"]==d["Topic"]["Value"]+"_updates"


def test_invalid_random_mode_is_rejected():
    with pytest.raises(ValueError):traffic_config(None,motion_mode="teleport")
