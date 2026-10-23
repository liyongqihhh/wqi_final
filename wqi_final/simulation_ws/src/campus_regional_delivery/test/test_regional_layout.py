import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import pytest
import yaml

from campus_regional_delivery.layout import RegionalLayout

PACKAGE=Path(__file__).parents[1]
LAYOUT=PACKAGE/"config/campus_regions.yaml"
ASSETS=PACKAGE/"assets"
SOURCE=PACKAGE.parent/"ugvcar_description/world/campus_delivery.world"


def test_generated_shape_has_distinct_visual_and_collision_names():
    from campus_regional_delivery.assets import add_shape
    link=ET.Element("link",name="body")
    add_shape(link,"post",[0,0,1,0,0,0],[.1,.1,2],[.2,.4,.8,1],collision=True)
    assert link.find("visual").get("name")=="post"
    assert link.find("collision").get("name")=="post_collision"
    assert ET.tostring(link.find("visual/geometry"))==ET.tostring(link.find("collision/geometry"))


def test_exactly_three_regions_and_eleven_original_buildings():
    layout=RegionalLayout(LAYOUT)
    world=ET.parse(ASSETS/"campus_regional.world").getroot().find("world")
    original=ET.parse(SOURCE).getroot().find("world")
    report=json.loads((ASSETS/"layout_validation.json").read_text())
    assert set(layout.regions)=={"A","B","C"}
    assert report["building_count"]==11 and len(layout.buildings)==10
    for name,b in report["buildings"].items():
        before=original.find(f"model[@name='{b['model_name']}']/link")
        after=world.find(f"model[@name='{b['model_name']}']/link")
        # Reindent before hashing because XML pretty-printing changes whitespace.
        for element in (before,after):
            for child in element.iter():
                child.text=child.text.strip() if child.text and child.text.strip() else None
                child.tail=None
        assert ET.tostring(before)==ET.tostring(after),name
        assert b["floors"]>=3
        assert len(world.findall(f"model[@name='{b['model_name']}']"))==1
    assert world.find("model[@name='campus_roads']") is None


def test_world_poses_are_exactly_the_single_layout_coordinates():
    layout=RegionalLayout(LAYOUT)
    world=ET.parse(ASSETS/"campus_regional.world").getroot().find("world")
    for b in layout.buildings.values():
        pose=[float(v) for v in world.findtext(f"model[@name='{b['model_name']}']/pose").split()]
        assert pose[:2]==b["center"] and pose[3:]==[0,0,0]
    assert yaml.safe_load((ASSETS/"regional_map.yaml").read_text())["origin"]==[-90.,-80.,0.]
    report=json.loads((ASSETS/"layout_validation.json").read_text())
    assert report["layout_sha256"]==hashlib.sha256(LAYOUT.read_bytes()).hexdigest()


def test_all_fourteen_stops_connected_with_vehicle_clearance():
    layout=RegionalLayout(LAYOUT)
    report=json.loads((ASSETS/"layout_validation.json").read_text())
    free,_,distance=layout.raster(report["obstacles"])
    checked=layout.validate_raster(free,distance,report["obstacles"])
    assert checked["stop_count"]==14 and checked["road_components"]==1
    actual=np.asarray(Image.open(ASSETS/"regional_map.pgm"))>200
    assert np.array_equal(actual,free)
    assert min(checked["uav_corridor_clearance_m"].values())>=2.5


def test_dormitories_have_four_distinct_front_door_stops():
    layout=RegionalLayout(LAYOUT)
    points=[]
    for name in layout.regions["B"]["targets"]:
        b=layout.buildings[name]
        stop=b["delivery_pose"]
        assert stop[0]==b["center"][0]
        assert stop[1]>b["center"][1] if name in ("dormitory_1","dormitory_2") else stop[1]<b["center"][1]
        points.append(tuple(stop[:2]))
    assert len(set(points))==4
    centers=[layout.buildings[n]["center"] for n in layout.regions["B"]["targets"]]
    assert centers[1][0]-centers[0][0]==centers[2][1]-centers[0][1]==24


def test_main_ring_is_central_and_buildings_are_outside_it():
    layout=RegionalLayout(LAYOUT)
    ring=next(r for r in layout.roads if r["name"]=="main_ring")
    assert ring["closed"] and ring["corner_radius"]>=6
    points=np.asarray(ring["points"])
    low,high=points.min(axis=0),points.max(axis=0)
    for building in layout.buildings.values():
        x,y=building["center"]
        assert not (low[0]<=x<=high[0] and low[1]<=y<=high[1])
    assert high[0]-low[0]<.6*(layout.bounds[1]-layout.bounds[0])


def test_air_target_schema_has_only_validated_direct_routes():
    from uav_navigation.waypoint_navigator import WaypointMap
    graph=WaypointMap(ASSETS/"uav_targets.yaml")
    assert set(graph.waypoints)=={"logistics_center","depot_a","depot_b","depot_c"}
    assert len(graph.corridor_edges)==3
    assert graph.flight["cruise_altitude"]==15.


def test_duplicate_namespace_is_rejected(tmp_path):
    data=yaml.safe_load(LAYOUT.read_text())
    data["regions"]["B"]["namespace"]="ugv_01"
    path=tmp_path/"bad.yaml";path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        RegionalLayout(path)


def test_no_building_is_assigned_to_two_vehicles(tmp_path):
    data=yaml.safe_load(LAYOUT.read_text())
    data["regions"]["C"]["targets"].append("library")
    path=tmp_path/"bad.yaml";path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        RegionalLayout(path)


def test_central_carrier_and_regional_parking_are_unambiguous():
    from campus_regional_delivery.layout import segment_distance
    layout=RegionalLayout(LAYOUT)
    assert [v["namespace"] for v in layout.vehicles.values()]==["ugv_00","ugv_01","ugv_02","ugv_03"]
    assert layout.hub["uav_pad"][:2]==layout.hub["depot_pose"][:2]
    assert layout.hub["uav_pad"][2]==pytest.approx(layout.hub["depot_pose"][2]+.42)
    for region in layout.regions.values():
        x,y=region["depot_pose"][:2]
        for road in layout.roads:
            if road["name"].startswith("depot_"):
                continue
            clearance=min(float(segment_distance(x,y,a,b))
                for a,b in zip(road["sampled_points"],road["sampled_points"][1:]))
            assert clearance>road["width"]/2+layout.data["navigation"]["vehicle_radius"]+.2
