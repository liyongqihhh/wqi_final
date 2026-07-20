import math
from pathlib import Path

import pytest

from uav_navigation.waypoint_navigator import (
    WaypointConfigurationError,
    WaypointMap,
)


CONFIG = Path(__file__).parents[1] / "config" / "uav_delivery_waypoints.yaml"


def test_required_delivery_points_exist():
    waypoints = WaypointMap(CONFIG)
    assert waypoints.home.name == "logistics_center"
    assert "teaching_building" in waypoints.waypoints
    assert "laboratory" in waypoints.waypoints
    assert "library" in waypoints.waypoints
    assert all(
        f"dormitory_{index}" in waypoints.waypoints
        for index in range(1, 5)
    )


def test_cruise_altitude_uses_low_altitude_corridor():
    waypoints = WaypointMap(CONFIG)
    assert waypoints.flight["cruise_altitude"] == 15.0
    assert waypoints.flight["cruise_altitude"] < (
        waypoints.flight["maximum_obstacle_height"]
    )


def test_all_delivery_points_have_connected_corridor_routes():
    waypoints = WaypointMap(CONFIG)
    for target in waypoints.waypoints:
        route = waypoints.plan_route(waypoints.home.name, target)
        if target == waypoints.home.name:
            assert route == []
        else:
            assert route[-1].name == target


def test_corridor_graph_contains_closed_loops():
    waypoints = WaypointMap(CONFIG)
    edge_count = len(waypoints.corridor_edges)
    node_count = len(waypoints.corridor_nodes)
    assert edge_count >= node_count


def test_library_route_bypasses_laboratory_delivery_pad():
    waypoints = WaypointMap(CONFIG)
    route = waypoints.plan_route(waypoints.home.name, "library")
    route_names = [node.name for node in route]
    assert "laboratory_transit" in route_names
    assert "laboratory" not in route_names
    assert waypoints.corridor_nodes["east_gate_south"].x == 78.0


def test_library_delivery_point_matches_south_door_and_routes_are_connected():
    waypoints = WaypointMap(CONFIG)
    library = waypoints.corridor_nodes["library"]
    return_route = waypoints.plan_route("library", waypoints.home.name)
    north_route = waypoints.plan_route("library", "north_east")

    assert (library.x, library.y) == (62.0, 40.5)
    assert return_route[0].name == "east_gate_north"
    assert [node.name for node in north_route] == [
        "east_gate_north",
        "library_east_mid",
        "library_north",
        "north_lane_east",
        "library_west_south",
        "north_east",
    ]

    east_clearance_nodes = (
        waypoints.corridor_nodes["library_east_mid"],
        waypoints.corridor_nodes["library_north"],
    )
    assert all(node.x >= 78.0 for node in east_clearance_nodes)

    north_clearance_nodes = (
        waypoints.corridor_nodes["library_north"],
        waypoints.corridor_nodes["north_lane_east"],
    )
    assert all(node.y >= 65.0 for node in north_clearance_nodes)

    west_clearance_nodes = (
        waypoints.corridor_nodes["north_lane_east"],
        waypoints.corridor_nodes["library_west_south"],
    )
    assert all(node.x <= 42.0 for node in west_clearance_nodes)

    road_start = (76.0, 38.0)
    road_end = (62.0, 40.5)
    road_dx = road_end[0] - road_start[0]
    road_dy = road_end[1] - road_start[1]
    projection = (
        (library.x - road_start[0]) * road_dx
        + (library.y - road_start[1]) * road_dy
    ) / (road_dx * road_dx + road_dy * road_dy)
    projection = max(0.0, min(1.0, projection))
    nearest_x = road_start[0] + projection * road_dx
    nearest_y = road_start[1] + projection * road_dy
    distance_to_centerline = math.hypot(
        library.x - nearest_x,
        library.y - nearest_y,
    )
    assert distance_to_centerline <= 2.0


def test_teaching_pad_keeps_safety_sphere_clear_of_building():
    waypoints = WaypointMap(CONFIG)
    teaching = waypoints.waypoints["teaching_building"]
    corridor = waypoints.corridor_nodes["teaching_building"]

    south_wall_y = 12.0
    safety_radius = 1.8
    control_margin = 0.5
    assert south_wall_y - teaching.y >= safety_radius + control_margin
    assert (teaching.x, teaching.y) == (corridor.x, corridor.y)
    assert waypoints.delivery_altitude_for(teaching) == 8.0
    assert teaching.delivery_floor == 3


def test_dormitories_hover_in_front_of_each_building_door():
    waypoints = WaypointMap(CONFIG)
    expected = {
        "dormitory_1": (
            (18.0, 8.5), (11.0, 25.0, -5.0, 5.0), (18.0, 5.0), -1.571
        ),
        "dormitory_2": (
            (42.0, 8.5), (35.0, 49.0, -5.0, 5.0), (42.0, 5.0), -1.571
        ),
        "dormitory_3": (
            (18.0, 13.5), (11.0, 25.0, 17.0, 27.0), (18.0, 17.0), 1.571
        ),
        "dormitory_4": (
            (42.0, 13.5), (35.0, 49.0, 17.0, 27.0), (42.0, 17.0), 1.571
        ),
    }

    safety_radius = 1.8
    cruise_speed = 1.2
    braking_acceleration = 0.8
    target_tolerance = 0.4
    residual_margin = 0.4
    required_standoff = (
        safety_radius
        + cruise_speed**2 / (2.0 * braking_acceleration)
        + target_tolerance
        + residual_margin
    )

    for name, (position, footprint, door_center, yaw) in expected.items():
        target = waypoints.waypoints[name]
        assert (target.x, target.y) == position
        assert target.x == door_center[0]
        assert math.dist(position, door_center) == pytest.approx(3.5)
        assert math.dist(position, door_center) >= required_standoff
        assert target.yaw == pytest.approx(yaw)
        assert [node.name for node in waypoints.plan_route("dormitory", name)] == [
            name
        ]

        min_x, max_x, min_y, max_y = footprint
        dx = max(min_x - target.x, 0.0, target.x - max_x)
        dy = max(min_y - target.y, 0.0, target.y - max_y)
        assert math.hypot(dx, dy) >= required_standoff


def test_unknown_target_is_rejected():
    waypoints = WaypointMap(CONFIG)
    with pytest.raises(WaypointConfigurationError):
        waypoints.resolve(["not_a_real_building"])


def test_cooperative_home_can_use_any_air_corridor_node():
    waypoints = WaypointMap(CONFIG)
    home = waypoints.resolve_home("west_return_3")
    assert (home.x, home.y) == (-8.0, 7.0)
    assert waypoints.plan_route(home.name, "teaching_building")[-1].name == (
        "teaching_building"
    )


def test_building_delivery_altitudes_are_below_cruise_height():
    waypoints = WaypointMap(CONFIG)
    cruise = float(waypoints.flight["cruise_altitude"])
    for waypoint in waypoints.waypoints.values():
        if waypoint.name != waypoints.home.name:
            assert 0.0 < waypoints.delivery_altitude_for(waypoint) < cruise


def test_empty_cooperative_home_keeps_standalone_logistics_home():
    waypoints = WaypointMap(CONFIG)
    assert waypoints.resolve_home("") == waypoints.home


def test_requested_floor_changes_delivery_altitude():
    waypoints = WaypointMap(CONFIG)
    target = waypoints.resolve_delivery_targets(
        ["teaching_building"], [6]
    )[0]
    assert target.delivery_floor == 6
    assert target.delivery_altitude == pytest.approx(17.6)


def test_requested_floor_above_building_limit_is_rejected():
    waypoints = WaypointMap(CONFIG)
    with pytest.raises(WaypointConfigurationError):
        waypoints.resolve_delivery_targets(["teaching_building"], [9])


def test_route_distance_uses_the_air_corridor_graph_and_is_symmetric():
    waypoints = WaypointMap(CONFIG)
    outbound = waypoints.route_distance(
        "logistics_center", "teaching_building"
    )
    inbound = waypoints.route_distance(
        "teaching_building", "logistics_center"
    )
    direct = math.dist(
        (
            waypoints.home.x,
            waypoints.home.y,
        ),
        (
            waypoints.waypoints["teaching_building"].x,
            waypoints.waypoints["teaching_building"].y,
        ),
    )
    assert outbound == pytest.approx(inbound)
    assert outbound >= direct
