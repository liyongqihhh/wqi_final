from pathlib import Path
from types import SimpleNamespace

from uav_application.delivery_mission_manager import DeliveryMissionManager
from uav_navigation.waypoint_navigator import WaypointMap


CONFIG = (
    Path(__file__).parents[2]
    / "uav_navigation"
    / "config"
    / "uav_delivery_waypoints.yaml"
)


class _PayloadResolver:
    @staticmethod
    def resolve_payload_masses(_targets, payloads):
        return [float(value) for value in payloads]


def test_uav_route_optimization_keeps_floor_and_payload_mapping():
    waypoint_map = WaypointMap(CONFIG)
    manager = SimpleNamespace(
        waypoint_map=waypoint_map,
        energy_planner=_PayloadResolver(),
    )
    requested_names = ["library", "teaching_building", "laboratory"]
    requested_floors = [5, 2, 7]
    requested_payloads = [0.15, 0.45, 0.25]

    mission = DeliveryMissionManager._resolve_optimized_delivery(
        manager,
        requested_names,
        requested_floors,
        requested_payloads,
        "",
        True,
    )
    mapping = {
        name: (floor, payload)
        for name, floor, payload in zip(
            mission.target_names,
            mission.target_floors,
            mission.payload_masses,
        )
    }
    assert mapping == {
        "library": (5, 0.15),
        "teaching_building": (2, 0.45),
        "laboratory": (7, 0.25),
    }

    original_route = [waypoint_map.home.name, *requested_names]
    original_cost = sum(
        waypoint_map.route_distance(start, end)
        for start, end in zip(original_route, original_route[1:])
    ) + waypoint_map.route_distance(
        requested_names[-1], waypoint_map.home.name
    )
    assert mission.route_plan.total_cost <= original_cost
