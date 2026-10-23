from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from uav_application.delivery_mission_manager import DeliveryMissionManager
from uav_navigation.waypoint_navigator import WaypointMap


def test_mission_leaves_dormitory_at_floor_height_before_vertical_transition():
    waypoint_map = WaypointMap(
        Path(__file__).parents[2] / "uav_navigation/config/uav_delivery_waypoints.yaml"
    )
    target = waypoint_map.resolve_delivery_targets(["dormitory_2"], [11])[0]
    manager = Mock()
    manager.waypoint_map = waypoint_map
    manager.energy_constraints_enabled = False
    manager._resolve_optimized_delivery.return_value = SimpleNamespace(
        targets=[target], home=waypoint_map.home, payload_masses=[0.2]
    )
    manager._wait_interfaces.return_value = True
    manager._snapshot.return_value = (None, 1, None)
    manager._wait_vehicle_state.return_value = True
    manager.get_parameter.return_value = SimpleNamespace(value=30.0)
    manager._wait_sim_duration.return_value = True
    manager._wait_landed.return_value = True
    manager._call_trigger.return_value = (True, "accepted")
    calls = []

    def fly(kind, waypoint, altitude, *args, **kwargs):
        calls.append((kind, waypoint.name, altitude))
        return True, "reached"

    manager._fly_to.side_effect = lambda *args, **kwargs: fly("pose", *args, **kwargs)
    manager._fly_with_local_replan.side_effect = (
        lambda *args, **kwargs: fly("local", *args, **kwargs)
    )
    manager._fly_route.return_value = (True, "reached")
    goal = Mock()
    goal.request = SimpleNamespace(
        targets=["dormitory_2"], target_floors=[11], payload_masses_kg=[0.2],
        home_name="", return_home=True, landing_height=0.0,
    )
    goal.is_cancel_requested = False
    result = DeliveryMissionManager._execute_delivery(manager, goal)
    assert result.success
    departure = calls.index(("local", "dormitory", 33.6))
    assert calls[departure + 1] == ("pose", "dormitory", 15.0)
    assert ("pose", "dormitory_2", 15.0) not in calls
    assert manager._fly_route.call_args_list[-1].args[:2] == (
        "dormitory", "logistics_center"
    )
