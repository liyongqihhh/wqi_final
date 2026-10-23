from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import threading

from geometry_msgs.msg import PoseStamped
import pytest

from uav_application.delivery_mission_manager import DeliveryMissionManager
from uav_application.mission_energy import MissionEnergyPlanner
from uav_control.battery_model import BatteryParameters
from uav_navigation.waypoint_navigator import WaypointMap


def planner():
    source = Path(__file__).parents[2]
    parameters = BatteryParameters.from_yaml(source / "uav_control/config/battery_model.yaml")
    return MissionEnergyPlanner(WaypointMap(source / "uav_navigation/config/uav_delivery_waypoints.yaml"),
                                parameters), parameters


def carrier_node(x=29.9, y=10.7, stamp=100, frame="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp.sec = stamp
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    return SimpleNamespace(use_carrier_home=True, carrier_home_pose=pose,
        lock=threading.Lock(), get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)))


def test_return_home_uses_physical_carrier_not_nominal_marker():
    energy, _ = planner()
    nominal = energy.waypoint_map.resolve_home("dormitory")
    actual = DeliveryMissionManager._carrier_home(carrier_node(), nominal, True)
    assert (actual.x, actual.y) == (29.9, 10.7)
    assert actual.name == nominal.name and nominal.y == 11.0
    reference = energy.plan(["dormitory_2"], "dormitory", .42,
                            target_floors=[11], payload_masses_kg=[.65])
    profile = energy.plan(["dormitory_2"], "dormitory", .42,
                          target_floors=[11], payload_masses_kg=[.65], home_override=actual)
    assert profile.horizontal_distance_m != reference.horizontal_distance_m
    assert profile.ascent_m == reference.ascent_m


@pytest.mark.parametrize("kwargs", [dict(stamp=90), dict(frame="odom"),
                                     dict(stamp=101),
                                     dict(x=0, y=-43.5), dict(x=float("nan"))])
def test_execution_rejects_stale_wrong_frame_or_distant_carrier(kwargs):
    energy, _ = planner()
    home = energy.waypoint_map.resolve_home("dormitory")
    node = carrier_node(**kwargs)
    with pytest.raises(ValueError, match="Fresh carrier pose"):
        DeliveryMissionManager._carrier_home(node, home, True)
    # Before arrival, batch scheduling can still estimate from the nominal stop.
    assert DeliveryMissionManager._carrier_home(node, home) == home


def test_one_clock_tick_of_transport_skew_does_not_reject_fresh_carrier():
    energy, _ = planner()
    home = energy.waypoint_map.resolve_home("dormitory")
    node = carrier_node()
    node.carrier_home_pose.header.stamp.nanosec = 150_000_000
    actual = DeliveryMissionManager._carrier_home(node, home, True)
    assert (actual.x, actual.y) == (29.9, 10.7)


def test_independent_uav_does_not_require_carrier_and_wrong_override_is_rejected():
    energy, _ = planner()
    home = energy.waypoint_map.resolve_home("dormitory")
    assert DeliveryMissionManager._carrier_home(SimpleNamespace(use_carrier_home=False), home, True) == home
    with pytest.raises(ValueError, match="match the configured home"):
        energy.plan(["dormitory_2"], "dormitory", .42,
                    home_override=replace(home, name="logistics_center"))
