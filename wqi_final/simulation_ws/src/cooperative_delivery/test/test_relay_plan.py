from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from cooperative_delivery.mission_config import CooperativeMissionConfig
from cooperative_delivery.relay_plan import (
    RelayConfig, can_move_carrier, partition_payloads,
)
from uav_application.mission_energy import MissionEnergyPlanner
from uav_control.battery_model import BatteryModel, BatteryParameters
from uav_navigation.waypoint_navigator import WaypointMap

SRC = Path(__file__).parents[2]
CONFIG = Path(__file__).parents[1] / "config"


@pytest.fixture
def waypoints():
    return WaypointMap(SRC / "uav_navigation/config/uav_delivery_waypoints.yaml")


def test_relay_uses_distinct_connected_platforms(waypoints):
    config = RelayConfig.load(CONFIG / "relay_delivery.yaml", waypoints)
    route = config.air_route(waypoints, "a", "b")
    assert route[0] == (0.0, -18.0, 15.0)
    assert route[-1] == (6.0, 11.0, 15.0)
    assert len(route) > 2
    assert config.air_route(waypoints, "b", "a")[-1] == route[0]


def test_smoke_route_stays_on_the_short_access_road(waypoints):
    config = RelayConfig.load(CONFIG / "relay_smoke.yaml", waypoints)
    assert config.air_route(waypoints, "a", "b") == ((1.5, -41.0, 3.0), (0.0, -38.0, 3.0))


@pytest.mark.parametrize("field,value", [("transfer_altitude", 0), ("landing_height", 3),
    ("transfer_speed_mps", float("nan")), ("telemetry_timeout_wall_s", -1)])
def test_invalid_relay_limits_rejected(tmp_path, waypoints, field, value):
    data = yaml.safe_load((CONFIG / "relay_delivery.yaml").read_text())
    data[field] = value
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        RelayConfig.load(path, waypoints)


def test_overlapping_platforms_rejected(tmp_path, waypoints):
    data = yaml.safe_load((CONFIG / "relay_delivery.yaml").read_text())
    data["platform_b"] = data["platform_a"]
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="2.5"):
        RelayConfig.load(path, waypoints)


def test_payload_partition_preserves_floors_masses_and_duplicate_orders():
    target = CooperativeMissionConfig(CONFIG / "cooperative_waypoints.yaml").resolve(["teaching_building"])[0]
    targets = [replace(target, payload_mass_kg=.6, delivery_floor=floor) for floor in (2, 3, 4)]
    batches = partition_payloads(targets, 1.0)
    assert [len(batch) for batch in batches] == [1, 1, 1]
    assert [batch[0].delivery_floor for batch in batches] == [2, 3, 4]
    assert len(partition_payloads([replace(t, payload_mass_kg=.3) for t in targets], 1)) == 1
    with pytest.raises(ValueError, match="capacity"):
        partition_payloads([replace(target, payload_mass_kg=1.01)], 1)


def test_live_rotors_block_loaded_car_but_not_empty_receiver():
    assert not can_move_carrier("a", "a", False)
    assert can_move_carrier("b", "a", False)
    assert can_move_carrier("b", "b", True)


def test_transfer_energy_uses_payload_and_vertical_propulsion(waypoints):
    config = RelayConfig.load(CONFIG / "relay_delivery.yaml", waypoints)
    parameters = BatteryParameters.from_yaml(SRC / "uav_control/config/battery_model.yaml").for_backend("px4")
    model = BatteryModel(parameters)
    planner = MissionEnergyPlanner(waypoints, parameters)
    route = config.air_route(waypoints, "a", "b")
    empty = model.estimate(planner.plan_transfer(route, .42, 0, config.transfer_hover_seconds))
    loaded = model.estimate(planner.plan_transfer(route, .42, .8, config.transfer_hover_seconds))
    assert loaded.required_energy_wh > empty.required_energy_wh > 0
    assert loaded.payload_energy_penalty_wh > 0


def test_capture_needs_physical_position_and_both_linear_and_angular_rates():
    from cooperative_delivery.relay_plan import capture_pose_is_safe
    def vector(x=0., y=0., z=0.):
        return SimpleNamespace(x=x, y=y, z=z)
    def odom(height):
        return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=vector(z=height))),
            twist=SimpleNamespace(twist=SimpleNamespace(linear=vector(), angular=vector())))
    car, aircraft = odom(.001), odom(.421)
    assert capture_pose_is_safe(car, aircraft, .42)
    aircraft.twist.twist.angular.z = .21
    assert not capture_pose_is_safe(car, aircraft, .42)
    aircraft.twist.twist.angular.z = 0
    car.twist.twist.linear.x = .06
    assert not capture_pose_is_safe(car, aircraft, .42)
    car.twist.twist.linear.x = 0
    aircraft.pose.pose.position.x = .19
    assert not capture_pose_is_safe(car, aircraft, .42)
    aircraft.pose.pose.position.x = float("nan")
    assert not capture_pose_is_safe(car, aircraft, .42)


def test_capture_window_requires_continuous_sim_time_and_resets_on_motion_or_clock_reset():
    from cooperative_delivery.relay_plan import CaptureSettleWindow
    window = CaptureSettleWindow()
    assert not window.observe(10., True)
    assert not window.observe(10., True)  # Wall time cannot substitute for sim time.
    assert not window.observe(10.8, False)
    assert not window.observe(10.9, True)
    assert not window.observe(11.5, True)
    assert window.observe(12., True)
    assert not window.observe(1., True)
    assert not window.observe(1.1, True)


def test_capture_velocity_filter_uses_new_sim_samples_and_does_not_hide_sustained_motion():
    from cooperative_delivery.relay_plan import CaptureVelocityFilter
    f = CaptureVelocityFilter()
    assert f.observe(1., (0.,) * 12) == (0.,) * 12
    impulse = (.4,) * 12
    first = f.observe(1.05, impulse)
    assert 0 < first[0] < .1
    assert f.observe(1.05, impulse) == first  # No refiltering a cached sample.
    for i in range(1, 21):
        sustained = f.observe(1.05 + .05*i, impulse)
    assert sustained[0] > .38
    assert f.observe(3., (float("nan"),) * 12) is None
