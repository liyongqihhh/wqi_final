from pathlib import Path
import ast
import threading
import pytest
from types import SimpleNamespace
from unittest.mock import Mock
from concurrent.futures import Future
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from cooperative_delivery.mission_config import CooperativeMissionConfig
from cooperative_delivery.relay_mission_manager import RelayMissionManager
from cooperative_delivery.relay_plan import RelayConfig
from uav_navigation.waypoint_navigator import WaypointMap


def test_relay_constructor_does_not_shadow_read_only_node_properties():
    path = Path(__file__).parents[1] / "cooperative_delivery/relay_mission_manager.py"
    tree = ast.parse(path.read_text())
    assigned = {target.attr for item in ast.walk(tree) if isinstance(item, ast.Assign)
                for target in item.targets if isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name) and target.value.id == "self"}
    readonly = {name for name, value in vars(Node).items()
                if isinstance(value, property) and value.fset is None}
    assert not assigned & readonly


def test_late_joining_manager_receives_latched_landed_state():
    path = Path(__file__).parents[1] / "cooperative_delivery/cooperative_mission_manager.py"
    tree = ast.parse(path.read_text())
    subscriptions = [call for call in ast.walk(tree) if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute) and call.func.attr == "create_subscription"
        and len(call.args) >= 4 and isinstance(call.args[1], ast.Constant)
        and call.args[1].value == "/uav/state"]
    expression = ast.Expression(subscriptions[0].args[3])
    qos = eval(compile(expression, str(path), "eval"), {
        "QoSProfile": QoSProfile, "DurabilityPolicy": DurabilityPolicy,
        "ReliabilityPolicy": ReliabilityPolicy})
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_relay_waits_for_active_nav2_not_just_advertised_actions():
    ready_client = Mock()
    ready_client.server_is_ready.return_value = True
    ready_client.service_is_ready.return_value = True
    node = SimpleNamespace(_fresh=lambda role: True,
        vehicle_clients={"a": (ready_client,) * 3, "b": (ready_client,) * 3},
        uav_client=ready_client, fly_client=ready_client, uav_state=0,
        select_client=ready_client, land_client=ready_client, takeoff_client=ready_client,
        attach_client=ready_client, detach_client=ready_client,
        readiness_pub=Mock(), last_readiness_log=float("inf"))
    for state, expected in ((2, False), (3, True)):
        future = Future()
        future.set_result(SimpleNamespace(current_state=SimpleNamespace(id=state)))
        node.lifecycle_probes = {"/bt_navigator/get_state": [ready_client, future, False, float("inf")]}
        assert RelayMissionManager._interfaces_ready(node) is expected
        assert (node.readiness_pub.publish.call_args.args[0].data == "READY") is expected


def fake_manager(payloads):
    config_dir = Path(__file__).parents[1] / "config"
    config = CooperativeMissionConfig(config_dir / "cooperative_waypoints.yaml")
    waypoints = WaypointMap(Path(__file__).parents[2] / "uav_navigation/config/uav_delivery_waypoints.yaml")
    from dataclasses import replace
    target = config.resolve(["teaching_building"])[0]
    targets = [replace(target, delivery_floor=i+1, payload_mass_kg=mass)
               for i, mass in enumerate(payloads)]
    trace = []
    node = SimpleNamespace(config=config,
        relay=RelayConfig.load(config_dir / "relay_delivery.yaml", waypoints),
        battery_parameters=SimpleNamespace(maximum_payload_mass_kg=1.0),
        energy_constraints_enabled=False, docked=True, carrier_role="a", uav_state=0,
        _resolve_request_targets=lambda _: targets, _ordered=lambda batch: list(batch),
        _wait=lambda *args: True, _fresh=lambda role: True, _interfaces_ready=lambda: True,
        _require=RelayMissionManager._require, cargo={"a": 0, "b": 0},
        _set_phase=Mock(), _bridge=Mock(), _finish_failure=Mock(),
        home_role="a", optimized_route_pub=Mock(),
        stop_pubs={"a": Mock(), "b": Mock()}, lock=threading.Lock(), mission_active=True)
    node._cargo = lambda role, mass: node.cargo.__setitem__(role, mass)
    node._drive = lambda role, stop, phase, goal: trace.append(("drive", role, stop.name))
    def transfer(source, destination, mass, phase, goal):
        trace.append(("transfer", source, destination))
        node.docked, node.carrier_role, node.uav_state = True, destination, 0
    node._transfer = transfer
    node._release = lambda role, mass, goal: trace.append(("release", role))
    def floor(target, goal):
        trace.append(("floor", target.delivery_floor))
        return True, "Delivered"
    node._execute_uav_delivery = floor
    node._dock = lambda role, goal: trace.append(("dock", role))
    goal = SimpleNamespace(request=object(), succeed=Mock(), is_cancel_requested=False)
    return node, goal, trace


def test_closed_loop_requires_b_reception_floor_return_and_a_return():
    node, goal, trace = fake_manager([.3])
    result = RelayMissionManager._execute_mission(node, goal)
    assert result.success and result.completed_targets == 1
    assert trace == [
        ("drive", "b", "platform_b"), ("drive", "a", "platform_a"),
        ("transfer", "a", "b"), ("drive", "b", "teaching_door"),
        ("release", "b"), ("floor", 1), ("dock", "b"),
        ("drive", "b", "platform_b"), ("transfer", "b", "a"),
        ("drive", "a", "logistics_center")]
    goal.succeed.assert_called_once()
    assert not node.mission_active


def test_multiple_payload_batches_repeat_air_relay_without_losing_floors():
    node, goal, trace = fake_manager([.6, .6, .6])
    result = RelayMissionManager._execute_mission(node, goal)
    assert result.success and result.completed_targets == 3
    assert [step for step in trace if step[0] == "floor"] == [("floor", 1), ("floor", 2), ("floor", 3)]
    assert trace.count(("transfer", "a", "b")) == trace.count(("transfer", "b", "a")) == 3
    assert trace[-1] == ("drive", "a", "logistics_center")


def test_failed_handoff_does_not_start_floor_delivery_or_report_success():
    node, goal, trace = fake_manager([.3])
    node._transfer = Mock(side_effect=RuntimeError("B landing failed"))
    RelayMissionManager._execute_mission(node, goal)
    assert not any(step[0] == "floor" for step in trace)
    goal.succeed.assert_not_called()
    assert node._finish_failure.call_args.args[2] == "B landing failed"
    assert not node.mission_active


def test_infeasible_whole_mission_does_not_start_either_car():
    node, goal, trace = fake_manager([.3])
    node.energy_constraints_enabled = True
    node._mission_energy_forecast = Mock(return_value=SimpleNamespace(
        feasible=False, reason="b_charge would fall below reserve"))
    node.energy_plan_pub = Mock()
    # asdict deliberately requires the real typed forecast for status serialization.
    from cooperative_delivery.relay_energy import RelayEnergyForecast
    node._mission_energy_forecast.return_value = RelayEnergyForecast(
        False, "b_charge would fall below reserve", {}, {}, 0., 0.)
    RelayMissionManager._execute_mission(node, goal)
    assert not trace
    goal.succeed.assert_not_called()
    assert "b_charge" in node._finish_failure.call_args.args[2]
    node.energy_plan_pub.publish.assert_called_once()


def test_ground_transit_charging_target_follows_next_sortie_not_old_default():
    from cooperative_delivery.relay_energy import RelayEnergyForecast
    node, goal, trace = fake_manager([.3])
    node.energy_constraints_enabled = True
    node.energy_plan_pub = Mock()
    node._mission_energy_forecast = lambda batches: RelayEnergyForecast(True, "OK", {}, {}, 1., 0.)
    node._transfer_energy_leg = lambda a, b, mass: f"{a}_to_{b}"
    node._floor_energy_leg = lambda target: "floor"
    node._prepare_charge_target = lambda leg: trace.append(("charge_target", leg))
    node._wait_charge_ready = lambda *args: (True, "Ready")
    result = RelayMissionManager._execute_mission(node, goal)
    assert result.success
    assert trace[0] == ("charge_target", "a_to_b")
    assert trace.index(("charge_target", "floor")) < trace.index(("drive", "b", "teaching_door"))
    assert trace[trace.index(("charge_target", "b_to_a")) + 1] == ("drive", "b", "platform_b")


def test_transfer_charging_timeout_uses_simulation_time_and_vm_wall_allowance():
    from cooperative_delivery.relay_energy import FlightLeg
    node = SimpleNamespace(energy_constraints_enabled=True,
        _transfer_energy_leg=lambda *args: FlightLeg("a", "b", 10., 30., 35.),
        battery_parameters=SimpleNamespace(capacity_wh=100.),
        get_parameter=lambda name: SimpleNamespace(value=2000.),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
        uav_soc=.8, charge_target_pub=Mock(), energy_plan_pub=Mock(),
        _require=RelayMissionManager._require)
    node._wait = Mock(side_effect=lambda predicate, goal, timeout: predicate())
    RelayMissionManager._charge_for_transfer(node, "a", "b", .3, object())
    assert node._wait.call_args.args[2] == 20000.
    assert node.charge_target_pub.publish.call_args.args[0].data == pytest.approx(.35)


def test_capture_retries_only_transient_physics_guard_after_new_settle_window():
    node = SimpleNamespace(_role=Mock(), _fresh=lambda role: True,
        _wait_ugv_settled=lambda goal: True, uav_state=0,
        _wait_capture_ready=Mock(return_value=True), attach_client=object(),
        _call_docking_service=Mock(side_effect=[
            (False, "Both vehicles must be settled before capture; uav_angular=0.21"),
            (True, "Docked")]), _require=RelayMissionManager._require,
        get_logger=lambda: Mock(), _wait=lambda *args: True,
        payload_pub=Mock(), _clear_costmaps=lambda: True)
    RelayMissionManager._dock(node, "b", object())
    assert node._wait_capture_ready.call_count == 2
    assert node._call_docking_service.call_count == 2
    node._call_docking_service = Mock(return_value=(False, "UAV is outside the docking capture distance"))
    with pytest.raises(RuntimeError, match="outside"):
        RelayMissionManager._dock(node, "b", object())
    assert node._call_docking_service.call_count == 1


def test_live_configuration_whole_mission_forecast_preserves_mass_floor_and_positions():
    from uav_control.battery_model import BatteryParameters
    from cooperative_delivery.ugv_energy_model import UgvEnergyParameters
    root = Path(__file__).parents[2]
    node, _, _ = fake_manager([.3])
    node.waypoints = WaypointMap(root / "uav_navigation/config/uav_delivery_waypoints.yaml")
    node.battery_parameters = BatteryParameters.from_yaml(
        root / "uav_control/config/battery_model.yaml").for_backend("px4")
    node.ugv_energy_parameters = UgvEnergyParameters.from_mapping({})
    node.uav_soc, node.charge_policy = .8, "as_needed"
    node.energy = {"a": [240., 200.], "b": [240., 200.]}
    def pose(stop):
        return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=stop)))
    node.poses = {"a": pose(node.config.ugv_home), "b": pose(node.relay.platform_b.stop)}
    node._distance = lambda a, b: ((a.x - b.x)**2 + (a.y - b.y)**2)**.5
    node.get_parameter = lambda name: SimpleNamespace(value=1.)
    node._transfer_energy_leg = lambda *args: RelayMissionManager._transfer_energy_leg(node, *args)
    node._floor_energy_leg = lambda target: RelayMissionManager._floor_energy_leg(node, target)
    targets = node._resolve_request_targets(None)
    first = RelayMissionManager._mission_energy_forecast(node, [targets])
    assert first.feasible and first.duration_s > 0
    from dataclasses import replace
    higher = [replace(targets[0], delivery_floor=3, payload_mass_kg=.6)]
    second = RelayMissionManager._mission_energy_forecast(node, [higher])
    assert second.duration_s >= first.duration_s
    # Both may finish at the same SOC after replenishment; source packs pay
    # the extra flight energy, so compare complete-system net energy instead.
    assert sum(second.consumed_wh.values()) > sum(first.consumed_wh.values())
    assert node.energy == {"a": [240., 200.], "b": [240., 200.]}
