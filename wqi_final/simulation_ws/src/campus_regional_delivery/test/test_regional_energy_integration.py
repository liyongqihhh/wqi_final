import json
import time
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from builtin_interfaces.msg import Time
from std_msgs.msg import String

from campus_regional_delivery.energy_manager import RegionalEnergyManager, battery_message
from campus_regional_delivery.energy_mission import RegionalEnergyMixin
from campus_regional_delivery.energy_model import FleetEnergyModel, NAMESPACES
from campus_regional_delivery.energy_model import CargoLedger
from campus_regional_delivery.layout import RegionalLayout
from campus_regional_delivery.mission_manager import RegionalMissionManager
from test_regional_energy_model import configuration
from pathlib import Path


def adapter():
    settings, air, ground, _ = configuration()
    node = SimpleNamespace(model=FleetEnergyModel(settings,air,ground),air_parameters=air,
        session_id="current",command_sequence=0,command_error="",publish=Mock(),get_logger=Mock())
    return node


def command(node, **overrides):
    values = dict(session_id="current",previous_sequence=0,sequence=1,
        cargo_kg=dict.fromkeys(NAMESPACES,0.),payload_kg=0.,charge_target_soc=.9)
    values.update(overrides)
    RegionalEnergyManager.command(node,String(data=json.dumps(values)))


def test_cargo_command_acknowledges_only_complete_current_session_transaction():
    node=adapter()
    cargo=dict.fromkeys(NAMESPACES,0.);cargo["ugv_02"]=.3
    command(node,cargo_kg=cargo,payload_kg=.2)
    assert node.command_sequence==1 and not node.command_error
    assert node.model.ledger.cargo["ugv_02"]==.3 and node.model.ledger.payload==.2
    assert node.model.charge_target==.9
    command(node,sequence=2,previous_sequence=1,payload_kg=.1)
    assert node.command_sequence==2 and node.model.ledger.payload==.1


@pytest.mark.parametrize("override",[
    {"session_id":"previous"},{"sequence":2},{"previous_sequence":7},
    {"payload_kg":2.},{"cargo_kg":{"ugv_00":.3}},{"charge_target_soc":1.1},
])
def test_invalid_command_cannot_partially_change_cargo_or_charge_target(override):
    node=adapter();before=node.model.status()
    command(node,**override)
    assert node.command_sequence==0 and node.command_error
    assert node.model.status()==before


def test_battery_messages_have_negative_discharge_current_and_constant_ah_capacity():
    low=battery_message(Time(),"uav/base_link","uav",20.,100.,13.,130.,14.8)
    high=battery_message(Time(),"uav/base_link","uav",80.,100.,16.,-160.,14.8)
    assert low.current==-10 and high.current==10
    assert low.capacity==high.capacity==pytest.approx(100/14.8)
    assert low.percentage==pytest.approx(.2) and high.percentage==pytest.approx(.8)
    assert low.power_supply_status==low.POWER_SUPPLY_STATUS_DISCHARGING
    assert high.power_supply_status==high.POWER_SUPPLY_STATUS_CHARGING


def gate():
    settings, air, ground, _ = configuration()
    cargo=dict.fromkeys(NAMESPACES,0.)
    data={"energy_wh":FleetEnergyModel(settings,air,ground).energies(),"cargo_kg":cargo,"payload_kg":0.}
    p=SimpleNamespace(x=-56.,y=-28.,z=0.)
    node=SimpleNamespace(energy_data=lambda:data,carrier="ugv_00",uav_state=0,
        poses={"ugv_00":SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=p)))},
        energy_planner=SimpleNamespace(flight=lambda *_:(10.,30.)),cargo_ledger=SimpleNamespace(payload=.3),
        energy_settings=settings,uav_energy_parameters=air,ugv_energy_parameters=ground,
        layout=SimpleNamespace(hub={"uav_pad":[-56.,-28.,.42,0.]}),
        sync_energy_cargo=Mock(),phase_to=Mock(),wait=Mock(),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000)))
    return node,data


def test_sufficient_energy_takes_off_without_wait_but_sets_finite_charge_target():
    node,data=gate()
    RegionalEnergyMixin.energy_before_flight(node,(-18.,22.),Mock())
    node.sync_energy_cargo.assert_called_once()
    target=node.sync_energy_cargo.call_args.args[1]
    assert target==pytest.approx(.405)
    node.wait.assert_not_called()


def test_low_uav_energy_waits_for_real_source_instead_of_resetting_soc():
    node,data=gate();data["energy_wh"]["uav"]=25.
    RegionalEnergyMixin.energy_before_flight(node,(-18.,22.),Mock())
    node.wait.assert_called_once()
    assert data["energy_wh"]["uav"]==25.
    predicate=node.wait.call_args.args[0]
    assert not predicate()
    data["energy_wh"]["uav"]=40.5
    assert predicate()


def test_empty_actual_carrier_charger_rejects_before_any_takeoff_or_cargo_command():
    node,data=gate();data["energy_wh"]["uav"]=25.
    data["energy_wh"]["ugv_00_charge"]=node.ugv_energy_parameters.charging_reserve_wh
    with pytest.raises(RuntimeError,match="charging pack cannot supply"):
        RegionalEnergyMixin.energy_before_flight(node,(-18.,22.),Mock())
    node.sync_energy_cargo.assert_not_called()
    node.wait.assert_not_called()


def test_airborne_carrier_flag_never_allows_charging_admission():
    node,_=gate();node.uav_state=1
    with pytest.raises(RuntimeError,match="landed UAV locked"):
        RegionalEnergyMixin.energy_before_flight(node,(-18.,22.),Mock())


def test_low_drive_pack_interrupts_mission_even_when_its_vehicle_is_parked():
    node,data=gate();node.last_energy_guard=0.;node.delivery_mode="cooperative"
    data["energy_wh"]["ugv_03_drive"]=node.ugv_energy_parameters.drive_reserve_wh-.1
    with pytest.raises(RuntimeError,match="ugv_03 drive battery"):
        RegionalEnergyMixin.energy_guard(node)


def test_charge_gate_exhaustion_fails_without_fabricating_battery_progress():
    node,data=gate();data["energy_wh"]["uav"]=25.
    RegionalEnergyMixin.energy_before_flight(node,(-18.,22.),Mock())
    data["energy_wh"]["ugv_00_charge"]=node.ugv_energy_parameters.charging_reserve_wh
    with pytest.raises(RuntimeError,match="charging pack exhausted"):
        node.wait.call_args.args[0]()


def telemetry_gate():
    node,data=gate()
    data.update(ready=True,session_id="current",sample_sim_ns=1_000_000_000)
    node.energy_snapshot=data
    node.energy_received=time.monotonic()
    node.energy_session=""
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=900_000_000))
    return node,data


def test_one_clock_delivery_cycle_of_future_energy_does_not_spuriously_reject():
    node,data=telemetry_gate()
    assert RegionalEnergyMixin.energy_data(node) is data


def test_large_future_timestamp_is_rejected_as_mixed_or_reset_clock():
    node,data=telemetry_gate();data["sample_sim_ns"]=3_000_000_000
    with pytest.raises(RuntimeError,match="Energy telemetry stale"):
        RegionalEnergyMixin.energy_data(node)


def test_changed_energy_session_aborts_instead_of_resetting_mission_budget():
    node,_=telemetry_gate();node.energy_session="previous"
    with pytest.raises(RuntimeError,match="restarted during a mission"):
        RegionalEnergyMixin.energy_data(node)


def test_task_energy_readiness_distinguishes_late_subscription_from_ready():
    node=SimpleNamespace(energy_data=Mock(side_effect=RuntimeError("not received")),energy_ready_pub=Mock())
    RegionalEnergyMixin.publish_energy_readiness(node)
    assert node.energy_ready_pub.publish.call_args.args[0].data=="WAITING:not received"
    node.energy_data.side_effect=None
    RegionalEnergyMixin.publish_energy_readiness(node)
    assert node.energy_ready_pub.publish.call_args.args[0].data=="READY"


@pytest.mark.parametrize("mode",["cooperative","ugv_only"])
def test_full_task_state_machine_conserves_parcel_ownership_with_energy_enabled(mode):
    # Actions are stubbed: this checks state-machine wiring, not physical flight.
    manager=Mock(spec=RegionalMissionManager)
    manager.layout=RegionalLayout(Path(__file__).parents[1]/"config/campus_regions.yaml")
    manager.delivery_mode=mode;manager.enable_energy=True;manager.energy_session="current"
    manager.shutdown_requested=False;manager.uav_state=0;manager.carrier="ugv_00"
    manager.paused_navigation=set();manager.busy=True;manager.busy_lock=threading.Lock()
    manager.payload_pub=Mock();manager.event_pub=Mock()
    p=SimpleNamespace(x=-56.,y=-28.,z=.425)
    manager.uav_pose=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=p)))
    manager.poses={ns:SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=p))) for ns in NAMESPACES}
    manager.cargo_ledger=CargoLedger()
    manager.batches.return_value=[("A",[
        {"target":"innovation_center","mass":.3,"floor":3 if mode=="cooperative" else 0},
        {"target":"library","mass":.4,"floor":0}])]
    def load(*_):
        manager.cargo_ledger.assign(dict.fromkeys(NAMESPACES,0.)|{"ugv_00":.7},0.)
    manager.energy_start.side_effect=load
    transfers=[]
    def transfer(source,destination,mass,goal):
        manager.cargo_ledger.transfer(source,destination,mass)
        transfers.append((source,destination,mass))
    manager.energy_transfer.side_effect=transfer
    manager.air_transport.side_effect=lambda point,goal,namespace:setattr(manager,"carrier",namespace)
    manager.floor_delivery.side_effect=lambda *args:RegionalMissionManager.floor_delivery(manager,*args)
    manager.ground_delivery.side_effect=lambda *args:RegionalMissionManager.ground_delivery(manager,*args)
    goal=Mock(request=SimpleNamespace(return_home=True),is_cancel_requested=False)
    result=RegionalMissionManager.execute(manager,goal)
    assert result.success and result.completed_targets==2
    assert sum(manager.cargo_ledger.cargo.values())+manager.cargo_ledger.payload<1e-6
    assert sum(m for _,destination,m in transfers if destination=="delivered")==pytest.approx(.7)
    if mode=="cooperative":
        assert transfers[:2]==[("ugv_00","uav",.7),("uav","ugv_01",.7)]
        manager.energy_before_flight.assert_called_once()
    else:
        assert all(source=="ugv_00" and destination=="delivered" for source,destination,_ in transfers)
        manager.air_transport.assert_not_called()
    manager.energy_finish.assert_called_once()
    goal.succeed.assert_called_once()
    goal.abort.assert_not_called()
