from dataclasses import replace
from pathlib import Path

import pytest

from campus_regional_delivery.energy_model import FleetEnergyModel, NAMESPACES
from campus_regional_delivery.energy_plan import RegionalEnergyPlanner
from campus_regional_delivery.layout import RegionalLayout,partition_orders
from test_regional_energy_model import configuration


def setup():
    settings,air,ground,_=configuration()
    layout=RegionalLayout(Path(__file__).parents[1]/"config/campus_regions.yaml")
    planner=RegionalEnergyPlanner(layout,settings,air,ground)
    initial=FleetEnergyModel(settings,air,ground).energies()
    positions={v["namespace"]:v["depot_pose"][:2] for v in layout.vehicles.values()}
    return planner,initial,positions


@pytest.mark.parametrize("target,floor",[("teaching_building",3),("dormitory_1",3),("cafeteria",2)])
def test_region_floor_forecast_budgets_both_air_legs_and_all_nine_batteries(target,floor):
    planner,energy,positions=setup()
    before=dict(energy)
    batches=partition_orders(planner.layout,[target],[.3],[floor])
    result=planner.assess(batches,True,positions,energy)
    assert result["feasible"] and len(result["remaining_wh"])==9
    assert result["duration_s"]>0 and energy==before
    air=[step for step in result["steps"] if step["type"]=="FlightLeg"]
    assert len(air)==3 and air[0]["source"]==air[-1]["destination"]=="ugv_00"
    for ns in NAMESPACES:
        assert result["consumed_wh"][ns+"_drive"]>0


@pytest.mark.parametrize("key",["uav",*[f"ugv_0{i}_drive" for i in range(4)],*[f"ugv_0{i}_charge" for i in range(4)]])
def test_infeasible_budget_is_refused_before_movement(key):
    planner,energy,positions=setup()
    energy[key]=0.
    if key=="uav":
        energy["ugv_00_charge"]=planner.ugv.charging_reserve_wh
    batches=partition_orders(planner.layout,["innovation_center"],[.3],[3])
    assert not planner.assess(batches,True,positions,energy)["feasible"]


def test_low_aircraft_soc_can_only_be_replenished_from_a_finite_correct_source():
    planner,energy,positions=setup();energy["uav"]=10.
    batches=partition_orders(planner.layout,["innovation_center"],[.3],[3])
    result=planner.assess(batches,True,positions,energy)
    assert result["feasible"] and result["charging_wait_s"]>0
    assert result["consumed_wh"]["ugv_00_charge"]>0
    assert result["consumed_wh"]["ugv_02_charge"]==result["consumed_wh"]["ugv_03_charge"]==0


def test_offline_charger_does_not_invent_idle_debt_when_uav_can_finish_without_charging():
    planner,energy,positions=setup();energy["uav"]=planner.uav.capacity_wh
    for ns in NAMESPACES:
        energy[ns+"_charge"]=planner.ugv.charging_reserve_wh
    batches=partition_orders(planner.layout,["innovation_center"],[.3],[3])
    result=planner.assess(batches,True,positions,energy)
    assert result["feasible"] and result["charging_wait_s"]==0
    for ns in NAMESPACES:
        assert result["remaining_wh"][ns+"_charge"]==planner.ugv.charging_reserve_wh


def test_nonreturn_order_still_reserves_safe_hub_return():
    planner,energy,positions=setup()
    batches=partition_orders(planner.layout,["innovation_center"],[.3])
    no_return=planner.assess(batches,False,positions,energy)
    returned=planner.assess(batches,True,positions,energy)
    assert no_return["remaining_wh"]==returned["remaining_wh"]


def test_multi_region_capacity_batches_and_payload_energy_are_not_lost():
    planner,energy,positions=setup()
    batches=partition_orders(planner.layout,["library","laboratory","dormitory_4","gymnasium"],[.6,.6,.2,.3])
    result=planner.assess(batches,True,positions,energy)
    assert [r for r,_ in batches]==["A","A","B","C"]
    assert len([s for s in result["steps"] if s["type"]=="FlightLeg"])==8


def test_ground_baseline_uses_same_road_network_and_eight_real_packs():
    planner,energy,positions=setup();energy.pop("uav")
    batches=partition_orders(planner.layout,["teaching_building","cafeteria"],[.3,.4])
    result=planner.assess(batches,True,positions,energy,ground_only=True)
    assert result["feasible"] and len(result["remaining_wh"])==8
    assert not any(step["type"]=="FlightLeg" for step in result["steps"])
    assert result["consumed_wh"]["ugv_00_drive"]>result["consumed_wh"]["ugv_01_drive"]


def test_new_road_graph_connects_all_stops_without_modifying_layout():
    planner,_,_=setup()
    stops=list(planner.layout.all_stops().values())
    for first in stops:
        for second in stops:
            distance=planner.road.distance(first[:2],second[:2])
            assert distance>=0 and distance==pytest.approx(planner.road.distance(second[:2],first[:2]))


def test_heavier_air_payload_costs_more_and_full_charging_has_real_source_cost():
    planner,energy,positions=setup()
    assert planner.flight((-56,-28),(-18,22),.8)[0]>planner.flight((-56,-28),(-18,22),.1)[0]
    batches=partition_orders(planner.layout,["innovation_center"],[.3])
    partial=planner.assess(batches,True,positions,energy)
    planner.settings["charging_policy"]="full"
    full=planner.assess(batches,True,positions,energy)
    assert full["charging_wait_s"]>partial["charging_wait_s"]
    assert full["consumed_wh"]["ugv_00_charge"]>partial["consumed_wh"]["ugv_00_charge"]
