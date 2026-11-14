from copy import deepcopy
from pathlib import Path

import pytest

from campus_regional_delivery.energy_model import (
    CargoLedger, FleetEnergyModel, NAMESPACES, charge_pose_safe, fresh_sample, load_energy_configuration,
)


ROOT = Path(__file__).parents[2]


def configuration():
    return load_energy_configuration(ROOT/"campus_regional_delivery/config/regional_energy.yaml",
        ROOT/"uav_control/config/battery_model.yaml",ROOT/"cooperative_delivery/config/ugv_energy_model.yaml")


def model(with_uav=True):
    settings, air, ground, _ = configuration()
    settings["uav_initial_soc"] = .4
    return FleetEnergyModel(settings,air,ground,with_uav)


def stationary():
    return dict.fromkeys(NAMESPACES,(0.,0.,0.))


def test_nine_independent_initial_packs_and_identical_px4_mass():
    settings,air,ground,digest = configuration()
    settings["initial_drive_soc"]["ugv_01"] = .32
    settings["initial_charging_soc"]["ugv_02"] = .47
    fleet = FleetEnergyModel(settings,air,ground)
    energy = fleet.energies()
    assert len(energy)==9 and len(digest)==64
    assert energy["ugv_01_drive"]==pytest.approx(.32*ground.drive_capacity_wh)
    assert energy["ugv_02_charge"]==pytest.approx(.47*ground.charging_capacity_wh)
    assert ground.docked_uav_mass_kg==pytest.approx(air.base_mass_kg)


@pytest.mark.parametrize("owner",NAMESPACES)
def test_only_actual_carrier_supplies_charge_and_losses_are_conserved(owner):
    fleet = model()
    before = fleet.energies()
    fleet.charge_target = .8
    fleet.step(10.,stationary(),carrying=owner,charge_allowed=True)
    after = fleet.energies()
    air,ground = fleet.uav_parameters,fleet.ugv_parameters
    stored = (air.charge_power_w*air.charge_efficiency-air.docked_idle_power_w)*10/3600
    source = (air.charge_power_w/ground.charger_transfer_efficiency+ground.charger_idle_power_w)*10/3600
    assert after["uav"]-before["uav"]==pytest.approx(stored)
    assert before[owner+"_charge"]-after[owner+"_charge"]==pytest.approx(source)
    for ns in NAMESPACES:
        if ns!=owner:
            assert before[ns+"_charge"]==after[ns+"_charge"]
    drive = sum(before[ns+"_drive"]-after[ns+"_drive"] for ns in NAMESPACES)
    assert sum(before.values())-sum(after.values())==pytest.approx(drive+source-stored)
    assert fleet.charging_carrier==owner


def test_exhausted_source_cannot_create_uav_energy_or_cross_source_reserve():
    fleet = model()
    reserve = fleet.ugv_parameters.charging_reserve_wh
    fleet.cars["ugv_03"].charging_energy_wh=reserve+.001
    before = fleet.uav.energy_wh
    for _ in range(10):
        fleet.step(1.,stationary(),carrying="ugv_03",charge_allowed=True)
    assert fleet.cars["ugv_03"].charging_energy_wh==pytest.approx(reserve)
    assert fleet.uav.energy_wh<before
    assert fleet.uav.charged_energy_wh==0
    assert fleet.input_power_w==0 and fleet.charging_carrier==""


def test_full_target_still_debits_converter_and_idle_load_but_does_not_overcharge():
    fleet = model()
    fleet.charge_target=fleet.uav.soc
    initial = fleet.energies()
    fleet.step(10.,stationary(),carrying="ugv_00",charge_allowed=True)
    assert fleet.uav.energy_wh==initial["uav"]
    assert fleet.cars["ugv_00"].charging_energy_wh<initial["ugv_00_charge"]


@pytest.mark.parametrize("flying,carrying,allowed",[(True,"ugv_00",True),(False,"",True),(False,"ugv_00",False)])
def test_no_charge_during_takeoff_or_without_verified_docking(flying,carrying,allowed):
    fleet=model()
    initial=fleet.uav.energy_wh
    fleet.step(1.,stationary(),flying=flying,carrying=carrying,charge_allowed=allowed)
    assert fleet.uav.energy_wh<initial and fleet.uav.charged_energy_wh==0


def test_mass_is_transferred_once_through_car_uav_receiver_and_delivery():
    ledger=CargoLedger()
    cargo=dict(ledger.cargo,ugv_00=.6)
    ledger.assign(cargo,0.)
    for source,destination in (("ugv_00","uav"),("uav","ugv_02"),("ugv_02","uav")):
        ledger.transfer(source,destination,.6)
        assert sum(ledger.cargo.values())+ledger.payload==pytest.approx(.6)
    ledger.transfer("uav","delivered",.6)
    assert sum(ledger.cargo.values())+ledger.payload==0
    with pytest.raises(ValueError):
        ledger.transfer("ugv_02","uav",.6)


def test_docked_uav_payload_affects_car_mass_without_duplicating_ledger_cargo():
    fleet=model()
    fleet.ledger.assign(dict(fleet.ledger.cargo,ugv_01=.2),.3)
    motion=stationary();motion["ugv_01"]=(.4,0.,.2)
    fleet.step(1.,motion,carrying="ugv_01",charge_allowed=True)
    snapshot=fleet.cars["ugv_01"].snapshot()
    assert snapshot.total_carried_mass_kg==pytest.approx(fleet.ugv_parameters.base_mass_kg+.2+.3+fleet.uav_parameters.base_mass_kg)
    assert fleet.status()["cargo_kg"]["ugv_01"]==.2 and fleet.status()["payload_kg"]==.3
    assert fleet.cars["ugv_00"].snapshot().total_carried_mass_kg==fleet.ugv_parameters.base_mass_kg


@pytest.mark.parametrize("dt,value",[(-1.,0.),(1.,float("nan")),(float("inf"),0.)])
def test_invalid_integration_does_not_mutate_any_pack(dt,value):
    fleet=model();before=fleet.energies();motion=stationary();motion["ugv_00"]=(value,0.,0.)
    with pytest.raises(ValueError):
        fleet.step(dt,motion)
    assert fleet.energies()==before


def test_ground_only_publishes_no_fictitious_uav_energy():
    fleet=model(False)
    assert len(fleet.energies())==8 and "uav" not in fleet.energies()
    fleet.step(1.,stationary())
    assert fleet.status()["uav_charged_wh"]==0


@pytest.mark.parametrize("car,air,expected",[
    ((0,0,.005),(0,0,.425),True),((0,0,.005),(0,0,.8),False),
    ((0,0,.005),(.4,0,.425),False),((0,0,.005),(0,0,float("nan")),False)])
def test_carrier_flag_does_not_override_actual_landing_geometry(car,air,expected):
    assert charge_pose_safe(car,air) is expected


@pytest.mark.parametrize("stamp,wall,now,expected",[(10_000_000_000,100.,11_000_000_000,True),
    (11_100_000_000,100.,11_000_000_000,True),
    (1_000_000_000,100.,11_000_000_000,False),(12_000_000_000,100.,11_000_000_000,False),
    (None,None,11_000_000_000,False)])
def test_energy_freshness_uses_simulation_time_and_bounded_wall_watchdog(stamp,wall,now,expected):
    settings,_,_,_=configuration()
    assert fresh_sample(stamp,wall,now,101.,settings) is expected
