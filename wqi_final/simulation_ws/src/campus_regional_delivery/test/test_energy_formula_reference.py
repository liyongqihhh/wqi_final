"""Independent arithmetic references, not calls back into model helpers."""
from pathlib import Path
import math
import random

import pytest

from campus_regional_delivery.energy_model import FleetEnergyModel, NAMESPACES, load_energy_configuration

ROOT=Path(__file__).parents[2]


def configuration():
    return load_energy_configuration(ROOT/"campus_regional_delivery/config/regional_energy.yaml",
        ROOT/"uav_control/config/battery_model.yaml",ROOT/"cooperative_delivery/config/ugv_energy_model.yaml")


def reference_air_power(p, payload, vh, vz, ah, az, dot):
    mass=p.base_mass_kg+payload
    weight=mass*p.gravity_mps2
    rho,area=p.air_density_kgpm3,p.total_rotor_area_m2
    pi=(1+p.induced_power_correction)*weight**1.5/math.sqrt(2*rho*area)
    v0=math.sqrt(weight/(2*rho*area))
    kappa=math.sqrt(1+max(0.,4*mass**2*ah**2+rho**2*p.horizontal_flat_plate_area_m2**2*vh**4
        +4*mass*rho*p.horizontal_flat_plate_area_m2*dot*vh)/(4*weight**2))
    horizontal=(p.blade_profile_power_w*(1+3*vh**2/p.rotor_tip_speed_mps**2)
        +pi*kappa*math.sqrt(max(0.,math.sqrt(kappa**2+vh**4/(4*v0**4))-vh**2/(2*v0**2)))
        +.5*p.fuselage_drag_ratio*rho*p.rotor_solidity*area*vh**3)
    thrust=max(.05*weight,weight+mass*az+.5*rho*p.vertical_flat_plate_area_m2*vz*abs(vz))
    vertical=p.blade_profile_power_w+(1+p.induced_power_correction)*.5*thrust*(vz+math.sqrt(vz*vz+2*thrust/(rho*area)))
    hover=p.blade_profile_power_w+pi
    auxiliary=p.computer_power_w+p.lidar_power_w+p.camera_power_w+p.communication_power_w
    return (max(.2*hover,horizontal+vertical-hover)+auxiliary)/p.discharge_efficiency


@pytest.mark.parametrize("payload",[0.,.3,1.])
@pytest.mark.parametrize("vh,vz,ah,az,dot",[
    (0.,0.,0.,0.,0.),(1.2,0.,0.,0.,0.),(4.,0.,.8,0.,3.2),
    (1.2,0.,.8,0.,0.),(0.,.6,0.,.5,0.),(0.,-.6,0.,.5,0.),
    (0.,-.6,0.,-.5,0.),(2.,.4,.4,.2,.8)])
def test_air_power_and_energy_integral_match_reference(payload,vh,vz,ah,az,dot):
    settings,p,g,_=configuration()
    fleet=FleetEnergyModel(settings,p,g)
    fleet.ledger.assign(dict.fromkeys(NAMESPACES,0.),payload)
    before=fleet.uav.energy_wh
    fleet.step(10.,dict.fromkeys(NAMESPACES,(0.,0.,0.)),velocity=(vh,0.,vz),
        acceleration=(dot/vh if vh else 0., math.sqrt(max(0.,ah*ah-(dot/vh)**2)) if vh else ah,az),
        phase="CRUISE",flying=True)
    expected=reference_air_power(p,payload,vh,vz,ah,az,dot)
    assert fleet.uav.last_power_w==pytest.approx(expected,rel=1e-12)
    assert before-fleet.uav.energy_wh==pytest.approx(expected*10/3600,abs=1e-12)
    assert fleet.status()["energy_balance_ok"]


@pytest.mark.parametrize("owner",NAMESPACES)
@pytest.mark.parametrize("cargo,payload,v,omega,acc",[(0.,0.,0.,0.,0.),(.6,.3,.7,.3,.35),(2.,.5,.4,-.2,-.2)])
def test_drive_mass_two_packs_and_charge_losses_match_reference(owner,cargo,payload,v,omega,acc):
    settings,p,g,_=configuration();settings["uav_initial_soc"]=.4
    fleet=FleetEnergyModel(settings,p,g)
    fleet.charge_target=.8
    fleet.ledger.assign(dict.fromkeys(NAMESPACES,cargo),payload)
    before=fleet.energies()
    motion=dict.fromkeys(NAMESPACES,(0.,0.,0.));motion[owner]=(v,omega,acc)
    dt=10.
    fleet.step(dt,motion,carrying=owner,charge_allowed=True)
    mass=g.base_mass_kg+g.docked_uav_mass_kg+cargo+payload
    drive=(g.idle_power_w+(g.rolling_resistance_coefficient*mass*g.gravity_mps2*v
        +.5*g.air_density_kgpm3*g.aerodynamic_drag_area_m2*v**3+max(0.,mass*acc*v))/g.drivetrain_efficiency
        +g.linear_loss_power_w_per_mps*v+g.angular_power_w_per_radps*abs(omega))
    expected_source=(p.charge_power_w/g.charger_transfer_efficiency+g.charger_idle_power_w)*dt/3600
    expected_stored=(p.charge_power_w*p.charge_efficiency-p.docked_idle_power_w)*dt/3600
    after=fleet.energies()
    assert before[owner+"_drive"]-after[owner+"_drive"]==pytest.approx(drive*dt/3600,abs=1e-12)
    assert before[owner+"_charge"]-after[owner+"_charge"]==pytest.approx(expected_source,abs=1e-12)
    assert after["uav"]-before["uav"]==pytest.approx(expected_stored,abs=1e-12)
    assert fleet.status()["energy_balance_error_wh"]==pytest.approx(0.,abs=1e-10)


def test_nine_pack_accounting_survives_mixed_flight_docking_and_source_depletion():
    settings,p,g,_=configuration()
    settings["initial_drive_soc"]=dict.fromkeys(NAMESPACES,.001)
    settings["initial_charging_soc"]=dict.fromkeys(NAMESPACES,g.charging_reserve_percentage+.001)
    fleet=FleetEnergyModel(settings,p,g)
    rng=random.Random(42)
    for n in range(4000):
        owner=NAMESPACES[n%4]
        flying=n%3==0
        fleet.step(.1,{ns:(rng.uniform(0,.7),rng.uniform(-.4,.4),rng.uniform(-.3,.3)) for ns in NAMESPACES},
            velocity=(rng.uniform(0,2),0.,rng.uniform(-.6,.6)),flying=flying,
            carrying="" if flying else owner,charge_allowed=not flying)
        status=fleet.status()
        assert status["energy_balance_error_wh"]==pytest.approx(0.,abs=1e-8)
        assert all(v>=0 for v in status["energy_wh"].values())
    assert all(car.drive_energy_wh==0 for car in fleet.cars.values())
    assert all(car.charging_energy_wh==pytest.approx(g.charging_reserve_wh) for car in fleet.cars.values())


def test_balance_check_detects_unaccounted_energy_injection():
    settings,p,g,_=configuration();fleet=FleetEnergyModel(settings,p,g)
    fleet.cars["ugv_01"].drive_energy_wh+=.01
    assert not fleet.status()["energy_balance_ok"]
