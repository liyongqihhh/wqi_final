"""Nine finite packs sharing one simulation-time integration transaction."""
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path

import yaml

from cooperative_delivery.ugv_energy_model import UgvEnergyModel, UgvEnergyParameters
from uav_control.battery_model import BatteryModel, BatteryParameters


NAMESPACES = tuple(f"ugv_0{i}" for i in range(4))


def load_energy_configuration(settings_file, uav_file, ugv_file):
    settings = yaml.safe_load(Path(settings_file).read_text())
    if not isinstance(settings, dict):
        raise ValueError("Regional energy settings must be a mapping")
    for name in ("uav_initial_soc", "idle_charge_target_soc"):
        value = settings[name]
        if not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 1:
            raise ValueError(name + " must be in [0, 1]")
    for name in ("initial_drive_soc", "initial_charging_soc"):
        values = settings[name]
        if set(values) != set(NAMESPACES) or any(
                not isinstance(v, (int, float)) or not math.isfinite(v) or not 0 <= v <= 1
                for v in values.values()):
            raise ValueError(name + " must contain four finite fractions")
    if settings["charging_policy"] not in ("as_needed", "full"):
        raise ValueError("charging_policy must be as_needed or full")
    for name in ("telemetry_timeout_sim_s", "telemetry_timeout_wall_s", "integration_step_s",
                 "update_rate_hz", "publish_rate_hz", "maximum_acceleration_mps2"):
        if not math.isfinite(settings[name]) or settings[name] <= 0:
            raise ValueError(name + " must be positive")
    for name in ("parked_service_allowance_s", "drive_turn_allowance_s"):
        if not math.isfinite(settings[name]) or settings[name] < 0:
            raise ValueError(name + " must be nonnegative")
    if not math.isfinite(settings["flight_time_margin"]) or settings["flight_time_margin"] < 1:
        raise ValueError("flight_time_margin must be at least one")
    uav = BatteryParameters.from_yaml(uav_file).for_backend("px4")
    ugv = replace(UgvEnergyParameters.from_yaml(ugv_file), docked_uav_mass_kg=uav.base_mass_kg)
    digest = hashlib.sha256(b"\0".join(Path(p).read_bytes()
        for p in (settings_file, uav_file, ugv_file))).hexdigest()
    return settings, uav, ugv, digest


class CargoLedger:
    """A parcel is on one carrier, on the UAV, or delivered, never on both."""
    def __init__(self):
        self.cargo = dict.fromkeys(NAMESPACES, 0.)
        self.payload = 0.

    def assign(self, cargo, payload, maximum_payload=1.):
        if set(cargo) != set(NAMESPACES):
            raise ValueError("All four cargo values are required")
        values = [*cargo.values(), payload]
        if any(not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0 for v in values):
            raise ValueError("Cargo masses must be finite and nonnegative")
        if payload > maximum_payload + 1e-6:
            raise ValueError("UAV cargo exceeds capacity")
        self.cargo = {key: float(value) for key, value in cargo.items()}
        self.payload = float(payload)

    def transfer(self, source, destination, mass):
        if (source == destination or source not in (*NAMESPACES, "uav")
                or destination not in (*NAMESPACES, "uav", "delivered")
                or not math.isfinite(mass) or mass < 0):
            raise ValueError("Invalid cargo transfer")
        cargo, payload = dict(self.cargo), self.payload
        available = payload if source == "uav" else cargo[source]
        if available + 1e-6 < mass:
            raise ValueError("Transfer exceeds cargo actually on the source")
        if source == "uav":
            payload = max(0., payload - mass)
        else:
            cargo[source] = max(0., cargo[source] - mass)
        if destination == "uav":
            payload += mass
        elif destination != "delivered":
            cargo[destination] += mass
        self.assign(cargo, payload)


class FleetEnergyModel:
    def __init__(self, settings, uav_parameters, ugv_parameters, with_uav=True):
        self.settings, self.uav_parameters, self.ugv_parameters = settings, uav_parameters, ugv_parameters
        self.with_uav = with_uav
        self.uav = BatteryModel(uav_parameters, settings["uav_initial_soc"]) if with_uav else None
        self.cars = {ns: UgvEnergyModel(ugv_parameters, settings["initial_drive_soc"][ns],
                    settings["initial_charging_soc"][ns]) for ns in NAMESPACES}
        self.ledger = CargoLedger()
        self.charge_target = settings["idle_charge_target_soc"]
        self.charging_carrier = ""
        self.input_power_w = 0.
        self.initial_energy_wh = sum(self.energies().values())

    def energies(self):
        return ({"uav": self.uav.energy_wh} if self.uav else {}) | {
            f"{ns}_{kind}": getattr(model, attribute) for ns, model in self.cars.items()
            for kind, attribute in (("drive", "drive_energy_wh"), ("charge", "charging_energy_wh"))}

    def step(self, dt, motion, velocity=(0., 0., 0.), acceleration=(0., 0., 0.),
             phase="IDLE", flying=False, carrying="", charge_allowed=False):
        if (not math.isfinite(dt) or dt <= 0 or set(motion) != set(NAMESPACES)
                or carrying not in ("", *NAMESPACES)
                or len(velocity) != 3 or len(acceleration) != 3
                or any(len(values) != 3 for values in motion.values())
                or not all(math.isfinite(v) for values in [*motion.values(), velocity, acceleration] for v in values)):
            raise ValueError("Finite complete motion inputs and a positive dt are required")
        owner = carrying if self.uav and charge_allowed and not flying else ""
        requested = 0.
        if owner:
            b = self.uav_parameters
            deficit = max(0., b.capacity_wh * self.charge_target - self.uav.energy_wh)
            requested = min(b.charge_power_w,
                (deficit * 3600 / dt + b.docked_idle_power_w) / b.charge_efficiency)
        delivered = 0.
        for ns, car in self.cars.items():
            # Payload still aboard a docked UAV is carried by the car too, but
            # is not inserted into the ground cargo ledger a second time.
            cargo = self.ledger.cargo[ns] + (self.ledger.payload if carrying == ns else 0.)
            snapshot = car.step(dt, *motion[ns], cargo, carrying == ns, 0.,
                uav_charging_input_power_w=requested if ns == owner else 0.)
            if ns == owner:
                delivered = snapshot.charging_output_power_w
        if self.uav:
            vh = math.hypot(*velocity[:2])
            self.uav.step(dt, phase, flying, bool(owner), horizontal_speed_mps=vh,
                vertical_speed_mps=velocity[2], horizontal_acceleration_mps2=math.hypot(*acceleration[:2]),
                vertical_acceleration_mps2=acceleration[2],
                horizontal_acceleration_velocity_dot_m2ps3=sum(a*v for a,v in zip(acceleration[:2], velocity[:2])),
                payload_mass_kg=self.ledger.payload, charge_target_soc=self.charge_target,
                charging_input_power_w=delivered if owner else None)
        self.charging_carrier = owner if delivered > 0 else ""
        self.input_power_w = delivered

    def status(self):
        return {"energy_wh": self.energies(), "cargo_kg": dict(self.ledger.cargo),
            "payload_kg": self.ledger.payload, "charging_carrier": self.charging_carrier,
            "charging_input_power_w": self.input_power_w, "charge_target_soc": self.charge_target,
            "system_net_consumed_wh": self.initial_energy_wh - sum(self.energies().values()),
            "uav_consumed_wh": self.uav.consumed_energy_wh if self.uav else 0.,
            "uav_charged_wh": self.uav.charged_energy_wh if self.uav else 0.,
            "drive_consumed_wh": {ns: car.drive_consumed_wh for ns, car in self.cars.items()},
            "charging_source_consumed_wh": {ns: car.charging_consumed_wh for ns, car in self.cars.items()}}


def fresh_sample(stamp_ns, received_wall, now_ns, now_wall, settings):
    return (stamp_ns is not None and received_wall is not None
        # Independent /clock subscriptions may trail odometry/status by one
        # delivery cycle. Large future timestamps (reset/mixed worlds) fail.
        and -.25 <= (now_ns - stamp_ns)/1e9 <= settings["telemetry_timeout_sim_s"]
        and 0 <= now_wall - received_wall <= settings["telemetry_timeout_wall_s"])


def charge_pose_safe(car_position, air_position, offset=.42):
    values = (*car_position, *air_position)
    return (all(math.isfinite(v) for v in values)
        and math.dist(car_position[:2], air_position[:2]) <= .12
        and abs(air_position[2] - car_position[2] - offset) <= .05)
