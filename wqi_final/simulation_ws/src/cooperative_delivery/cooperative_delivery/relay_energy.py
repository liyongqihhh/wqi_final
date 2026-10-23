"""Serial five-battery admission forecast; no ROS, simulation or battery mutation."""

from dataclasses import dataclass
import math

from cooperative_delivery.ugv_energy_model import UgvEnergyModel


@dataclass(frozen=True)
class DriveLeg:
    role: str
    distance_m: float
    cargo_kg: float
    docked_owner: str


@dataclass(frozen=True)
class FlightLeg:
    source: str
    destination: str
    consumption_wh: float
    duration_s: float
    departure_target_wh: float


@dataclass(frozen=True)
class RelayEnergyForecast:
    feasible: bool
    reason: str
    remaining_wh: dict
    consumed_wh: dict
    duration_s: float
    charging_wait_s: float


def evaluate_relay_energy(steps, ugv, uav, initial_wh):
    """Forecast serial travel, idle loads, payload changes and finite charging.

    Opportunistic charging during ground transit is not credited. Travel uses
    configured speed and margin, not a bound on arbitrary dynamic-obstacle waits.
    """
    capacities = {"uav": uav.capacity_wh, **{
        f"{r}_{kind}": capacity for r in ("a", "b") for kind, capacity in (
            ("drive", ugv.drive_capacity_wh), ("charge", ugv.charging_capacity_wh))}}
    reserve = {"uav": uav.capacity_wh * uav.reserve_percentage, **{
        f"{r}_{kind}": value for r in ("a", "b") for kind, value in (
            ("drive", ugv.drive_reserve_wh), ("charge", ugv.charging_reserve_wh))}}
    if set(initial_wh) != set(capacities) or any(
            not isinstance(v, (float, int)) or not math.isfinite(v)
            or not 0 <= v <= capacities[k] for k, v in initial_wh.items()):
        raise ValueError("A fresh finite energy value is required for all five batteries")
    energy = dict(initial_wh)
    duration = charging = 0.0
    model = UgvEnergyModel(ugv)

    def failure(airborne=True):
        return next((f"{key} would fall below reserve ({value:.3f} Wh)"
                     for key, value in energy.items()
                     if value < (reserve[key] if key != "uav" or airborne else 0) - 1e-9), "")

    def idle(seconds, owner, moving=None, cargo=0.0):
        nonlocal duration
        duration += seconds
        for role in ("a", "b"):
            power = model.drive_power_w(ugv.planning_speed_mps, 0, 0, cargo, owner == role) \
                if moving == role else ugv.idle_power_w
            energy[f"{role}_drive"] -= power * seconds / 3600
        if owner:
            energy[f"{owner}_charge"] -= ugv.charger_idle_power_w * seconds / 3600

    reason = failure(False)
    for step in steps:
        if reason:
            break
        if isinstance(step, DriveLeg):
            if (step.role not in ("a", "b") or step.docked_owner not in ("", "a", "b")
                    or not all(math.isfinite(v) and v >= 0 for v in (step.distance_m, step.cargo_kg))):
                raise ValueError("Invalid relay drive leg")
            seconds = step.distance_m / ugv.planning_speed_mps * ugv.prediction_margin_factor
            idle(seconds, step.docked_owner, step.role, step.cargo_kg)
            # The runtime battery manager may use landed idle when charging is
            # inhibited by the target-SOC hysteresis; budget the larger load.
            standby_w = max(uav.docked_idle_power_w, uav.landed_idle_power_w)
            energy["uav"] -= standby_w * seconds / (3600 * uav.discharge_efficiency)
        elif isinstance(step, FlightLeg):
            if (step.source not in ("a", "b") or step.destination not in ("a", "b")
                    or not all(math.isfinite(v) and v >= 0 for v in (
                        step.consumption_wh, step.duration_s, step.departure_target_wh))
                    or step.departure_target_wh < step.consumption_wh + reserve["uav"]):
                raise ValueError("Invalid relay flight energy or missing safe return reserve")
            if step.departure_target_wh > uav.capacity_wh + 1e-9:
                reason = "Sortie plus return reserve exceeds UAV capacity"
                break
            deficit = max(0, step.departure_target_wh - energy["uav"])
            if deficit and uav.net_charge_power_w <= 0:
                reason = "Charger cannot overcome UAV docked idle power"
                break
            seconds = deficit * 3600 / uav.net_charge_power_w if deficit else 0.0
            # UAV charger efficiency and UGV DC transfer efficiency are distinct.
            source_wh = (deficit + uav.docked_idle_power_w * seconds / 3600) / (
                uav.charge_efficiency * ugv.charger_transfer_efficiency)
            idle(seconds, step.source)
            charging += seconds
            energy[f"{step.source}_charge"] -= source_wh
            energy["uav"] += deficit
            reason = failure()
            if reason:
                break
            energy["uav"] -= step.consumption_wh
            idle(step.duration_s, "")
        else:
            raise ValueError("Unknown relay energy step")
        reason = failure(isinstance(step, FlightLeg))
    return RelayEnergyForecast(not bool(reason), reason or "Nominal complete relay is feasible",
        energy, {k: initial_wh[k] - v for k, v in energy.items()}, duration, charging)
