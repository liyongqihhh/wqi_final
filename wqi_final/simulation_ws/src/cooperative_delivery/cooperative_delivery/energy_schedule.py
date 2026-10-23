"""Bounded search over delivery order, battery reserves and docked charging."""

from dataclasses import dataclass
from itertools import permutations
import math

from cooperative_delivery.ugv_energy_model import UgvEnergyModel


@dataclass(frozen=True)
class DeliveryEnergy:
    payload_kg: float
    energy_wh: float
    duration_s: float


@dataclass(frozen=True)
class SchedulePlan:
    order: tuple[int, ...]
    feasible: bool
    distance_m: float
    duration_s: float
    energy_wh: float
    drive_energy_wh: float
    charging_source_wh: float
    charge_wait_s: float
    objective: float
    method: str = "EVALUATION"


def evaluate_schedule(order, deliveries, distances, ugv_parameters, battery_parameters,
                      initial_uav_wh, initial_drive_wh, available_charger_wh,
                      return_home=True, charge_policy="as_needed", time_weight=1.0,
                      energy_weight=1.0, charging_buffer_wh=1.0):
    """Use battery-side Wh and account for both charger conversion stages."""
    if charge_policy not in ("full", "as_needed"):
        raise ValueError("Unknown charge policy")
    order = tuple(order)
    n = len(deliveries)
    if sorted(order) != list(range(n)) or n == 0:
        raise ValueError("Order must contain each delivery exactly once")
    if len(distances) != n + 1 or any(len(row) != n + 1 for row in distances):
        raise ValueError("Distance matrix must include deliveries and home")
    values = [initial_uav_wh, initial_drive_wh, available_charger_wh,
              time_weight, energy_weight, charging_buffer_wh]
    values += [value for row in distances for value in row]
    values += [value for d in deliveries for value in
               (d.payload_kg, d.energy_wh, d.duration_s)]
    if not all(math.isfinite(v) and v >= 0 for v in values):
        raise ValueError("Schedule inputs must be finite and non-negative")
    if time_weight + energy_weight <= 0:
        raise ValueError("At least one objective weight must be positive")
    p, b = ugv_parameters, battery_parameters
    vehicle = UgvEnergyModel(p)
    energy = min(initial_uav_wh, b.capacity_wh)
    distance = duration = drive = source = wait_total = charged = consumed = 0.0
    payload = sum(d.payload_kg for d in deliveries)
    previous = n
    feasible = True
    for index in order:
        d = deliveries[index]
        leg = distances[previous][index]
        transit = leg / p.planning_speed_mps
        required = d.energy_wh + b.capacity_wh * b.reserve_percentage
        if required > b.capacity_wh:
            feasible = False
        target = (b.capacity_wh if charge_policy == "full"
                  else min(b.capacity_wh, required + charging_buffer_wh))
        idle = b.landed_idle_power_w
        net = b.net_charge_power_w
        # If charging finishes before arrival, account for subsequent standby
        # drain; if it cannot finish in transit, wait docked at the service stop.
        deficit = target - energy
        if deficit >= net * transit / 3600.0:
            stored = max(0.0, deficit)
            charge_s = stored * 3600.0 / net
            waiting = max(0.0, charge_s - transit)
            standby_wh = 0.0
        else:
            stored = max(0.0, (deficit + idle * transit / 3600.0) / (1 + idle / net))
            charge_s = stored * 3600.0 / net
            waiting = 0.0
            standby_wh = idle * max(0.0, transit - charge_s) / 3600.0
        charger_input_wh = (stored + b.docked_idle_power_w * charge_s / 3600.0) / b.charge_efficiency
        leg_source = charger_input_wh / p.charger_transfer_efficiency
        leg_source += p.charger_idle_power_w * (transit + waiting) / 3600.0
        source += leg_source
        energy += stored - standby_wh
        charged += stored
        consumed += standby_wh + d.energy_wh
        feasible = feasible and energy + 1e-8 >= required
        energy -= d.energy_wh
        drive += vehicle.drive_power_w(p.planning_speed_mps, 0, 0, payload, True) * transit / 3600.0
        payload = max(0.0, payload - d.payload_kg)
        drive += vehicle.drive_power_w(0, 0, 0, payload, False) * (waiting + d.duration_s) / 3600.0
        distance += leg
        duration += transit + waiting + d.duration_s
        wait_total += waiting
        previous = index
    if return_home:
        leg = distances[previous][n]
        transit = leg / p.planning_speed_mps
        drive += vehicle.drive_power_w(p.planning_speed_mps, 0, 0, 0, True) * transit / 3600.0
        # Optimized runs turn charging off after the final delivery. Full-policy
        # post-mission top-ups are not part of this finite delivery window.
        consumed += b.landed_idle_power_w * transit / 3600.0
        energy -= b.landed_idle_power_w * transit / 3600.0
        source += p.charger_idle_power_w * transit / 3600.0
        duration += transit
        distance += leg
    drive *= p.prediction_margin_factor
    feasible = (feasible and energy >= 0
                and drive + p.drive_reserve_wh <= initial_drive_wh
                and source <= available_charger_wh)
    total = drive + consumed + source - charged
    # Normalized time/energy references keep weights dimensionless.
    objective = time_weight * duration / 3600.0 + energy_weight * total / 100.0
    return SchedulePlan(order, feasible, distance, duration, total, drive,
                        source, wait_total, objective)


def optimize_schedule(deliveries, distances, ugv_parameters, battery_parameters,
                      initial_uav_wh, initial_drive_wh, available_charger_wh,
                      seed_order, **options):
    n = len(deliveries)
    if not 1 <= n <= 10:
        raise ValueError("Energy scheduling supports 1..10 delivery items")

    def evaluate(order):
        return evaluate_schedule(order, deliveries, distances, ugv_parameters,
                                 battery_parameters, initial_uav_wh,
                                 initial_drive_wh, available_charger_wh, **options)

    def rank(plan):
        return (not plan.feasible, plan.objective, plan.order)

    if n <= 6:
        best = min((evaluate(order) for order in permutations(range(n))), key=rank)
        method = "EXACT_PERMUTATIONS"
    else:
        seeds = {tuple(seed_order), tuple(range(n)),
                 tuple(sorted(range(n), key=lambda i: -deliveries[i].payload_kg))}
        best = min((evaluate(order) for order in seeds), key=rank)
        # Bounded 2-opt refinement; no global-optimality claim for 7..10 items.
        for _ in range(4):
            candidates = [best]
            for first in range(n - 1):
                for last in range(first + 1, n):
                    order = best.order[:first] + best.order[first:last + 1][::-1] + best.order[last + 1:]
                    candidates.append(evaluate(order))
            updated = min(candidates, key=rank)
            if updated.order == best.order:
                break
            best = updated
        method = "BOUNDED_2OPT"
    return SchedulePlan(**{**best.__dict__, "method": method})
