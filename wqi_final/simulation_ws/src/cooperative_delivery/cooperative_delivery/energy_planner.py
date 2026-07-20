from dataclasses import dataclass
import math


@dataclass(frozen=True)
class EnergySortie:
    target_name: str
    launch_x: float
    launch_y: float
    mission_energy_wh: float


@dataclass(frozen=True)
class EnergyPlanStep:
    target_name: str
    ugv_distance_m: float
    minimum_charge_wh: float
    takeoff_energy_wh: float
    mission_energy_wh: float
    landing_energy_wh: float


@dataclass(frozen=True)
class CooperativeEnergyPlan:
    feasible: bool
    steps: tuple[EnergyPlanStep, ...]
    final_energy_wh: float
    message: str


def plan_cooperative_energy(
    initial_energy_wh: float,
    battery_capacity_wh: float,
    reserve_energy_wh: float,
    net_charge_power_w: float,
    ugv_planning_speed_mps: float,
    initial_x: float,
    initial_y: float,
    sorties: list[EnergySortie],
) -> CooperativeEnergyPlan:
    values = (
        initial_energy_wh,
        battery_capacity_wh,
        reserve_energy_wh,
        net_charge_power_w,
        ugv_planning_speed_mps,
        initial_x,
        initial_y,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Cooperative energy planning values must be finite")
    if battery_capacity_wh <= 0.0 or ugv_planning_speed_mps <= 0.0:
        raise ValueError("Battery capacity and UGV speed must be positive")
    if (
        initial_energy_wh < 0.0
        or reserve_energy_wh < 0.0
        or net_charge_power_w < 0.0
    ):
        raise ValueError("Energy and charging values cannot be negative")
    if not sorties:
        raise ValueError("At least one UAV sortie is required")

    energy = min(initial_energy_wh, battery_capacity_wh)
    previous_x = initial_x
    previous_y = initial_y
    steps = []
    for sortie in sorties:
        sortie_values = (
            sortie.launch_x,
            sortie.launch_y,
            sortie.mission_energy_wh,
        )
        if not all(math.isfinite(value) for value in sortie_values):
            raise ValueError("Sortie planning values must be finite")
        if sortie.mission_energy_wh < 0.0:
            raise ValueError("Sortie mission energy cannot be negative")
        ugv_distance = math.hypot(
            sortie.launch_x - previous_x,
            sortie.launch_y - previous_y,
        )
        minimum_transit_seconds = ugv_distance / ugv_planning_speed_mps
        minimum_charge = (
            net_charge_power_w * minimum_transit_seconds / 3600.0
        )
        energy = min(battery_capacity_wh, energy + minimum_charge)
        required = sortie.mission_energy_wh + reserve_energy_wh
        if energy + 1e-9 < required:
            message = (
                f"REJECT at {sortie.target_name}: takeoff {energy:.2f} Wh, "
                f"sortie {sortie.mission_energy_wh:.2f} Wh, "
                f"reserve {reserve_energy_wh:.2f} Wh"
            )
            return CooperativeEnergyPlan(
                feasible=False,
                steps=tuple(steps),
                final_energy_wh=energy,
                message=message,
            )
        landing_energy = energy - sortie.mission_energy_wh
        steps.append(EnergyPlanStep(
            target_name=sortie.target_name,
            ugv_distance_m=ugv_distance,
            minimum_charge_wh=minimum_charge,
            takeoff_energy_wh=energy,
            mission_energy_wh=sortie.mission_energy_wh,
            landing_energy_wh=landing_energy,
        ))
        energy = landing_energy
        previous_x = sortie.launch_x
        previous_y = sortie.launch_y

    summary = ", ".join(
        f"{step.target_name}:{step.takeoff_energy_wh:.1f}->"
        f"{step.landing_energy_wh:.1f}Wh"
        for step in steps
    )
    return CooperativeEnergyPlan(
        feasible=True,
        steps=tuple(steps),
        final_energy_wh=energy,
        message=f"PASS energy-aware sequence: {summary}",
    )
