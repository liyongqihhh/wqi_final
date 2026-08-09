from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class UgvEnergyParameters:
    drive_capacity_wh: float
    charging_capacity_wh: float
    drive_nominal_voltage: float
    charging_nominal_voltage: float
    initial_drive_percentage: float
    initial_charging_percentage: float
    drive_reserve_percentage: float
    charging_reserve_percentage: float
    base_mass_kg: float
    docked_uav_mass_kg: float
    gravity_mps2: float
    rolling_resistance_coefficient: float
    air_density_kgpm3: float
    aerodynamic_drag_area_m2: float
    idle_power_w: float
    linear_loss_power_w_per_mps: float
    angular_power_w_per_radps: float
    drivetrain_efficiency: float
    charger_transfer_efficiency: float
    charger_idle_power_w: float
    planning_speed_mps: float
    planned_target_service_time_s: float
    prediction_margin_factor: float
    acceleration_filter_alpha: float
    update_rate: float
    publish_rate: float
    maximum_update_step_s: float

    @classmethod
    def defaults(cls) -> dict[str, float]:
        return {
            "drive_capacity_wh": 300.0,
            "charging_capacity_wh": 250.0,
            "drive_nominal_voltage": 24.0,
            "charging_nominal_voltage": 24.0,
            "initial_drive_percentage": 0.80,
            "initial_charging_percentage": 0.80,
            "drive_reserve_percentage": 0.20,
            "charging_reserve_percentage": 0.10,
            "base_mass_kg": 1.28,
            "docked_uav_mass_kg": 1.477,
            "gravity_mps2": 9.80665,
            "rolling_resistance_coefficient": 0.030,
            "air_density_kgpm3": 1.225,
            "aerodynamic_drag_area_m2": 0.080,
            "idle_power_w": 42.0,
            "linear_loss_power_w_per_mps": 115.0,
            "angular_power_w_per_radps": 18.0,
            "drivetrain_efficiency": 0.85,
            "charger_transfer_efficiency": 0.90,
            "charger_idle_power_w": 2.0,
            "planning_speed_mps": 0.40,
            "planned_target_service_time_s": 300.0,
            "prediction_margin_factor": 1.20,
            "acceleration_filter_alpha": 0.25,
            "update_rate": 10.0,
            "publish_rate": 2.0,
            "maximum_update_step_s": 1.0,
        }

    @classmethod
    def from_mapping(cls, values) -> "UgvEnergyParameters":
        defaults = cls.defaults()
        try:
            parameters = cls(**{
                name: float(values.get(name, default))
                for name, default in defaults.items()
            })
        except (TypeError, ValueError) as error:
            raise ValueError(f"Invalid UGV energy configuration: {error}") from error
        parameters.validate()
        return parameters

    @classmethod
    def from_yaml(cls, path) -> "UgvEnergyParameters":
        with Path(path).open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        try:
            values = data["/ugv/energy_manager"]["ros__parameters"]
        except (KeyError, TypeError) as error:
            raise ValueError(
                "UGV energy YAML must define /ugv/energy_manager.ros__parameters"
            ) from error
        return cls.from_mapping(values)

    def validate(self) -> None:
        if not all(math.isfinite(value) for value in self.__dict__.values()):
            raise ValueError("UGV energy parameters must be finite")
        positive = (
            self.drive_capacity_wh,
            self.charging_capacity_wh,
            self.drive_nominal_voltage,
            self.charging_nominal_voltage,
            self.base_mass_kg,
            self.gravity_mps2,
            self.planning_speed_mps,
            self.update_rate,
            self.publish_rate,
            self.maximum_update_step_s,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("UGV capacities, mass, voltage and rates must be positive")
        non_negative = (
            self.docked_uav_mass_kg,
            self.rolling_resistance_coefficient,
            self.air_density_kgpm3,
            self.aerodynamic_drag_area_m2,
            self.idle_power_w,
            self.linear_loss_power_w_per_mps,
            self.angular_power_w_per_radps,
            self.charger_idle_power_w,
            self.planned_target_service_time_s,
        )
        if any(value < 0.0 for value in non_negative):
            raise ValueError("UGV power and physical parameters cannot be negative")
        fractions = (
            self.initial_drive_percentage,
            self.initial_charging_percentage,
            self.drive_reserve_percentage,
            self.charging_reserve_percentage,
            self.drivetrain_efficiency,
            self.charger_transfer_efficiency,
            self.acceleration_filter_alpha,
        )
        if any(not 0.0 <= value <= 1.0 for value in fractions):
            raise ValueError("UGV percentages and efficiencies must be in [0, 1]")
        if self.drivetrain_efficiency <= 0.0 or self.charger_transfer_efficiency <= 0.0:
            raise ValueError("UGV energy efficiencies must be greater than zero")
        if self.prediction_margin_factor < 1.0:
            raise ValueError("UGV prediction margin must be at least one")

    @property
    def drive_reserve_wh(self) -> float:
        return self.drive_capacity_wh * self.drive_reserve_percentage

    @property
    def charging_reserve_wh(self) -> float:
        return self.charging_capacity_wh * self.charging_reserve_percentage


@dataclass(frozen=True)
class UgvEnergySnapshot:
    drive_energy_wh: float
    charging_energy_wh: float
    drive_soc: float
    charging_soc: float
    drive_power_w: float
    charging_source_power_w: float
    charging_output_power_w: float
    drive_consumed_wh: float
    charging_consumed_wh: float
    cargo_mass_kg: float
    total_carried_mass_kg: float
    charger_available: bool


@dataclass(frozen=True)
class UgvDriveEstimate:
    raw_energy_wh: float
    predicted_energy_wh: float
    reserve_energy_wh: float
    required_energy_wh: float
    final_energy_wh: float
    feasible: bool


class UgvEnergyModel:
    def __init__(
        self,
        parameters: UgvEnergyParameters,
        initial_drive_percentage: float | None = None,
        initial_charging_percentage: float | None = None,
    ) -> None:
        self.parameters = parameters
        drive_soc = (
            parameters.initial_drive_percentage
            if initial_drive_percentage is None
            else float(initial_drive_percentage)
        )
        charging_soc = (
            parameters.initial_charging_percentage
            if initial_charging_percentage is None
            else float(initial_charging_percentage)
        )
        if not 0.0 <= drive_soc <= 1.0 or not 0.0 <= charging_soc <= 1.0:
            raise ValueError("Initial UGV battery percentages must be in [0, 1]")
        self.drive_energy_wh = parameters.drive_capacity_wh * drive_soc
        self.charging_energy_wh = parameters.charging_capacity_wh * charging_soc
        self.drive_consumed_wh = 0.0
        self.charging_consumed_wh = 0.0
        self.last_drive_power_w = 0.0
        self.last_charging_source_power_w = 0.0
        self.last_charging_output_power_w = 0.0
        self.last_cargo_mass_kg = 0.0
        self.last_total_carried_mass_kg = parameters.base_mass_kg

    @staticmethod
    def _mass(value: float) -> float:
        mass = float(value)
        if not math.isfinite(mass) or mass < 0.0:
            raise ValueError("Cargo mass must be finite and non-negative")
        return mass

    def total_mass_kg(self, cargo_mass_kg: float, uav_docked: bool) -> float:
        cargo = self._mass(cargo_mass_kg)
        return (
            self.parameters.base_mass_kg
            + cargo
            + (self.parameters.docked_uav_mass_kg if uav_docked else 0.0)
        )

    def drive_power_w(
        self,
        linear_speed_mps: float,
        angular_speed_radps: float,
        linear_acceleration_mps2: float,
        cargo_mass_kg: float,
        uav_docked: bool,
    ) -> float:
        speed = abs(float(linear_speed_mps))
        angular = abs(float(angular_speed_radps))
        acceleration = float(linear_acceleration_mps2)
        if not all(math.isfinite(value) for value in (speed, angular, acceleration)):
            raise ValueError("UGV motion values must be finite")
        p = self.parameters
        total_mass = self.total_mass_kg(cargo_mass_kg, uav_docked)
        rolling = (
            p.rolling_resistance_coefficient
            * total_mass
            * p.gravity_mps2
            * speed
        )
        aerodynamic = (
            0.5
            * p.air_density_kgpm3
            * p.aerodynamic_drag_area_m2
            * speed ** 3
        )
        acceleration_power = max(0.0, total_mass * acceleration * speed)
        motion = (rolling + aerodynamic + acceleration_power) / p.drivetrain_efficiency
        losses = (
            p.linear_loss_power_w_per_mps * speed
            + p.angular_power_w_per_radps * angular
        )
        return p.idle_power_w + motion + losses

    def step(
        self,
        dt_seconds: float,
        linear_speed_mps: float,
        angular_speed_radps: float,
        linear_acceleration_mps2: float,
        cargo_mass_kg: float,
        uav_docked: bool,
        uav_battery_power_w: float,
    ) -> UgvEnergySnapshot:
        dt = float(dt_seconds)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("UGV energy update interval must be positive")
        cargo = self._mass(cargo_mass_kg)
        total_mass = self.total_mass_kg(cargo, uav_docked)
        requested_drive_power = self.drive_power_w(
            linear_speed_mps,
            angular_speed_radps,
            linear_acceleration_mps2,
            cargo,
            uav_docked,
        )
        requested_drive_energy = requested_drive_power * dt / 3600.0
        accepted_drive_energy = min(self.drive_energy_wh, requested_drive_energy)
        self.drive_energy_wh -= accepted_drive_energy
        self.drive_consumed_wh += accepted_drive_energy
        self.last_drive_power_w = accepted_drive_energy * 3600.0 / dt

        requested_output = (
            max(0.0, -float(uav_battery_power_w)) if uav_docked else 0.0
        )
        requested_source_power = 0.0
        if uav_docked and self.charging_energy_wh > 0.0:
            requested_source_power = self.parameters.charger_idle_power_w
            requested_source_power += (
                requested_output / self.parameters.charger_transfer_efficiency
            )
        requested_source_energy = requested_source_power * dt / 3600.0
        usable_charging_energy = max(
            0.0,
            self.charging_energy_wh - self.parameters.charging_reserve_wh,
        )
        accepted_source_energy = min(
            usable_charging_energy, requested_source_energy
        )
        self.charging_energy_wh -= accepted_source_energy
        self.charging_consumed_wh += accepted_source_energy
        source_power = accepted_source_energy * 3600.0 / dt
        output_power = max(
            0.0,
            (source_power - self.parameters.charger_idle_power_w)
            * self.parameters.charger_transfer_efficiency,
        ) if uav_docked else 0.0
        self.last_charging_source_power_w = source_power
        self.last_charging_output_power_w = min(requested_output, output_power)
        self.last_cargo_mass_kg = cargo
        self.last_total_carried_mass_kg = total_mass
        return self.snapshot()

    def snapshot(self) -> UgvEnergySnapshot:
        p = self.parameters
        return UgvEnergySnapshot(
            drive_energy_wh=self.drive_energy_wh,
            charging_energy_wh=self.charging_energy_wh,
            drive_soc=self.drive_energy_wh / p.drive_capacity_wh,
            charging_soc=self.charging_energy_wh / p.charging_capacity_wh,
            drive_power_w=self.last_drive_power_w,
            charging_source_power_w=self.last_charging_source_power_w,
            charging_output_power_w=self.last_charging_output_power_w,
            drive_consumed_wh=self.drive_consumed_wh,
            charging_consumed_wh=self.charging_consumed_wh,
            cargo_mass_kg=self.last_cargo_mass_kg,
            total_carried_mass_kg=self.last_total_carried_mass_kg,
            charger_available=(
                self.charging_energy_wh
                > self.parameters.charging_reserve_wh + 1e-6
            ),
        )


def estimate_ugv_drive_energy(
    parameters: UgvEnergyParameters,
    available_energy_wh: float,
    leg_distances_m,
    payload_masses_kg,
    return_distance_m: float = 0.0,
) -> UgvDriveEstimate:
    distances = [float(value) for value in leg_distances_m]
    payloads = [float(value) for value in payload_masses_kg]
    if len(distances) != len(payloads):
        raise ValueError("UGV leg distances and payload masses must have equal length")
    values = [float(available_energy_wh), float(return_distance_m), *distances, *payloads]
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError("UGV energy planning values must be finite and non-negative")

    model = UgvEnergyModel(parameters, 1.0, 1.0)
    remaining_payload = sum(payloads)
    raw_energy = 0.0
    speed = parameters.planning_speed_mps
    for distance, delivered_payload in zip(distances, payloads):
        transit_power = model.drive_power_w(
            speed, 0.0, 0.0, remaining_payload, True
        )
        raw_energy += transit_power * (distance / speed) / 3600.0
        remaining_payload = max(0.0, remaining_payload - delivered_payload)
        service_power = model.drive_power_w(
            0.0, 0.0, 0.0, remaining_payload, False
        )
        raw_energy += (
            service_power * parameters.planned_target_service_time_s / 3600.0
        )

    if return_distance_m > 0.0:
        return_power = model.drive_power_w(
            speed, 0.0, 0.0, remaining_payload, True
        )
        raw_energy += return_power * (float(return_distance_m) / speed) / 3600.0

    predicted = raw_energy * parameters.prediction_margin_factor
    reserve = parameters.drive_reserve_wh
    required = predicted + reserve
    available = min(float(available_energy_wh), parameters.drive_capacity_wh)
    return UgvDriveEstimate(
        raw_energy_wh=raw_energy,
        predicted_energy_wh=predicted,
        reserve_energy_wh=reserve,
        required_energy_wh=required,
        final_energy_wh=max(0.0, available - predicted),
        feasible=available + 1e-9 >= required,
    )
