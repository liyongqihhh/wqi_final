from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class UgvEnergyParameters:
    idle_power_w: float
    linear_power_w_per_mps: float
    angular_power_w_per_radps: float
    payload_power_w_per_kg: float
    drivetrain_efficiency: float
    maximum_update_step_s: float
    maximum_position_jump_m: float

    @classmethod
    def from_yaml(cls, path):
        with Path(path).open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        raw = data.get("ugv_energy_model", {}) if isinstance(data, dict) else {}
        try:
            parameters = cls(**{
                field: float(raw[field])
                for field in cls.__dataclass_fields__
            })
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"Invalid UGV energy model: {error}") from error
        values = tuple(parameters.__dict__.values())
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("UGV energy parameters must be finite and non-negative")
        if not 0.0 < parameters.drivetrain_efficiency <= 1.0:
            raise ValueError("drivetrain_efficiency must be in (0, 1]")
        if parameters.maximum_update_step_s <= 0.0:
            raise ValueError("maximum_update_step_s must be positive")
        return parameters


class UgvEnergyIntegrator:
    def __init__(self, parameters: UgvEnergyParameters) -> None:
        self.parameters = parameters
        self.reset()

    def reset(self) -> None:
        self.energy_wh = 0.0
        self.last_stamp_ns = None
        self.last_power_w = None

    def power_w(
        self,
        linear_speed_mps: float,
        angular_speed_radps: float,
        payload_mass_kg: float,
    ) -> float:
        p = self.parameters
        mechanical = (
            p.linear_power_w_per_mps * abs(float(linear_speed_mps))
            + p.angular_power_w_per_radps * abs(float(angular_speed_radps))
            + p.payload_power_w_per_kg * max(0.0, float(payload_mass_kg))
        )
        return p.idle_power_w + mechanical / p.drivetrain_efficiency

    def update(
        self,
        stamp_ns: int,
        linear_speed_mps: float,
        angular_speed_radps: float,
        payload_mass_kg: float = 0.0,
    ) -> bool:
        stamp = int(stamp_ns)
        current_power = self.power_w(
            linear_speed_mps, angular_speed_radps, payload_mass_kg
        )
        if self.last_stamp_ns is None:
            self.last_stamp_ns = stamp
            self.last_power_w = current_power
            return True
        if stamp <= self.last_stamp_ns:
            return False
        dt = (stamp - self.last_stamp_ns) / 1.0e9
        self.last_stamp_ns = stamp
        if dt > self.parameters.maximum_update_step_s:
            self.last_power_w = current_power
            return False
        average_power = 0.5 * (self.last_power_w + current_power)
        self.energy_wh += average_power * dt / 3600.0
        self.last_power_w = current_power
        return True
