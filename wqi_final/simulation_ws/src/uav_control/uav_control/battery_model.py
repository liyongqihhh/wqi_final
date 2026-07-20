from dataclasses import dataclass
import math
from pathlib import Path

import yaml


class BatteryConfigurationError(ValueError):
    pass


PHASE_NAMES = (
    "ascent_acceleration",
    "climb",
    "ascent_deceleration",
    "horizontal_acceleration",
    "cruise",
    "horizontal_deceleration",
    "hover",
    "descent_acceleration",
    "descent",
    "descent_deceleration",
)


@dataclass(frozen=True)
class MissionEnergySegment:
    phase: str
    duration_seconds: float
    horizontal_speed_start_mps: float = 0.0
    horizontal_speed_end_mps: float = 0.0
    vertical_speed_start_mps: float = 0.0
    vertical_speed_end_mps: float = 0.0
    payload_mass_kg: float = 0.0

    def __post_init__(self) -> None:
        if not self.phase:
            raise ValueError("Mission energy segment phase cannot be empty")
        values = (
            self.duration_seconds,
            self.horizontal_speed_start_mps,
            self.horizontal_speed_end_mps,
            self.vertical_speed_start_mps,
            self.vertical_speed_end_mps,
            self.payload_mass_kg,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Mission energy segment values must be finite")
        if self.duration_seconds < 0.0:
            raise ValueError("Mission energy segment duration cannot be negative")
        if (
            self.horizontal_speed_start_mps < 0.0
            or self.horizontal_speed_end_mps < 0.0
            or self.payload_mass_kg < 0.0
        ):
            raise ValueError(
                "Horizontal speeds and payload mass must be non-negative"
            )

    @property
    def horizontal_acceleration_mps2(self) -> float:
        if self.duration_seconds <= 0.0:
            return 0.0
        return (
            self.horizontal_speed_end_mps
            - self.horizontal_speed_start_mps
        ) / self.duration_seconds

    @property
    def vertical_acceleration_mps2(self) -> float:
        if self.duration_seconds <= 0.0:
            return 0.0
        return (
            self.vertical_speed_end_mps
            - self.vertical_speed_start_mps
        ) / self.duration_seconds


@dataclass(frozen=True)
class MissionPowerProfile:
    segments: tuple[MissionEnergySegment, ...] = ()
    ascent_acceleration_seconds: float = 0.0
    climb_seconds: float = 0.0
    ascent_deceleration_seconds: float = 0.0
    horizontal_acceleration_seconds: float = 0.0
    cruise_seconds: float = 0.0
    horizontal_deceleration_seconds: float = 0.0
    hover_seconds: float = 0.0
    descent_acceleration_seconds: float = 0.0
    descent_seconds: float = 0.0
    descent_deceleration_seconds: float = 0.0
    horizontal_distance_m: float = 0.0
    ascent_m: float = 0.0
    descent_m: float = 0.0
    initial_payload_mass_kg: float = 0.0

    def __post_init__(self) -> None:
        for name in (
            "ascent_acceleration_seconds",
            "climb_seconds",
            "ascent_deceleration_seconds",
            "horizontal_acceleration_seconds",
            "cruise_seconds",
            "horizontal_deceleration_seconds",
            "hover_seconds",
            "descent_acceleration_seconds",
            "descent_seconds",
            "descent_deceleration_seconds",
            "horizontal_distance_m",
            "ascent_m",
            "descent_m",
            "initial_payload_mass_kg",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")


@dataclass(frozen=True)
class EnergyAssessment:
    feasible: bool
    current_soc: float
    current_energy_wh: float
    raw_mission_energy_wh: float
    estimated_mission_energy_wh: float
    safety_reserve_wh: float
    required_energy_wh: float
    estimated_final_soc: float
    propulsion_energy_wh: float
    auxiliary_energy_wh: float
    payload_energy_penalty_wh: float
    initial_payload_mass_kg: float


@dataclass(frozen=True)
class PowerBreakdown:
    propulsion_power_w: float
    auxiliary_power_w: float
    battery_power_w: float
    horizontal_propulsion_power_w: float
    vertical_propulsion_power_w: float
    hover_propulsion_power_w: float
    thrust_to_weight_ratio: float
    payload_mass_kg: float
    total_mass_kg: float
    mode: str


@dataclass(frozen=True)
class EnergyBreakdown:
    phase_energy_wh: dict[str, float]
    propulsion_energy_wh: float
    auxiliary_energy_wh: float
    payload_energy_penalty_wh: float


@dataclass(frozen=True)
class BatterySnapshot:
    energy_wh: float
    soc: float
    voltage: float
    current_a: float
    battery_power_w: float
    propulsion_power_w: float
    auxiliary_power_w: float
    payload_mass_kg: float
    total_mass_kg: float
    mode: str
    consumed_energy_wh: float
    charged_energy_wh: float


@dataclass(frozen=True)
class BatteryParameters:
    capacity_wh: float
    nominal_voltage: float
    empty_voltage: float
    full_voltage: float
    initial_percentage: float
    discharge_efficiency: float
    reserve_percentage: float
    prediction_margin_factor: float
    warning_percentage: float
    critical_percentage: float
    airframe_mass_kg: float
    sensor_mass_kg: float
    maximum_payload_mass_kg: float
    default_payload_mass_kg: float
    rotor_count: int
    rotor_radius_m: float
    air_density_kgpm3: float
    gravity_mps2: float
    blade_profile_power_w: float
    induced_power_correction: float
    rotor_tip_speed_mps: float
    fuselage_drag_ratio: float
    rotor_solidity: float
    horizontal_flat_plate_area_m2: float
    vertical_flat_plate_area_m2: float
    computer_power_w: float
    lidar_power_w: float
    camera_power_w: float
    communication_power_w: float
    landed_idle_power_w: float
    docked_idle_power_w: float
    charge_power_w: float
    charge_efficiency: float
    estimated_horizontal_speed_mps: float
    estimated_horizontal_acceleration_mps2: float
    estimated_vertical_speed_mps: float
    estimated_vertical_acceleration_mps2: float
    model_integration_step_s: float
    hover_speed_threshold_mps: float
    vertical_motion_threshold_mps: float
    motion_acceleration_threshold_mps2: float
    acceleration_filter_alpha: float
    update_rate: float
    publish_rate: float
    maximum_update_step_s: float

    @classmethod
    def from_mapping(cls, values) -> "BatteryParameters":
        try:
            parsed = {}
            for field in cls.__dataclass_fields__:
                raw_value = values[field]
                if field == "rotor_count":
                    numeric = float(raw_value)
                    if not numeric.is_integer():
                        raise ValueError("rotor_count must be an integer")
                    parsed[field] = int(numeric)
                else:
                    parsed[field] = float(raw_value)
            parameters = cls(**parsed)
        except (KeyError, TypeError, ValueError) as error:
            raise BatteryConfigurationError(
                f"Invalid battery model configuration: {error}"
            ) from error
        parameters.validate()
        return parameters

    @classmethod
    def from_yaml(cls, path) -> "BatteryParameters":
        config_path = Path(path)
        with config_path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        try:
            values = data["/uav/battery_manager"]["ros__parameters"]
        except (KeyError, TypeError) as error:
            raise BatteryConfigurationError(
                "Battery YAML must define /uav/battery_manager.ros__parameters"
            ) from error
        return cls.from_mapping(values)

    @property
    def base_mass_kg(self) -> float:
        return self.airframe_mass_kg + self.sensor_mass_kg

    @property
    def total_rotor_area_m2(self) -> float:
        return self.rotor_count * math.pi * self.rotor_radius_m ** 2

    @property
    def auxiliary_power_w(self) -> float:
        return (
            self.computer_power_w
            + self.lidar_power_w
            + self.camera_power_w
            + self.communication_power_w
        )

    @property
    def net_charge_power_w(self) -> float:
        return (
            self.charge_power_w * self.charge_efficiency
            - self.docked_idle_power_w
        )

    def validate(self) -> None:
        positive = (
            "capacity_wh",
            "nominal_voltage",
            "empty_voltage",
            "full_voltage",
            "airframe_mass_kg",
            "rotor_radius_m",
            "air_density_kgpm3",
            "gravity_mps2",
            "blade_profile_power_w",
            "rotor_tip_speed_mps",
            "fuselage_drag_ratio",
            "rotor_solidity",
            "horizontal_flat_plate_area_m2",
            "vertical_flat_plate_area_m2",
            "landed_idle_power_w",
            "charge_power_w",
            "estimated_horizontal_speed_mps",
            "estimated_horizontal_acceleration_mps2",
            "estimated_vertical_speed_mps",
            "estimated_vertical_acceleration_mps2",
            "model_integration_step_s",
            "update_rate",
            "publish_rate",
            "maximum_update_step_s",
        )
        for name in positive:
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0.0:
                raise BatteryConfigurationError(f"{name} must be positive")
        if self.rotor_count < 2:
            raise BatteryConfigurationError("rotor_count must be at least 2")
        if self.full_voltage <= self.empty_voltage:
            raise BatteryConfigurationError(
                "full_voltage must be greater than empty_voltage"
            )
        for name in (
            "initial_percentage",
            "reserve_percentage",
            "warning_percentage",
            "critical_percentage",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0.0 <= value <= 1.0:
                raise BatteryConfigurationError(f"{name} must be in [0, 1]")
        for name in (
            "discharge_efficiency",
            "charge_efficiency",
            "acceleration_filter_alpha",
        ):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0.0 < value <= 1.0:
                raise BatteryConfigurationError(f"{name} must be in (0, 1]")
        if self.prediction_margin_factor < 1.0:
            raise BatteryConfigurationError(
                "prediction_margin_factor must be at least 1"
            )
        if self.critical_percentage > self.warning_percentage:
            raise BatteryConfigurationError(
                "critical_percentage cannot exceed warning_percentage"
            )
        nonnegative = (
            "sensor_mass_kg",
            "maximum_payload_mass_kg",
            "default_payload_mass_kg",
            "induced_power_correction",
            "computer_power_w",
            "lidar_power_w",
            "camera_power_w",
            "communication_power_w",
            "docked_idle_power_w",
            "hover_speed_threshold_mps",
            "vertical_motion_threshold_mps",
            "motion_acceleration_threshold_mps2",
        )
        for name in nonnegative:
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0.0:
                raise BatteryConfigurationError(
                    f"{name} must be finite and non-negative"
                )
        if self.default_payload_mass_kg > self.maximum_payload_mass_kg:
            raise BatteryConfigurationError(
                "default payload cannot exceed maximum payload"
            )
        if self.net_charge_power_w <= 0.0:
            raise BatteryConfigurationError(
                "effective charging power must exceed docked idle power"
            )


class BatteryModel:
    def __init__(
        self,
        parameters: BatteryParameters,
        initial_percentage: float | None = None,
    ) -> None:
        self.parameters = parameters
        initial = (
            parameters.initial_percentage
            if initial_percentage is None
            else float(initial_percentage)
        )
        if not math.isfinite(initial) or not 0.0 <= initial <= 1.0:
            raise ValueError("initial_percentage must be in [0, 1]")
        self.energy_wh = parameters.capacity_wh * initial
        self.last_power_w = parameters.landed_idle_power_w
        self.last_propulsion_power_w = 0.0
        self.last_auxiliary_power_w = parameters.landed_idle_power_w
        self.last_payload_mass_kg = 0.0
        self.last_total_mass_kg = parameters.base_mass_kg
        self.last_mode = "LANDED_IDLE"
        self.consumed_energy_wh = 0.0
        self.charged_energy_wh = 0.0

    @property
    def soc(self) -> float:
        return min(1.0, max(0.0, self.energy_wh / self.parameters.capacity_wh))

    def voltage(self) -> float:
        span = self.parameters.full_voltage - self.parameters.empty_voltage
        return self.parameters.empty_voltage + span * self.soc

    @staticmethod
    def phase_name(phase: str) -> str:
        return str(phase).split(":", 1)[0].strip().upper()

    def _validated_payload(self, payload_mass_kg: float) -> float:
        payload = float(payload_mass_kg)
        if not math.isfinite(payload) or payload < 0.0:
            raise ValueError("payload_mass_kg must be finite and non-negative")
        if payload > self.parameters.maximum_payload_mass_kg + 1e-9:
            raise ValueError(
                f"payload {payload:.3f} kg exceeds "
                f"{self.parameters.maximum_payload_mass_kg:.3f} kg limit"
            )
        return payload

    def total_mass_kg(self, payload_mass_kg: float = 0.0) -> float:
        return self.parameters.base_mass_kg + self._validated_payload(
            payload_mass_kg
        )

    def hover_induced_power_w(self, payload_mass_kg: float = 0.0) -> float:
        weight = self.total_mass_kg(payload_mass_kg) * self.parameters.gravity_mps2
        denominator = math.sqrt(
            2.0
            * self.parameters.air_density_kgpm3
            * self.parameters.total_rotor_area_m2
        )
        return (
            (1.0 + self.parameters.induced_power_correction)
            * weight ** 1.5
            / denominator
        )

    def mean_induced_velocity_mps(
        self, payload_mass_kg: float = 0.0
    ) -> float:
        weight = self.total_mass_kg(payload_mass_kg) * self.parameters.gravity_mps2
        return math.sqrt(
            weight
            / (
                2.0
                * self.parameters.air_density_kgpm3
                * self.parameters.total_rotor_area_m2
            )
        )

    def hover_propulsion_power_w(
        self, payload_mass_kg: float = 0.0
    ) -> float:
        return (
            self.parameters.blade_profile_power_w
            + self.hover_induced_power_w(payload_mass_kg)
        )

    def dynamic_thrust_to_weight_ratio(
        self,
        horizontal_speed_mps: float,
        horizontal_acceleration_mps2: float,
        horizontal_acceleration_velocity_dot_m2ps3: float,
        payload_mass_kg: float = 0.0,
    ) -> float:
        speed = max(0.0, float(horizontal_speed_mps))
        acceleration = abs(float(horizontal_acceleration_mps2))
        acceleration_velocity_dot = float(
            horizontal_acceleration_velocity_dot_m2ps3
        )
        mass = self.total_mass_kg(payload_mass_kg)
        weight = mass * self.parameters.gravity_mps2
        rho = self.parameters.air_density_kgpm3
        flat_plate_area = self.parameters.horizontal_flat_plate_area_m2
        horizontal_force_squared_times_four = (
            4.0 * mass ** 2 * acceleration ** 2
            + rho ** 2 * flat_plate_area ** 2 * speed ** 4
            + 4.0
            * mass
            * rho
            * flat_plate_area
            * acceleration_velocity_dot
            * speed
        )
        horizontal_force_squared_times_four = max(
            0.0, horizontal_force_squared_times_four
        )
        return math.sqrt(
            1.0
            + horizontal_force_squared_times_four / (4.0 * weight ** 2)
        )

    def horizontal_propulsion_power_w(
        self,
        horizontal_speed_mps: float,
        horizontal_acceleration_mps2: float = 0.0,
        horizontal_acceleration_velocity_dot_m2ps3: float = 0.0,
        payload_mass_kg: float = 0.0,
    ) -> tuple[float, float]:
        speed = max(0.0, float(horizontal_speed_mps))
        kappa = self.dynamic_thrust_to_weight_ratio(
            speed,
            horizontal_acceleration_mps2,
            horizontal_acceleration_velocity_dot_m2ps3,
            payload_mass_kg,
        )
        p0 = self.parameters.blade_profile_power_w
        induced_hover = self.hover_induced_power_w(payload_mass_kg)
        induced_velocity = self.mean_induced_velocity_mps(payload_mass_kg)
        blade_profile = p0 * (
            1.0
            + 3.0 * speed ** 2 / self.parameters.rotor_tip_speed_mps ** 2
        )
        induced_inner = (
            math.sqrt(
                kappa ** 2
                + speed ** 4 / (4.0 * induced_velocity ** 4)
            )
            - speed ** 2 / (2.0 * induced_velocity ** 2)
        )
        induced = (
            induced_hover
            * kappa
            * math.sqrt(max(0.0, induced_inner))
        )
        parasite = (
            0.5
            * self.parameters.fuselage_drag_ratio
            * self.parameters.air_density_kgpm3
            * self.parameters.rotor_solidity
            * self.parameters.total_rotor_area_m2
            * speed ** 3
        )
        return blade_profile + induced + parasite, kappa

    def vertical_propulsion_power_w(
        self,
        vertical_speed_mps: float,
        vertical_acceleration_mps2: float = 0.0,
        payload_mass_kg: float = 0.0,
    ) -> float:
        vertical_speed = float(vertical_speed_mps)
        vertical_acceleration = float(vertical_acceleration_mps2)
        mass = self.total_mass_kg(payload_mass_kg)
        weight = mass * self.parameters.gravity_mps2
        signed_drag = (
            0.5
            * self.parameters.air_density_kgpm3
            * self.parameters.vertical_flat_plate_area_m2
            * vertical_speed
            * abs(vertical_speed)
        )
        thrust = weight + mass * vertical_acceleration + signed_drag
        thrust = max(0.05 * weight, thrust)
        induced_flow_power = 0.5 * thrust * (
            vertical_speed
            + math.sqrt(
                vertical_speed ** 2
                + 2.0
                * thrust
                / (
                    self.parameters.air_density_kgpm3
                    * self.parameters.total_rotor_area_m2
                )
            )
        )
        return self.parameters.blade_profile_power_w + (
            1.0 + self.parameters.induced_power_correction
        ) * max(0.0, induced_flow_power)

    def _operating_mode(
        self,
        phase: str,
        flying: bool,
        horizontal_speed_mps: float,
        vertical_speed_mps: float,
        horizontal_acceleration_mps2: float,
        vertical_acceleration_mps2: float,
        horizontal_parallel_acceleration_mps2: float,
    ) -> str:
        if not flying:
            return "LANDED_IDLE"
        phase_name = self.phase_name(phase)
        velocity_threshold = self.parameters.vertical_motion_threshold_mps
        acceleration_threshold = (
            self.parameters.motion_acceleration_threshold_mps2
        )
        if phase_name == "TAKEOFF":
            if vertical_acceleration_mps2 > acceleration_threshold:
                return "ASCENT_ACCELERATION"
            if vertical_acceleration_mps2 < -acceleration_threshold:
                return "ASCENT_DECELERATION"
            return "CLIMB"
        if phase_name == "LANDING":
            if vertical_acceleration_mps2 < -acceleration_threshold:
                return "DESCENT_ACCELERATION"
            if vertical_acceleration_mps2 > acceleration_threshold:
                return "DESCENT_DECELERATION"
            return "DESCENT"
        if vertical_speed_mps > velocity_threshold:
            if vertical_acceleration_mps2 > acceleration_threshold:
                return "ASCENT_ACCELERATION"
            if vertical_acceleration_mps2 < -acceleration_threshold:
                return "ASCENT_DECELERATION"
            return "CLIMB"
        if vertical_speed_mps < -velocity_threshold:
            if vertical_acceleration_mps2 < -acceleration_threshold:
                return "DESCENT_ACCELERATION"
            if vertical_acceleration_mps2 > acceleration_threshold:
                return "DESCENT_DECELERATION"
            return "DESCENT"
        if phase_name in ("HOVER", "DELIVERING"):
            return "HOVER"
        if horizontal_speed_mps <= self.parameters.hover_speed_threshold_mps:
            return "HOVER"
        if horizontal_parallel_acceleration_mps2 > acceleration_threshold:
            return "HORIZONTAL_ACCELERATION"
        if horizontal_parallel_acceleration_mps2 < -acceleration_threshold:
            return "HORIZONTAL_DECELERATION"
        if horizontal_acceleration_mps2 > acceleration_threshold:
            return "MANEUVER"
        return "CRUISE"

    def flight_power(
        self,
        phase: str,
        flying: bool,
        horizontal_speed_mps: float = 0.0,
        vertical_speed_mps: float = 0.0,
        horizontal_acceleration_mps2: float = 0.0,
        vertical_acceleration_mps2: float = 0.0,
        horizontal_acceleration_velocity_dot_m2ps3: float = 0.0,
        payload_mass_kg: float = 0.0,
    ) -> PowerBreakdown:
        motion_values = (
            horizontal_speed_mps,
            vertical_speed_mps,
            horizontal_acceleration_mps2,
            vertical_acceleration_mps2,
            horizontal_acceleration_velocity_dot_m2ps3,
        )
        if not all(math.isfinite(float(value)) for value in motion_values):
            raise ValueError("motion inputs must be finite")
        payload = self._validated_payload(payload_mass_kg)
        total_mass = self.total_mass_kg(payload)
        if not flying:
            return PowerBreakdown(
                propulsion_power_w=0.0,
                auxiliary_power_w=self.parameters.landed_idle_power_w,
                battery_power_w=self.parameters.landed_idle_power_w,
                horizontal_propulsion_power_w=0.0,
                vertical_propulsion_power_w=0.0,
                hover_propulsion_power_w=self.hover_propulsion_power_w(payload),
                thrust_to_weight_ratio=1.0,
                payload_mass_kg=payload,
                total_mass_kg=total_mass,
                mode="LANDED_IDLE",
            )

        horizontal_speed = max(0.0, float(horizontal_speed_mps))
        horizontal_acceleration = abs(float(horizontal_acceleration_mps2))
        dot = float(horizontal_acceleration_velocity_dot_m2ps3)
        parallel_acceleration = (
            dot / horizontal_speed if horizontal_speed > 1e-6 else 0.0
        )
        horizontal_power, kappa = self.horizontal_propulsion_power_w(
            horizontal_speed,
            horizontal_acceleration,
            dot,
            payload,
        )
        vertical_power = self.vertical_propulsion_power_w(
            vertical_speed_mps,
            vertical_acceleration_mps2,
            payload,
        )
        hover_power = self.hover_propulsion_power_w(payload)
        propulsion_power = max(
            0.2 * hover_power,
            horizontal_power + vertical_power - hover_power,
        )
        auxiliary_power = self.parameters.auxiliary_power_w
        battery_power = (
            propulsion_power + auxiliary_power
        ) / self.parameters.discharge_efficiency
        mode = self._operating_mode(
            phase,
            flying,
            horizontal_speed,
            float(vertical_speed_mps),
            horizontal_acceleration,
            float(vertical_acceleration_mps2),
            parallel_acceleration,
        )
        return PowerBreakdown(
            propulsion_power_w=propulsion_power,
            auxiliary_power_w=auxiliary_power,
            battery_power_w=battery_power,
            horizontal_propulsion_power_w=horizontal_power,
            vertical_propulsion_power_w=vertical_power,
            hover_propulsion_power_w=hover_power,
            thrust_to_weight_ratio=kappa,
            payload_mass_kg=payload,
            total_mass_kg=total_mass,
            mode=mode,
        )

    def discharge_power(
        self,
        phase: str,
        flying: bool,
        speed_mps: float,
        vertical_speed_mps: float = 0.0,
        acceleration_mps2: float = 0.0,
        payload_mass_kg: float = 0.0,
    ) -> tuple[float, str]:
        speed = max(0.0, float(speed_mps))
        vertical_speed = float(vertical_speed_mps)
        horizontal_speed = math.sqrt(
            max(0.0, speed ** 2 - vertical_speed ** 2)
        )
        vertical_context = (
            abs(vertical_speed)
            > self.parameters.vertical_motion_threshold_mps
            or self.phase_name(phase) in ("TAKEOFF", "LANDING")
        )
        vertical_acceleration = (
            float(acceleration_mps2) if vertical_context else 0.0
        )
        horizontal_parallel_acceleration = (
            0.0 if vertical_context else float(acceleration_mps2)
        )
        breakdown = self.flight_power(
            phase,
            flying,
            horizontal_speed,
            vertical_speed,
            abs(horizontal_parallel_acceleration),
            vertical_acceleration,
            horizontal_parallel_acceleration * horizontal_speed,
            payload_mass_kg,
        )
        return breakdown.battery_power_w, breakdown.mode

    def step(
        self,
        dt_seconds: float,
        phase: str,
        flying: bool,
        docked: bool,
        speed_mps: float = 0.0,
        vertical_speed_mps: float = 0.0,
        acceleration_mps2: float = 0.0,
        *,
        horizontal_speed_mps: float | None = None,
        horizontal_acceleration_mps2: float | None = None,
        vertical_acceleration_mps2: float | None = None,
        horizontal_acceleration_velocity_dot_m2ps3: float | None = None,
        payload_mass_kg: float = 0.0,
    ) -> BatterySnapshot:
        if not math.isfinite(dt_seconds) or dt_seconds < 0.0:
            raise ValueError("dt_seconds must be finite and non-negative")
        payload = self._validated_payload(payload_mass_kg)
        self.last_payload_mass_kg = payload
        self.last_total_mass_kg = self.total_mass_kg(payload)
        if docked and self.soc < 1.0:
            stored_power = self.parameters.net_charge_power_w
            stored_energy = stored_power * dt_seconds / 3600.0
            available_capacity = self.parameters.capacity_wh - self.energy_wh
            accepted_energy = min(stored_energy, max(0.0, available_capacity))
            self.energy_wh += accepted_energy
            self.charged_energy_wh += accepted_energy
            self.last_power_w = -stored_power
            self.last_propulsion_power_w = 0.0
            self.last_auxiliary_power_w = self.parameters.docked_idle_power_w
            self.last_mode = "CHARGING"
        elif docked:
            self.last_power_w = 0.0
            self.last_propulsion_power_w = 0.0
            self.last_auxiliary_power_w = self.parameters.docked_idle_power_w
            self.last_mode = "FULL"
        else:
            if horizontal_speed_mps is None:
                speed = max(0.0, float(speed_mps))
                horizontal_speed = math.sqrt(
                    max(0.0, speed ** 2 - float(vertical_speed_mps) ** 2)
                )
            else:
                horizontal_speed = max(0.0, float(horizontal_speed_mps))
            horizontal_acceleration = (
                abs(float(acceleration_mps2))
                if horizontal_acceleration_mps2 is None
                else abs(float(horizontal_acceleration_mps2))
            )
            vertical_acceleration = (
                float(acceleration_mps2)
                if vertical_acceleration_mps2 is None
                else float(vertical_acceleration_mps2)
            )
            dot = (
                float(acceleration_mps2) * horizontal_speed
                if horizontal_acceleration_velocity_dot_m2ps3 is None
                else float(horizontal_acceleration_velocity_dot_m2ps3)
            )
            breakdown = self.flight_power(
                phase,
                flying,
                horizontal_speed,
                float(vertical_speed_mps),
                horizontal_acceleration,
                vertical_acceleration,
                dot,
                payload,
            )
            requested_energy = (
                breakdown.battery_power_w * dt_seconds / 3600.0
            )
            consumed_energy = min(
                requested_energy, max(0.0, self.energy_wh)
            )
            self.energy_wh -= consumed_energy
            self.consumed_energy_wh += consumed_energy
            self.last_power_w = breakdown.battery_power_w
            self.last_propulsion_power_w = breakdown.propulsion_power_w
            self.last_auxiliary_power_w = breakdown.auxiliary_power_w
            self.last_mode = breakdown.mode
        self.energy_wh = min(
            self.parameters.capacity_wh,
            max(0.0, self.energy_wh),
        )
        return self.snapshot()

    def snapshot(self) -> BatterySnapshot:
        voltage = self.voltage()
        current = 0.0 if voltage <= 0.0 else -self.last_power_w / voltage
        return BatterySnapshot(
            energy_wh=self.energy_wh,
            soc=self.soc,
            voltage=voltage,
            current_a=current,
            battery_power_w=self.last_power_w,
            propulsion_power_w=self.last_propulsion_power_w,
            auxiliary_power_w=self.last_auxiliary_power_w,
            payload_mass_kg=self.last_payload_mass_kg,
            total_mass_kg=self.last_total_mass_kg,
            mode=self.last_mode,
            consumed_energy_wh=self.consumed_energy_wh,
            charged_energy_wh=self.charged_energy_wh,
        )

    def _legacy_segments(
        self, profile: MissionPowerProfile
    ) -> tuple[MissionEnergySegment, ...]:
        horizontal_speed = self.parameters.estimated_horizontal_speed_mps
        vertical_speed = self.parameters.estimated_vertical_speed_mps
        payload = profile.initial_payload_mass_kg
        definitions = (
            (
                "ascent_acceleration",
                profile.ascent_acceleration_seconds,
                0.0,
                0.0,
                0.0,
                vertical_speed,
            ),
            (
                "climb",
                profile.climb_seconds,
                0.0,
                0.0,
                vertical_speed,
                vertical_speed,
            ),
            (
                "ascent_deceleration",
                profile.ascent_deceleration_seconds,
                0.0,
                0.0,
                vertical_speed,
                0.0,
            ),
            (
                "horizontal_acceleration",
                profile.horizontal_acceleration_seconds,
                0.0,
                horizontal_speed,
                0.0,
                0.0,
            ),
            (
                "cruise",
                profile.cruise_seconds,
                horizontal_speed,
                horizontal_speed,
                0.0,
                0.0,
            ),
            (
                "horizontal_deceleration",
                profile.horizontal_deceleration_seconds,
                horizontal_speed,
                0.0,
                0.0,
                0.0,
            ),
            ("hover", profile.hover_seconds, 0.0, 0.0, 0.0, 0.0),
            (
                "descent_acceleration",
                profile.descent_acceleration_seconds,
                0.0,
                0.0,
                0.0,
                -vertical_speed,
            ),
            (
                "descent",
                profile.descent_seconds,
                0.0,
                0.0,
                -vertical_speed,
                -vertical_speed,
            ),
            (
                "descent_deceleration",
                profile.descent_deceleration_seconds,
                0.0,
                0.0,
                -vertical_speed,
                0.0,
            ),
        )
        return tuple(
            MissionEnergySegment(
                phase=name,
                duration_seconds=duration,
                horizontal_speed_start_mps=horizontal_start,
                horizontal_speed_end_mps=horizontal_end,
                vertical_speed_start_mps=vertical_start,
                vertical_speed_end_mps=vertical_end,
                payload_mass_kg=payload,
            )
            for (
                name,
                duration,
                horizontal_start,
                horizontal_end,
                vertical_start,
                vertical_end,
            ) in definitions
            if duration > 0.0
        )

    def _segments(
        self, profile: MissionPowerProfile
    ) -> tuple[MissionEnergySegment, ...]:
        return profile.segments or self._legacy_segments(profile)

    def _integrate_segment(
        self,
        segment: MissionEnergySegment,
        payload_mass_kg: float | None = None,
    ) -> tuple[float, float, float]:
        duration = segment.duration_seconds
        if duration <= 0.0:
            return 0.0, 0.0, 0.0
        count = max(
            1,
            math.ceil(duration / self.parameters.model_integration_step_s),
        )
        dt = duration / count
        horizontal_acceleration = segment.horizontal_acceleration_mps2
        vertical_acceleration = segment.vertical_acceleration_mps2
        payload = (
            segment.payload_mass_kg
            if payload_mass_kg is None
            else payload_mass_kg
        )
        propulsion_energy = 0.0
        auxiliary_energy = 0.0
        total_energy = 0.0
        for index in range(count):
            fraction = (index + 0.5) / count
            horizontal_speed = (
                segment.horizontal_speed_start_mps
                + fraction
                * (
                    segment.horizontal_speed_end_mps
                    - segment.horizontal_speed_start_mps
                )
            )
            vertical_speed = (
                segment.vertical_speed_start_mps
                + fraction
                * (
                    segment.vertical_speed_end_mps
                    - segment.vertical_speed_start_mps
                )
            )
            breakdown = self.flight_power(
                segment.phase,
                True,
                horizontal_speed,
                vertical_speed,
                abs(horizontal_acceleration),
                vertical_acceleration,
                horizontal_acceleration * horizontal_speed,
                payload,
            )
            propulsion_energy += (
                breakdown.propulsion_power_w
                / self.parameters.discharge_efficiency
                * dt
                / 3600.0
            )
            auxiliary_energy += (
                breakdown.auxiliary_power_w
                / self.parameters.discharge_efficiency
                * dt
                / 3600.0
            )
            total_energy += breakdown.battery_power_w * dt / 3600.0
        return total_energy, propulsion_energy, auxiliary_energy

    def energy_breakdown_wh(
        self, profile: MissionPowerProfile
    ) -> EnergyBreakdown:
        phase_energy = {name: 0.0 for name in PHASE_NAMES}
        propulsion_energy = 0.0
        auxiliary_energy = 0.0
        unloaded_energy = 0.0
        for segment in self._segments(profile):
            total, propulsion, auxiliary = self._integrate_segment(segment)
            unloaded, _, _ = self._integrate_segment(
                segment, payload_mass_kg=0.0
            )
            phase_energy.setdefault(segment.phase, 0.0)
            phase_energy[segment.phase] += total
            propulsion_energy += propulsion
            auxiliary_energy += auxiliary
            unloaded_energy += unloaded
        total_energy = propulsion_energy + auxiliary_energy
        return EnergyBreakdown(
            phase_energy_wh=phase_energy,
            propulsion_energy_wh=propulsion_energy,
            auxiliary_energy_wh=auxiliary_energy,
            payload_energy_penalty_wh=max(0.0, total_energy - unloaded_energy),
        )

    def phase_energy_wh(
        self, profile: MissionPowerProfile
    ) -> dict[str, float]:
        return self.energy_breakdown_wh(profile).phase_energy_wh

    def estimate(self, profile: MissionPowerProfile) -> EnergyAssessment:
        breakdown = self.energy_breakdown_wh(profile)
        raw_energy = (
            breakdown.propulsion_energy_wh + breakdown.auxiliary_energy_wh
        )
        estimated_energy = (
            raw_energy * self.parameters.prediction_margin_factor
        )
        reserve = (
            self.parameters.capacity_wh * self.parameters.reserve_percentage
        )
        required = estimated_energy + reserve
        final_soc = max(
            0.0,
            (self.energy_wh - estimated_energy) / self.parameters.capacity_wh,
        )
        return EnergyAssessment(
            feasible=self.energy_wh + 1e-9 >= required,
            current_soc=self.soc,
            current_energy_wh=self.energy_wh,
            raw_mission_energy_wh=raw_energy,
            estimated_mission_energy_wh=estimated_energy,
            safety_reserve_wh=reserve,
            required_energy_wh=required,
            estimated_final_soc=final_soc,
            propulsion_energy_wh=(
                breakdown.propulsion_energy_wh
                * self.parameters.prediction_margin_factor
            ),
            auxiliary_energy_wh=(
                breakdown.auxiliary_energy_wh
                * self.parameters.prediction_margin_factor
            ),
            payload_energy_penalty_wh=(
                breakdown.payload_energy_penalty_wh
                * self.parameters.prediction_margin_factor
            ),
            initial_payload_mass_kg=profile.initial_payload_mass_kg,
        )
