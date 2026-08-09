from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass(frozen=True)
class PoseTarget:
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class DeliverySpec:
    name: str
    floor: int
    payload_kg: float
    ugv_pose: PoseTarget
    uav_pose: PoseTarget


@dataclass(frozen=True)
class ExperimentScenario:
    name: str
    deliveries: tuple[DeliverySpec, ...]
    ugv_home: PoseTarget
    uav_home: PoseTarget
    precondition_position_tolerance_m: float
    precondition_speed_tolerance_mps: float
    action_timeout_s: float
    settle_time_s: float


@dataclass
class RunRecord:
    run_id: str
    mode: str
    scenario: str
    repetition: int
    obstacle_density: str
    random_seed: int
    targets: list[str]
    success: bool = False
    failure_reason: str = ""
    sim_duration_s: float = 0.0
    wall_duration_s: float = 0.0
    real_time_factor: float = 0.0
    phase_durations_s: dict[str, float] = field(default_factory=dict)
    delivery_rate_per_min: float = 0.0
    energy_per_completed_target_wh: float = 0.0
    ugv_path_length_m: float = 0.0
    uav_path_length_m: float = 0.0
    ugv_endpoint_error_m: float | None = None
    uav_endpoint_error_m: float | None = None
    nav_recovery_count: int = 0
    uav_replan_count: int = 0
    uav_safety_hold_count: int = 0
    uav_safety_hold_s: float = 0.0
    minimum_uav_clearance_m: float | None = None
    minimum_ugv_obstacle_clearance_m: float | None = None
    minimum_uav_dynamic_clearance_m: float | None = None
    ugv_collision_count: int = 0
    uav_collision_count: int = 0
    collision_free: bool = True
    detach_success: bool = False
    redock_success: bool = False
    ugv_energy_wh: float = 0.0
    ugv_drive_energy_wh: float = 0.0
    ugv_charging_energy_wh: float = 0.0
    uav_energy_wh: float = 0.0
    uav_charged_wh: float = 0.0
    total_energy_wh: float = 0.0
    initial_uav_soc: float | None = None
    final_uav_soc: float | None = None
    initial_ugv_drive_soc: float | None = None
    final_ugv_drive_soc: float | None = None
    initial_ugv_charging_soc: float | None = None
    final_ugv_charging_soc: float | None = None
    completed_targets: int = 0

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
