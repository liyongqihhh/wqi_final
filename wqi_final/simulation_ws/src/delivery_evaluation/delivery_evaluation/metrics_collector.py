from collections import defaultdict
import math
import threading
import time

from delivery_evaluation.energy_estimator import UgvEnergyIntegrator
from delivery_evaluation.models import RunRecord
from delivery_evaluation.path_metrics import PathAccumulator, endpoint_error


SAFETY_MEASUREMENT_PHASES = frozenset({"CRUISE", "RETURNING", "APPROACH"})


class MissionMetricsCollector:
    def __init__(
        self,
        ugv_energy: UgvEnergyIntegrator,
        ugv_collision_radius_m: float = 0.22,
        uav_collision_radius_m: float = 0.56,
    ) -> None:
        self.lock = threading.Lock()
        self.ugv_energy = ugv_energy
        self.ugv_path = PathAccumulator(
            ugv_energy.parameters.maximum_position_jump_m
        )
        self.uav_path = PathAccumulator(maximum_jump_m=4.0)
        self.latest_ugv = None
        self.latest_uav = None
        self.latest_dynamic_obstacles = {}
        self.ugv_collision_radius_m = float(ugv_collision_radius_m)
        self.uav_collision_radius_m = float(uav_collision_radius_m)
        if min(self.ugv_collision_radius_m, self.uav_collision_radius_m) <= 0.0:
            raise ValueError("Robot collision radii must be positive")
        self._active_ugv_contacts = set()
        self._active_uav_contacts = set()
        self.latest_battery_soc = None
        self.latest_consumed_wh = None
        self.latest_charged_wh = None
        self.latest_ugv_drive_soc = None
        self.latest_ugv_charging_soc = None
        self.latest_ugv_drive_consumed_wh = None
        self.latest_ugv_charging_consumed_wh = None
        self.latest_docked = None
        self.latest_uav_replans = 0
        self.active = False
        self.record = None
        self._phase = None
        self._phase_started_ns = None
        self._phase_durations = defaultdict(float)
        self._hold_started_ns = None
        self._start_sim_ns = 0
        self._start_wall = 0.0
        self._baseline_consumed_wh = 0.0
        self._baseline_charged_wh = 0.0
        self._baseline_ugv_drive_consumed_wh = None
        self._baseline_ugv_charging_consumed_wh = None
        self._delivery_errors_uav = []
        self._delivery_errors_ugv = []
        self._marked_uav_targets = set()
        self._marked_ugv_targets = set()

    @staticmethod
    def _stamp_ns(message) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _position(message):
        point = message.pose.pose.position
        return float(point.x), float(point.y), float(point.z)

    @staticmethod
    def _linear_speed(message) -> float:
        vector = message.twist.twist.linear
        return math.sqrt(vector.x ** 2 + vector.y ** 2 + vector.z ** 2)

    def update_ugv(self, message, payload_mass_kg: float = 0.0) -> None:
        stamp = self._stamp_ns(message)
        position = self._position(message)
        linear = self._linear_speed(message)
        angular = float(message.twist.twist.angular.z)
        with self.lock:
            self.latest_ugv = (position, linear)
            if self.active:
                self.ugv_path.add(stamp, *position)
                self.ugv_energy.update(stamp, linear, angular, payload_mass_kg)
                self._measure_dynamic_clearances_locked()

    def update_uav(self, message) -> None:
        stamp = self._stamp_ns(message)
        position = self._position(message)
        speed = self._linear_speed(message)
        with self.lock:
            self.latest_uav = (position, speed)
            if self.active:
                self.uav_path.add(stamp, *position)
                self._measure_dynamic_clearances_locked()

    def update_dynamic_obstacle(
        self,
        name: str,
        layer: str,
        radius_m: float,
        message,
    ) -> None:
        position = self._position(message)
        obstacle_layer = str(layer)
        if obstacle_layer not in ("ground", "air"):
            raise ValueError(f"Unknown dynamic obstacle layer: {obstacle_layer}")
        obstacle_radius = float(radius_m)
        if obstacle_radius <= 0.0:
            raise ValueError("Dynamic obstacle radius must be positive")
        with self.lock:
            self.latest_dynamic_obstacles[str(name)] = (
                position,
                obstacle_layer,
                obstacle_radius,
            )
            if self.active:
                self._measure_dynamic_clearances_locked()

    def _record_dynamic_clearance_locked(
        self,
        robot: str,
        obstacle_name: str,
        clearance_m: float,
    ) -> None:
        if robot == "ugv":
            field = "minimum_ugv_obstacle_clearance_m"
            contacts = self._active_ugv_contacts
            collision_field = "ugv_collision_count"
        else:
            field = "minimum_uav_dynamic_clearance_m"
            contacts = self._active_uav_contacts
            collision_field = "uav_collision_count"
        current = getattr(self.record, field)
        if current is None or clearance_m < current:
            setattr(self.record, field, clearance_m)
        if clearance_m <= 0.0:
            if obstacle_name not in contacts:
                setattr(
                    self.record,
                    collision_field,
                    getattr(self.record, collision_field) + 1,
                )
            contacts.add(obstacle_name)
            self.record.collision_free = False
        elif clearance_m > 0.05:
            contacts.discard(obstacle_name)

    def _measure_dynamic_clearances_locked(self) -> None:
        for name, (obstacle, layer, radius_m) in (
            self.latest_dynamic_obstacles.items()
        ):
            if layer == "ground" and self.latest_ugv is not None:
                robot = self.latest_ugv[0]
                clearance = math.hypot(
                    robot[0] - obstacle[0],
                    robot[1] - obstacle[1],
                ) - (self.ugv_collision_radius_m + radius_m)
                self._record_dynamic_clearance_locked("ugv", name, clearance)
            elif layer == "air" and self.latest_uav is not None:
                robot = self.latest_uav[0]
                clearance = math.sqrt(
                    sum(
                        (robot[index] - obstacle[index]) ** 2
                        for index in range(3)
                    )
                ) - (self.uav_collision_radius_m + radius_m)
                self._record_dynamic_clearance_locked("uav", name, clearance)

    def update_battery(self, soc: float) -> None:
        with self.lock:
            self.latest_battery_soc = float(soc)

    def update_consumed(self, value: float) -> None:
        with self.lock:
            self.latest_consumed_wh = float(value)

    def update_charged(self, value: float) -> None:
        with self.lock:
            self.latest_charged_wh = float(value)

    def update_ugv_drive_battery(self, soc: float) -> None:
        with self.lock:
            self.latest_ugv_drive_soc = float(soc)

    def update_ugv_charging_battery(self, soc: float) -> None:
        with self.lock:
            self.latest_ugv_charging_soc = float(soc)

    def update_ugv_drive_consumed(self, value: float) -> None:
        with self.lock:
            self.latest_ugv_drive_consumed_wh = float(value)

    def update_ugv_charging_consumed(self, value: float) -> None:
        with self.lock:
            self.latest_ugv_charging_consumed_wh = float(value)

    def update_docked(self, docked: bool) -> None:
        with self.lock:
            value = bool(docked)
            previous = self.latest_docked
            self.latest_docked = value
            if self.active and previous is True and value is False:
                self.record.detach_success = True
            if self.active and previous is False and value is True:
                self.record.redock_success = True

    def update_replans(self, count: int) -> None:
        with self.lock:
            self.latest_uav_replans = max(0, int(count))
            if self.active:
                self.record.uav_replan_count = max(
                    self.record.uav_replan_count, self.latest_uav_replans
                )

    def update_clearance(self, value: float) -> None:
        distance = float(value)
        if not math.isfinite(distance):
            return
        with self.lock:
            if (
                not self.active
                or self._phase not in SAFETY_MEASUREMENT_PHASES
            ):
                return
            current = self.record.minimum_uav_clearance_m
            if current is None or distance < current:
                self.record.minimum_uav_clearance_m = distance

    def update_safety(self, blocked: bool, sim_ns: int) -> None:
        with self.lock:
            if not self.active:
                return
            if self._phase not in SAFETY_MEASUREMENT_PHASES:
                return
            if blocked and self._hold_started_ns is None:
                self._hold_started_ns = int(sim_ns)
                self.record.uav_safety_hold_count += 1
            elif not blocked and self._hold_started_ns is not None:
                self.record.uav_safety_hold_s += max(
                    0.0, (int(sim_ns) - self._hold_started_ns) / 1.0e9
                )
                self._hold_started_ns = None

    def set_phase(self, phase: str, sim_ns: int) -> None:
        phase_name = str(phase).split(":", 1)[0].strip() or "UNKNOWN"
        with self.lock:
            if not self.active or phase_name == self._phase:
                return
            previous_phase = self._phase
            if self._phase is not None and self._phase_started_ns is not None:
                self._phase_durations[self._phase] += max(
                    0.0, (int(sim_ns) - self._phase_started_ns) / 1.0e9
                )
            if (
                previous_phase in SAFETY_MEASUREMENT_PHASES
                and phase_name not in SAFETY_MEASUREMENT_PHASES
                and self._hold_started_ns is not None
            ):
                self.record.uav_safety_hold_s += max(
                    0.0, (int(sim_ns) - self._hold_started_ns) / 1.0e9
                )
                self._hold_started_ns = None
            self._phase = phase_name
            self._phase_started_ns = int(sim_ns)

    def start(self, record: RunRecord, sim_ns: int) -> None:
        with self.lock:
            self.record = record
            self.active = True
            self.ugv_path.reset()
            self.uav_path.reset()
            self.ugv_energy.reset()
            self._phase = None
            self._phase_started_ns = None
            self._phase_durations = defaultdict(float)
            self._hold_started_ns = None
            self._start_sim_ns = int(sim_ns)
            self._start_wall = time.monotonic()
            self._baseline_consumed_wh = self.latest_consumed_wh
            self._baseline_charged_wh = self.latest_charged_wh
            self._baseline_ugv_drive_consumed_wh = (
                self.latest_ugv_drive_consumed_wh
            )
            self._baseline_ugv_charging_consumed_wh = (
                self.latest_ugv_charging_consumed_wh
            )
            self._delivery_errors_uav = []
            self._delivery_errors_ugv = []
            self._marked_uav_targets = set()
            self._marked_ugv_targets = set()
            self._active_ugv_contacts = set()
            self._active_uav_contacts = set()
            self._measure_dynamic_clearances_locked()
            record.initial_uav_soc = self.latest_battery_soc
            record.initial_ugv_drive_soc = self.latest_ugv_drive_soc
            record.initial_ugv_charging_soc = self.latest_ugv_charging_soc
            if self.latest_docked:
                record.redock_success = True

    def mark_ugv_endpoint(self, target, target_name=None) -> None:
        with self.lock:
            key = target_name or (target.x, target.y, target.z)
            if (
                not self.active
                or self.latest_ugv is None
                or key in self._marked_ugv_targets
            ):
                return
            position = self.latest_ugv[0]
            self._delivery_errors_ugv.append(
                endpoint_error(position, (target.x, target.y, target.z), 2)
            )
            self._marked_ugv_targets.add(key)

    def mark_uav_endpoint(self, target, target_name=None) -> None:
        with self.lock:
            key = target_name or (target.x, target.y, target.z)
            if (
                not self.active
                or self.latest_uav is None
                or key in self._marked_uav_targets
            ):
                return
            position = self.latest_uav[0]
            self._delivery_errors_uav.append(
                endpoint_error(position, (target.x, target.y, target.z), 3)
            )
            self._marked_uav_targets.add(key)

    def finish(self, sim_ns: int) -> RunRecord:
        with self.lock:
            if not self.active or self.record is None:
                raise RuntimeError("No active evaluation run")
            self._measure_dynamic_clearances_locked()
            end_sim_ns = int(sim_ns)
            if self._phase is not None and self._phase_started_ns is not None:
                self._phase_durations[self._phase] += max(
                    0.0, (end_sim_ns - self._phase_started_ns) / 1.0e9
                )
            if self._hold_started_ns is not None:
                self.record.uav_safety_hold_s += max(
                    0.0, (end_sim_ns - self._hold_started_ns) / 1.0e9
                )
                self._hold_started_ns = None
            record = self.record
            record.sim_duration_s = max(
                0.0, (end_sim_ns - self._start_sim_ns) / 1.0e9
            )
            record.wall_duration_s = max(0.0, time.monotonic() - self._start_wall)
            record.real_time_factor = (
                record.sim_duration_s / record.wall_duration_s
                if record.wall_duration_s > 0.0 else 0.0
            )
            record.phase_durations_s = dict(sorted(self._phase_durations.items()))
            record.ugv_path_length_m = self.ugv_path.length_m
            record.uav_path_length_m = self.uav_path.length_m
            if (
                self.latest_ugv_drive_consumed_wh is not None
                and self._baseline_ugv_drive_consumed_wh is not None
            ):
                record.ugv_drive_energy_wh = max(
                    0.0,
                    self.latest_ugv_drive_consumed_wh
                    - self._baseline_ugv_drive_consumed_wh,
                )
            else:
                record.ugv_drive_energy_wh = self.ugv_energy.energy_wh
            if (
                self.latest_ugv_charging_consumed_wh is not None
                and self._baseline_ugv_charging_consumed_wh is not None
            ):
                record.ugv_charging_energy_wh = max(
                    0.0,
                    self.latest_ugv_charging_consumed_wh
                    - self._baseline_ugv_charging_consumed_wh,
                )
            record.ugv_energy_wh = (
                record.ugv_drive_energy_wh
                + record.ugv_charging_energy_wh
            )
            if (
                self.latest_consumed_wh is not None
                and self._baseline_consumed_wh is not None
            ):
                record.uav_energy_wh = max(
                    0.0,
                    self.latest_consumed_wh - self._baseline_consumed_wh,
                )
            if (
                self.latest_charged_wh is not None
                and self._baseline_charged_wh is not None
            ):
                record.uav_charged_wh = max(
                    0.0,
                    self.latest_charged_wh - self._baseline_charged_wh,
                )
            # Energy delivered from the UGV charging pack is already counted
            # at its source. Subtract UAV charged energy to avoid double count.
            record.total_energy_wh = record.ugv_energy_wh + max(
                0.0,
                record.uav_energy_wh - record.uav_charged_wh,
            )
            if record.completed_targets > 0:
                record.delivery_rate_per_min = (
                    record.completed_targets * 60.0 / record.sim_duration_s
                    if record.sim_duration_s > 0.0 else 0.0
                )
                record.energy_per_completed_target_wh = (
                    record.total_energy_wh / record.completed_targets
                )
            record.final_uav_soc = self.latest_battery_soc
            record.final_ugv_drive_soc = self.latest_ugv_drive_soc
            record.final_ugv_charging_soc = self.latest_ugv_charging_soc
            if self._delivery_errors_ugv:
                record.ugv_endpoint_error_m = max(self._delivery_errors_ugv)
            if self._delivery_errors_uav:
                record.uav_endpoint_error_m = max(self._delivery_errors_uav)
            self.active = False
            self.record = None
            return record
