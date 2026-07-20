from dataclasses import dataclass
import math
from pathlib import Path

import yaml
from uav_navigation.route_optimizer import RoutePlan, optimize_visit_order


class MissionConfigurationError(ValueError):
    pass


@dataclass(frozen=True)
class GroundWaypoint:
    name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class DeliveryTarget:
    name: str
    ugv_launch: GroundWaypoint
    uav_target: str
    uav_home_node: str
    payload_mass_kg: float
    delivery_floor: int | None = None


class CooperativeMissionConfig:
    def __init__(self, path) -> None:
        self.path = Path(path)
        with self.path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        if not isinstance(data, dict):
            raise MissionConfigurationError("Cooperative config must be a mapping")

        self.settings = data.get("mission", {})
        self.waypoints = self._load_waypoints(data.get("ugv_waypoints", {}))
        self.targets = self._load_targets(data.get("targets", {}))
        self._validate_settings()

    @staticmethod
    def _load_waypoints(raw_waypoints) -> dict[str, GroundWaypoint]:
        if not isinstance(raw_waypoints, dict) or not raw_waypoints:
            raise MissionConfigurationError("No UGV launch waypoints are configured")
        result = {}
        for name, raw in raw_waypoints.items():
            try:
                waypoint = GroundWaypoint(
                    name=str(name),
                    x=float(raw["x"]),
                    y=float(raw["y"]),
                    yaw=float(raw.get("yaw", 0.0)),
                )
            except (AttributeError, KeyError, TypeError, ValueError) as error:
                raise MissionConfigurationError(
                    f"Invalid UGV waypoint '{name}': {error}"
                ) from error
            if not all(math.isfinite(value) for value in (
                waypoint.x, waypoint.y, waypoint.yaw
            )):
                raise MissionConfigurationError(
                    f"UGV waypoint '{name}' contains a non-finite value"
                )
            result[waypoint.name] = waypoint
        return result

    def _load_targets(self, raw_targets) -> dict[str, DeliveryTarget]:
        if not isinstance(raw_targets, dict) or not raw_targets:
            raise MissionConfigurationError("No cooperative targets are configured")
        result = {}
        for name, raw in raw_targets.items():
            try:
                launch_name = str(raw["ugv_launch"])
                launch = self.waypoints[launch_name]
                target = DeliveryTarget(
                    name=str(name),
                    ugv_launch=launch,
                    uav_target=str(raw["uav_target"]),
                    uav_home_node=str(raw["uav_home_node"]),
                    payload_mass_kg=float(raw["payload_mass_kg"]),
                )
            except (AttributeError, KeyError, TypeError, ValueError) as error:
                raise MissionConfigurationError(
                    f"Invalid cooperative target '{name}': {error}"
                ) from error
            if not target.uav_target or not target.uav_home_node:
                raise MissionConfigurationError(
                    f"Target '{name}' must define UAV target and home node"
                )
            if (
                not math.isfinite(target.payload_mass_kg)
                or target.payload_mass_kg < 0.0
            ):
                raise MissionConfigurationError(
                    f"Target '{name}' payload mass must be non-negative"
                )
            result[target.name] = target
        return result

    def _positive_float(self, name: str) -> float:
        try:
            value = float(self.settings[name])
        except (KeyError, TypeError, ValueError) as error:
            raise MissionConfigurationError(
                f"Invalid mission setting '{name}'"
            ) from error
        if not math.isfinite(value) or value <= 0.0:
            raise MissionConfigurationError(
                f"Mission setting '{name}' must be positive"
            )
        return value

    def _nonnegative_int(self, name: str) -> int:
        try:
            raw_value = self.settings[name]
            value = int(raw_value)
        except (KeyError, TypeError, ValueError) as error:
            raise MissionConfigurationError(
                f"Invalid mission setting '{name}'"
            ) from error
        if isinstance(raw_value, float) and not raw_value.is_integer():
            raise MissionConfigurationError(
                f"Mission setting '{name}' must be an integer"
            )
        if value < 0:
            raise MissionConfigurationError(
                f"Mission setting '{name}' cannot be negative"
            )
        return value

    def _validate_settings(self) -> None:
        home = str(self.settings.get("ugv_home", ""))
        if home not in self.waypoints:
            raise MissionConfigurationError("Configured UGV home does not exist")
        for name in (
            "navigation_timeout_min",
            "navigation_timeout_per_meter",
            "navigation_timeout_max",
            "navigation_stall_timeout",
            "navigation_progress_distance",
            "navigation_progress_angle",
            "navigation_retry_delay",
            "uav_mission_timeout",
            "feedback_rate",
            "ugv_settle_speed",
            "ugv_settle_duration",
            "ugv_settle_timeout",
            "docking_timeout",
            "uav_landing_height",
            "ugv_energy_planning_speed",
        ):
            self._positive_float(name)
        self._nonnegative_int("navigation_retry_count")
        if (
            self._positive_float("navigation_timeout_min")
            > self._positive_float("navigation_timeout_max")
        ):
            raise MissionConfigurationError(
                "Minimum navigation timeout cannot exceed its maximum"
            )

    @property
    def ugv_home(self) -> GroundWaypoint:
        return self.waypoints[str(self.settings["ugv_home"])]

    def resolve(self, names) -> list[DeliveryTarget]:
        resolved = []
        for name in names:
            if name not in self.targets:
                available = ", ".join(sorted(self.targets))
                raise MissionConfigurationError(
                    f"Unknown cooperative target '{name}'. Available: {available}"
                )
            resolved.append(self.targets[name])
        return resolved

    def optimize_targets(
        self,
        targets: list[DeliveryTarget],
        return_home: bool,
    ) -> tuple[list[DeliveryTarget], RoutePlan]:
        values = list(targets)
        home = self.ugv_home

        def distance(first: GroundWaypoint, second: GroundWaypoint) -> float:
            return math.hypot(second.x - first.x, second.y - first.y)

        plan = optimize_visit_order(
            len(values),
            lambda index: distance(home, values[index].ugv_launch),
            lambda origin, destination: distance(
                values[origin].ugv_launch,
                values[destination].ugv_launch,
            ),
            (
                lambda index: distance(values[index].ugv_launch, home)
            ) if return_home else None,
        )
        return [values[index] for index in plan.order], plan
