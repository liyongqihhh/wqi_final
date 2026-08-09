import math
from pathlib import Path

import yaml

from delivery_evaluation.models import (
    DeliverySpec,
    ExperimentScenario,
    PoseTarget,
)


class ScenarioConfigurationError(ValueError):
    pass


def _finite(value, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ScenarioConfigurationError(f"{name} must be finite")
    return result


def _pose(raw, name: str) -> PoseTarget:
    if not isinstance(raw, dict):
        raise ScenarioConfigurationError(f"{name} must be a mapping")
    try:
        return PoseTarget(
            x=_finite(raw["x"], f"{name}.x"),
            y=_finite(raw["y"], f"{name}.y"),
            z=_finite(raw.get("z", 0.0), f"{name}.z"),
            yaw=_finite(raw.get("yaw", 0.0), f"{name}.yaw"),
        )
    except KeyError as error:
        raise ScenarioConfigurationError(
            f"{name} is missing {error.args[0]}"
        ) from error


def load_scenarios(path) -> dict[str, ExperimentScenario]:
    config_path = Path(path)
    with config_path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ScenarioConfigurationError("Scenario file must contain a mapping")
    defaults = data.get("defaults", {})
    raw_scenarios = data.get("scenarios", {})
    if not isinstance(raw_scenarios, dict) or not raw_scenarios:
        raise ScenarioConfigurationError("No experiment scenarios are configured")

    ugv_home = _pose(defaults.get("ugv_home"), "defaults.ugv_home")
    uav_home = _pose(defaults.get("uav_home"), "defaults.uav_home")
    position_tolerance = _finite(
        defaults.get("precondition_position_tolerance_m", 1.5),
        "precondition_position_tolerance_m",
    )
    speed_tolerance = _finite(
        defaults.get("precondition_speed_tolerance_mps", 0.1),
        "precondition_speed_tolerance_mps",
    )
    action_timeout = _finite(
        defaults.get("action_timeout_s", 1800.0), "action_timeout_s"
    )
    settle_time = _finite(defaults.get("settle_time_s", 3.0), "settle_time_s")
    if min(position_tolerance, speed_tolerance, action_timeout, settle_time) <= 0.0:
        raise ScenarioConfigurationError("Scenario limits must be positive")

    scenarios = {}
    for scenario_name, raw_scenario in raw_scenarios.items():
        raw_deliveries = raw_scenario.get("deliveries", [])
        if not isinstance(raw_deliveries, list) or not raw_deliveries:
            raise ScenarioConfigurationError(
                f"Scenario '{scenario_name}' has no deliveries"
            )
        deliveries = []
        for index, raw in enumerate(raw_deliveries):
            prefix = f"scenarios.{scenario_name}.deliveries[{index}]"
            try:
                floor = int(raw["floor"])
                payload = _finite(raw["payload_kg"], f"{prefix}.payload_kg")
                name = str(raw["name"])
            except (KeyError, TypeError, ValueError) as error:
                raise ScenarioConfigurationError(
                    f"Invalid delivery '{prefix}': {error}"
                ) from error
            if not name or floor <= 0 or payload < 0.0:
                raise ScenarioConfigurationError(
                    f"Invalid name, floor, or payload in '{prefix}'"
                )
            deliveries.append(DeliverySpec(
                name=name,
                floor=floor,
                payload_kg=payload,
                ugv_pose=_pose(raw.get("ugv_pose"), f"{prefix}.ugv_pose"),
                uav_pose=_pose(raw.get("uav_pose"), f"{prefix}.uav_pose"),
            ))
        scenarios[str(scenario_name)] = ExperimentScenario(
            name=str(scenario_name),
            deliveries=tuple(deliveries),
            ugv_home=ugv_home,
            uav_home=uav_home,
            precondition_position_tolerance_m=position_tolerance,
            precondition_speed_tolerance_mps=speed_tolerance,
            action_timeout_s=action_timeout,
            settle_time_s=settle_time,
        )
    return scenarios
