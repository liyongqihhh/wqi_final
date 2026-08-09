from dataclasses import dataclass
import math
from pathlib import Path
import random

import yaml


@dataclass(frozen=True)
class ObstacleRoute:
    name: str
    layer: str
    speed_mps: float
    radius_m: float
    height_m: float
    waypoints: tuple[tuple[float, float, float], ...]


class ObstacleScenarioError(ValueError):
    pass


def obstacle_instance_name(index: int, route: ObstacleRoute) -> str:
    number = int(index)
    if number <= 0:
        raise ObstacleScenarioError("Obstacle instance index must be positive")
    return f"dynamic_obstacle_{number:02d}_{route.name}"


def load_obstacle_config(path):
    with Path(path).open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ObstacleScenarioError("Obstacle file must contain a mapping")
    densities = data.get("densities", {})
    raw_routes = data.get("routes", {})
    if not isinstance(densities, dict) or not isinstance(raw_routes, dict):
        raise ObstacleScenarioError("Obstacle densities and routes are required")
    routes = []
    for name, raw in raw_routes.items():
        try:
            layer = str(raw["layer"])
            speed = float(raw["speed_mps"])
            radius = float(raw["radius_m"])
            height = float(raw["height_m"])
            waypoints = tuple(
                tuple(float(value) for value in waypoint)
                for waypoint in raw["waypoints"]
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ObstacleScenarioError(
                f"Invalid obstacle route '{name}': {error}"
            ) from error
        if layer not in ("ground", "air"):
            raise ObstacleScenarioError(
                f"Route '{name}' layer must be ground or air"
            )
        values = (speed, radius, height, *(item for point in waypoints for item in point))
        if not all(math.isfinite(value) for value in values):
            raise ObstacleScenarioError(f"Route '{name}' contains non-finite values")
        if min(speed, radius, height) <= 0.0 or len(waypoints) < 2:
            raise ObstacleScenarioError(f"Route '{name}' has invalid geometry")
        routes.append(ObstacleRoute(
            name=str(name),
            layer=layer,
            speed_mps=speed,
            radius_m=radius,
            height_m=height,
            waypoints=waypoints,
        ))
    return densities, tuple(routes)


def select_routes(densities, routes, density: str, random_seed: int):
    name = str(density)
    if name not in densities:
        available = ", ".join(sorted(densities))
        raise ObstacleScenarioError(
            f"Unknown obstacle density '{name}'. Available: {available}"
        )
    requested = densities[name]
    if not isinstance(requested, dict):
        raise ObstacleScenarioError(f"Density '{name}' must be a mapping")
    generator = random.Random(int(random_seed))
    selected = []
    for layer in ("ground", "air"):
        count = int(requested.get(layer, 0))
        candidates = [route for route in routes if route.layer == layer]
        if count < 0 or count > len(candidates):
            raise ObstacleScenarioError(
                f"Density '{name}' requests {count} {layer} routes, "
                f"but {len(candidates)} are available"
            )
        selected.extend(generator.sample(candidates, count))
    generator.shuffle(selected)
    return tuple(selected)
