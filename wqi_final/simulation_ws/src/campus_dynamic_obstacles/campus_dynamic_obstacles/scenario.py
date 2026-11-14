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
    group: str = ""
    priority: int = 1
    activation_models: tuple[str, ...] = ()
    activation_point: tuple[float, float, float] = (0., 0., 0.)
    activation_distance: float = 0.


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
            models = tuple(str(v) for v in raw.get("activation_models", ()))
            activation_point = tuple(float(v) for v in raw.get("activation_point", (0,0,0)))
            activation_distance = float(raw.get("activation_distance", 0.))
            priority = int(raw.get("priority", 1))
        except (KeyError, TypeError, ValueError) as error:
            raise ObstacleScenarioError(
                f"Invalid obstacle route '{name}': {error}"
            ) from error
        if layer not in ("ground", "air"):
            raise ObstacleScenarioError(
                f"Route '{name}' layer must be ground or air"
            )
        if any(len(point)!=3 for point in waypoints) or len(activation_point)!=3:
            raise ObstacleScenarioError(f"Route '{name}' requires xyz coordinates")
        if activation_distance < 0 or (models and activation_distance <= 0) or any(not v for v in models):
            raise ObstacleScenarioError(f"Route '{name}' has an invalid activation gate")
        values = (speed, radius, height, activation_distance, *activation_point,
                  *(item for point in waypoints for item in point))
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
            group=str(raw.get("group", "")), priority=priority,
            activation_models=models, activation_point=activation_point,
            activation_distance=activation_distance,
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
        if candidates and all(route.group for route in candidates):
            # Every region receives traffic, and higher densities are strict
            # supersets for the same seed. Core crossings precede extra routes.
            groups = {}
            for group in sorted({route.group for route in candidates}):
                ordered=[]
                for priority in sorted({r.priority for r in candidates if r.group==group}):
                    bucket=[r for r in candidates if r.group==group and r.priority==priority]
                    random.Random(f"{random_seed}:{layer}:{group}:{priority}").shuffle(bucket)
                    ordered.extend(bucket)
                groups[group]=ordered
            ordered=[group[i] for i in range(max(map(len,groups.values())))
                     for group in groups.values() if i < len(group)]
            selected.extend(ordered[:count])
        else:
            selected.extend(generator.sample(candidates, count))
    generator.shuffle(selected)
    return tuple(selected)
