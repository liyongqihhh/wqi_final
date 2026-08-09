from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
import math

import pytest
import yaml

from campus_dynamic_obstacles.scenario import (
    ObstacleScenarioError,
    ObstacleRoute,
    load_obstacle_config,
    obstacle_instance_name,
    select_routes,
)


CONFIG = """
densities:
  none: {ground: 0, air: 0}
  mixed: {ground: 1, air: 1}
routes:
  ground_a:
    layer: ground
    speed_mps: 0.4
    radius_m: 0.5
    height_m: 1.5
    waypoints: [[0, 0, 0.75], [2, 0, 0.75]]
  air_a:
    layer: air
    speed_mps: 0.6
    radius_m: 0.7
    height_m: 1.4
    waypoints: [[0, 0, 15], [0, 5, 15]]
"""


def test_selection_is_deterministic_and_respects_layers(tmp_path: Path):
    path = tmp_path / "obstacles.yaml"
    path.write_text(CONFIG, encoding="utf-8")
    densities, routes = load_obstacle_config(path)
    first = select_routes(densities, routes, "mixed", 7)
    second = select_routes(densities, routes, "mixed", 7)
    assert first == second
    assert {route.layer for route in first} == {"ground", "air"}


def test_unknown_density_is_rejected(tmp_path: Path):
    path = tmp_path / "obstacles.yaml"
    path.write_text(CONFIG, encoding="utf-8")
    densities, routes = load_obstacle_config(path)
    with pytest.raises(ObstacleScenarioError):
        select_routes(densities, routes, "missing", 1)


def test_obstacle_instance_name_matches_spawn_and_evaluation_topics():
    route = _route("ground")
    assert (
        obstacle_instance_name(3, route)
        == "dynamic_obstacle_03_ground_test"
    )
    with pytest.raises(ObstacleScenarioError):
        obstacle_instance_name(0, route)


def test_spawner_script_is_executable():
    path = (
        Path(__file__).parents[1]
        / "scripts"
        / "dynamic_obstacle_spawner"
    )
    assert path.is_file()
    assert path.stat().st_mode & 0o111


def _load_spawner_module():
    path = (
        Path(__file__).parents[1]
        / "scripts"
        / "dynamic_obstacle_spawner"
    )
    loader = SourceFileLoader("dynamic_obstacle_spawner_test", str(path))
    spec = spec_from_loader(loader.name, loader)
    assert spec is not None
    module = module_from_spec(spec)
    loader.exec_module(module)
    return module


def _route(layer: str):
    z = 0.8 if layer == "ground" else 15.0
    return ObstacleRoute(
        name=f"{layer}_test",
        layer=layer,
        speed_mps=0.4,
        radius_m=0.45,
        height_m=1.6,
        waypoints=((0.0, 0.0, z), (2.0, 0.0, z)),
    )


def test_spawned_models_follow_fixed_routes_without_robot_yielding():
    spawner = _load_spawner_module()
    ground_sdf = spawner.DynamicObstacleSpawner._sdf("ground", _route("ground"))
    air_sdf = spawner.DynamicObstacleSpawner._sdf("air", _route("air"))

    for token in (
        "<max_acceleration>1.20</max_acceleration>",
        "libgazebo_ros_p3d.so",
        "name='ground_truth_ground'",
        "odom:=/dynamic_obstacles/ground/odom",
        "<update_rate>20.0</update_rate>",
    ):
        assert token in ground_sdf
    for forbidden in (
        "yield_speed_ratio",
        "minimum_yield_speed",
        "yield_radius",
        "resume_radius",
        "avoidance_gain",
        "yield_model",
        "ugvcar",
        "campus_uav",
    ):
        assert forbidden not in ground_sdf
        assert forbidden not in air_sdf
    assert "<mass>1.000</mass>" in ground_sdf
    assert "<mass>0.500</mass>" in air_sdf


def _point_to_segment_distance(point, start, end):
    delta_x = end[0] - start[0]
    delta_y = end[1] - start[1]
    length_squared = delta_x * delta_x + delta_y * delta_y
    if length_squared <= 1.0e-12:
        return math.dist(point, start[:2])
    projection = (
        (point[0] - start[0]) * delta_x
        + (point[1] - start[1]) * delta_y
    ) / length_squared
    ratio = min(1.0, max(0.0, projection))
    closest = (
        start[0] + ratio * delta_x,
        start[1] + ratio * delta_y,
    )
    return math.dist(point, closest)


def test_ground_routes_leave_combined_robot_clear_at_delivery_bays():
    config_path = Path(__file__).parents[1] / "config" / "obstacle_routes.yaml"
    with config_path.open(encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    _, routes = load_obstacle_config(config_path)

    combined_robot_radius = 0.60
    clearance_margin = 0.15
    for stop_name, stop in raw["protected_stops"].items():
        for route in (route for route in routes if route.layer == "ground"):
            minimum_distance = min(
                _point_to_segment_distance(stop, start, end)
                for start, end in zip(route.waypoints, route.waypoints[1:])
            )
            required = combined_robot_radius + route.radius_m + clearance_margin
            assert minimum_distance >= required, (
                f"{route.name} passes too close to {stop_name}: "
                f"{minimum_distance:.2f} m < {required:.2f} m"
            )


def test_rear_diagonal_regression_config_is_reproducible():
    config_path = (
        Path(__file__).parents[1]
        / "config"
        / "rear_diagonal_regression.yaml"
    )
    densities, routes = load_obstacle_config(config_path)
    selected = select_routes(densities, routes, "rear_catch", 42)

    assert len(selected) == 1
    assert selected[0].name == "rear_diagonal_catch"
    assert selected[0].speed_mps == pytest.approx(0.70)
    assert selected[0].waypoints[0][1] > selected[0].waypoints[1][1]
    assert selected[0].waypoints[-1][0] <= 30.0
