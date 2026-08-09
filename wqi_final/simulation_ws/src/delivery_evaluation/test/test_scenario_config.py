from pathlib import Path

import pytest

from delivery_evaluation.scenario_config import (
    ScenarioConfigurationError,
    load_scenarios,
)


def test_loads_delivery_scenario(tmp_path: Path):
    path = tmp_path / "scenarios.yaml"
    path.write_text(
        """
defaults:
  ugv_home: {x: 0, y: 0}
  uav_home: {x: 0, y: 0, z: 0.4}
scenarios:
  sample:
    deliveries:
      - name: building
        floor: 2
        payload_kg: 0.5
        ugv_pose: {x: 2, y: 3}
        uav_pose: {x: 2, y: 3, z: 5}
""",
        encoding="utf-8",
    )
    scenario = load_scenarios(path)["sample"]
    assert scenario.deliveries[0].name == "building"
    assert scenario.deliveries[0].uav_pose.z == pytest.approx(5.0)


def test_rejects_empty_scenario_file(tmp_path: Path):
    path = tmp_path / "scenarios.yaml"
    path.write_text("scenarios: {}\n", encoding="utf-8")
    with pytest.raises(ScenarioConfigurationError):
        load_scenarios(path)


def test_logistics_smoke_target_matches_floor_one_delivery_altitude():
    config = Path(__file__).parents[1] / "config" / "experiment_scenarios.yaml"
    scenario = load_scenarios(config)["logistics_center_smoke"]
    delivery = scenario.deliveries[0]

    assert delivery.floor == 1
    assert delivery.uav_pose.z == pytest.approx(1.6)
