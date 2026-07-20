from pathlib import Path
from dataclasses import replace
import math

import pytest

from cooperative_delivery.mission_config import (
    CooperativeMissionConfig,
    MissionConfigurationError,
)
from cooperative_delivery.mission_states import navigation_timeout_for_distance


CONFIG = Path(__file__).parents[1] / "config" / "cooperative_waypoints.yaml"


def test_teaching_mission_stops_at_door_and_delivers_vertically():
    config = CooperativeMissionConfig(CONFIG)
    target = config.resolve(["teaching_building"])[0]

    assert target.ugv_launch.name == "teaching_door"
    assert (target.ugv_launch.x, target.ugv_launch.y) == (-40.0, 9.5)
    assert target.uav_target == "teaching_building"
    assert target.uav_home_node == "teaching_building"


def test_dormitories_share_ugv_stop_but_use_distinct_uav_delivery_points():
    config = CooperativeMissionConfig(CONFIG)
    targets = config.resolve([f"dormitory_{index}" for index in range(1, 5)])

    assert [target.uav_target for target in targets] == [
        "dormitory_1",
        "dormitory_2",
        "dormitory_3",
        "dormitory_4",
    ]
    assert {target.ugv_launch.name for target in targets} == {
        "dormitory_service_point"
    }
    assert {target.uav_home_node for target in targets} == {"dormitory"}


def test_generic_dormitory_name_keeps_backward_compatible_first_building():
    config = CooperativeMissionConfig(CONFIG)
    target = config.resolve(["dormitory"])[0]

    assert target.uav_target == "dormitory_1"
    assert target.uav_home_node == "dormitory"


def test_representative_multi_target_mission_preserves_requested_order():
    config = CooperativeMissionConfig(CONFIG)
    targets = config.resolve(["laboratory", "library", "dormitory"])

    assert [target.name for target in targets] == [
        "laboratory",
        "library",
        "dormitory",
    ]
    assert [target.ugv_launch.name for target in targets] == [
        "laboratory_door",
        "library_door",
        "dormitory_service_point",
    ]
    assert [target.uav_home_node for target in targets] == [
        "laboratory",
        "library",
        "dormitory",
    ]


def test_optimizer_shortens_route_and_keeps_payload_with_its_target():
    config = CooperativeMissionConfig(CONFIG)
    requested = [
        replace(target, payload_mass_kg=payload, delivery_floor=floor)
        for target, payload, floor in zip(
            config.resolve([
                "library", "teaching_building", "laboratory"
            ]),
            (0.15, 0.35, 0.55),
            (2, 4, 6),
        )
    ]
    optimized, plan = config.optimize_targets(requested, return_home=True)

    mapping = {
        target.name: (target.payload_mass_kg, target.delivery_floor)
        for target in optimized
    }
    assert mapping == {
        "library": (0.15, 2),
        "teaching_building": (0.35, 4),
        "laboratory": (0.55, 6),
    }

    points = [config.ugv_home, *(target.ugv_launch for target in requested)]
    requested_cost = sum(
        math.hypot(second.x - first.x, second.y - first.y)
        for first, second in zip(points, points[1:])
    ) + math.hypot(
        requested[-1].ugv_launch.x - config.ugv_home.x,
        requested[-1].ugv_launch.y - config.ugv_home.y,
    )
    assert plan.total_cost < requested_cost


def test_logistics_target_supports_fast_closed_loop_regression():
    config = CooperativeMissionConfig(CONFIG)
    target = config.resolve(["logistics_center"])[0]

    assert target.ugv_launch == config.ugv_home
    assert target.uav_target == "logistics_center"
    assert target.uav_home_node == "logistics_center"


def test_unknown_cooperative_target_is_rejected():
    config = CooperativeMissionConfig(CONFIG)
    with pytest.raises(MissionConfigurationError):
        config.resolve(["unknown_building"])


def test_long_campus_route_gets_virtualbox_navigation_budget():
    config = CooperativeMissionConfig(CONFIG)
    settings = config.settings
    timeout = navigation_timeout_for_distance(
        112.0,
        float(settings["navigation_timeout_min"]),
        float(settings["navigation_timeout_per_meter"]),
        float(settings["navigation_timeout_max"]),
    )
    assert timeout > 300.0
    assert timeout <= float(settings["navigation_timeout_max"])
    assert float(settings["navigation_stall_timeout"]) >= 120.0
    assert float(settings["navigation_progress_distance"]) <= 0.1
    assert int(settings["navigation_retry_count"]) >= 1
