# Copyright 2026 liyongqihhh
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import itertools
import math

import pytest

from ugvcar_application.delivery_task_manager import optimize_order


def _cost(waypoints, order):
    route = ["home", *order, "home"]
    return sum(
        math.hypot(
            waypoints[second]["x"] - waypoints[first]["x"],
            waypoints[second]["y"] - waypoints[first]["y"],
        )
        for first, second in zip(route, route[1:])
    )


def test_ugv_delivery_order_is_the_exact_shortest_closed_route():
    waypoints = {
        "home": {"x": 0.0, "y": 0.0},
        "a": {"x": 8.0, "y": 1.0},
        "b": {"x": 2.0, "y": 7.0},
        "c": {"x": 4.0, "y": 2.0},
        "d": {"x": 9.0, "y": 8.0},
    }
    requested = ["a", "b", "c", "d"]
    optimized = optimize_order(waypoints, requested, "home")
    expected_cost = min(
        _cost(waypoints, order)
        for order in itertools.permutations(requested)
    )
    assert _cost(waypoints, optimized) == pytest.approx(expected_cost)


def test_ugv_optimizer_uses_supplied_road_costs():
    waypoints = {
        "home": {"x": 0.0, "y": 0.0},
        "a": {"x": 1.0, "y": 0.0},
        "b": {"x": 2.0, "y": 0.0},
        "c": {"x": 3.0, "y": 0.0},
    }
    costs = {
        ("home", "a"): 80.0,
        ("home", "b"): 20.0,
        ("home", "c"): 60.0,
        ("a", "b"): 20.0,
        ("a", "c"): 70.0,
        ("b", "a"): 20.0,
        ("b", "c"): 20.0,
        ("c", "a"): 20.0,
        ("c", "b"): 20.0,
        ("a", "home"): 20.0,
        ("b", "home"): 80.0,
        ("c", "home"): 60.0,
    }

    optimized = optimize_order(
        waypoints,
        ["a", "b", "c"],
        "home",
        route_distance=lambda first, second: costs[(first, second)],
    )

    assert optimized == ["b", "c", "a"]


def test_ugv_visits_duplicate_building_only_once():
    waypoints = {
        "home": {"x": 0.0, "y": 0.0},
        "a": {"x": 1.0, "y": 0.0},
        "b": {"x": 2.0, "y": 0.0},
    }
    assert optimize_order(waypoints, ["a", "b", "a"], "home") == [
        "a",
        "b",
    ]
