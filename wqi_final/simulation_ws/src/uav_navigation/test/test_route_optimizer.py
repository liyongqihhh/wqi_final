import itertools
import math

import pytest

from uav_navigation.route_optimizer import (
    MAX_EXACT_STOPS,
    optimize_visit_order,
)


def _route_cost(order, start, points, return_home):
    current = start
    total = 0.0
    for index in order:
        destination = points[index]
        total += math.dist(current, destination)
        current = destination
    if return_home:
        total += math.dist(current, start)
    return total


def test_optimizer_matches_exhaustive_closed_route():
    start = (0.0, 0.0)
    points = ((8.0, 1.0), (2.0, 7.0), (4.0, 2.0), (9.0, 8.0))
    plan = optimize_visit_order(
        len(points),
        lambda index: math.dist(start, points[index]),
        lambda origin, destination: math.dist(
            points[origin], points[destination]
        ),
        lambda index: math.dist(points[index], start),
    )
    expected = min(
        (_route_cost(order, start, points, True), order)
        for order in itertools.permutations(range(len(points)))
    )
    assert plan.total_cost == pytest.approx(expected[0])
    assert plan.order == expected[1]


def test_open_route_can_choose_a_different_final_stop():
    start = (0.0, 0.0)
    points = ((1.0, 0.0), (2.0, 0.0), (10.0, 0.0))
    plan = optimize_visit_order(
        len(points),
        lambda index: math.dist(start, points[index]),
        lambda origin, destination: math.dist(
            points[origin], points[destination]
        ),
    )
    assert plan.order == (0, 1, 2)
    assert plan.total_cost == pytest.approx(10.0)


def test_equal_cost_stops_preserve_input_order():
    plan = optimize_visit_order(
        3,
        lambda _index: 1.0,
        lambda _origin, _destination: 0.0,
        lambda _index: 1.0,
    )
    assert plan.order == (0, 1, 2)


def test_exact_optimizer_rejects_more_than_supported_item_count():
    with pytest.raises(ValueError, match=str(MAX_EXACT_STOPS)):
        optimize_visit_order(
            MAX_EXACT_STOPS + 1,
            lambda _index: 1.0,
            lambda _origin, _destination: 1.0,
        )
