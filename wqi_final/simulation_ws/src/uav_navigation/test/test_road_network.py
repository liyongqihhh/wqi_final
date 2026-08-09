import math
from pathlib import Path

import pytest

from uav_navigation.road_network import RoadNetwork, RoadNetworkError


LAYOUT = (
    Path(__file__).parents[2]
    / "ugvcar_description"
    / "config"
    / "campus_layout.yaml"
)


def test_campus_delivery_points_are_connected_by_roads():
    network = RoadNetwork.from_yaml(LAYOUT)
    logistics = (0.0, -43.5)
    cafeteria = (38.0, -20.0)
    library = (62.0, 40.5)

    south_leg = network.distance(logistics, cafeteria)
    north_leg = network.distance(cafeteria, library)

    assert south_leg > math.dist(logistics, cafeteria)
    assert north_leg > 0.0
    assert network.distance(library, cafeteria) == pytest.approx(north_leg)


def test_same_delivery_stop_has_zero_cost():
    network = RoadNetwork.from_yaml(LAYOUT)
    dormitory_stop = (30.0, 11.0)

    assert network.distance(dormitory_stop, dormitory_stop) == 0.0


def test_query_away_from_the_road_is_rejected():
    network = RoadNetwork.from_yaml(LAYOUT, projection_tolerance=1.0)

    with pytest.raises(RoadNetworkError, match="nearest road"):
        network.distance((90.0, 70.0), (0.0, -43.5))
