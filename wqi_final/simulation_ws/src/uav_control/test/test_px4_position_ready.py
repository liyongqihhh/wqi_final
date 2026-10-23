from types import SimpleNamespace

import pytest

from uav_control.px4_frames import px4_position_is_ready


def position(**changes):
    values = dict(xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
                  heading_good_for_control=True, x=1., y=2., z=-.4,
                  vx=0., vy=0., vz=0., heading=0.)
    values.update(changes)
    return SimpleNamespace(**values)


def test_valid_px4_estimate_is_ready():
    assert px4_position_is_ready(position())


@pytest.mark.parametrize("field", ["xy_valid", "z_valid", "v_xy_valid", "v_z_valid"])
def test_cached_map_alignment_does_not_hide_invalid_estimate(field):
    assert not px4_position_is_ready(position(**{field: False}))


@pytest.mark.parametrize("field", ["x", "y", "z", "vx", "vy", "vz", "heading"])
def test_nonfinite_estimates_are_not_ready(field):
    assert not px4_position_is_ready(position(**{field: float("nan")}))


def test_missing_px4_estimate_is_not_ready():
    assert not px4_position_is_ready(None)


def test_final_in_air_heading_alignment_does_not_block_ground_takeoff():
    assert px4_position_is_ready(position(heading_good_for_control=False))
