import math

import pytest

from uav_control.position_setpoint import (
    adaptive_position_setpoint,
    limited_position_setpoint,
)


def test_limited_setpoint_preserves_direction_in_three_dimensions():
    setpoint = limited_position_setpoint(
        (0.0, 0.0, 0.0),
        (3.0, 4.0, 12.0),
        2.0,
    )

    assert math.dist((0.0, 0.0, 0.0), setpoint) == pytest.approx(2.0)
    assert setpoint == pytest.approx((6.0 / 13.0, 8.0 / 13.0, 24.0 / 13.0))


def test_limited_setpoint_returns_final_target_inside_step():
    target = (1.0, -2.0, 3.0)
    assert limited_position_setpoint((0.8, -2.0, 3.0), target, 0.6) == target


def test_adaptive_setpoint_uses_shorter_step_near_target():
    far_setpoint, far_approaching = adaptive_position_setpoint(
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        cruise_step=2.0,
        approach_step=0.6,
        slowdown_distance=4.0,
    )
    near_setpoint, near_approaching = adaptive_position_setpoint(
        (7.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        cruise_step=2.0,
        approach_step=0.6,
        slowdown_distance=4.0,
    )

    assert far_setpoint == pytest.approx((2.0, 0.0, 0.0))
    assert far_approaching is False
    assert near_setpoint == pytest.approx((7.6, 0.0, 0.0))
    assert near_approaching is True


@pytest.mark.parametrize("invalid_step", [0.0, -0.1, float("inf")])
def test_limited_setpoint_rejects_invalid_step(invalid_step):
    with pytest.raises(ValueError):
        limited_position_setpoint((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), invalid_step)


def test_adaptive_setpoint_rejects_approach_step_above_cruise_step():
    with pytest.raises(ValueError):
        adaptive_position_setpoint(
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            cruise_step=0.5,
            approach_step=0.6,
            slowdown_distance=4.0,
        )
