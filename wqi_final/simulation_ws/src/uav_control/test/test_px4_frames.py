import math

import pytest

from uav_control.px4_frames import (
    local_ned_to_map_enu,
    map_enu_to_local_ned,
    reference_from_positions,
    yaw_enu_to_ned,
    landing_setpoint_ned,
    landing_is_aligned,
    landing_alignment_gate,
    map_reference_velocity_bias,
    landing_deadline_expired,
    map_velocity_to_ned,
    velocity_sample_is_fresh,
    deck_contact_is_stable,
)


def test_map_ned_round_trip_with_nonzero_reference():
    reference = (12.0, -43.5, 0.42)
    map_point = (28.0, 15.0, 12.0)
    ned = map_enu_to_local_ned(map_point, reference)
    assert local_ned_to_map_enu(ned, reference) == pytest.approx(map_point)


def test_reference_aligns_current_positions():
    map_point = (4.0, -8.0, 0.6)
    local_point = (2.0, -3.0, -0.1)
    reference = reference_from_positions(map_point, local_point)
    assert local_ned_to_map_enu(local_point, reference) == pytest.approx(map_point)


def test_yaw_axes_follow_ros_enu_and_px4_ned_conventions():
    assert yaw_enu_to_ned(0.0) == pytest.approx(math.pi / 2.0)
    assert yaw_enu_to_ned(math.pi / 2.0) == pytest.approx(0.0)


def test_precision_landing_keeps_xy_and_commands_downward_velocity():
    position, velocity = landing_setpoint_ned((2.0, -3.0, -1.0), .3)
    assert position[:2] == [2.0, -3.0]
    assert math.isnan(position[2])
    assert all(math.isnan(v) for v in velocity[:2])
    assert velocity[2] == .3


@pytest.mark.parametrize("speed", [math.inf, -0.1, math.nan])
def test_invalid_descent_commands_are_rejected(speed):
    with pytest.raises(ValueError):
        landing_setpoint_ned((2.0, -3.0, -1.0), speed)


def test_zero_descent_holds_altitude_during_xy_alignment():
    _, velocity = landing_setpoint_ned((2.0, -3.0, -1.0), 0.0)
    assert velocity[2] == 0.0


def test_landing_requires_close_position_and_low_horizontal_speed():
    assert landing_is_aligned((.02, .03), (0, 0), (.03, .04))
    assert not landing_is_aligned((.11, .03), (0, 0), (0, 0))
    assert not landing_is_aligned((.02, .03), (0, 0), (.2, 0))
    assert not landing_is_aligned((math.nan, 0), (0, 0), (0, 0))


def test_moving_frame_velocity_correction_preserves_real_descent():
    bias = map_reference_velocity_bias((.1, -.2, .05), (-.4, .3, .12))
    assert bias == pytest.approx((-.2, .2, .17))
    _, velocity = landing_setpoint_ned((0, 0, -.8), .3, bias)
    assert velocity == pytest.approx((-.2, .2, .47))
    # Subtracting the estimator offset yields the requested map-frame velocity.
    assert [velocity[i] - bias[i] for i in range(3)] == pytest.approx((0, 0, .3))


def test_alignment_holds_real_altitude_and_corrects_velocity_bias():
    position, velocity = landing_setpoint_ned((2, 3, -.8), 0, (.1, -.1, .12), True)
    assert position == [2, 3, -.8]
    assert velocity == [.1, -.1, .12]


def test_alignment_hysteresis_avoids_contact_chatter_but_blocks_large_errors():
    assert not landing_alignment_gate((.12, 0), (0, 0), (0, 0), False)
    assert landing_alignment_gate((.12, 0), (0, 0), (.2, 0), True)
    assert not landing_alignment_gate((.25, 0), (0, 0), (0, 0), True)
    assert not landing_alignment_gate((0, 0), (0, 0), (.5, 0), True)


def test_invalid_velocity_bias_is_not_published_to_px4():
    with pytest.raises(ValueError):
        map_reference_velocity_bias((0, math.nan, 0), (0, 0, 0))
    with pytest.raises(ValueError):
        landing_setpoint_ned((0, 0, 0), .3, (0, math.inf, 0))


def test_landing_watchdog_uses_simulation_time_and_handles_unstarted_sequence():
    assert not landing_deadline_expired(None, 200_000_000_000, 60)
    assert not landing_deadline_expired(100_000_000_000, 159_000_000_000, 60)
    assert landing_deadline_expired(100_000_000_000, 160_000_000_000, 60)
    assert not landing_deadline_expired(100_000_000_000, 90_000_000_000, 60)


@pytest.mark.parametrize("bias", [-1.18, -.28, -.04, 0., .5])
def test_landing_bias_cannot_request_ascent_or_break_native_descent_detection(bias):
    _, velocity = landing_setpoint_ned((0, 0, -.8), .3, (0, 0, bias))
    assert velocity[2] >= .3


def test_velocity_only_aiding_converts_world_enu_without_position_offset():
    assert map_velocity_to_ned((.7, -.2, .3)) == pytest.approx((-.2, .7, -.3))
    with pytest.raises(ValueError):
        map_velocity_to_ned((0, math.nan, 0))


def test_velocity_aiding_rejects_old_duplicate_future_and_zero_samples():
    assert velocity_sample_is_fresh(10_000_000_000, 10_100_000_000, 9_900_000_000)
    assert not velocity_sample_is_fresh(9_000_000_000, 10_100_000_000)
    assert not velocity_sample_is_fresh(10_000_000_000, 10_100_000_000, 10_000_000_000)
    assert not velocity_sample_is_fresh(11_000_000_000, 10_100_000_000)
    assert not velocity_sample_is_fresh(0, 0)


def test_native_deck_handoff_requires_stable_height_alignment_speed_and_attitude():
    assert deck_contact_is_stable((30, 11, .421), (30, 11), (0, 0, 0), (0, 0, 0, 1), .42)
    assert not deck_contact_is_stable((30, 11, .055), (30, 11), (0, 0, 0), (0, 0, 0, 1), .42)
    assert not deck_contact_is_stable((30, 11, .8), (30, 11), (0, 0, 0), (0, 0, 0, 1), .42)
    assert not deck_contact_is_stable((30.2, 11, .421), (30, 11), (0, 0, 0), (0, 0, 0, 1), .42)
    assert not deck_contact_is_stable((30, 11, .421), (30, 11), (.1, 0, 0), (0, 0, 0, 1), .42)
    assert not deck_contact_is_stable((30, 11, .421), (30, 11), (0, 0, -.2), (0, 0, 0, 1), .42)
    assert not deck_contact_is_stable((30, 11, .421), (30, 11), (0, 0, 0), (.5, 0, 0, .866), .42)
    assert not deck_contact_is_stable((30, 11, .421), (30, 11), (0, 0, 0), (0, 0, 0, 0), .42)
    assert not deck_contact_is_stable((30, 11, math.nan), (30, 11), (0, 0, 0), (0, 0, 0, 1), .42)
