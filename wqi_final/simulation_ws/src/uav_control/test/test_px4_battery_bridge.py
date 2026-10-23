import math

import pytest

from uav_control.px4_battery_bridge import battery_mavlink_fields, battery_sample_fresh


def test_slow_simulation_does_not_make_battery_telemetry_stale():
    assert battery_sample_fresh(0, 0.0, 500_000_000, 20.0, 2.0)


@pytest.mark.parametrize("received_sim,received_wall,sim_now,wall_now", [
    (0, 0.0, 2_100_000_000, 1.0),
    (0, 0.0, 500_000_000, 31.0),
    (1_000_000_000, 0.0, 0, 1.0),
    (None, None, 0, 0.0),
])
def test_stalled_missing_or_old_battery_samples_remain_rejected(
    received_sim, received_wall, sim_now, wall_now
):
    assert not battery_sample_fresh(received_sim, received_wall, sim_now, wall_now, 2.0)


def test_battery_status_encodes_pack_voltage_soc_and_current():
    fields = battery_mavlink_fields(22.2, -10.0, 0.8, 5.0, 4.0)
    assert fields["voltages"] == [22200] + [65535] * 9
    assert fields["battery_remaining"] == 80
    assert fields["current_battery"] == 1000
    assert fields["current_consumed"] == 1000


def test_charging_is_signed_and_does_not_reset_soc_on_disarm():
    assert battery_mavlink_fields(24.0, 5.0, 0.6, 5.0, 3.0)["current_battery"] == -500
    assert battery_mavlink_fields(24.0, 0.0, 0.6, 5.0, 3.0)["battery_remaining"] == 60


@pytest.mark.parametrize("values", [
    (22, math.nan, .8, 5, 4), (22, 1, -1, 5, 4),
    (22, 1, .8, 5, 6), (0, 1, .8, 5, 4), (22, 400, .8, 5, 4),
])
def test_invalid_data_is_not_sent_to_flight_controller(values):
    with pytest.raises(ValueError):
        battery_mavlink_fields(*values)
