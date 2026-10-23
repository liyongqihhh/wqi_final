import importlib.util
import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

backend = None
if importlib.util.find_spec("px4_msgs") is not None:
    from uav_control import px4_offboard_controller as backend
pytestmark = pytest.mark.skipif(backend is None, reason="Optional px4_msgs is not installed")


def make_controller():
    position = SimpleNamespace(x=30., y=11., z=.421)
    odom = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=position, orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0., y=0., z=0.))))
    stamp = SimpleNamespace(nanoseconds=10_000_000_000)
    controller = SimpleNamespace(enable_sim_velocity_aiding=True, precision_landing=True,
        _last_published_state=backend.LANDING, native_deck_handoff=False,
        target_landing_height=.42, map_target=(30., 11., .8),
        velocity_aiding_sample_ns=stamp.nanoseconds, deck_contact_started_ns=None,
        native_land_request_ns=None, vehicle_status=SimpleNamespace(nav_state=14),
        _snapshot=lambda: (odom, backend.LANDING),
        get_clock=lambda: SimpleNamespace(now=lambda: stamp),
        get_logger=Mock(return_value=Mock()), _send_vehicle_command=Mock())
    return controller, stamp, odom


def test_contact_handoff_waits_for_stability_and_retries_until_native_acknowledges():
    controller, stamp, _ = make_controller()
    backend.Px4OffboardController._try_native_deck_handoff(controller)
    controller._send_vehicle_command.assert_not_called()
    stamp.nanoseconds += 400_000_000
    controller.velocity_aiding_sample_ns = stamp.nanoseconds
    backend.Px4OffboardController._try_native_deck_handoff(controller)
    controller._send_vehicle_command.assert_not_called()
    stamp.nanoseconds += 200_000_000
    controller.velocity_aiding_sample_ns = stamp.nanoseconds
    backend.Px4OffboardController._try_native_deck_handoff(controller)
    assert controller.native_deck_handoff
    assert controller.precision_landing  # Keep Offboard until NAV_LAND is acknowledged.
    controller._send_vehicle_command.assert_called_once_with(backend.VehicleCommand.VEHICLE_CMD_NAV_LAND)
    backend.Px4OffboardController._try_native_deck_handoff(controller)
    assert controller._send_vehicle_command.call_count == 1
    stamp.nanoseconds += 600_000_000
    controller.velocity_aiding_sample_ns = stamp.nanoseconds
    backend.Px4OffboardController._try_native_deck_handoff(controller)
    assert controller._send_vehicle_command.call_count == 2
    controller.vehicle_status.nav_state = backend.VehicleStatus.NAVIGATION_STATE_AUTO_LAND
    backend.Px4OffboardController._try_native_deck_handoff(controller)
    assert not controller.precision_landing
    assert controller._last_published_state == backend.LANDING  # Never synthesize LANDED.


@pytest.mark.parametrize("invalid", ["old_sample", "ground_not_deck", "independent_backend"])
def test_invalid_contact_cannot_request_native_deck_handoff(invalid):
    controller, stamp, odom = make_controller()
    controller.deck_contact_started_ns = stamp.nanoseconds - 1_000_000_000
    if invalid == "old_sample":
        controller.velocity_aiding_sample_ns -= 1_000_000_000
    elif invalid == "ground_not_deck":
        odom.pose.pose.position.z = .055
    else:
        controller.enable_sim_velocity_aiding = False
    backend.Px4OffboardController._try_native_deck_handoff(controller)
    controller._send_vehicle_command.assert_not_called()
    assert not controller.native_deck_handoff


def test_external_observation_converts_height_and_velocity_without_attitude():
    controller = object.__new__(backend.Px4OffboardController)
    controller.lock = threading.Lock()
    controller.enable_sim_velocity_aiding = True
    controller.velocity_aiding_sample_ns = None
    controller.velocity_aiding_pub = Mock()
    controller.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    controller._timestamp = lambda: 10_000_000
    message = Odometry()
    message.header.frame_id = 'uav_odom'
    message.header.stamp.sec = 10
    message.pose.pose.position.x = 30.
    message.pose.pose.position.y = 11.
    message.pose.pose.position.z = .421
    message.twist.twist.linear.x = 1.
    message.twist.twist.linear.y = 2.
    message.twist.twist.linear.z = 3.
    controller._odom_callback(message)
    observation = controller.velocity_aiding_pub.publish.call_args.args[0]
    assert list(observation.position) == pytest.approx([11., 30., -.421])
    assert list(observation.velocity) == pytest.approx([2., 1., -3.])
    assert all(math.isnan(value) for value in observation.q)
    assert list(observation.position_variance) == pytest.approx([.0004]*3)
    controller._odom_callback(message)  # Duplicate samples must not enter the EKF.
    assert controller.velocity_aiding_pub.publish.call_count == 1


def test_estimator_readiness_requires_height_and_velocity_fusion(monkeypatch):
    monkeypatch.setattr(backend.time, 'monotonic', lambda: 0.)
    controller = object.__new__(backend.Px4OffboardController)
    controller.lock = threading.Lock()
    controller.enable_sim_velocity_aiding = True
    controller.velocity_aiding_sample_ns = 10_000_000_000
    controller.velocity_aiding_received_at = 0.
    controller.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    callback = backend.Px4OffboardController._estimator_status_callback
    callback(controller, SimpleNamespace(cs_ev_vel=True, cs_ev_hgt=False))
    assert controller.velocity_fusion_active and not controller.height_fusion_active
    assert not backend.Px4OffboardController._velocity_aiding_ready(controller)
    callback(controller, SimpleNamespace(cs_ev_vel=True, cs_ev_hgt=True))
    assert controller.velocity_fusion_active and controller.height_fusion_active
    assert backend.Px4OffboardController._velocity_aiding_ready(controller)


def aiding_controller(monkeypatch):
    controller = object.__new__(backend.Px4OffboardController)
    controller.lock = threading.Lock()
    controller.enable_sim_velocity_aiding = True
    controller.velocity_aiding_sample_ns = 10_000_000_000
    controller.velocity_aiding_received_at = 0.
    controller.velocity_fusion_sim_ns = 10_000_000_000
    controller.velocity_fusion_at = 0.
    controller.velocity_fusion_active = controller.height_fusion_active = True
    controller.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    monkeypatch.setattr(backend.time, "monotonic", lambda: 0.)
    return controller


def test_receipt_after_clock_read_cannot_invalidate_the_previous_fresh_snapshot(monkeypatch):
    controller = aiding_controller(monkeypatch)
    def interleaved_clock():
        controller.velocity_aiding_received_at = .000001
        return 0.
    monkeypatch.setattr(backend.time, "monotonic", interleaved_clock)
    # The old implementation sampled the clock first, then read this newer
    # receipt and falsely interpreted a -1 us age as a lost EKF observation.
    assert controller._velocity_aiding_ready()


@pytest.mark.parametrize("invalid",["stale_sample","stale_flags","stale_wall", "no_velocity","no_height"])
def test_atomic_snapshot_keeps_original_loss_protections(monkeypatch,invalid):
    controller = aiding_controller(monkeypatch)
    if invalid == "stale_sample":controller.velocity_aiding_sample_ns -= 600_000_000
    if invalid == "stale_flags":controller.velocity_fusion_sim_ns -= 2_100_000_000
    if invalid == "stale_wall":controller.velocity_aiding_received_at = -31.
    if invalid == "no_velocity":controller.velocity_fusion_active = False
    if invalid == "no_height":controller.height_fusion_active = False
    assert not controller._velocity_aiding_ready()


def test_health_reads_its_clock_while_holding_the_shared_state_lock(monkeypatch):
    controller = aiding_controller(monkeypatch)
    def checked_clock():
        assert controller.lock.locked()
        return SimpleNamespace(nanoseconds=10_000_000_000)
    controller.get_clock = lambda: SimpleNamespace(now=checked_clock)
    health=controller._velocity_aiding_health()
    assert health["ready"] and health["sample_sim_age_s"] == 0.
