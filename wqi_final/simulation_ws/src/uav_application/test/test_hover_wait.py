from types import SimpleNamespace
from unittest.mock import Mock

from uav_application.delivery_mission_manager import DeliveryMissionManager


def manager_with_time(monkeypatch, real_time_factor):
    state = {"wall": 0.0}
    monkeypatch.setattr("uav_application.delivery_mission_manager.rclpy.ok", lambda: True)
    monkeypatch.setattr("uav_application.delivery_mission_manager.time.monotonic", lambda: state["wall"])

    def sleep(duration):
        state["wall"] += duration

    monkeypatch.setattr("uav_application.delivery_mission_manager.time.sleep", sleep)
    clock = SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=int(state["wall"] * real_time_factor * 1e9)))
    manager = SimpleNamespace(get_clock=lambda: clock, _publish_feedback=Mock(),
                              get_logger=Mock(return_value=Mock()))
    return manager, state


def test_hover_finishes_five_sim_seconds_below_old_rtf_limit(monkeypatch):
    manager, state = manager_with_time(monkeypatch, 0.10)
    goal = SimpleNamespace(is_cancel_requested=False)
    assert DeliveryMissionManager._wait_sim_duration(manager, 5.0, goal)
    assert state["wall"] >= 50.0
    assert state["wall"] < 51.0


def test_clock_stall_does_not_wait_forever(monkeypatch):
    manager, state = manager_with_time(monkeypatch, 0.0)
    assert not DeliveryMissionManager._wait_sim_duration(manager, 5.0,
                                                        SimpleNamespace(is_cancel_requested=False))
    assert 30.0 <= state["wall"] < 31.0
    manager.get_logger().error.assert_called_once()


def test_hover_cancellation_is_immediate(monkeypatch):
    manager, state = manager_with_time(monkeypatch, 0.10)
    assert not DeliveryMissionManager._wait_sim_duration(manager, 5.0,
                                                        SimpleNamespace(is_cancel_requested=True))
    assert state["wall"] == 0.0


def test_clock_reset_aborts_hover(monkeypatch):
    manager, _ = manager_with_time(monkeypatch, 0.10)
    clock = Mock()
    clock.now.side_effect = [SimpleNamespace(nanoseconds=2_000_000_000),
                             SimpleNamespace(nanoseconds=1_000_000_000)]
    manager.get_clock = lambda: clock
    assert not DeliveryMissionManager._wait_sim_duration(manager, 5.0,
                                                        SimpleNamespace(is_cancel_requested=False))
    manager.get_logger().error.assert_called_once()
