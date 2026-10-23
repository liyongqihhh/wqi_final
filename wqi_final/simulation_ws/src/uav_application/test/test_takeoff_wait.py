from types import SimpleNamespace
from unittest.mock import Mock

from uav_application.delivery_mission_manager import DeliveryMissionManager


def test_slow_simulation_takeoff_uses_sim_time_not_wall_seconds(monkeypatch):
    monkeypatch.setattr("uav_application.delivery_mission_manager.rclpy.ok", lambda: True)
    monkeypatch.setattr("uav_application.delivery_mission_manager.time.sleep", lambda _: None)
    wall = iter((0, 20, 21))
    monkeypatch.setattr("uav_application.delivery_mission_manager.time.monotonic", lambda: next(wall))
    clock = Mock()
    clock.now.side_effect = [SimpleNamespace(nanoseconds=0), SimpleNamespace(nanoseconds=1_000_000_000)]
    manager = SimpleNamespace(get_clock=lambda: clock,
        _snapshot=Mock(side_effect=[(None, 2, None), (None, 1, None)]))
    assert DeliveryMissionManager._wait_vehicle_state(manager, 1, 15)


def test_cancellation_interrupts_takeoff_wait(monkeypatch):
    monkeypatch.setattr("uav_application.delivery_mission_manager.rclpy.ok", lambda: True)
    manager = SimpleNamespace(get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=0)))
    assert not DeliveryMissionManager._wait_vehicle_state(
        manager, 1, 30, SimpleNamespace(is_cancel_requested=True))


def test_px4_requires_landed_state_even_when_settled_on_deck(monkeypatch):
    monkeypatch.setattr("uav_application.delivery_mission_manager.rclpy.ok", lambda: True)
    monkeypatch.setattr("uav_application.delivery_mission_manager.time.sleep", lambda _: None)
    wall = iter((0, 0, 0, 1.1, 1.2, 1.3))
    monkeypatch.setattr("uav_application.delivery_mission_manager.time.monotonic", lambda: next(wall))
    odom = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(z=.42))),
                           twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0,y=0,z=0))))
    manager = SimpleNamespace(require_landed_state=True,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
        _snapshot=Mock(side_effect=[(odom,3,None),(odom,3,None),(odom,0,None)]))
    assert DeliveryMissionManager._wait_landed(manager,.42)
    assert manager._snapshot.call_count == 3


def test_px4_landing_on_ground_is_not_accepted_as_carrier_landing(monkeypatch):
    monkeypatch.setattr("uav_application.delivery_mission_manager.rclpy.ok", lambda: True)
    monkeypatch.setattr("uav_application.delivery_mission_manager.time.sleep", lambda _: None)
    monkeypatch.setattr("uav_application.delivery_mission_manager.time.monotonic", lambda: 0)

    def odom(z):
        return SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(z=z))),
                               twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0,y=0,z=0))))

    manager = SimpleNamespace(require_landed_state=True,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
        _snapshot=Mock(side_effect=[(odom(.055),0,None),(odom(.42),0,None)]))
    assert DeliveryMissionManager._wait_landed(manager, .42)
    assert manager._snapshot.call_count == 2


def test_emergency_landing_does_not_complete_a_normal_delivery(monkeypatch):
    monkeypatch.setattr("uav_application.delivery_mission_manager.rclpy.ok", lambda: True)
    manager = SimpleNamespace(require_landed_state=True, px4_landing_fault="SIM_VELOCITY_FUSION_LOST",
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
        get_logger=Mock(return_value=Mock()), _snapshot=Mock())
    assert not DeliveryMissionManager._wait_landed(manager, .42, require_normal_landing=True)
    manager._snapshot.assert_not_called()
    manager.get_logger().error.assert_called_once()


def test_safe_recovery_can_confirm_landing_after_a_fault(monkeypatch):
    monkeypatch.setattr("uav_application.delivery_mission_manager.rclpy.ok", lambda: True)
    odom = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(z=.42))),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0, y=0, z=0))))
    manager = SimpleNamespace(require_landed_state=True, px4_landing_fault="SIM_VELOCITY_FUSION_LOST",
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=0)),
        _snapshot=Mock(return_value=(odom, 0, None)))
    assert DeliveryMissionManager._wait_landed(manager, .42)


def test_landing_timeout_reports_measured_state_not_command_acceptance(monkeypatch):
    monkeypatch.setattr("uav_application.delivery_mission_manager.rclpy.ok", lambda: True)
    monkeypatch.setattr("uav_application.delivery_mission_manager.time.sleep", lambda _: None)
    monkeypatch.setattr("uav_application.delivery_mission_manager.time.monotonic", lambda: 0)
    clock = Mock()
    clock.now.side_effect = [SimpleNamespace(nanoseconds=n) for n in (0, 0, 31_000_000_000)]
    odom = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(z=.421))),
        twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=.011, y=0, z=0))))
    manager = SimpleNamespace(require_landed_state=True, get_clock=lambda: clock,
        get_logger=Mock(return_value=Mock()), _snapshot=Mock(return_value=(odom, 3, None)))
    assert not DeliveryMissionManager._wait_landed(manager, .42)
    assert "state=3" in manager.last_landing_failure
    assert "altitude=0.421" in manager.last_landing_failure
    assert "expected_altitude=0.420" in manager.last_landing_failure
    assert "speed=0.011" in manager.last_landing_failure
