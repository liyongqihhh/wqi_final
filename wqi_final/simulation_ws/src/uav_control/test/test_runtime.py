from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from uav_control import runtime


def test_callbacks_finish_before_node_and_context_are_destroyed(monkeypatch):
    order=[]
    node=SimpleNamespace(timers=[SimpleNamespace(cancel=lambda:order.append("timer_cancel"))],
                         destroy_node=lambda:order.append("node_destroy"))
    executor=Mock()
    executor.spin_once.side_effect=KeyboardInterrupt
    executor.shutdown.side_effect=lambda **_:order.append("executor_join") or True
    monkeypatch.setattr(runtime,"MultiThreadedExecutor",lambda **_:executor)
    monkeypatch.setattr(runtime.rclpy,"init",lambda **_:order.append("init"))
    monkeypatch.setattr(runtime.rclpy,"ok",lambda:True)
    monkeypatch.setattr(runtime.rclpy,"shutdown",lambda:order.append("context_shutdown"))
    monkeypatch.setattr(runtime.signal,"signal",lambda *_:None)
    runtime.run_node(lambda:node)
    assert node.shutdown_requested
    assert order==["init","timer_cancel","executor_join","node_destroy","context_shutdown"]


def test_failed_constructor_still_closes_context(monkeypatch):
    closed=Mock()
    monkeypatch.setattr(runtime.rclpy,"init",lambda **_:None)
    monkeypatch.setattr(runtime.rclpy,"ok",lambda:True)
    monkeypatch.setattr(runtime.rclpy,"shutdown",closed)
    monkeypatch.setattr(runtime.signal,"signal",lambda *_:None)
    with pytest.raises(ValueError,match="config"):
        runtime.run_node(Mock(side_effect=ValueError("config")))
    closed.assert_called_once()


def test_flight_path_monitor_uses_the_existing_callback_safe_host(monkeypatch):
    from uav_control import flight_state_monitor
    host=Mock()
    monkeypatch.setattr(runtime,"run_node",host)
    flight_state_monitor.main(args=["--ros-args"])
    host.assert_called_once_with(flight_state_monitor.FlightStateMonitor,args=["--ros-args"])
