import importlib.util
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

backend=None
if importlib.util.find_spec('px4_msgs') is not None:
    from uav_control import px4_offboard_controller as backend
pytestmark=pytest.mark.skipif(backend is None,reason='Optional px4_msgs is not installed')


def controller(height,energy_ready=True):
    point=SimpleNamespace(x=0.,y=0.,z=15.)
    return SimpleNamespace(_snapshot=lambda:(SimpleNamespace(pose=SimpleNamespace(
        pose=SimpleNamespace(position=point))),backend.FLYING),
        land_detected=False,target_landing_height=height,map_reference=(0.,0.,0.),
        _energy_bridge_ready=lambda:energy_ready,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1)),
        _publish_state=Mock(),_set_landing_fault=Mock(),_send_vehicle_command=Mock(),
        _publish_offboard_setpoint=Mock(),completed_map_target=None,get_logger=Mock())


def test_no_deck_target_requests_native_landing_without_waiting_on_ground():
    node=controller(None)
    response=backend.Px4OffboardController._land_callback(node,None,SimpleNamespace())
    assert response.success and not node.precision_landing
    node._send_vehicle_command.assert_called_once_with(backend.VehicleCommand.VEHICLE_CMD_NAV_LAND)
    node._set_landing_fault.assert_called_once_with('')
    node._publish_state.assert_called_once_with(backend.LANDING)


def test_explicit_deck_height_preserves_existing_precision_control():
    node=controller(.42)
    backend.Px4OffboardController._land_callback(node,None,SimpleNamespace())
    assert node.precision_landing
    assert node._send_vehicle_command.call_args.args[0]!=backend.VehicleCommand.VEHICLE_CMD_NAV_LAND


def test_unavailable_deck_control_still_requests_native_landing_and_reports_fault():
    node=controller(.42,energy_ready=False)
    backend.Px4OffboardController._land_callback(node,None,SimpleNamespace())
    assert not node.precision_landing
    node._set_landing_fault.assert_called_once_with('PRECISION_LANDING_UNAVAILABLE')
    node._send_vehicle_command.assert_called_once_with(backend.VehicleCommand.VEHICLE_CMD_NAV_LAND)
