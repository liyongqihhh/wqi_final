import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext


@pytest.mark.parametrize("moving", [False, True])
@pytest.mark.parametrize("energy", [False, True])
def test_native_checks_and_calibration_are_scoped_to_moving_sitl(moving, energy, monkeypatch):
    path = Path(__file__).parents[1] / "launch" / "px4_sitl.launch.py"
    spec = importlib.util.spec_from_file_location("tested_px4_sitl", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "ExecuteProcess", lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update(
        px4_autopilot_dir="/tmp/px4", moving_base=str(moving).lower(),
        enable_energy_constraints=str(energy).lower())
    process = module._start_px4(context)[0]
    env = process["additional_env"]
    mask = int(env["PX4_PARAM_EKF2_GPS_CHECK"])
    assert mask == (1023 & ~(32 | 128) if moving else 1023)
    assert mask & 31 == 31  # Satellites, PDOP and position/speed accuracy remain checked.
    assert mask & (64 | 256 | 512) == 64 | 256 | 512
    assert env["PX4_PARAM_IMU_GYRO_CAL_EN"] == ("0" if moving else "1")
    assert env["PX4_PARAM_EKF2_EV_CTRL"] == ("6" if moving else "0")
    assert env["PX4_PARAM_EKF2_EVV_NOISE"] == ("0.01" if moving else "0.1")
    assert env["PX4_PARAM_EKF2_EVP_NOISE"] == ("0.02" if moving else "0.1")
    assert env["PX4_PARAM_EKF2_HGT_REF"] == ("3" if moving else "1")
    assert env["PX4_PARAM_EKF2_GPS_CTRL"] == ("5" if moving else "7")
    assert env["PX4_PARAM_MPC_USE_HTE"] == ("0" if moving else "1")
    assert env["PX4_PARAM_MPC_THR_HOVER"] == ("0.71" if moving else "0.5")
    assert env["PX4_PARAM_SIM_BAT_ENABLE"].perform(context) == ("0" if energy else "1")
    for gyro in range(3):
        for axis in "XYZ":
            key = f"PX4_PARAM_CAL_GYRO{gyro}_{axis}OFF"
            assert (key in env) == moving
            if moving:
                assert env[key] == "0"
