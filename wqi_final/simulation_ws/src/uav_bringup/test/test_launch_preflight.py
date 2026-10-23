import importlib.util
from pathlib import Path
import socket
from types import SimpleNamespace

import pytest


def _module():
    path = Path(__file__).parents[1] / "launch" / "uav_sim.launch.py"
    spec = importlib.util.spec_from_file_location("uav_sim_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_existing_gazebo_master_is_rejected_without_starting_children():
    with socket.socket() as server:
        server.bind(("127.0.0.1", 0))
        server.listen()
        uri = f"http://127.0.0.1:{server.getsockname()[1]}"
        with pytest.raises(RuntimeError, match="already running"):
            _module().check_gazebo_master(SimpleNamespace(
                environment={"GAZEBO_MASTER_URI": uri}
            ))


def test_unused_gazebo_master_allows_launch():
    with socket.socket() as server:
        server.bind(("127.0.0.1", 0))
        uri = f"http://127.0.0.1:{server.getsockname()[1]}"
        assert _module().check_gazebo_master(SimpleNamespace(
            environment={"GAZEBO_MASTER_URI": uri}
        )) == []
