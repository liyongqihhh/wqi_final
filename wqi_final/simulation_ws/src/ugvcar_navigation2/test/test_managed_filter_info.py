import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.msg import CostmapFilterInfo
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile


@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM])
def test_metadata_survives_late_subscription_and_exits_cleanly(stop_signal, tmp_path):
    executable = (Path(get_package_prefix("ugvcar_navigation2")) /
                  "lib/ugvcar_navigation2/managed_filter_info_server")
    domain = 203
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    executor = SingleThreadedExecutor(context=context)
    node = rclpy.create_node("filter_info_probe", context=context)
    executor.add_node(node)
    logfile = (tmp_path / "server.log").open("w")
    process = subprocess.Popen(
        [str(executable), "--ros-args", "-r", "__node:=filter_info_probe_server",
         "-p", "filter_info_topic:=/probe/filter_info", "-p", "mask_topic:=/probe/mask"],
        env=dict(os.environ, ROS_DOMAIN_ID=str(domain)), stdout=logfile,
        stderr=subprocess.STDOUT)
    try:
        client = node.create_client(ChangeState, "/filter_info_probe_server/change_state")
        assert client.wait_for_service(timeout_sec=15)
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=10)
            assert future.done() and future.result().success
        messages = []
        subscription = node.create_subscription(
            CostmapFilterInfo, "/probe/filter_info", messages.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic() + 10
        while not messages and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        assert messages and messages[0].type == 0
        assert messages[0].filter_mask_topic == "/probe/mask"
        assert messages[0].base == 0 and messages[0].multiplier == 1
        process.send_signal(stop_signal)
        assert process.wait(timeout=10) == 0
        node.destroy_subscription(subscription)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        node.destroy_node()
        executor.shutdown()
        context.shutdown()
        logfile.close()
