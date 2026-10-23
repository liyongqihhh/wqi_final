"""Native regression for lifecycle weak references into behavior plugin code."""
import os
from pathlib import Path
import signal
import subprocess
import pytest

from ament_index_python.packages import get_package_prefix
from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from lifecycle_msgs.srv import ChangeState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


@pytest.mark.parametrize("package,plugin,name,parameters,transitions", [
    ("nav2_behaviors", "behavior_server::BehaviorServer", "behavior_server", [], (1,3,4,2,5)),
    ("nav2_planner", "nav2_planner::PlannerServer", "planner_server", [
        Parameter(name="planner_plugins", value=ParameterValue(
            type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=["GridBased"])),
        Parameter(name="GridBased.plugin", value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value="nav2_smac_planner/SmacPlanner2D"))], (1,2,5)),
    ("nav2_controller", "nav2_controller::ControllerServer", "controller_server", [
        Parameter(name="controller_plugins", value=ParameterValue(
            type=ParameterType.PARAMETER_STRING_ARRAY, string_array_value=["FollowPath"])),
        Parameter(name="FollowPath.plugin", value=ParameterValue(
            type=ParameterType.PARAMETER_STRING,
            string_value="nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"))], (1,2,5)),
    ("nav2_smoother", "nav2_smoother::SmootherServer", "smoother_server", [], (1,2,5)),
])
def test_component_destroys_managed_entities_before_plugin_library(
        tmp_path, package, plugin, name, parameters, transitions):
    namespace = f"/lifetime_{os.getpid()}"
    domain = 100 + os.getpid() % 100
    executable = Path(get_package_prefix("campus_regional_delivery")) / "lib" / \
        "campus_regional_delivery/regional_component_container"
    context = Context()
    rclpy.init(context=context, domain_id=domain,
               signal_handler_options=SignalHandlerOptions.NO)
    node = Node("lifetime_probe", context=context, use_global_arguments=False)
    executor = SingleThreadedExecutor(context=context)
    output = (tmp_path / "container.log").open("w")
    child = subprocess.Popen([str(executable), "--ros-args", "-r", "__node:=container",
        "-r", "__ns:=" + namespace], env={**os.environ, "ROS_DOMAIN_ID": str(domain)},
        stdout=output, stderr=subprocess.STDOUT, start_new_session=True)

    def request(kind, name, message):
        client = node.create_client(kind, namespace + name)
        try:
            assert client.wait_for_service(timeout_sec=10), name
            pending = client.call_async(message)
            rclpy.spin_until_future_complete(node, pending, executor=executor, timeout_sec=15)
            assert pending.done() and pending.result() is not None, name
            return pending.result()
        finally:
            node.destroy_client(client)

    try:
        loaded = request(LoadNode, "/container/_container/load_node", LoadNode.Request(
            package_name=package, plugin_name=plugin,
            node_name=name, node_namespace=namespace,
            parameters=parameters+[Parameter(name="use_sim_time", value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=True))]))
        assert loaded.success, loaded.error_message
        for transition in transitions:
            message = ChangeState.Request()
            message.transition.id = transition
            assert request(ChangeState, "/"+name+"/change_state", message).success
        unloaded = request(UnloadNode, "/container/_container/unload_node",
                           UnloadNode.Request(unique_id=loaded.unique_id))
        assert unloaded.success, unloaded.error_message
        assert not request(ListNodes, "/container/_container/list_nodes",
                           ListNodes.Request()).unique_ids
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
        output.close()
    assert child.returncode == 0, (tmp_path / "container.log").read_text()
    log = (tmp_path / "container.log").read_text()
    assert "[ERROR]" not in log and "[FATAL]" not in log, log
