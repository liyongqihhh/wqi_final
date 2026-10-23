"""Keep a Nav2 component context alive until its components are unloaded."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
from composition_interfaces.srv import ListNodes,UnloadNode
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from campus_regional_delivery.fleet import NAVIGATION_COMPONENTS


def shutdown_components(namespace,request):
    """Lifecycle shutdown first, then destroy components while ROS is still live."""
    result={"managed_shutdown":False,"unloaded":[],"errors":[]}
    try:
        states=[request(GetState,f"/{namespace}/{name}/get_state",GetState.Request()).current_state.id
                for name in NAVIGATION_COMPONENTS]
        if not all(state==4 for state in states):
            response=request(ManageLifecycleNodes,f"/{namespace}/lifecycle_manager_navigation/manage_nodes",
                ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.SHUTDOWN))
            if not response.success:
                raise RuntimeError("Navigation lifecycle shutdown failed")
        result["managed_shutdown"]=True
        prefix=f"/{namespace}/nav2_container/_container/"
        prepared=request(Trigger,prefix+"prepare_shutdown",Trigger.Request())
        if not prepared.success:
            raise RuntimeError("Unsafe component cleanup: "+prepared.message)
        nodes=request(ListNodes,prefix+"list_nodes",ListNodes.Request())
        for identifier in reversed(nodes.unique_ids):
            response=request(UnloadNode,prefix+"unload_node",UnloadNode.Request(unique_id=identifier))
            if not response.success:
                raise RuntimeError("Component unload failed: "+response.error_message)
            result["unloaded"].append(identifier)
    except (RuntimeError,TimeoutError) as error:
        result["errors"].append(str(error))
    return result


def main():
    parser=argparse.ArgumentParser(add_help=False)
    parser.add_argument("--vehicle-namespace",required=True,choices=[f"ugv_0{i}" for i in range(4)])
    options,ros_arguments=parser.parse_known_args()
    executable=Path(get_package_prefix("campus_regional_delivery"))/"lib/campus_regional_delivery/regional_component_container"
    child=subprocess.Popen([str(executable),*ros_arguments],start_new_session=True)
    stopping=[False]
    signal.signal(signal.SIGINT,lambda *_:stopping.__setitem__(0,True))
    signal.signal(signal.SIGTERM,lambda *_:stopping.__setitem__(0,True))
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    node=Node("regional_container_guard",namespace="/"+options.vehicle_namespace,
        use_global_arguments=False,parameter_overrides=[Parameter("use_sim_time",value=True)])
    cleanup=None
    try:
        while child.poll() is None and not stopping[0]:
            rclpy.spin_once(node,timeout_sec=.1)
        if child.poll() is None:
            deadline=time.monotonic()+30.
            def request(kind,name,message):
                client=node.create_client(kind,name)
                try:
                    service_deadline=min(deadline,time.monotonic()+2.)
                    while time.monotonic()<service_deadline and not client.service_is_ready():
                        rclpy.spin_once(node,timeout_sec=.05)
                    if not client.service_is_ready():
                        raise TimeoutError("Cleanup service unavailable: "+name)
                    pending=client.call_async(message)
                    while time.monotonic()<deadline and not pending.done():
                        rclpy.spin_once(node,timeout_sec=.05)
                    if not pending.done() or pending.result() is None:
                        raise TimeoutError("Cleanup response missing: "+name)
                    return pending.result()
                finally:
                    node.destroy_client(client)
            cleanup=shutdown_components(options.vehicle_namespace,request)
            print("REGIONAL_CONTAINER_CLEANUP "+json.dumps(cleanup),flush=True)
    finally:
        for sig,timeout in ((signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,2)):
            if child.poll() is not None:
                break
            child.send_signal(sig)
            try:
                child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                node.get_logger().error("Nav2 child did not exit after "+signal.Signals(sig).name)
        node.destroy_node()
        rclpy.shutdown()
    return 1 if child.returncode is None or (cleanup and cleanup["errors"]) else child.returncode or 0


if __name__=="__main__":
    sys.exit(main())
