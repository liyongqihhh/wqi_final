"""Bounded lifecycle readiness gate for staged fleet startup."""
import re
import time

from lifecycle_msgs.srv import GetState
import rclpy


REQUIRED_NODES=("controller_server","smoother_server","planner_server","behavior_server",
                "bt_navigator","waypoint_follower","velocity_smoother","collision_monitor")


def navigation_states_ready(states):
    return all(states.get(name)==3 for name in REQUIRED_NODES)


def main():
    rclpy.init()
    node=rclpy.create_node("wait_for_regional_navigation")
    node.declare_parameter("vehicle_namespace","ugv_00")
    node.declare_parameter("timeout_wall_s",240.)
    namespace=node.get_parameter("vehicle_namespace").value
    timeout=float(node.get_parameter("timeout_wall_s").value)
    status=1
    try:
        if not re.fullmatch(r"ugv_0[0123]",namespace) or not 10<=timeout<=600:
            raise ValueError("Invalid regional navigation startup configuration")
        clients={name:node.create_client(GetState,f"/{namespace}/{name}/get_state") for name in REQUIRED_NODES}
        deadline=time.monotonic()+timeout
        states={}
        while rclpy.ok() and time.monotonic()<deadline:
            states={}
            for name,client in clients.items():
                if not client.service_is_ready():
                    continue
                future=client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node,future,timeout_sec=1.)
                if future.done() and future.result() is not None:
                    states[name]=future.result().current_state.id
                else:
                    future.cancel()
            if navigation_states_ready(states):
                node.get_logger().info(f"{namespace} navigation is active; releasing the next startup stage")
                status=0
                break
            rclpy.spin_once(node,timeout_sec=.2)
        if status:
            node.get_logger().error(f"{namespace} navigation did not activate within {timeout:g} wall seconds: {states}")
    except (KeyboardInterrupt,RuntimeError,ValueError) as error:
        node.get_logger().error(str(error))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status
