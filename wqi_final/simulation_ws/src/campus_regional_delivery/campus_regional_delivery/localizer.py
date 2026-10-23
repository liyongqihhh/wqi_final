"""Reuse native-wheel command watchdog with a scene-specific odometry frame."""
import math
import signal
import threading

from cooperative_delivery.relay_localizer import RelayLocalizer, yaw
from geometry_msgs.msg import TransformStamped, Twist
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


class RegionalLocalizer(RelayLocalizer):
    def _truth(self, message):
        if self.wheel is None:
            return
        p, w = message.pose.pose, self.wheel.pose.pose
        angle = yaw(p.orientation)-yaw(w.orientation)
        c, s = math.cos(angle), math.sin(angle)
        transform = TransformStamped()
        transform.header.stamp = (self.get_clock().now()+Duration(seconds=.2)).to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = self.get_namespace().strip("/") + "/odom"
        transform.transform.translation.x = p.position.x-(c*w.position.x-s*w.position.y)
        transform.transform.translation.y = p.position.y-(s*w.position.x+c*w.position.y)
        transform.transform.rotation.z = math.sin(angle/2)
        transform.transform.rotation.w = math.cos(angle/2)
        self.broadcaster.sendTransform(transform)


def main():
    # Finish the current DDS callback before invalidating its ROS context.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    stop=threading.Event()
    previous={s:signal.signal(s,lambda *_:stop.set()) for s in (signal.SIGINT,signal.SIGTERM)}
    node = RegionalLocalizer()
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if rclpy.ok():
            node.command_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for s,handler in previous.items():
            signal.signal(s,handler)
