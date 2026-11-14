"""Keep the ROS context alive until executor callbacks have finished."""
import signal
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def run_node(factory, args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stopping = threading.Event()
    previous = {sig: signal.signal(sig, lambda *_: stopping.set())
                for sig in (signal.SIGINT, signal.SIGTERM)}
    node = None
    executor = None
    try:
        node = factory()
        executor = MultiThreadedExecutor(num_threads=6)
        executor.add_node(node)
        while rclpy.ok() and not stopping.is_set():
            executor.spin_once(timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.shutdown_requested = True
                for timer in node.timers:
                    timer.cancel()
            if executor is not None and not executor.shutdown(timeout_sec=30.0):
                raise RuntimeError("Callbacks did not finish before ROS teardown")
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
