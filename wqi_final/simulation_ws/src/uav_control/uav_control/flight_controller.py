import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Int8, String
from std_srvs.srv import Trigger
from uav_interfaces.action import FlyToPose

from uav_control.position_setpoint import adaptive_position_setpoint
from uav_control.safety_policy import safety_issue


LANDED = 0
FLYING = 1
TAKING_OFF = 2
LANDING = 3


class FlightController(Node):
    """Expose stable ROS actions over the sjtu_drone position controller."""

    def __init__(self) -> None:
        super().__init__("flight_controller")
        self.declare_parameter("command_rate", 20.0)
        self.declare_parameter("default_position_tolerance", 0.4)
        self.declare_parameter("default_velocity_tolerance", 0.1)
        self.declare_parameter("settle_duration", 1.0)
        self.declare_parameter("default_timeout", 180.0)
        self.declare_parameter("minimum_altitude", 0.15)
        self.declare_parameter("maximum_altitude", 48.0)
        self.declare_parameter("odom_wait_timeout", 15.0)
        self.declare_parameter("safety_monitor_enabled", True)
        self.declare_parameter("safety_data_timeout", 1.5)
        self.declare_parameter("obstacle_wait_timeout", 10.0)
        self.declare_parameter("cruise_position_setpoint_step", 2.0)
        self.declare_parameter("approach_slowdown_distance", 4.0)
        self.declare_parameter("approach_position_setpoint_step", 0.6)

        self.command_rate = float(self.get_parameter("command_rate").value)
        self.default_tolerance = float(
            self.get_parameter("default_position_tolerance").value
        )
        self.default_velocity_tolerance = float(
            self.get_parameter("default_velocity_tolerance").value
        )
        self.settle_duration = float(self.get_parameter("settle_duration").value)
        self.default_timeout = float(self.get_parameter("default_timeout").value)
        self.minimum_altitude = float(self.get_parameter("minimum_altitude").value)
        self.maximum_altitude = float(self.get_parameter("maximum_altitude").value)
        self.odom_wait_timeout = float(self.get_parameter("odom_wait_timeout").value)
        self.safety_monitor_enabled = bool(
            self.get_parameter("safety_monitor_enabled").value
        )
        self.safety_data_timeout = float(
            self.get_parameter("safety_data_timeout").value
        )
        self.obstacle_wait_timeout = float(
            self.get_parameter("obstacle_wait_timeout").value
        )
        self.cruise_setpoint_step = float(
            self.get_parameter("cruise_position_setpoint_step").value
        )
        self.approach_slowdown_distance = float(
            self.get_parameter("approach_slowdown_distance").value
        )
        self.approach_setpoint_step = float(
            self.get_parameter("approach_position_setpoint_step").value
        )
        adaptive_position_setpoint(
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
            self.cruise_setpoint_step,
            self.approach_setpoint_step,
            self.approach_slowdown_distance,
        )

        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.current_odom = None
        self.vehicle_state = None
        self.safety_blocked = True
        self.safety_status = "WAITING_FOR_SAFETY_MONITOR"
        self.safety_updated_at = None
        self.platform_safety_blocked = True
        self.platform_safety_status = "WAITING_FOR_SAFETY_MONITOR"
        self.platform_safety_updated_at = None

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.posctrl_pub = self.create_publisher(Bool, "posctrl", 10)
        self.takeoff_pub = self.create_publisher(Empty, "takeoff", 10)
        self.land_pub = self.create_publisher(Empty, "land", 10)
        self.create_subscription(
            Odometry,
            "odom",
            self._odom_callback,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Int8,
            "state",
            self._state_callback,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            "safety/blocked",
            self._safety_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "safety/status",
            self._safety_status_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            "safety/platform_blocked",
            self._platform_safety_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "safety/platform_status",
            self._platform_safety_status_callback,
            10,
            callback_group=self.callback_group,
        )

        self.create_service(
            Trigger,
            "takeoff",
            self._takeoff_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "land",
            self._land_callback,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            FlyToPose,
            "fly_to_pose",
            execute_callback=self._execute_fly_to_pose,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info("UAV flight controller is ready")

    def _odom_callback(self, message: Odometry) -> None:
        with self.lock:
            self.current_odom = message

    def _state_callback(self, message: Int8) -> None:
        with self.lock:
            self.vehicle_state = int(message.data)

    def _safety_callback(self, message: Bool) -> None:
        with self.lock:
            self.safety_blocked = bool(message.data)
            self.safety_updated_at = time.monotonic()

    def _safety_status_callback(self, message: String) -> None:
        with self.lock:
            self.safety_status = str(message.data)

    def _platform_safety_callback(self, message: Bool) -> None:
        with self.lock:
            self.platform_safety_blocked = bool(message.data)
            self.platform_safety_updated_at = time.monotonic()

    def _platform_safety_status_callback(self, message: String) -> None:
        with self.lock:
            self.platform_safety_status = str(message.data)

    def _snapshot(self):
        with self.lock:
            return self.current_odom, self.vehicle_state

    def _safety_snapshot(self, allow_platform_proximity=False):
        with self.lock:
            if allow_platform_proximity:
                return (
                    self.platform_safety_blocked,
                    self.platform_safety_status,
                    self.platform_safety_updated_at,
                )
            return (
                self.safety_blocked,
                self.safety_status,
                self.safety_updated_at,
            )

    def _publish_repeated(self, publisher, message, count: int = 3) -> None:
        for _ in range(count):
            publisher.publish(message)
            time.sleep(0.05)

    def _takeoff_callback(self, _request, response):
        _, state = self._snapshot()
        if state in (FLYING, TAKING_OFF):
            response.success = True
            response.message = "UAV is already airborne or taking off"
            return response
        if state == LANDING:
            response.success = False
            response.message = "UAV is currently landing"
            return response

        self._publish_repeated(self.posctrl_pub, Bool(data=False))
        self.cmd_pub.publish(Twist())
        self._publish_repeated(self.takeoff_pub, Empty())
        response.success = True
        response.message = "Takeoff command accepted"
        self.get_logger().info(response.message)
        return response

    def _land_callback(self, _request, response):
        _, state = self._snapshot()
        if state == LANDED:
            response.success = True
            response.message = "UAV is already landed"
            return response

        self._publish_repeated(self.posctrl_pub, Bool(data=False))
        self.cmd_pub.publish(Twist())
        self._publish_repeated(self.land_pub, Empty())
        response.success = True
        response.message = "Landing command accepted"
        self.get_logger().info(response.message)
        return response

    def _goal_callback(self, goal_request):
        target = goal_request.target
        coordinates = (
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        )
        if target.header.frame_id not in ("", "map"):
            self.get_logger().error("FlyToPose goals must use the map frame")
            return GoalResponse.REJECT
        if not all(math.isfinite(value) for value in coordinates):
            return GoalResponse.REJECT
        if not self.minimum_altitude <= coordinates[2] <= self.maximum_altitude:
            self.get_logger().error(
                f"Target altitude {coordinates[2]:.2f} is outside the safety limits"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _wait_for_odom(self) -> bool:
        deadline = time.monotonic() + self.odom_wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            odom, _ = self._snapshot()
            if odom is not None:
                return True
            time.sleep(0.05)
        return False

    @staticmethod
    def _distance(odom: Odometry, target) -> float:
        position = odom.pose.pose.position
        return math.sqrt(
            (target.x - position.x) ** 2
            + (target.y - position.y) ** 2
            + (target.z - position.z) ** 2
        )

    @staticmethod
    def _speed(odom: Odometry) -> float:
        velocity = odom.twist.twist.linear
        return math.sqrt(
            velocity.x * velocity.x
            + velocity.y * velocity.y
            + velocity.z * velocity.z
        )

    def _feedback(self, odom: Odometry, distance: float, phase: str):
        feedback = FlyToPose.Feedback()
        feedback.phase = phase
        feedback.distance_remaining = float(distance)
        feedback.current_pose = PoseStamped()
        feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
        feedback.current_pose.header.frame_id = "map"
        feedback.current_pose.pose = odom.pose.pose
        return feedback

    def _execute_fly_to_pose(self, goal_handle):
        result = FlyToPose.Result()
        if not self._wait_for_odom():
            result.success = False
            result.message = "No UAV odometry received"
            goal_handle.abort()
            return result

        start_wait = time.monotonic()
        while rclpy.ok():
            _, state = self._snapshot()
            if state == FLYING:
                break
            if state == LANDED or time.monotonic() - start_wait > 8.0:
                result.success = False
                result.message = "UAV must be airborne before FlyToPose"
                goal_handle.abort()
                return result
            time.sleep(0.05)

        request = goal_handle.request
        target = request.target.pose.position
        allow_platform_proximity = bool(request.allow_platform_proximity)
        tolerance = (
            float(request.position_tolerance)
            if request.position_tolerance > 0.0
            else self.default_tolerance
        )
        timeout = (
            float(request.timeout)
            if request.timeout > 0.0
            else self.default_timeout
        )
        self._publish_repeated(self.posctrl_pub, Bool(data=True))
        started = time.monotonic()
        within_tolerance_since = None
        period = 1.0 / max(self.command_rate, 1.0)
        last_distance = float("inf")
        hold_started = None
        hold_reason = ""
        hold_command = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Flight goal canceled"
                result.final_position_error = float(last_distance)
                return result

            odom, state = self._snapshot()
            if odom is None:
                time.sleep(period)
                continue
            if state in (LANDED, LANDING):
                goal_handle.abort()
                result.success = False
                result.message = "UAV left flying state while navigating"
                result.final_position_error = float(last_distance)
                return result

            last_distance = self._distance(odom, target)
            current_speed = self._speed(odom)
            now = time.monotonic()
            active_safety_issue = ""
            if self.safety_monitor_enabled:
                blocked, safety_status, safety_updated_at = self._safety_snapshot(
                    allow_platform_proximity
                )
                if (
                    safety_updated_at is None
                    or now - safety_updated_at > self.safety_data_timeout
                ):
                    active_safety_issue = "Safety monitor data unavailable"
                else:
                    active_safety_issue = safety_issue(
                        blocked,
                        safety_status,
                        allow_platform_proximity,
                    )

            if active_safety_issue:
                if hold_started is None:
                    hold_started = now
                    hold_reason = active_safety_issue
                    hold_command = Twist()
                    hold_command.linear.x = odom.pose.pose.position.x
                    hold_command.linear.y = odom.pose.pose.position.y
                    hold_command.linear.z = odom.pose.pose.position.z
                    self.get_logger().warning(
                        f"Holding position for obstacle: {hold_reason}"
                    )
                self.cmd_pub.publish(hold_command)
                within_tolerance_since = None
                goal_handle.publish_feedback(
                    self._feedback(
                        odom,
                        last_distance,
                        "HOLDING_FOR_OBSTACLE",
                    )
                )
                if now - hold_started >= self.obstacle_wait_timeout:
                    goal_handle.abort()
                    result.success = False
                    result.message = (
                        "Obstacle did not clear within "
                        f"{self.obstacle_wait_timeout:.1f}s: {hold_reason}"
                    )
                    result.final_position_error = float(last_distance)
                    return result
                time.sleep(period)
                continue

            if hold_started is not None:
                self.get_logger().info("Safety envelope clear; resuming flight")
                hold_started = None
                hold_reason = ""
                hold_command = None

            position = odom.pose.pose.position
            setpoint, approaching = adaptive_position_setpoint(
                (position.x, position.y, position.z),
                (target.x, target.y, target.z),
                self.cruise_setpoint_step,
                self.approach_setpoint_step,
                self.approach_slowdown_distance,
            )
            command = Twist()
            command.linear.x, command.linear.y, command.linear.z = setpoint
            self.cmd_pub.publish(command)
            if last_distance <= tolerance:
                phase = "SETTLING"
            elif approaching:
                phase = "APPROACHING"
            else:
                phase = "FLYING"
            goal_handle.publish_feedback(
                self._feedback(odom, last_distance, phase)
            )

            if (
                last_distance <= tolerance
                and current_speed <= self.default_velocity_tolerance
            ):
                if within_tolerance_since is None:
                    within_tolerance_since = now
                elif now - within_tolerance_since >= self.settle_duration:
                    goal_handle.succeed()
                    result.success = True
                    result.message = "Target reached"
                    result.final_position_error = float(last_distance)
                    return result
            else:
                within_tolerance_since = None

            if now - started >= timeout:
                goal_handle.abort()
                result.success = False
                result.message = f"Flight goal timed out after {timeout:.1f}s"
                result.final_position_error = float(last_distance)
                return result
            time.sleep(period)

        goal_handle.abort()
        result.success = False
        result.message = "ROS shutdown during flight"
        result.final_position_error = float(last_distance)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FlightController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
