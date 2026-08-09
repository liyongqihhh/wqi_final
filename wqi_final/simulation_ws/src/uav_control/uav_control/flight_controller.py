import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, Int8, Int32, String
from std_srvs.srv import Trigger
from uav_interfaces.action import FlyToPose

from uav_control.dynamic_path_planner import (
    path_lookahead_point,
    plan_dynamic_path,
)
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
        self.declare_parameter("dynamic_replanning_enabled", True)
        self.declare_parameter("dynamic_replan_period", 0.5)
        self.declare_parameter("dynamic_path_clearance", 2.5)
        self.declare_parameter("dynamic_path_spacing", 0.5)
        self.declare_parameter("dynamic_path_lookahead", 1.5)
        self.declare_parameter("dynamic_obstacle_warning_distance", 6.0)
        self.declare_parameter("dynamic_rear_warning_distance", 4.0)
        self.declare_parameter("dynamic_target_clearance", 1.5)
        self.declare_parameter("obstacle_vector_timeout", 1.0)
        self.declare_parameter("dynamic_clear_confirmations", 3)
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
        self.dynamic_replanning_enabled = bool(
            self.get_parameter("dynamic_replanning_enabled").value
        )
        self.dynamic_replan_period = float(
            self.get_parameter("dynamic_replan_period").value
        )
        self.dynamic_path_clearance = float(
            self.get_parameter("dynamic_path_clearance").value
        )
        self.dynamic_path_spacing = float(
            self.get_parameter("dynamic_path_spacing").value
        )
        self.dynamic_path_lookahead = float(
            self.get_parameter("dynamic_path_lookahead").value
        )
        self.dynamic_warning_distance = float(
            self.get_parameter(
                "dynamic_obstacle_warning_distance"
            ).value
        )
        self.dynamic_rear_warning_distance = float(
            self.get_parameter("dynamic_rear_warning_distance").value
        )
        self.dynamic_target_clearance = float(
            self.get_parameter("dynamic_target_clearance").value
        )
        self.obstacle_vector_timeout = float(
            self.get_parameter("obstacle_vector_timeout").value
        )
        self.dynamic_clear_confirmations = max(
            1,
            int(self.get_parameter("dynamic_clear_confirmations").value),
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
        self.nearest_obstacle_vector = None
        self.nearest_obstacle_updated_at = None

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.dynamic_path_pub = self.create_publisher(
            Path, "replanned_path", transient_qos
        )
        self.dynamic_status_pub = self.create_publisher(
            String, "dynamic_replanning/status", transient_qos
        )
        self.dynamic_replan_count_pub = self.create_publisher(
            Int32, "dynamic_replanning/count", transient_qos
        )
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
        self.create_subscription(
            Vector3Stamped,
            "safety/nearest_obstacle",
            self._nearest_obstacle_callback,
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

    def _nearest_obstacle_callback(self, message: Vector3Stamped) -> None:
        if message.header.frame_id not in (
            "",
            "base_link",
            "uav/base_link",
        ):
            self.get_logger().warning(
                "Ignoring obstacle vector in unexpected frame "
                f"'{message.header.frame_id}'"
            )
            return
        with self.lock:
            self.nearest_obstacle_vector = (
                float(message.vector.x),
                float(message.vector.y),
                float(message.vector.z),
            )
            self.nearest_obstacle_updated_at = time.monotonic()

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

    def _nearest_obstacle_snapshot(self):
        with self.lock:
            return (
                self.nearest_obstacle_vector,
                self.nearest_obstacle_updated_at,
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

    def _publish_dynamic_path(self, points) -> None:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for point in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2])
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.dynamic_path_pub.publish(path)

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
        active_dynamic_path = ()
        avoidance_active = False
        last_dynamic_replan = -math.inf
        preferred_side = 1
        dynamic_replan_count = 0
        dynamic_clear_observations = 0

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
            obstacle_vector, vector_updated_at = (
                self._nearest_obstacle_snapshot()
            )
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
            vector_is_fresh = (
                obstacle_vector is not None
                and vector_updated_at is not None
                and now - vector_updated_at
                <= self.obstacle_vector_timeout
            )
            position = odom.pose.pose.position
            orientation = odom.pose.pose.orientation
            current_position = (
                float(position.x),
                float(position.y),
                float(position.z),
            )
            target_position = (
                float(target.x),
                float(target.y),
                float(target.z),
            )
            if (
                self.dynamic_replanning_enabled
                and not allow_platform_proximity
                and vector_is_fresh
                and now - last_dynamic_replan >= self.dynamic_replan_period
            ):
                try:
                    dynamic_plan = plan_dynamic_path(
                        current_position,
                        target_position,
                        (
                            orientation.x,
                            orientation.y,
                            orientation.z,
                            orientation.w,
                        ),
                        obstacle_vector,
                        self.dynamic_path_clearance,
                        self.dynamic_warning_distance,
                        self.dynamic_rear_warning_distance,
                        self.dynamic_target_clearance,
                        self.minimum_altitude,
                        self.maximum_altitude,
                        self.dynamic_path_spacing,
                        preferred_side,
                    )
                except ValueError as error:
                    self.get_logger().error(
                        f"Cannot calculate sensor-driven UAV path: {error}"
                    )
                else:
                    last_dynamic_replan = now
                    preferred_side = dynamic_plan.preferred_side
                    if dynamic_plan.avoiding:
                        dynamic_clear_observations = 0
                        active_dynamic_path = dynamic_plan.points
                        self._publish_dynamic_path(active_dynamic_path)
                        self.dynamic_status_pub.publish(String(
                            data=(
                                f"REPLANNING:{dynamic_plan.reason};"
                                f"points={len(active_dynamic_path)}"
                            )
                        ))
                        if not avoidance_active:
                            avoidance_active = True
                            dynamic_replan_count += 1
                            self.dynamic_replan_count_pub.publish(
                                Int32(data=dynamic_replan_count)
                            )
                            self.get_logger().warning(
                                "UAV switched to a sensor-replanned path: "
                                f"{dynamic_plan.reason}"
                            )
                    elif avoidance_active:
                        dynamic_clear_observations += 1
                        if (
                            dynamic_clear_observations
                            >= self.dynamic_clear_confirmations
                        ):
                            avoidance_active = False
                            active_dynamic_path = ()
                            dynamic_clear_observations = 0
                            self._publish_dynamic_path(dynamic_plan.points)
                            self.dynamic_status_pub.publish(
                                String(data="CLEAR:direct_path_restored")
                            )
                        else:
                            self.dynamic_status_pub.publish(String(
                                data=(
                                    "HOLDING_REPLANNED_PATH:"
                                    f"clear_confirmation="
                                    f"{dynamic_clear_observations}/"
                                    f"{self.dynamic_clear_confirmations}"
                                )
                            ))

            if (
                avoidance_active
                and not vector_is_fresh
                and not active_safety_issue
            ):
                avoidance_active = False
                active_dynamic_path = ()
                dynamic_clear_observations = 0
                self._publish_dynamic_path(
                    (current_position, target_position)
                )
                self.dynamic_status_pub.publish(
                    String(data="CLEAR:obstacle_left_sensor_range")
                )

            cannot_plan_safely = (
                bool(active_safety_issue)
                and not (
                    avoidance_active
                    and active_safety_issue.startswith("BLOCKED:")
                )
            )
            if cannot_plan_safely:
                hold = Twist()
                hold.linear.x, hold.linear.y, hold.linear.z = current_position
                self.cmd_pub.publish(hold)
                within_tolerance_since = None
                self.dynamic_status_pub.publish(String(
                    data=f"WAITING_FOR_VALID_SENSOR_PATH:{active_safety_issue}"
                ))
                goal_handle.publish_feedback(
                    self._feedback(
                        odom,
                        last_distance,
                        "WAITING_FOR_VALID_SENSOR_PATH",
                    )
                )
                if now - started >= timeout:
                    goal_handle.abort()
                    result.success = False
                    result.message = (
                        f"Flight goal timed out after {timeout:.1f}s while "
                        f"waiting for a valid sensor path"
                    )
                    result.final_position_error = float(last_distance)
                    return result
                time.sleep(period)
                continue

            navigation_target = target_position
            phase = "FLYING"
            if avoidance_active and active_dynamic_path:
                navigation_target = path_lookahead_point(
                    current_position,
                    active_dynamic_path,
                    self.dynamic_path_lookahead,
                )
                phase = "FOLLOWING_REPLANNED_PATH"
                within_tolerance_since = None
            setpoint, approaching = adaptive_position_setpoint(
                current_position,
                navigation_target,
                self.cruise_setpoint_step,
                self.approach_setpoint_step,
                self.approach_slowdown_distance,
            )
            command = Twist()
            command.linear.x, command.linear.y, command.linear.z = setpoint
            self.cmd_pub.publish(command)
            if last_distance <= tolerance:
                phase = "SETTLING"
            elif approaching and not avoidance_active:
                phase = "APPROACHING"
            elif not avoidance_active:
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
