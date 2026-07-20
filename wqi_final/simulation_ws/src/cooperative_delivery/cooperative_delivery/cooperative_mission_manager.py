import math
import os
import threading
import time
from dataclasses import replace

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from cooperative_delivery_interfaces.action import ExecuteCooperativeDelivery
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int8, String
from std_srvs.srv import Trigger
from uav_interfaces.action import ExecuteDelivery
from uav_interfaces.srv import CheckDeliveryEnergy

from cooperative_delivery.mission_config import (
    CooperativeMissionConfig,
    GroundWaypoint,
    MissionConfigurationError,
)
from cooperative_delivery.energy_planner import (
    EnergySortie,
    plan_cooperative_energy,
)
from cooperative_delivery.mission_states import (
    CooperativePhase,
    NavigationProgressTracker,
    is_vehicle_settled,
    navigation_timeout_for_distance,
)


LANDED = 0


class CooperativeMissionManager(Node):
    def __init__(self) -> None:
        super().__init__("cooperative_mission_manager")
        default_config = os.path.join(
            get_package_share_directory("cooperative_delivery"),
            "config",
            "cooperative_waypoints.yaml",
        )
        self.declare_parameter("mission_config", default_config)
        self.config = CooperativeMissionConfig(
            str(self.get_parameter("mission_config").value)
        )

        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.ugv_odom = None
        self.ugv_ground_truth = None
        self.uav_state = None
        self.docked = False
        self.nav_distance = 0.0
        self.uav_distance = 0.0
        self.mission_active = False
        self.phase = CooperativePhase.IDLE
        self.current_target = ""
        self.active_vehicle = "NONE"
        self.active_nav_goal = None
        self.active_uav_goal = None
        self.feedback_period = 1.0 / float(
            self.config.settings["feedback_rate"]
        )
        self.last_feedback_at = 0.0

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self.callback_group,
        )
        self.uav_client = ActionClient(
            self,
            ExecuteDelivery,
            "/uav/execute_delivery",
            callback_group=self.callback_group,
        )
        self.attach_client = self.create_client(
            Trigger, "/uav/attach_uav", callback_group=self.callback_group
        )
        self.detach_client = self.create_client(
            Trigger, "/uav/detach_uav", callback_group=self.callback_group
        )
        self.takeoff_client = self.create_client(
            Trigger, "/uav/takeoff", callback_group=self.callback_group
        )
        self.energy_check_client = self.create_client(
            CheckDeliveryEnergy,
            "/uav/check_delivery_energy",
            callback_group=self.callback_group,
        )
        self.clear_local_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=self.callback_group,
        )
        self.clear_global_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=self.callback_group,
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self._ugv_odom_callback,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self._ugv_ground_truth_callback,
            20,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Int8,
            "/uav/state",
            self._uav_state_callback,
            20,
            callback_group=self.callback_group,
        )
        dock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            "/uav/docked",
            self._docked_callback,
            dock_qos,
            callback_group=self.callback_group,
        )
        self.status_pub = self.create_publisher(
            String, "mission_status", dock_qos
        )
        self.energy_plan_pub = self.create_publisher(
            String, "energy_plan", dock_qos
        )
        self.optimized_route_pub = self.create_publisher(
            String, "optimized_route", dock_qos
        )

        self.action_server = ActionServer(
            self,
            ExecuteCooperativeDelivery,
            "execute_mission",
            execute_callback=self._execute_mission,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self._set_phase(CooperativePhase.IDLE)
        self.get_logger().info("Cooperative UGV-UAV mission manager is ready")

    def _ugv_odom_callback(self, message: Odometry) -> None:
        with self.lock:
            self.ugv_odom = message

    def _ugv_ground_truth_callback(self, message: Odometry) -> None:
        with self.lock:
            self.ugv_ground_truth = message

    def _uav_state_callback(self, message: Int8) -> None:
        with self.lock:
            self.uav_state = int(message.data)

    def _docked_callback(self, message: Bool) -> None:
        with self.lock:
            self.docked = bool(message.data)

    def _snapshot(self):
        with self.lock:
            return self.ugv_odom, self.uav_state, self.docked

    def _physical_ugv_odom(self):
        with self.lock:
            return self.ugv_ground_truth or self.ugv_odom

    def _goal_callback(self, request):
        with self.lock:
            if self.mission_active:
                self.get_logger().error("A cooperative mission is already active")
                return GoalResponse.REJECT
        if not request.targets:
            self.get_logger().error("Cooperative mission contains no targets")
            return GoalResponse.REJECT
        try:
            self._resolve_request_targets(request)
        except (MissionConfigurationError, ValueError) as error:
            self.get_logger().error(str(error))
            return GoalResponse.REJECT
        with self.lock:
            self.mission_active = True
        return GoalResponse.ACCEPT

    def _resolve_request_targets(self, request):
        targets = self.config.resolve(request.targets)
        floors = list(request.target_floors)
        payloads = list(request.payload_masses_kg)
        if floors and len(floors) != len(targets):
            raise ValueError(
                "target_floors must be empty or match the target count"
            )
        if payloads and len(payloads) != len(targets):
            raise ValueError(
                "payload_masses_kg must be empty or match the target count"
            )

        resolved = []
        for index, target in enumerate(targets):
            floor = int(floors[index]) if floors else None
            payload = (
                float(payloads[index])
                if payloads
                else target.payload_mass_kg
            )
            if floor is not None and floor <= 0:
                raise ValueError("Delivery floors must be positive")
            if not math.isfinite(payload) or payload < 0.0:
                raise ValueError(
                    "Payload masses must be finite and non-negative"
                )
            resolved.append(replace(
                target,
                payload_mass_kg=payload,
                delivery_floor=floor,
            ))
        return self.config.optimize_targets(
            resolved, bool(request.return_home)
        )

    def _publish_optimized_route(self, targets, route_plan, return_home):
        names = [self.config.ugv_home.name]
        names.extend(target.name for target in targets)
        if return_home:
            names.append(self.config.ugv_home.name)
        message = (
            f"OPTIMAL_UGV_ROUTE {route_plan.total_cost:.2f} m: "
            + " -> ".join(names)
        )
        self.optimized_route_pub.publish(String(data=message))
        self.get_logger().info(message)

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _set_phase(
        self,
        phase: CooperativePhase,
        target: str = "",
        vehicle: str = "NONE",
    ) -> None:
        self.phase = phase
        self.current_target = target
        self.active_vehicle = vehicle
        fields = [phase.value]
        if target:
            fields.append(target)
        if vehicle != "NONE":
            fields.append(vehicle)
        status = ":".join(fields)
        self.status_pub.publish(String(data=status))
        self.get_logger().info(f"Cooperative phase: {status}")

    def _publish_feedback(self, goal_handle) -> None:
        now = time.monotonic()
        if now - self.last_feedback_at < self.feedback_period:
            return
        self.last_feedback_at = now
        _, _, docked = self._snapshot()
        feedback = ExecuteCooperativeDelivery.Feedback()
        feedback.phase = self.phase.value
        feedback.current_target = self.current_target
        feedback.active_vehicle = self.active_vehicle
        feedback.uav_docked = bool(docked)
        if self.active_vehicle == "UGV":
            feedback.distance_remaining = float(self.nav_distance)
        elif self.active_vehicle == "UAV":
            feedback.distance_remaining = float(self.uav_distance)
        else:
            feedback.distance_remaining = 0.0
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _wait_future(future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done()

    def _wait_interfaces(self, timeout: float = 45.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self.nav_client.server_is_ready()
                and self.uav_client.server_is_ready()
                and self.attach_client.service_is_ready()
                and self.detach_client.service_is_ready()
                and self.takeoff_client.service_is_ready()
                and self.energy_check_client.service_is_ready()
                and self.clear_local_client.service_is_ready()
                and self.clear_global_client.service_is_ready()
            ):
                return True
            self.nav_client.wait_for_server(timeout_sec=0.2)
            self.uav_client.wait_for_server(timeout_sec=0.2)
            self.attach_client.wait_for_service(timeout_sec=0.2)
            self.detach_client.wait_for_service(timeout_sec=0.2)
            self.takeoff_client.wait_for_service(timeout_sec=0.2)
            self.energy_check_client.wait_for_service(timeout_sec=0.2)
            self.clear_local_client.wait_for_service(timeout_sec=0.2)
            self.clear_global_client.wait_for_service(timeout_sec=0.2)
        return False

    def _clear_costmaps(self) -> bool:
        for client in (self.clear_local_client, self.clear_global_client):
            future = client.call_async(ClearEntireCostmap.Request())
            if not self._wait_future(future, 10.0) or future.result() is None:
                return False
        return True

    def _call_docking_service(self, client, expected: bool):
        future = client.call_async(Trigger.Request())
        timeout = float(self.config.settings["docking_timeout"])
        if not self._wait_future(future, timeout):
            return False, "Docking service timed out"
        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else "No response"
            return False, str(message)

        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            _, _, docked = self._snapshot()
            if docked == expected:
                return True, str(response.message)
            time.sleep(0.05)
        return False, "Docking status did not reach the requested state"

    def _call_trigger(self, client, timeout: float = 10.0):
        future = client.call_async(Trigger.Request())
        if not self._wait_future(future, timeout):
            return False, "Service call timed out"
        response = future.result()
        if response is None:
            return False, "Service returned no response"
        return bool(response.success), str(response.message)

    def _request_uav_energy(self, target):
        request = CheckDeliveryEnergy.Request()
        request.targets = [target.uav_target]
        request.return_home = True
        request.home_name = target.uav_home_node
        request.landing_height = float(
            self.config.settings["uav_landing_height"]
        )
        request.payload_masses_kg = [float(target.payload_mass_kg)]
        request.target_floors = (
            []
            if target.delivery_floor is None
            else [int(target.delivery_floor)]
        )
        future = self.energy_check_client.call_async(request)
        if not self._wait_future(future, 10.0):
            return None, "UAV energy preflight service timed out"
        response = future.result()
        if response is None:
            return None, "UAV energy preflight returned no response"
        return response, str(response.message)

    def _check_uav_energy(self, target):
        response, message = self._request_uav_energy(target)
        if response is None:
            return False, message
        if not response.feasible:
            return False, message
        self.get_logger().info(message)
        return True, message

    def _plan_uav_energy_sequence(self, targets):
        responses = []
        sorties = []
        for target in targets:
            response, message = self._request_uav_energy(target)
            if response is None:
                return False, message
            responses.append(response)
            sorties.append(EnergySortie(
                target_name=target.name,
                launch_x=target.ugv_launch.x,
                launch_y=target.ugv_launch.y,
                mission_energy_wh=float(
                    response.estimated_mission_energy_wh
                ),
            ))
        first = responses[0]
        plan = plan_cooperative_energy(
            initial_energy_wh=float(first.current_energy_wh),
            battery_capacity_wh=float(first.battery_capacity_wh),
            reserve_energy_wh=float(first.safety_reserve_wh),
            net_charge_power_w=float(first.net_charge_power_w),
            ugv_planning_speed_mps=float(
                self.config.settings["ugv_energy_planning_speed"]
            ),
            initial_x=self.config.ugv_home.x,
            initial_y=self.config.ugv_home.y,
            sorties=sorties,
        )
        self.energy_plan_pub.publish(String(data=plan.message))
        log = self.get_logger().info if plan.feasible else self.get_logger().error
        log(plan.message)
        return plan.feasible, plan.message

    @staticmethod
    def _pose(waypoint: GroundWaypoint, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        half_yaw = waypoint.yaw / 2.0
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def _nav_feedback_callback(self, message) -> None:
        self.nav_distance = float(message.feedback.distance_remaining)

    @staticmethod
    def _yaw_from_odometry(odometry: Odometry) -> float:
        orientation = odometry.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y ** 2 + orientation.z ** 2
        )
        return math.atan2(sin_yaw, cos_yaw)

    def _cancel_navigation_goal(self, nested_goal) -> None:
        cancel_future = nested_goal.cancel_goal_async()
        if not self._wait_future(cancel_future, 5.0):
            self.get_logger().warning(
                "Timed out waiting for Nav2 to acknowledge cancellation"
            )
        self.active_nav_goal = None

    def _cancel_uav_goal(self, nested_goal) -> None:
        cancel_future = nested_goal.cancel_goal_async()
        if not self._wait_future(cancel_future, 5.0):
            self.get_logger().warning(
                "Timed out waiting for UAV action cancellation"
            )
        self.active_uav_goal = None

    def _cancel_active_goals(self) -> None:
        if self.active_nav_goal is not None:
            self._cancel_navigation_goal(self.active_nav_goal)
        if self.active_uav_goal is not None:
            self._cancel_uav_goal(self.active_uav_goal)

    def _navigate(self, waypoint: GroundWaypoint, goal_handle):
        self.nav_distance = 0.0
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(waypoint, self.get_clock().now().to_msg())
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_callback
        )
        if not self._wait_future(send_future, 15.0):
            return False, "Nav2 goal submission timed out"
        nested_goal = send_future.result()
        if nested_goal is None or not nested_goal.accepted:
            return False, "Nav2 rejected the UGV goal"

        self.active_nav_goal = nested_goal
        result_future = nested_goal.get_result_async()
        started = self.get_clock().now()
        last_progress_at = started
        best_remaining_distance = math.inf
        observed_route_distance = 0.0
        progress_tracker = NavigationProgressTracker(
            float(self.config.settings["navigation_progress_distance"]),
            float(self.config.settings["navigation_progress_angle"]),
        )
        last_progress_log_at = time.monotonic()
        minimum_timeout = float(
            self.config.settings["navigation_timeout_min"]
        )
        seconds_per_meter = float(
            self.config.settings["navigation_timeout_per_meter"]
        )
        maximum_timeout = float(
            self.config.settings["navigation_timeout_max"]
        )
        stall_timeout = float(
            self.config.settings["navigation_stall_timeout"]
        )
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_navigation_goal(nested_goal)
                return False, "Cooperative mission canceled"
            observed_route_distance = max(
                observed_route_distance,
                self.nav_distance,
            )
            clock_now = self.get_clock().now()
            progressed = False
            if (
                self.nav_distance > 0.0
                and self.nav_distance <= best_remaining_distance - 0.05
            ):
                best_remaining_distance = self.nav_distance
                progressed = True

            physical_odom = self._physical_ugv_odom()
            if physical_odom is not None:
                position = physical_odom.pose.pose.position
                if progress_tracker.update(
                    position.x,
                    position.y,
                    self._yaw_from_odometry(physical_odom),
                ):
                    progressed = True

            if progressed:
                last_progress_at = clock_now
            stall_elapsed = max(
                0.0,
                (clock_now - last_progress_at).nanoseconds / 1.0e9,
            )
            if (
                math.isfinite(best_remaining_distance)
                and stall_elapsed >= stall_timeout
            ):
                self._cancel_navigation_goal(nested_goal)
                return False, (
                    "UGV ground-truth pose and Nav2 path made no progress for "
                    f"{stall_timeout:.1f}s of simulated time "
                    f"(remaining {self.nav_distance:.1f}m)"
                )
            timeout = navigation_timeout_for_distance(
                observed_route_distance,
                minimum_timeout,
                seconds_per_meter,
                maximum_timeout,
            )
            elapsed = max(
                0.0,
                (self.get_clock().now() - started).nanoseconds / 1.0e9,
            )
            if elapsed >= timeout:
                self._cancel_navigation_goal(nested_goal)
                return False, (
                    f"UGV navigation exceeded {elapsed:.1f}s of simulated time "
                    f"(route {observed_route_distance:.1f}m, limit {timeout:.1f}s)"
                )
            wall_now = time.monotonic()
            if wall_now - last_progress_log_at >= 30.0:
                pose_text = "unavailable"
                if physical_odom is not None:
                    position = physical_odom.pose.pose.position
                    pose_text = f"({position.x:.2f}, {position.y:.2f})"
                self.get_logger().info(
                    "UGV navigation active: "
                    f"remaining {self.nav_distance:.1f}m, "
                    f"physical pose {pose_text}, "
                    f"no-motion {stall_elapsed:.1f}s sim"
                )
                last_progress_log_at = wall_now
            self._publish_feedback(goal_handle)
            time.sleep(0.1)
        self.active_nav_goal = None
        if not result_future.done():
            return False, "ROS shutdown during UGV navigation"
        wrapped_result = result_future.result()
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f"UGV navigation ended with status {wrapped_result.status}"
        return True, f"UGV reached {waypoint.name}"

    def _navigate_with_recovery(self, waypoint: GroundWaypoint, goal_handle):
        retry_count = int(self.config.settings["navigation_retry_count"])
        retry_delay = float(self.config.settings["navigation_retry_delay"])
        for attempt in range(retry_count + 1):
            success, message = self._navigate(waypoint, goal_handle)
            if success:
                return True, message
            if (
                goal_handle.is_cancel_requested
                or message == "Cooperative mission canceled"
                or not rclpy.ok()
            ):
                return False, message
            if attempt >= retry_count:
                return False, message

            self.get_logger().warning(
                f"UGV navigation attempt {attempt + 1} failed: {message}; "
                "clearing costmaps before retry "
                f"{attempt + 1}/{retry_count}"
            )
            if not self._wait_ugv_settled(goal_handle):
                return False, "UGV did not settle before navigation retry"
            if not self._clear_costmaps():
                return False, "Failed to clear costmaps before navigation retry"
            deadline = time.monotonic() + retry_delay
            while rclpy.ok() and time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    return False, "Cooperative mission canceled"
                self._publish_feedback(goal_handle)
                time.sleep(0.1)

        return False, "UGV navigation retry loop ended unexpectedly"

    def _wait_ugv_settled(self, goal_handle) -> bool:
        maximum_speed = float(self.config.settings["ugv_settle_speed"])
        settle_duration = float(self.config.settings["ugv_settle_duration"])
        timeout = float(self.config.settings["ugv_settle_timeout"])
        deadline = time.monotonic() + timeout
        settled_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            odom = self._physical_ugv_odom()
            if odom is not None:
                velocity = odom.twist.twist
                speed = math.sqrt(
                    velocity.linear.x ** 2
                    + velocity.linear.y ** 2
                    + velocity.angular.z ** 2
                )
                if is_vehicle_settled(speed, maximum_speed):
                    if settled_since is None:
                        settled_since = time.monotonic()
                    elif time.monotonic() - settled_since >= settle_duration:
                        return True
                else:
                    settled_since = None
            if goal_handle.is_cancel_requested:
                return False
            self._publish_feedback(goal_handle)
            time.sleep(0.1)
        return False

    def _uav_feedback_callback(self, message) -> None:
        self.uav_distance = float(message.feedback.distance_remaining)

    def _execute_uav_delivery(self, target, goal_handle):
        goal = ExecuteDelivery.Goal()
        goal.targets = [target.uav_target]
        goal.return_home = True
        goal.home_name = target.uav_home_node
        goal.landing_height = float(
            self.config.settings["uav_landing_height"]
        )
        goal.payload_masses_kg = [float(target.payload_mass_kg)]
        goal.target_floors = (
            []
            if target.delivery_floor is None
            else [int(target.delivery_floor)]
        )
        send_future = self.uav_client.send_goal_async(
            goal, feedback_callback=self._uav_feedback_callback
        )
        if not self._wait_future(send_future, 15.0):
            return False, "UAV goal submission timed out"
        nested_goal = send_future.result()
        if nested_goal is None or not nested_goal.accepted:
            return False, "UAV rejected the delivery goal"

        self.active_uav_goal = nested_goal
        result_future = nested_goal.get_result_async()
        deadline = time.monotonic() + float(
            self.config.settings["uav_mission_timeout"]
        )
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self._cancel_uav_goal(nested_goal)
                return False, "Cooperative mission canceled"
            if time.monotonic() >= deadline:
                self._cancel_uav_goal(nested_goal)
                return False, "UAV delivery timed out"
            self._publish_feedback(goal_handle)
            time.sleep(0.1)
        self.active_uav_goal = None
        if not result_future.done():
            return False, "ROS shutdown during UAV delivery"
        wrapped_result = result_future.result()
        result = wrapped_result.result
        if (
            wrapped_result.status != GoalStatus.STATUS_SUCCEEDED
            or not result.success
        ):
            return False, str(result.message)
        return True, str(result.message)

    def _finish_failure(self, goal_handle, result, message):
        if goal_handle.is_cancel_requested:
            return self._finish_canceled(goal_handle, result)
        self._cancel_active_goals()
        self._set_phase(CooperativePhase.FAILED, self.current_target)
        result.success = False
        result.message = message
        goal_handle.abort()
        self.get_logger().error(message)
        return result

    def _finish_canceled(self, goal_handle, result):
        self._cancel_active_goals()
        self._set_phase(CooperativePhase.CANCELED, self.current_target)
        result.success = False
        result.message = "Cooperative mission canceled by client"
        goal_handle.canceled()
        self.get_logger().warning(result.message)
        return result

    def _execute_mission(self, goal_handle):
        result = ExecuteCooperativeDelivery.Result()
        result.completed_targets = 0
        self.last_feedback_at = 0.0
        try:
            targets, route_plan = self._resolve_request_targets(
                goal_handle.request
            )
            self._publish_optimized_route(
                targets,
                route_plan,
                bool(goal_handle.request.return_home),
            )
            if not self._wait_interfaces():
                return self._finish_failure(
                    goal_handle, result, "Cooperative interfaces are unavailable"
                )

            self._set_phase(CooperativePhase.PREPARING)
            _, uav_state, _ = self._snapshot()
            if uav_state is not None and uav_state != LANDED:
                return self._finish_failure(
                    goal_handle, result, "UAV must be landed before UGV transit"
                )
            success, message = self._call_docking_service(
                self.attach_client, True
            )
            if not success:
                return self._finish_failure(
                    goal_handle, result, f"Initial docking failed: {message}"
                )
            time.sleep(1.0)
            if not self._clear_costmaps():
                return self._finish_failure(
                    goal_handle, result, "Failed to clear costmaps after docking"
                )

            # Predict the complete sequence before the UGV starts. The plan
            # subtracts every payload-specific sortie and conservatively adds
            # only the minimum charging available during UGV transit.
            success, message = self._plan_uav_energy_sequence(targets)
            if not success:
                return self._finish_failure(
                    goal_handle,
                    result,
                    "Cooperative mission rejected by UAV battery plan: "
                    + message,
                )

            current_ugv_stop = self.config.ugv_home.name
            for index, target in enumerate(targets):
                if target.ugv_launch.name != current_ugv_stop:
                    self._set_phase(
                        CooperativePhase.UGV_TRANSIT, target.name, "UGV"
                    )
                    success, message = self._navigate_with_recovery(
                        target.ugv_launch, goal_handle
                    )
                    if not success:
                        return self._finish_failure(
                            goal_handle, result, message
                        )
                    current_ugv_stop = target.ugv_launch.name
                else:
                    self.get_logger().info(
                        "UGV already at shared launch stop "
                        f"'{current_ugv_stop}'; starting next UAV sortie"
                    )

                self._set_phase(
                    CooperativePhase.UGV_SETTLING, target.name, "UGV"
                )
                if not self._wait_ugv_settled(goal_handle):
                    return self._finish_failure(
                        goal_handle, result, "UGV did not settle before UAV release"
                    )

                # Charging continues during UGV transit. Re-evaluate with the
                # latest state of charge immediately before physical release.
                success, message = self._check_uav_energy(target)
                if not success:
                    return self._finish_failure(
                        goal_handle,
                        result,
                        "UAV takeoff rejected by battery preflight: " + message,
                    )

                self._set_phase(
                    CooperativePhase.UAV_DETACHING, target.name, "UAV"
                )
                success, message = self._call_trigger(self.takeoff_client)
                if not success:
                    return self._finish_failure(
                        goal_handle, result, f"UAV pre-takeoff failed: {message}"
                    )
                success, message = self._call_docking_service(
                    self.detach_client, False
                )
                if not success:
                    return self._finish_failure(
                        goal_handle, result, f"UAV detach failed: {message}"
                    )

                self._set_phase(
                    CooperativePhase.UAV_DELIVERING, target.name, "UAV"
                )
                success, message = self._execute_uav_delivery(
                    target, goal_handle
                )
                if not success:
                    return self._finish_failure(
                        goal_handle, result, f"UAV delivery failed: {message}"
                    )

                self._set_phase(
                    CooperativePhase.UAV_DOCKING, target.name, "UAV"
                )
                success, message = self._call_docking_service(
                    self.attach_client, True
                )
                if not success:
                    return self._finish_failure(
                        goal_handle, result, f"UAV redocking failed: {message}"
                    )
                self.get_logger().info(
                    "UAV redocked; waiting for the platform to settle before "
                    "refreshing UGV costmaps"
                )
                if not self._wait_ugv_settled(goal_handle):
                    return self._finish_failure(
                        goal_handle,
                        result,
                        "Docked platform did not settle before UGV navigation",
                    )
                if not self._clear_costmaps():
                    return self._finish_failure(
                        goal_handle,
                        result,
                        "Failed to clear costmaps after UAV redocking",
                    )
                result.completed_targets = index + 1

            if goal_handle.request.return_home:
                self._set_phase(
                    CooperativePhase.RETURNING_HOME, "logistics_center", "UGV"
                )
                success, message = self._navigate_with_recovery(
                    self.config.ugv_home, goal_handle
                )
                if not success:
                    return self._finish_failure(goal_handle, result, message)

            if goal_handle.is_cancel_requested:
                return self._finish_canceled(goal_handle, result)

            self._set_phase(CooperativePhase.COMPLETED)
            result.success = True
            result.message = "Cooperative UGV-UAV delivery completed"
            goal_handle.succeed()
            return result
        except Exception as error:  # Keep both vehicles stopped on unexpected errors.
            self.get_logger().error(
                f"Unhandled cooperative mission error: {error!r}"
            )
            return self._finish_failure(goal_handle, result, str(error))
        finally:
            with self.lock:
                self.mission_active = False


def main(args=None):
    rclpy.init(args=args)
    node = CooperativeMissionManager()
    executor = MultiThreadedExecutor(num_threads=6)
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
