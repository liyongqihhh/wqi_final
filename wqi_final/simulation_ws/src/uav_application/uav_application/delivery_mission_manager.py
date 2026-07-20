import math
import os
import threading
import time
from dataclasses import dataclass

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Float64, Int8, String
from std_srvs.srv import Trigger
from uav_control.battery_model import BatteryModel, BatteryParameters
from uav_interfaces.action import ExecuteDelivery, FlyToPose
from uav_interfaces.srv import CheckDeliveryEnergy
from uav_navigation.waypoint_navigator import (
    Waypoint,
    WaypointConfigurationError,
    WaypointMap,
)
from uav_navigation.route_optimizer import RoutePlan, optimize_visit_order

from uav_application.mission_states import (
    MissionPhase,
    is_settled_at_altitude,
    uses_local_delivery_profile,
)
from uav_application.mission_energy import MissionEnergyPlanner


LANDED = 0
FLYING = 1


@dataclass(frozen=True)
class OptimizedDelivery:
    target_names: tuple[str, ...]
    targets: tuple[Waypoint, ...]
    payload_masses: tuple[float, ...]
    target_floors: tuple[int, ...]
    home: Waypoint
    route_plan: RoutePlan
    return_home: bool


class DeliveryMissionManager(Node):
    def __init__(self) -> None:
        super().__init__("delivery_mission_manager")
        default_waypoint_file = os.path.join(
            get_package_share_directory("uav_navigation"),
            "config",
            "uav_delivery_waypoints.yaml",
        )
        self.declare_parameter("waypoint_file", default_waypoint_file)
        default_battery_file = os.path.join(
            get_package_share_directory("uav_control"),
            "config",
            "battery_model.yaml",
        )
        self.declare_parameter("battery_config", default_battery_file)
        self.declare_parameter("battery_state_timeout", 3.0)
        self.waypoint_map = WaypointMap(
            str(self.get_parameter("waypoint_file").value)
        )
        self.battery_parameters = BatteryParameters.from_yaml(
            str(self.get_parameter("battery_config").value)
        )
        self.energy_planner = MissionEnergyPlanner(
            self.waypoint_map, self.battery_parameters
        )

        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.current_odom = None
        self.vehicle_state = None
        self.nested_distance = 0.0
        self.active_fly_goal = None
        self.phase = MissionPhase.IDLE
        self.current_target = ""
        self.battery_state = None
        self.battery_received_at_ns = 0

        self.fly_client = ActionClient(
            self,
            FlyToPose,
            "fly_to_pose",
            callback_group=self.callback_group,
        )
        self.takeoff_client = self.create_client(
            Trigger, "takeoff", callback_group=self.callback_group
        )
        self.land_client = self.create_client(
            Trigger, "land", callback_group=self.callback_group
        )
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

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            BatteryState,
            "battery_state",
            self._battery_callback,
            transient_qos,
            callback_group=self.callback_group,
        )
        self.status_pub = self.create_publisher(
            String, "mission_status", transient_qos
        )
        self.delivery_event_pub = self.create_publisher(
            String, "delivery_event", transient_qos
        )
        self.planned_path_pub = self.create_publisher(
            Path, "planned_path", transient_qos
        )
        self.landing_height_pub = self.create_publisher(
            Float64, "landing_height", 10
        )
        self.energy_preflight_pub = self.create_publisher(
            String, "energy_preflight", transient_qos
        )
        self.optimized_route_pub = self.create_publisher(
            String, "optimized_route", transient_qos
        )
        self.can_execute_pub = self.create_publisher(
            Bool, "can_execute_task", transient_qos
        )
        self.payload_mass_pub = self.create_publisher(
            Float32, "payload_mass", transient_qos
        )
        self.energy_service = self.create_service(
            CheckDeliveryEnergy,
            "check_delivery_energy",
            self._check_delivery_energy_callback,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            ExecuteDelivery,
            "execute_delivery",
            execute_callback=self._execute_delivery,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self._set_phase(MissionPhase.IDLE)
        self._publish_payload_mass(0.0)
        self.can_execute_pub.publish(Bool(data=False))
        self.get_logger().info("UAV delivery mission manager is ready")

    def _odom_callback(self, message: Odometry) -> None:
        with self.lock:
            self.current_odom = message

    def _state_callback(self, message: Int8) -> None:
        with self.lock:
            self.vehicle_state = int(message.data)

    def _battery_callback(self, message: BatteryState) -> None:
        with self.lock:
            self.battery_state = message
            self.battery_received_at_ns = self.get_clock().now().nanoseconds

    def _snapshot(self):
        with self.lock:
            return self.current_odom, self.vehicle_state, self.nested_distance

    def _goal_callback(self, goal_request):
        if not goal_request.targets:
            self.get_logger().error("Delivery goal contains no targets")
            return GoalResponse.REJECT
        try:
            mission = self._resolve_optimized_delivery(
                goal_request.targets,
                goal_request.target_floors,
                goal_request.payload_masses_kg,
                goal_request.home_name,
                bool(goal_request.return_home),
            )
        except WaypointConfigurationError as error:
            self.get_logger().error(str(error))
            return GoalResponse.REJECT
        except ValueError as error:
            self.get_logger().error(str(error))
            return GoalResponse.REJECT
        landing_height = float(goal_request.landing_height)
        if not math.isfinite(landing_height) or not 0.0 <= landing_height <= 2.0:
            self.get_logger().error("Landing height must be between 0 and 2 metres")
            return GoalResponse.REJECT
        assessment, message = self._assess_delivery_energy(
            mission.target_names,
            goal_request.return_home,
            goal_request.home_name,
            landing_height,
            mission.payload_masses,
            mission.target_floors,
        )
        self.energy_preflight_pub.publish(String(data=message))
        self.can_execute_pub.publish(
            Bool(data=bool(assessment and assessment.feasible))
        )
        if assessment is None or not assessment.feasible:
            self.get_logger().error(message)
            return GoalResponse.REJECT
        self.get_logger().info(message)
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _battery_percentage(self):
        with self.lock:
            message = self.battery_state
            received_at_ns = self.battery_received_at_ns
        age = (self.get_clock().now().nanoseconds - received_at_ns) / 1e9
        timeout = float(self.get_parameter("battery_state_timeout").value)
        if message is None or age < 0.0 or age > timeout:
            return None
        percentage = float(message.percentage)
        if not math.isfinite(percentage) or not 0.0 <= percentage <= 1.0:
            return None
        return percentage

    def _publish_payload_mass(self, payload_mass_kg: float) -> None:
        self.payload_mass_pub.publish(
            Float32(data=float(max(0.0, payload_mass_kg)))
        )

    def _resolve_optimized_delivery(
        self,
        target_names,
        target_floors,
        payload_masses,
        home_name: str,
        return_home: bool,
    ) -> OptimizedDelivery:
        names = list(target_names)
        floors = list(target_floors)
        targets = self.waypoint_map.resolve_delivery_targets(names, floors)
        home = self.waypoint_map.resolve_home(home_name)
        masses = self.energy_planner.resolve_payload_masses(
            targets, payload_masses
        )
        plan = optimize_visit_order(
            len(targets),
            lambda index: self.waypoint_map.route_distance(
                home.name, targets[index].name
            ),
            lambda origin, destination: self.waypoint_map.route_distance(
                targets[origin].name, targets[destination].name
            ),
            (
                lambda index: self.waypoint_map.route_distance(
                    targets[index].name, home.name
                )
            ) if return_home else None,
        )
        order = plan.order
        return OptimizedDelivery(
            tuple(names[index] for index in order),
            tuple(targets[index] for index in order),
            tuple(float(masses[index]) for index in order),
            tuple(int(floors[index]) for index in order) if floors else (),
            home,
            plan,
            bool(return_home),
        )

    def _publish_optimized_route(self, mission: OptimizedDelivery) -> None:
        names = [mission.home.name, *mission.target_names]
        if mission.return_home:
            names.append(mission.home.name)
        message = (
            f"OPTIMAL_UAV_ROUTE {mission.route_plan.total_cost:.2f} m: "
            + " -> ".join(names)
        )
        self.optimized_route_pub.publish(String(data=message))
        self.get_logger().info(message)

    def _assess_delivery_energy(
        self,
        targets,
        return_home: bool,
        home_name: str,
        landing_height: float,
        payload_masses_kg=None,
        target_floors=None,
    ):
        percentage = self._battery_percentage()
        if percentage is None:
            return None, "Energy preflight rejected: UAV battery state is unavailable or stale"
        try:
            profile = self.energy_planner.plan(
                targets,
                home_name,
                landing_height,
                return_home,
                payload_masses_kg=payload_masses_kg,
                target_floors=target_floors,
            )
        except (ValueError, WaypointConfigurationError) as error:
            return None, f"Energy preflight rejected: {error}"
        model = BatteryModel(
            self.battery_parameters, initial_percentage=percentage
        )
        assessment = model.estimate(profile)
        phase_energy = {
            name: energy * self.battery_parameters.prediction_margin_factor
            for name, energy in model.phase_energy_wh(profile).items()
        }
        ascent_energy = sum(
            phase_energy[name]
            for name in (
                "ascent_acceleration",
                "climb",
                "ascent_deceleration",
            )
        )
        horizontal_dynamic_energy = (
            phase_energy["horizontal_acceleration"]
            + phase_energy["horizontal_deceleration"]
        )
        descent_energy = sum(
            phase_energy[name]
            for name in (
                "descent_acceleration",
                "descent",
                "descent_deceleration",
            )
        )
        decision = "PASS" if assessment.feasible else "REJECT"
        message = (
            f"Energy preflight {decision}: SOC {assessment.current_soc * 100.0:.1f}%, "
            f"payload {assessment.initial_payload_mass_kg:.2f} kg, "
            f"mission {assessment.estimated_mission_energy_wh:.2f} Wh, "
            f"reserve {assessment.safety_reserve_wh:.2f} Wh, "
            f"required {assessment.required_energy_wh:.2f} Wh, "
            f"predicted final {assessment.estimated_final_soc * 100.0:.1f}% "
            "(safe return included); estimated breakdown: "
            f"propulsion {assessment.propulsion_energy_wh:.2f}, "
            f"auxiliary {assessment.auxiliary_energy_wh:.2f}, "
            f"payload penalty {assessment.payload_energy_penalty_wh:.2f}, "
            f"ascent {ascent_energy:.2f}, "
            f"horizontal accel/decel {horizontal_dynamic_energy:.2f}, "
            f"cruise {phase_energy['cruise']:.2f}, "
            f"hover {phase_energy['hover']:.2f}, "
            f"descent {descent_energy:.2f} Wh"
        )
        return assessment, message

    @staticmethod
    def _fill_energy_response(response, assessment, message: str) -> None:
        response.feasible = bool(assessment and assessment.feasible)
        response.message = message
        if assessment is None:
            return
        response.current_soc = float(assessment.current_soc)
        response.current_energy_wh = float(assessment.current_energy_wh)
        response.estimated_mission_energy_wh = float(
            assessment.estimated_mission_energy_wh
        )
        response.safety_reserve_wh = float(assessment.safety_reserve_wh)
        response.required_energy_wh = float(assessment.required_energy_wh)
        response.estimated_final_soc = float(assessment.estimated_final_soc)
        response.raw_mission_energy_wh = float(
            assessment.raw_mission_energy_wh
        )
        response.propulsion_energy_wh = float(
            assessment.propulsion_energy_wh
        )
        response.auxiliary_energy_wh = float(
            assessment.auxiliary_energy_wh
        )
        response.payload_energy_penalty_wh = float(
            assessment.payload_energy_penalty_wh
        )
        response.initial_payload_mass_kg = float(
            assessment.initial_payload_mass_kg
        )

    def _check_delivery_energy_callback(self, request, response):
        try:
            mission = self._resolve_optimized_delivery(
                request.targets,
                request.target_floors,
                request.payload_masses_kg,
                request.home_name,
                bool(request.return_home),
            )
            landing_height = float(request.landing_height)
            if not math.isfinite(landing_height) or not 0.0 <= landing_height <= 2.0:
                raise ValueError("Landing height must be between 0 and 2 metres")
            assessment, message = self._assess_delivery_energy(
                mission.target_names,
                request.return_home,
                request.home_name,
                landing_height,
                mission.payload_masses,
                mission.target_floors,
            )
        except (ValueError, WaypointConfigurationError) as error:
            assessment = None
            message = f"Energy preflight rejected: {error}"
        self._fill_energy_response(response, assessment, message)
        response.battery_capacity_wh = float(
            self.battery_parameters.capacity_wh
        )
        response.net_charge_power_w = float(
            self.battery_parameters.net_charge_power_w
        )
        self.energy_preflight_pub.publish(String(data=message))
        self.can_execute_pub.publish(
            Bool(data=bool(assessment and assessment.feasible))
        )
        return response

    def _set_phase(self, phase: MissionPhase, target: str = "") -> None:
        self.phase = phase
        if target:
            self.current_target = target
        message = phase.value
        if self.current_target:
            message += f":{self.current_target}"
        self.status_pub.publish(String(data=message))
        self.get_logger().info(f"Mission phase: {message}")

    def _publish_feedback(self, goal_handle) -> None:
        _, _, distance = self._snapshot()
        feedback = ExecuteDelivery.Feedback()
        feedback.phase = self.phase.value
        feedback.current_target = self.current_target
        feedback.distance_remaining = float(distance)
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _wait_future(future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done()

    def _wait_interfaces(self, timeout: float = 30.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self.fly_client.server_is_ready()
                and self.takeoff_client.service_is_ready()
                and self.land_client.service_is_ready()
            ):
                return True
            self.fly_client.wait_for_server(timeout_sec=0.2)
            self.takeoff_client.wait_for_service(timeout_sec=0.2)
            self.land_client.wait_for_service(timeout_sec=0.2)
        return False

    def _call_trigger(self, client, timeout: float = 10.0):
        future = client.call_async(Trigger.Request())
        if not self._wait_future(future, timeout):
            return False, "Service call timed out"
        response = future.result()
        return bool(response.success), str(response.message)

    def _wait_vehicle_state(self, expected: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            _, state, _ = self._snapshot()
            if state == expected:
                return True
            time.sleep(0.05)
        return False

    def _wait_landed(
        self, expected_altitude: float = 0.0, timeout: float = 30.0
    ) -> bool:
        deadline = time.monotonic() + timeout
        settled_since = None
        last_state = None
        last_altitude = float("inf")
        last_speed = float("inf")
        while rclpy.ok() and time.monotonic() < deadline:
            odom, state, _ = self._snapshot()
            last_state = state
            if odom is None:
                time.sleep(0.05)
                continue

            last_altitude = float(odom.pose.pose.position.z)
            velocity = odom.twist.twist.linear
            last_speed = math.sqrt(
                velocity.x * velocity.x
                + velocity.y * velocity.y
                + velocity.z * velocity.z
            )
            if state == LANDED and last_altitude <= expected_altitude + 0.25:
                return True

            now = time.monotonic()
            if is_settled_at_altitude(
                last_altitude, last_speed, expected_altitude
            ):
                if settled_since is None:
                    settled_since = now
                elif now - settled_since >= 1.0:
                    self.get_logger().warn(
                        "Accepting settled ground contact without a fresh LANDED state"
                    )
                    return True
            else:
                settled_since = None
            time.sleep(0.05)
        self.get_logger().error(
            "Landing confirmation timed out "
            f"(state={last_state}, altitude={last_altitude:.3f}, "
            f"speed={last_speed:.3f})"
        )
        return False

    def _configure_landing_height(self, landing_height: float) -> None:
        message = Float64(data=landing_height)
        for _ in range(3):
            self.landing_height_pub.publish(message)
            time.sleep(0.05)

    def _wait_sim_duration(self, duration: float, goal_handle) -> bool:
        start = self.get_clock().now().nanoseconds
        wall_deadline = time.monotonic() + max(duration * 5.0, duration + 10.0)
        while rclpy.ok() and time.monotonic() < wall_deadline:
            if goal_handle.is_cancel_requested:
                return False
            elapsed = (self.get_clock().now().nanoseconds - start) / 1e9
            self._publish_feedback(goal_handle)
            if elapsed >= duration:
                return True
            time.sleep(0.1)
        return False

    def _fly_feedback_callback(self, message) -> None:
        with self.lock:
            self.nested_distance = float(message.feedback.distance_remaining)

    @staticmethod
    def _pose(waypoint: Waypoint, altitude: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint.x
        pose.pose.position.y = waypoint.y
        pose.pose.position.z = altitude
        half_yaw = waypoint.yaw / 2.0
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def _fly_to(
        self,
        waypoint,
        altitude,
        goal_handle,
        allow_platform_proximity=False,
    ):
        settings = self.waypoint_map.flight
        goal = FlyToPose.Goal()
        goal.target = self._pose(waypoint, altitude)
        goal.target.header.stamp = self.get_clock().now().to_msg()
        goal.position_tolerance = float(settings["position_tolerance"])
        goal.timeout = float(settings["segment_timeout"])
        goal.allow_platform_proximity = bool(allow_platform_proximity)

        send_future = self.fly_client.send_goal_async(
            goal, feedback_callback=self._fly_feedback_callback
        )
        if not self._wait_future(send_future, 10.0):
            return False, "FlyToPose goal submission timed out"
        nested_goal = send_future.result()
        if not nested_goal.accepted:
            return False, "FlyToPose goal was rejected"
        self.active_fly_goal = nested_goal
        result_future = nested_goal.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                nested_goal.cancel_goal_async()
                return False, "Delivery mission canceled"
            self._publish_feedback(goal_handle)
            time.sleep(0.1)
        self.active_fly_goal = None
        if not result_future.done():
            return False, "ROS shutdown during flight"
        result = result_future.result().result
        return bool(result.success), str(result.message)

    def _fly_route(self, start, destination, altitude, goal_handle):
        route = self.waypoint_map.plan_route(start, destination)
        planned_path = Path()
        planned_path.header.frame_id = "map"
        planned_path.header.stamp = self.get_clock().now().to_msg()
        route_nodes = [self.waypoint_map.corridor_nodes[start], *route]
        for node in route_nodes:
            pose = self._pose(node, altitude)
            pose.header.stamp = planned_path.header.stamp
            planned_path.poses.append(pose)
        self.planned_path_pub.publish(planned_path)
        route_names = " -> ".join([start, *(node.name for node in route)])
        self.get_logger().info(f"UAV air corridor route: {route_names}")
        for node in route:
            success, message = self._fly_to(node, altitude, goal_handle)
            if not success:
                return False, f"segment to '{node.name}' failed: {message}"
        return True, "Air corridor route completed"

    def _publish_direct_path(
        self,
        start: Waypoint,
        destination: Waypoint,
        start_altitude: float,
        destination_altitude: float,
    ) -> None:
        planned_path = Path()
        planned_path.header.frame_id = "map"
        planned_path.header.stamp = self.get_clock().now().to_msg()
        for waypoint, altitude in (
            (start, start_altitude),
            (destination, destination_altitude),
        ):
            pose = self._pose(waypoint, altitude)
            pose.header.stamp = planned_path.header.stamp
            planned_path.poses.append(pose)
        self.planned_path_pub.publish(planned_path)
        self.get_logger().info(
            "UAV local direct route: "
            f"{start.name} ({start_altitude:.2f} m) -> "
            f"{destination.name} ({destination_altitude:.2f} m)"
        )

    def _fail(self, goal_handle, result, message):
        self._set_phase(MissionPhase.FAILED)
        result.success = False
        result.message = message
        goal_handle.abort()
        self.get_logger().error(message)
        return result

    def _execute_delivery(self, goal_handle):
        result = ExecuteDelivery.Result()
        result.completed_targets = 0
        try:
            mission = self._resolve_optimized_delivery(
                goal_handle.request.targets,
                goal_handle.request.target_floors,
                goal_handle.request.payload_masses_kg,
                goal_handle.request.home_name,
                bool(goal_handle.request.return_home),
            )
        except (ValueError, WaypointConfigurationError) as error:
            return self._fail(goal_handle, result, str(error))
        targets = mission.targets
        home = mission.home
        payload_masses = mission.payload_masses
        self._publish_optimized_route(mission)
        if not self._wait_interfaces():
            return self._fail(goal_handle, result, "UAV control interfaces are unavailable")

        assessment, preflight_message = self._assess_delivery_energy(
            mission.target_names,
            goal_handle.request.return_home,
            goal_handle.request.home_name,
            float(goal_handle.request.landing_height),
            payload_masses,
            mission.target_floors,
        )
        self.energy_preflight_pub.publish(String(data=preflight_message))
        if assessment is None or not assessment.feasible:
            return self._fail(goal_handle, result, preflight_message)
        self.get_logger().info(preflight_message)

        settings = self.waypoint_map.flight
        local_delivery = uses_local_delivery_profile(
            str(goal_handle.request.home_name)
        )
        delivery_altitudes = [
            self.waypoint_map.delivery_altitude_for(target)
            for target in targets
        ]
        takeoff_altitude = (
            delivery_altitudes[0]
            if local_delivery
            else float(settings["takeoff_altitude"])
        )
        landing_height = float(goal_handle.request.landing_height)
        remaining_payload = sum(payload_masses)
        self._publish_payload_mass(remaining_payload)
        self._configure_landing_height(landing_height)
        route_position = home.name
        self.current_target = home.name
        self._set_phase(MissionPhase.TAKEOFF, home.name)
        _, vehicle_state, _ = self._snapshot()
        if vehicle_state == FLYING:
            success, message = True, "UAV already released for takeoff"
        else:
            success, message = self._call_trigger(self.takeoff_client)
        if not success or not self._wait_vehicle_state(FLYING, 15.0):
            return self._fail(goal_handle, result, f"Takeoff failed: {message}")

        success, message = self._fly_to(
            home,
            takeoff_altitude,
            goal_handle,
            allow_platform_proximity=True,
        )
        if not success:
            return self._fail(goal_handle, result, f"Takeoff climb failed: {message}")

        self._set_phase(MissionPhase.HOVER, home.name)
        if not self._wait_sim_duration(
            float(settings["takeoff_hover_duration"]), goal_handle
        ):
            return self._fail(goal_handle, result, "Takeoff hover interrupted")

        if local_delivery:
            self.get_logger().info(
                "Using cooperative local-delivery profile at the requested "
                f"floor altitude ({takeoff_altitude:.2f} m)"
            )
        else:
            self._set_phase(MissionPhase.CRUISE, home.name)
            success, message = self._fly_to(
                home, float(settings["cruise_altitude"]), goal_handle
            )
            if not success:
                return self._fail(
                    goal_handle, result, f"Cruise climb failed: {message}"
                )

        for index, target in enumerate(targets):
            delivery_altitude = delivery_altitudes[index]
            self._set_phase(MissionPhase.CRUISE, target.name)
            if local_delivery:
                route_start = self.waypoint_map.corridor_nodes[route_position]
                start_altitude = (
                    takeoff_altitude
                    if index == 0
                    else delivery_altitudes[index - 1]
                )
                self._publish_direct_path(
                    route_start,
                    target,
                    start_altitude,
                    delivery_altitude,
                )
            else:
                success, message = self._fly_route(
                    route_position,
                    target.name,
                    float(settings["cruise_altitude"]),
                    goal_handle,
                )
                if not success:
                    return self._fail(
                        goal_handle, result, f"Cruise failed: {message}"
                    )
                self._set_phase(MissionPhase.APPROACH, target.name)

            floor_text = (
                f"floor {target.delivery_floor}"
                if target.delivery_floor is not None
                else "configured delivery level"
            )
            self.get_logger().info(
                f"Approaching {target.name} {floor_text} at "
                f"({target.x:.2f}, {target.y:.2f}, {delivery_altitude:.2f})"
            )
            success, message = self._fly_to(
                target, delivery_altitude, goal_handle
            )
            if not success:
                return self._fail(goal_handle, result, f"Approach failed: {message}")
            route_position = target.name

            self._set_phase(MissionPhase.DELIVERING, target.name)
            if not self._wait_sim_duration(
                float(settings["delivery_hover_duration"]), goal_handle
            ):
                return self._fail(goal_handle, result, "Delivery hover interrupted")
            self.delivery_event_pub.publish(String(data=f"DELIVERED:{target.name}"))
            remaining_payload = max(
                0.0, remaining_payload - payload_masses[index]
            )
            self._publish_payload_mass(remaining_payload)
            result.completed_targets = index + 1

            if (
                not local_delivery
                and (index < len(targets) - 1 or goal_handle.request.return_home)
            ):
                self._set_phase(MissionPhase.CRUISE, target.name)
                success, message = self._fly_to(
                    target, float(settings["cruise_altitude"]), goal_handle
                )
                if not success:
                    return self._fail(
                        goal_handle,
                        result,
                        f"Post-delivery climb failed: {message}",
                    )

        if goal_handle.request.return_home:
            self._set_phase(MissionPhase.RETURNING, home.name)
            if local_delivery:
                route_start = self.waypoint_map.corridor_nodes[route_position]
                return_altitude = delivery_altitudes[-1]
                self._publish_direct_path(
                    route_start,
                    home,
                    return_altitude,
                    return_altitude,
                )
                success, message = self._fly_to(
                    home,
                    return_altitude,
                    goal_handle,
                    allow_platform_proximity=True,
                )
            else:
                success, message = self._fly_route(
                    route_position,
                    home.name,
                    float(settings["cruise_altitude"]),
                    goal_handle,
                )
            if not success:
                return self._fail(goal_handle, result, f"Return flight failed: {message}")
            success, message = self._fly_to(
                home,
                float(settings["landing_approach_altitude"]),
                goal_handle,
                allow_platform_proximity=True,
            )
            if not success:
                return self._fail(goal_handle, result, f"Landing approach failed: {message}")

            self._set_phase(MissionPhase.LANDING, home.name)
            success, message = self._call_trigger(self.land_client)
            if not success or not self._wait_landed(landing_height):
                return self._fail(goal_handle, result, f"Landing failed: {message}")
            self._publish_payload_mass(0.0)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "Delivery mission canceled"
            return result

        self._set_phase(MissionPhase.COMPLETED)
        result.success = True
        result.message = "UAV delivery mission completed"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryMissionManager()
    executor = MultiThreadedExecutor(num_threads=5)
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
