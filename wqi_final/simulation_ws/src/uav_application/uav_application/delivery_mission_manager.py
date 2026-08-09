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
from std_msgs.msg import Bool, Float32, Float64, Int8, Int32, String
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
    is_same_delivery_pad,
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
        self.declare_parameter("energy_constraints_enabled", True)
        self.energy_constraints_enabled = bool(
            self.get_parameter("energy_constraints_enabled").value
        )
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
        self.replan_count = 0
        self.recovery_home = None
        self.recovery_landing_height = 0.0

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
        self.replanned_path_pub = self.create_publisher(
            Path, "replanned_path", transient_qos
        )
        self.blocked_edge_pub = self.create_publisher(
            String, "blocked_edge", transient_qos
        )
        self.replan_count_pub = self.create_publisher(
            Int32, "replan_count", transient_qos
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
        self.can_execute_pub.publish(
            Bool(data=not self.energy_constraints_enabled)
        )
        energy_state = (
            "enabled" if self.energy_constraints_enabled else "disabled"
        )
        self.get_logger().info(
            f"UAV delivery mission manager is ready; energy constraints "
            f"{energy_state}"
        )

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
        if not self.energy_constraints_enabled:
            message = "Energy constraints disabled for this simulation stage"
            self.energy_preflight_pub.publish(String(data=message))
            self.can_execute_pub.publish(Bool(data=True))
            self.get_logger().info(message)
            return GoalResponse.ACCEPT
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
            if not self.energy_constraints_enabled:
                message = "Energy constraints disabled for this simulation stage"
                response.feasible = True
                response.message = message
                response.current_soc = 1.0
                response.current_energy_wh = float(
                    self.battery_parameters.capacity_wh
                )
                response.battery_capacity_wh = float(
                    self.battery_parameters.capacity_wh
                )
                response.estimated_final_soc = 1.0
                response.net_charge_power_w = float(
                    self.battery_parameters.net_charge_power_w
                )
                self.energy_preflight_pub.publish(String(data=message))
                self.can_execute_pub.publish(Bool(data=True))
                return response
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
        timeout_override=None,
        honor_mission_cancel=True,
    ):
        settings = self.waypoint_map.flight
        goal = FlyToPose.Goal()
        goal.target = self._pose(waypoint, altitude)
        goal.target.header.stamp = self.get_clock().now().to_msg()
        goal.position_tolerance = float(settings["position_tolerance"])
        goal.timeout = float(
            timeout_override
            if timeout_override is not None
            else settings["segment_timeout"]
        )
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
            if honor_mission_cancel and goal_handle.is_cancel_requested:
                cancel_future = nested_goal.cancel_goal_async()
                self._wait_future(cancel_future, 3.0)
                self.active_fly_goal = None
                return False, "Delivery mission canceled"
            self._publish_feedback(goal_handle)
            time.sleep(0.1)
        self.active_fly_goal = None
        if not result_future.done():
            return False, "ROS shutdown during flight"
        result = result_future.result().result
        return bool(result.success), str(result.message)

    @staticmethod
    def _is_obstacle_failure(message: str) -> bool:
        text = str(message).lower()
        return "obstacle did not clear" in text or "blocked:" in text

    def _publish_path(self, points, publisher=None) -> None:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for waypoint, altitude in points:
            pose = self._pose(waypoint, altitude)
            pose.header.stamp = path.header.stamp
            path.poses.append(pose)
        (publisher or self.planned_path_pub).publish(path)

    def _record_replan(self, reason: str) -> None:
        self.replan_count += 1
        self.replan_count_pub.publish(Int32(data=self.replan_count))
        self.blocked_edge_pub.publish(String(data=str(reason)))
        self.get_logger().warning(
            f"UAV replanning event {self.replan_count}: {reason}"
        )

    def _fly_with_local_replan(
        self,
        waypoint,
        altitude,
        goal_handle,
        allow_platform_proximity=False,
    ):
        # The flight controller follows a continuously replanned 3D path from
        # live lidar data. The mission layer must not inject scripted
        # left/right/up/down escape motions.
        return self._fly_to(
            waypoint,
            altitude,
            goal_handle,
            allow_platform_proximity=allow_platform_proximity,
        )

    def _fly_route(self, start, destination, altitude, goal_handle):
        current = str(start)
        blocked_edges = set()
        maximum = int(
            self.waypoint_map.flight.get("maximum_route_replans", 3)
        )
        route_replans = 0
        while current != destination:
            route = self.waypoint_map.plan_route(
                current, destination, blocked_edges=blocked_edges
            )
            if not route:
                break
            current_node = self.waypoint_map.corridor_nodes[current]
            self._publish_path(
                tuple(
                    (node, altitude)
                    for node in (current_node, *route)
                )
            )
            route_names = " -> ".join(
                [current, *(node.name for node in route)]
            )
            self.get_logger().info(f"UAV air corridor route: {route_names}")
            node = route[0]
            success, message = self._fly_to(node, altitude, goal_handle)
            if success:
                current = node.name
                continue
            if not self._is_obstacle_failure(message):
                return False, f"segment to '{node.name}' failed: {message}"
            edge = self.waypoint_map.normalized_edge(current, node.name)
            if route_replans >= maximum:
                return self._fly_with_local_replan(
                    node, altitude, goal_handle
                )
            blocked_edges.add(edge)
            route_replans += 1
            self._record_replan(f"EDGE:{edge[0]}<->{edge[1]}")
            returned, return_message = self._fly_to(
                current_node, altitude, goal_handle
            )
            if not returned:
                return False, (
                    f"Could not return to corridor node '{current}' after "
                    f"blockage: {return_message}"
                )
            try:
                self.waypoint_map.plan_route(
                    current, destination, blocked_edges=blocked_edges
                )
            except WaypointConfigurationError:
                blocked_edges.remove(edge)
                local_success, local_message = self._fly_with_local_replan(
                    node, altitude, goal_handle
                )
                if not local_success:
                    return False, local_message
                current = node.name
        return True, "Air corridor route completed"

    def _publish_direct_path(
        self,
        start: Waypoint,
        destination: Waypoint,
        start_altitude: float,
        destination_altitude: float,
    ) -> None:
        self._publish_path((
            (start, start_altitude),
            (destination, destination_altitude),
        ))
        self.get_logger().info(
            "UAV local direct route: "
            f"{start.name} ({start_altitude:.2f} m) -> "
            f"{destination.name} ({destination_altitude:.2f} m)"
        )

    def _recover_after_failure(self, goal_handle) -> str:
        home = self.recovery_home
        landing_height = float(self.recovery_landing_height)
        odom, vehicle_state, _ = self._snapshot()
        altitude = (
            float(odom.pose.pose.position.z) if odom is not None else 0.0
        )
        airborne = (
            vehicle_state != LANDED
            if vehicle_state is not None
            else altitude > landing_height + 0.25
        )
        airborne = airborne or altitude > landing_height + 0.35
        if home is None or not airborne:
            return "UAV was already on the ground"

        self.get_logger().error(
            "Mission failed while airborne; starting automatic return and landing"
        )
        if self.active_fly_goal is not None:
            cancel_future = self.active_fly_goal.cancel_goal_async()
            self._wait_future(cancel_future, 3.0)
            self.active_fly_goal = None

        settings = self.waypoint_map.flight
        approach_altitude = max(
            landing_height + 0.8,
            float(settings["landing_approach_altitude"]),
        )
        horizontal_error = (
            math.hypot(
                float(odom.pose.pose.position.x) - float(home.x),
                float(odom.pose.pose.position.y) - float(home.y),
            )
            if odom is not None
            else float("inf")
        )
        recovery_notes = []
        self._set_phase(MissionPhase.RETURNING, home.name)
        if horizontal_error > 0.5:
            return_altitude = max(approach_altitude, altitude)
            returned, return_message = self._fly_to(
                home,
                return_altitude,
                goal_handle,
                allow_platform_proximity=True,
                timeout_override=60.0,
                honor_mission_cancel=False,
            )
            recovery_notes.append(f"return={return_message}")
        else:
            returned = True
            recovery_notes.append("return=already above home")

        if returned:
            approached, approach_message = self._fly_to(
                home,
                approach_altitude,
                goal_handle,
                allow_platform_proximity=True,
                timeout_override=45.0,
                honor_mission_cancel=False,
            )
            recovery_notes.append(f"approach={approach_message}")
        else:
            approached = False
            recovery_notes.append("approach=skipped after return failure")

        self._set_phase(MissionPhase.LANDING, home.name)
        land_accepted, land_message = self._call_trigger(
            self.land_client, timeout=10.0
        )
        landed = land_accepted and self._wait_landed(
            landing_height, timeout=45.0
        )
        recovery_notes.append(f"land={land_message}")
        if landed:
            self._publish_payload_mass(0.0)
            recovery_notes.append("confirmed=landed")
        else:
            recovery_notes.append("confirmed=failed")
            self.get_logger().error(
                "Automatic landing recovery could not confirm ground contact"
            )
        if not approached:
            recovery_notes.append("warning=landed at current safe position")
        return ", ".join(recovery_notes)

    def _fail(self, goal_handle, result, message):
        recovery_message = self._recover_after_failure(goal_handle)
        self._set_phase(MissionPhase.FAILED)
        result.success = False
        result.message = f"{message}; recovery: {recovery_message}"
        goal_handle.abort()
        self.get_logger().error(result.message)
        self.recovery_home = None
        return result

    def _execute_delivery(self, goal_handle):
        result = ExecuteDelivery.Result()
        result.completed_targets = 0
        self.replan_count = 0
        self.recovery_home = None
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

        if self.energy_constraints_enabled:
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
        else:
            self.energy_preflight_pub.publish(String(
                data="Energy constraints disabled for this simulation stage"
            ))

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
        self.recovery_home = home
        self.recovery_landing_height = landing_height
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
            platform_approach = local_delivery and is_same_delivery_pad(
                home.x,
                home.y,
                target.x,
                target.y,
            )
            success, message = self._fly_with_local_replan(
                target,
                delivery_altitude,
                goal_handle,
                allow_platform_proximity=platform_approach,
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
                success, message = self._fly_with_local_replan(
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
        self.recovery_home = None
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
