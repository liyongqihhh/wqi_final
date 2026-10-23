"""Two-car relay orchestration; reuse the proven single-car navigation helpers."""

from dataclasses import asdict, replace
from enum import Enum
import math
import json
from pathlib import Path
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Float64, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from uav_interfaces.action import FlyToPose
from uav_interfaces.srv import SelectCarrier
from cooperative_delivery_interfaces.action import ExecuteCooperativeDelivery
from uav_control.battery_model import BatteryModel
from uav_application.mission_energy import MissionEnergyPlanner
from uav_navigation.route_optimizer import optimize_visit_order
from uav_navigation.waypoint_navigator import WaypointMap

from cooperative_delivery.cooperative_mission_manager import CooperativeMissionManager
from cooperative_delivery.mission_states import CooperativePhase
from cooperative_delivery.relay_plan import (
    CaptureSettleWindow, CaptureVelocityFilter, RelayConfig, can_move_carrier,
    capture_pose_is_safe, partition_payloads,
)
from cooperative_delivery.relay_energy import DriveLeg, FlightLeg, evaluate_relay_energy
from cooperative_delivery.ugv_energy_model import estimate_ugv_drive_energy


class RelayPhase(str, Enum):
    PREPARING = "RELAY_PREPARING"
    A_TO_PLATFORM = "A_TO_PLATFORM"
    B_TO_PLATFORM = "B_TO_PLATFORM"
    CROSS_REGION_OUTBOUND = "CROSS_REGION_OUTBOUND"
    B_TO_BUILDING = "B_TO_BUILDING"
    FLOOR_DELIVERY = "FLOOR_DELIVERY"
    B_RETURN_PLATFORM = "B_RETURN_PLATFORM"
    CROSS_REGION_RETURN = "CROSS_REGION_RETURN"
    A_RETURN_HUB = "A_RETURN_HUB"


class RelayMissionManager(CooperativeMissionManager):
    def __init__(self):
        super().__init__()
        share = Path(get_package_share_directory("cooperative_delivery"))
        self.declare_parameter("relay_config", str(share / "config/relay_delivery.yaml"))
        self.waypoints = WaypointMap(Path(get_package_share_directory("uav_navigation")) /
                                     "config/uav_delivery_waypoints.yaml")
        self.relay = RelayConfig.load(self.get_parameter("relay_config").value, self.waypoints)
        self.navigation_role = "a"
        self.home_role = "a"
        self.carrier_role = ""
        self.poses = {"a": None, "b": None}
        self.pose_received = {"a": 0.0, "b": 0.0}
        self.wheel = {"a": None, "b": None}
        self.energy = {"a": [None, None], "b": [None, None]}
        self.chargers = {"a": False, "b": False}
        self.cargo = {"a": 0.0, "b": 0.0}
        self.uav_odom = None
        self.uav_odom_received = 0.0
        self.uav_soc = None
        self.landing_fault = ""
        self.vehicle_clients = {"a": (self.nav_client, self.clear_local_client, self.clear_global_client)}
        group = self.callback_group
        self.vehicle_clients["b"] = (
            ActionClient(self, NavigateToPose, "/ugv_b/navigate_to_pose", callback_group=group),
            self.create_client(ClearEntireCostmap,
                "/ugv_b/local_costmap/clear_entirely_local_costmap", callback_group=group),
            self.create_client(ClearEntireCostmap,
                "/ugv_b/global_costmap/clear_entirely_global_costmap", callback_group=group),
        )
        self.fly_client = ActionClient(self, FlyToPose, "/uav/fly_to_pose", callback_group=group)
        self.land_client = self.create_client(Trigger, "/uav/land", callback_group=group)
        self.select_client = self.create_client(SelectCarrier, "/uav/select_carrier", callback_group=group)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Odometry, "/ugv_b/ground_truth/odom",
            lambda msg: self._truth("b", msg), 20, callback_group=group)
        self.create_subscription(Odometry, "/ugv_b/odom",
            lambda msg: self._wheel("b", msg), 20, callback_group=group)
        self.create_subscription(Odometry, "/uav/odom", self._uav_pose, 20, callback_group=group)
        self.create_subscription(String, "/uav/docked_carrier", self._carrier, qos, callback_group=group)
        self.create_subscription(String, "/uav/px4/landing_fault",
            lambda msg: setattr(self, "landing_fault", msg.data), 10, callback_group=group)
        self.create_subscription(BatteryState, "/uav/battery_state",
            lambda msg: setattr(self, "uav_soc", float(msg.percentage)), qos, callback_group=group)
        self.cargo_pubs, self.dock_pubs, self.stop_pubs = {}, {}, {}
        for role in ("a", "b"):
            ns = f"/ugv_{role}"
            self.cargo_pubs[role] = self.create_publisher(Float32, ns + "/cargo_mass_kg", qos)
            self.dock_pubs[role] = self.create_publisher(Bool, ns + "/uav_docked", qos)
            self.stop_pubs[role] = self.create_publisher(Twist,
                "/cmd_vel_safe" if role == "a" else ns + "/cmd_vel_safe", 10)
            for index, name in enumerate(("drive_remaining_wh", "charging_remaining_wh")):
                self.create_subscription(Float32, ns + "/" + name,
                    lambda msg, r=role, i=index: self._energy(r, i, msg.data), qos, callback_group=group)
            self.create_subscription(Bool, ns + "/charger_available",
                lambda msg, r=role: self.chargers.__setitem__(r, bool(msg.data)), qos, callback_group=group)
        self.charger_pub = self.create_publisher(Bool, "/ugv/charger_available", qos)
        self.payload_pub = self.create_publisher(Float32, "/uav/payload_mass", qos)
        self.flight_phase_pub = self.create_publisher(String, "/uav/mission_status", qos)
        self.height_pub = self.create_publisher(Float64, "/uav/landing_height", 10)
        self.path_pub = self.create_publisher(NavPath, "/uav/planned_path", qos)
        self.markers_pub = self.create_publisher(MarkerArray, "markers", qos)
        self.carrier_state_pub = self.create_publisher(String, "carrier_state", qos)
        self.readiness_pub = self.create_publisher(String, "readiness", qos)
        self.last_readiness_log = 0.0
        self.lifecycle_probes = {}
        for namespace in ("", "/ugv_b"):
            for name in ("controller_server", "bt_navigator"):
                topic = namespace + "/" + name + "/get_state"
                self.lifecycle_probes[topic] = [self.create_client(
                    GetState, topic, callback_group=group), None, False, 0.0]
        self.create_timer(.2, self._bridge, callback_group=group)
        self.get_logger().info("Two-UGV relay ready: A -> UAV -> B -> floor -> B -> UAV -> A")

    def _ugv_ground_truth_callback(self, message):
        self._truth("a", message)

    def _ugv_odom_callback(self, message):
        self._wheel("a", message)

    def _truth(self, role, message):
        with self.lock:
            self.poses[role] = message
            self.pose_received[role] = time.monotonic()

    def _wheel(self, role, message):
        with self.lock:
            self.wheel[role] = message

    def _uav_pose(self, message):
        with self.lock:
            self.uav_odom = message
            self.uav_odom_received = time.monotonic()

    def _carrier(self, message):
        self.carrier_role = {"ugvcar": "a", "ugv_b": "b"}.get(message.data, "")

    def _energy(self, role, index, value):
        if math.isfinite(value) and value >= 0:
            self.energy[role][index] = float(value)

    def _physical_ugv_odom(self):
        with self.lock:
            return self.poses[self.navigation_role]

    def _snapshot(self):
        with self.lock:
            return self.wheel[self.navigation_role], self.uav_state, self.docked

    def _ugv_energy_snapshot(self):
        return tuple(self.energy[self.navigation_role])

    def _bridge(self):
        # Route charger permission by confirmed joint ownership, never by target.
        for role in ("a", "b"):
            self.dock_pubs[role].publish(Bool(data=self.docked and self.carrier_role == role))
        available = self.docked and self.chargers.get(self.carrier_role, False)
        self.charger_pub.publish(Bool(data=bool(available)))
        role = self.home_role
        if self._fresh(role):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose = self.poses[role].pose.pose
            self.carrier_home_pub.publish(pose)
        self.carrier_state_pub.publish(String(data=(
            f"docked={self.carrier_role or 'NONE'};landing_target={role};"
            f"cargo_a_kg={self.cargo['a']:.3f};cargo_b_kg={self.cargo['b']:.3f}")))
        markers = MarkerArray()
        for index, role in enumerate(("a", "b")):
            stop = self.relay.platform(role).stop
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type = "relay_platforms", index, Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = stop.x, stop.y, 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.z = .75
            marker.color.a = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0 if role == "a" else .2
            marker.text = f"P{index+1} / UGV {role.upper()}\n{self.phase.value}"
            markers.markers.append(marker)
        self.markers_pub.publish(markers)

    def _fresh(self, role):
        return (self.poses[role] is not None and time.monotonic() - self.pose_received[role]
                <= self.relay.telemetry_timeout_wall_s)

    def _interfaces_ready(self):
        checks = {f"pose_{r}": self._fresh(r) for r in ("a", "b")}
        checks.update({f"nav_{r}": clients[0].server_is_ready()
                       for r, clients in self.vehicle_clients.items()})
        checks.update(uav_delivery=self.uav_client.server_is_ready(),
                      uav_flight=self.fly_client.server_is_ready(), landed=self.uav_state == 0)
        for client in (self.select_client, self.land_client, self.takeoff_client,
                       self.attach_client, self.detach_client,
                       *[c for clients in self.vehicle_clients.values() for c in clients[1:]]):
            checks[client.srv_name] = client.service_is_ready()
        for topic, probe in self.lifecycle_probes.items():
            client, future, active, last = probe
            if future is not None and future.done():
                try:
                    active = future.result().current_state.id == 3
                except Exception:
                    active = False
                future = None
            if future is None and client.service_is_ready() and time.monotonic() - last >= 1.0:
                future, last = client.call_async(GetState.Request()), time.monotonic()
            probe[:] = [client, future, active, last]
            checks[topic] = active
        missing = [name for name, ready in checks.items() if not ready]
        message = "READY" if not missing else "WAITING: " + ", ".join(missing)
        self.readiness_pub.publish(String(data=message))
        if missing and time.monotonic() - self.last_readiness_log >= 10:
            self.get_logger().info(message)
            self.last_readiness_log = time.monotonic()
        return not missing

    def _cargo(self, role, mass):
        self.cargo[role] = max(0.0, mass)
        self.cargo_pubs[role].publish(Float32(data=float(self.cargo[role])))

    def _goal_callback(self, request):
        try:
            if not request.return_home:
                raise ValueError("Relay missions require return_home=true for the complete return chain")
            targets = self._resolve_request_targets(request)
            if not 1 <= len(targets) <= 10:
                raise ValueError("Relay missions support 1 to 10 delivery items")
            self.waypoints.resolve_delivery_targets([t.uav_target for t in targets],
                [t.delivery_floor or self.waypoints.waypoints[t.uav_target].delivery_floor or 1
                 for t in targets])
            partition_payloads(targets, self.battery_parameters.maximum_payload_mass_kg)
        except ValueError as error:
            self.get_logger().error(str(error))
            return GoalResponse.REJECT
        return super()._goal_callback(request)

    def _wait(self, predicate, goal, timeout=180.0):
        deadline = time.monotonic() + timeout
        last_clock = self.get_clock().now().nanoseconds
        clock_progress = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            if goal.is_cancel_requested:
                raise RuntimeError("Relay mission canceled")
            if predicate():
                return True
            clock = self.get_clock().now().nanoseconds
            if clock < last_clock:
                raise RuntimeError("Simulation clock reset during relay mission")
            if clock > last_clock:
                last_clock, clock_progress = clock, time.monotonic()
            if time.monotonic() - clock_progress > 30:
                raise RuntimeError("Simulation clock stopped for 30 wall seconds")
            self._publish_feedback(goal)
            time.sleep(.05)
        return False

    @staticmethod
    def _require(ok, message):
        if not ok:
            raise RuntimeError(message)

    def _role(self, role):
        self.navigation_role = role
        self.nav_client, self.clear_local_client, self.clear_global_client = self.vehicle_clients[role]

    def _publish_feedback(self, goal):
        # Preserve parent helpers but label the active ground vehicle explicitly.
        now = time.monotonic()
        if now - self.last_feedback_at < self.feedback_period:
            return
        self.last_feedback_at = now
        feedback = ExecuteCooperativeDelivery.Feedback()
        feedback.phase = self.phase.value
        feedback.current_target = self.current_target
        feedback.active_vehicle = ("UGV_" + self.navigation_role.upper()
                                   if self.active_vehicle == "UGV" else self.active_vehicle)
        feedback.uav_docked = self.docked
        feedback.distance_remaining = float(self.nav_distance if self.active_vehicle == "UGV"
                                            else self.uav_distance)
        goal.publish_feedback(feedback)

    def _distance(self, a, b):
        if self.road_network is None:
            raise RuntimeError("Relay routing requires the campus road graph")
        return self.road_network.distance((a.x, a.y), (b.x, b.y))

    def _ordered(self, targets):
        start = self.relay.platform_b.stop
        plan = optimize_visit_order(len(targets),
            lambda i: self._distance(start, targets[i].ugv_launch),
            lambda i, j: self._distance(targets[i].ugv_launch, targets[j].ugv_launch),
            lambda i: self._distance(targets[i].ugv_launch, start))
        return [targets[i] for i in plan.order]

    def _drive(self, role, stop, phase, goal):
        self._role(role)
        self._require(self._fresh(role), f"UGV {role} ground-truth telemetry is stale")
        self._require(can_move_carrier(role, self.carrier_role, self.uav_state == 0),
                      "A carrier cannot drive before UAV rotors stop")
        if self.carrier_role == role:
            self._require(self.docked, "Carrier joint is not confirmed")
        self._set_phase(phase, stop.name, "UGV")
        if self.energy_constraints_enabled:
            p = self.poses[role].pose.pose.position
            origin = replace(stop, x=p.x, y=p.y)
            available = self.energy[role][0]
            self._require(available is not None, f"UGV {role} battery is unavailable")
            safe_stop = self.config.ugv_home if role == "a" else self.relay.platform_b.stop
            estimate = estimate_ugv_drive_energy(self.ugv_energy_parameters, available,
                [self._distance(origin, stop)], [self.cargo[role]], self._distance(stop, safe_stop))
            self._require(estimate.feasible, f"UGV {role} lacks drive energy including return reserve")
        self._require(*self._navigate_with_recovery(stop, goal))
        self._require(self._wait_ugv_settled(goal), f"UGV {role} did not settle at {stop.name}")
        self.stop_pubs[role].publish(Twist())

    def _select(self, role):
        request = SelectCarrier.Request()
        request.model_name = "ugvcar" if role == "a" else "ugv_b"
        request.link_name = "base_footprint" if role == "a" else "ugv_b/base_footprint"
        future = self.select_client.call_async(request)
        self._require(self._wait_future(future, 15), "Carrier selection timed out")
        response = future.result()
        self._require(response is not None and response.success,
                      response.message if response else "Carrier selection failed")
        self.home_role = role
        self._bridge()

    def _dock(self, role, goal):
        self._role(role)
        self._require(self._fresh(role), "Receiver telemetry is stale")
        self._require(self._wait_ugv_settled(goal), "Receiver moved during landing")
        self._require(self.uav_state == 0, "PX4 has not confirmed landing/disarming")
        for attempt in range(3):
            self._require(self._wait_capture_ready(role, goal),
                          f"UAV and car {role} did not settle inside the capture envelope")
            ok, message = self._call_docking_service(self.attach_client, True)
            if ok:
                break
            # A fresh Gazebo physics sample can invalidate a ROS-side check.
            # Retry only this transient guard; never bypass plugin admission.
            self._require(message.startswith("Both vehicles must be settled") and attempt < 2, message)
            self.get_logger().warning("Capture deferred by physics; waiting for a new stable interval")
        self._require(self._wait(lambda: self.carrier_role == role, goal, 15),
                      "UAV joint ownership did not match receiving car")
        self.home_role = role
        self.payload_pub.publish(Float32(data=0.0))
        self._require(self._clear_costmaps(), "Failed to refresh receiver costmaps")

    def _wait_capture_ready(self, role, goal):
        window = CaptureSettleWindow()
        velocity_filter = CaptureVelocityFilter()
        def ready():
            self._require(self.uav_state == 0, "UAV motors restarted before capture")
            self._require(not self.landing_fault, f"PX4 landing fault: {self.landing_fault}")
            fresh = (self._fresh(role) and self.uav_odom is not None
                and time.monotonic() - self.uav_odom_received <= self.relay.telemetry_timeout_wall_s)
            filtered = None
            if fresh:
                car, aircraft = self.poses[role], self.uav_odom
                velocities = []
                for odom in (car, aircraft):
                    for vector in (odom.twist.twist.linear, odom.twist.twist.angular):
                        velocities.extend((vector.x, vector.y, vector.z))
                stamp = min(odom.header.stamp.sec + odom.header.stamp.nanosec / 1e9
                            for odom in (car, aircraft))
                filtered = velocity_filter.observe(stamp, velocities)
            safe = filtered is not None and capture_pose_is_safe(
                self.poses[role], self.uav_odom, self.relay.landing_height, filtered)
            return window.observe(self.get_clock().now().nanoseconds / 1e9, safe)
        return self._wait(ready, goal, 180.)

    def _release(self, role, mass, goal):
        self._role(role)
        self._require(self.docked and self.carrier_role == role and self.uav_state == 0,
                      "UAV must be landed and attached to its departure car")
        self._require(self._wait_ugv_settled(goal), "Departure car is not stationary")
        self.home_role = role
        self._bridge()
        self.payload_pub.publish(Float32(data=float(mass)))
        self._require(*self._call_trigger(self.takeoff_client))
        self._require(*self._call_docking_service(self.detach_client, False))
        self._require(self._wait(lambda: self.uav_state == 1, goal, 180),
                      "PX4 did not complete takeoff")

    def _fly(self, point, goal, platform=False, tolerance=.35):
        request = FlyToPose.Goal()
        request.target.header.frame_id = "map"
        request.target.header.stamp = self.get_clock().now().to_msg()
        request.target.pose.position.x, request.target.pose.position.y, request.target.pose.position.z = point
        request.target.pose.orientation.w = 1.0
        request.position_tolerance = float(tolerance)
        request.timeout = 240.0
        request.allow_platform_proximity = platform
        sent = self.fly_client.send_goal_async(request, feedback_callback=self._uav_feedback_callback)
        self._require(self._wait_future(sent, 20), "FlyToPose goal submission timed out")
        handle = sent.result()
        self._require(handle is not None and handle.accepted, "FlyToPose rejected relay waypoint")
        self.active_uav_goal = handle
        result = handle.get_result_async()
        self._require(self._wait(result.done, goal, self.relay.transfer_timeout_wall_s),
                      "Relay flight waypoint timed out")
        wrapped = result.result()
        self._require(wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success,
                      wrapped.result.message)
        self.active_uav_goal = None

    def _transfer_energy_leg(self, source, destination, mass):
        route = self.relay.air_route(self.waypoints, source, destination)
        parameters = replace(self.battery_parameters,
            estimated_horizontal_speed_mps=min(self.relay.transfer_speed_mps,
                self.battery_parameters.estimated_horizontal_speed_mps),
            estimated_vertical_speed_mps=min(self.relay.transfer_vertical_speed_mps,
                self.battery_parameters.estimated_vertical_speed_mps))
        planner = MissionEnergyPlanner(self.waypoints, parameters)
        profile = planner.plan_transfer(route, self.relay.landing_height, mass,
                                        self.relay.transfer_hover_seconds)
        back = planner.plan_transfer(tuple(reversed(route)), self.relay.landing_height, 0.0,
                                     self.relay.transfer_hover_seconds)
        model = BatteryModel(self.battery_parameters)
        forward = model.estimate(profile)
        required = forward.required_energy_wh + model.estimate(back).estimated_mission_energy_wh
        capacity = self.battery_parameters.capacity_wh
        self._require(required <= capacity, "Transfer plus return reserve exceeds UAV battery capacity")
        target = capacity if self.charge_policy == "full" else min(capacity, required + .02 * capacity)
        return FlightLeg(source, destination, forward.estimated_mission_energy_wh,
            sum(segment.duration_seconds for segment in profile.segments), target)

    def _mission_energy_forecast(self, batches):
        steps = []
        cargo = {"a": sum(t.payload_mass_kg for batch in batches for t in batch), "b": 0.0}
        positions = {role: replace(self.relay.platform(role).stop,
            x=self.poses[role].pose.pose.position.x, y=self.poses[role].pose.pose.position.y)
            for role in ("a", "b")}
        owner = "a"

        def drive(role, destination):
            steps.append(DriveLeg(role, self._distance(positions[role], destination), cargo[role], owner))
            positions[role] = destination

        drive("b", self.relay.platform_b.stop)
        drive("a", self.relay.platform_a.stop)
        for batch in batches:
            mass = sum(t.payload_mass_kg for t in batch)
            steps.append(self._transfer_energy_leg("a", "b", mass))
            cargo["a"] = max(0.0, cargo["a"] - mass)
            cargo["b"], owner = mass, "b"
            for target in batch:
                drive("b", target.ugv_launch)
                steps.append(self._floor_energy_leg(target))
                cargo["b"] = max(0.0, cargo["b"] - target.payload_mass_kg)
            drive("b", self.relay.platform_b.stop)
            steps.append(self._transfer_energy_leg("b", "a", 0.0))
            cargo["b"], owner = 0.0, "a"
        drive("a", self.config.ugv_home)
        initial = {"uav": self.uav_soc * self.battery_parameters.capacity_wh}
        initial.update({f"{role}_{name}": self.energy[role][index]
            for role in ("a", "b") for index, name in enumerate(("drive", "charge"))})
        return evaluate_relay_energy(steps, self.ugv_energy_parameters, self.battery_parameters, initial)

    def _floor_energy_leg(self, target):
        home = replace(self.waypoints.resolve_home(target.uav_home_node),
            x=target.ugv_launch.x, y=target.ugv_launch.y)
        profile = MissionEnergyPlanner(self.waypoints, self.battery_parameters).plan(
            [target.uav_target], target.uav_home_node, self.relay.landing_height, True,
            [target.payload_mass_kg],
            [target.delivery_floor or self.waypoints.waypoints[target.uav_target].delivery_floor or 1],
            home_override=home)
        estimate = BatteryModel(self.battery_parameters).estimate(profile)
        capacity = self.battery_parameters.capacity_wh
        self._require(estimate.required_energy_wh <= capacity,
                      f"{target.name} sortie exceeds UAV battery capacity")
        target_wh = capacity if self.charge_policy == "full" else min(capacity,
            estimate.required_energy_wh + float(self.get_parameter("charging_buffer_wh").value))
        return FlightLeg("b", "b", estimate.estimated_mission_energy_wh,
            max(self.ugv_energy_parameters.planned_target_service_time_s,
                sum(s.duration_seconds for s in profile.segments)), target_wh)

    def _prepare_charge_target(self, leg):
        if self.energy_constraints_enabled:
            self.charge_target_pub.publish(Float32(
                data=leg.departure_target_wh / self.battery_parameters.capacity_wh))

    def _charge_for_transfer(self, source, destination, mass, goal):
        if not self.energy_constraints_enabled:
            return
        leg = self._transfer_energy_leg(source, destination, mass)
        capacity = self.battery_parameters.capacity_wh
        charging_timeout = float(self.get_parameter("charging_wait_timeout_s").value)
        started = self.get_clock().now().nanoseconds
        self.charge_target_pub.publish(Float32(data=leg.departure_target_wh / capacity))
        def ready():
            if self.uav_soc is None or not math.isfinite(self.uav_soc):
                return False
            remaining = self.uav_soc * capacity
            if remaining >= leg.departure_target_wh - .002 * capacity:
                return True
            elapsed = (self.get_clock().now().nanoseconds - started) / 1e9
            self._require(elapsed < charging_timeout, "Transfer charging simulation-time limit exceeded")
            self._require(self.chargers[source], "Departure car charging pack is unavailable")
            # Include finite source energy and both conversion stages before waiting.
            p, b = self.ugv_energy_parameters, self.battery_parameters
            deficit = leg.departure_target_wh - remaining
            self._require(b.net_charge_power_w > 0, "Charger cannot overcome UAV idle power")
            seconds = deficit * 3600 / b.net_charge_power_w
            needed = (deficit + b.docked_idle_power_w * seconds / 3600) / (
                b.charge_efficiency * p.charger_transfer_efficiency)
            drive, charger = self.energy[source]
            self._require(drive is not None and charger is not None,
                          "Departure car battery telemetry is unavailable")
            self._require(charger - needed - p.charger_idle_power_w * seconds / 3600
                          >= p.charging_reserve_wh, "Departure car cannot supply the transfer charge")
            self._require(drive - p.idle_power_w * seconds / 3600 >= p.drive_reserve_wh,
                          "Departure car drive reserve would be exhausted while charging")
            return False
        self._require(self._wait(ready, goal, max(30.0, charging_timeout * 10.0)),
                      "Charging timed out before inter-platform flight")
        self.energy_plan_pub.publish(String(data=(
            f"TRANSFER_{source.upper()}_{destination.upper()}: departure_target_wh={leg.departure_target_wh:.3f}")))

    def _transfer(self, source, destination, mass, phase, goal):
        self._require(self._fresh(source) and self._fresh(destination), "Relay car telemetry unavailable")
        self._role(destination)
        self._require(self._wait_ugv_settled(goal), "Receiving car must wait stationary")
        self._charge_for_transfer(source, destination, mass, goal)
        self._set_phase(phase, f"platform_{source}_to_{destination}", "UAV")
        route = list(self.relay.air_route(self.waypoints, source, destination))
        # Use measured car centres for both the vertical departure and final approach.
        src, dst = self.poses[source].pose.pose.position, self.poses[destination].pose.pose.position
        route[0] = (src.x, src.y, self.relay.transfer_altitude)
        route[-1] = (dst.x, dst.y, self.relay.transfer_altitude)
        path = NavPath()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for point in route:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.flight_phase_pub.publish(String(data="TAKEOFF"))
        self.height_pub.publish(Float64(data=self.relay.landing_height))
        self._release(source, mass, goal)
        if source == "a":
            self._cargo("a", self.cargo["a"] - mass)
        self._select(destination)
        for index, point in enumerate(route):
            self.flight_phase_pub.publish(String(data="CRUISE"))
            self._fly(point, goal, platform=(index == 0))
            if index == 0:
                self.flight_phase_pub.publish(String(data="HOVER"))
                until = self.get_clock().now().nanoseconds + int(
                    self.relay.transfer_hover_seconds * 1e9)
                self._require(self._wait(lambda: self.get_clock().now().nanoseconds >= until,
                    goal, self.relay.transfer_timeout_wall_s), "Departure hover timed out")
        self.flight_phase_pub.publish(String(data="LANDING"))
        self._fly((dst.x, dst.y, max(.8, self.relay.landing_height + .35)), goal, platform=True,
                  tolerance=.10)
        self.height_pub.publish(Float64(data=self.relay.landing_height))
        self._require(*self._call_trigger(self.land_client))
        self._require(self._wait(lambda: self.uav_state == 0, goal, 300),
                      f"PX4 did not land on car {destination}: {self.landing_fault}")
        self._require(not self.landing_fault, f"PX4 landing fault: {self.landing_fault}")
        self._dock(destination, goal)
        self.flight_phase_pub.publish(String(data="COMPLETED"))
        if destination == "b":
            self._cargo("b", mass)

    def _execute_mission(self, goal):
        result = ExecuteCooperativeDelivery.Result()
        result.completed_targets = 0
        try:
            self._set_phase(RelayPhase.PREPARING)
            self._require(self._wait(self._interfaces_ready, goal, 180),
                          "Two-car relay interfaces are not ready; inspect /relay_delivery/readiness")
            self._require(self.docked and self.carrier_role == "a",
                          "Start relay with UAV docked to A at the logistics hub")
            targets = self._resolve_request_targets(goal.request)
            batches = partition_payloads(targets, self.battery_parameters.maximum_payload_mass_kg)
            batches = tuple(tuple(self._ordered(batch)) for batch in batches)
            if self.energy_constraints_enabled:
                self._require(self._wait(lambda: self.uav_soc is not None and all(
                    v is not None for pair in self.energy.values() for v in pair), goal, 30),
                    "Five-battery telemetry is not ready")
                forecast = self._mission_energy_forecast(batches)
                self.energy_plan_pub.publish(String(data=json.dumps(asdict(forecast))))
                self._require(forecast.feasible, "Relay energy admission refused: " + forecast.reason)
                self._prepare_charge_target(self._transfer_energy_leg(
                    "a", "b", sum(t.payload_mass_kg for t in batches[0])))
            self._cargo("a", sum(t.payload_mass_kg for t in targets))
            self._cargo("b", 0.0)
            self._drive("b", self.relay.platform_b.stop, RelayPhase.B_TO_PLATFORM, goal)
            self._drive("a", self.relay.platform_a.stop, RelayPhase.A_TO_PLATFORM, goal)
            for batch in batches:
                ordered = batch
                self.optimized_route_pub.publish(String(data="B_ROAD_DISTANCE: " + " -> ".join(
                    ["platform_b"] + [t.name for t in ordered] + ["platform_b"])))
                self._transfer("a", "b", sum(t.payload_mass_kg for t in ordered),
                               RelayPhase.CROSS_REGION_OUTBOUND, goal)
                for target in ordered:
                    if self.energy_constraints_enabled:
                        self._prepare_charge_target(self._floor_energy_leg(target))
                    self._drive("b", target.ugv_launch, RelayPhase.B_TO_BUILDING, goal)
                    self.home_role = "b"
                    self._bridge()
                    if self.energy_constraints_enabled:
                        self._require(*self._wait_charge_ready(target, goal))
                    self._set_phase(RelayPhase.FLOOR_DELIVERY, target.name, "UAV")
                    self._release("b", target.payload_mass_kg, goal)
                    self._cargo("b", self.cargo["b"] - target.payload_mass_kg)
                    self._require(*self._execute_uav_delivery(target, goal))
                    self._dock("b", goal)
                    result.completed_targets += 1
                if self.energy_constraints_enabled:
                    self._prepare_charge_target(self._transfer_energy_leg("b", "a", 0.0))
                self._drive("b", self.relay.platform_b.stop, RelayPhase.B_RETURN_PLATFORM, goal)
                self._transfer("b", "a", 0.0, RelayPhase.CROSS_REGION_RETURN, goal)
            self._drive("a", self.config.ugv_home, RelayPhase.A_RETURN_HUB, goal)
            self._require(self.carrier_role == "a" and self.docked and self.uav_state == 0,
                          "Final UAV return to A was not confirmed")
            self._set_phase(CooperativePhase.COMPLETED)
            result.success = True
            result.message = "Two-car relay completed; UAV returned to logistics hub on A"
            goal.succeed()
            return result
        except Exception as error:
            return self._finish_failure(goal, result, str(error))
        finally:
            if rclpy.ok():
                for publisher in self.stop_pubs.values():
                    publisher.publish(Twist())
            with self.lock:
                self.mission_active = False


def main(args=None):
    rclpy.init(args=args)
    node = RelayMissionManager()
    executor = MultiThreadedExecutor(num_threads=8)
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
