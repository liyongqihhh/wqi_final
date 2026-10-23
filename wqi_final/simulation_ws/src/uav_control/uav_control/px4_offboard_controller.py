import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, Int8, String
from sensor_msgs.msg import BatteryState

from uav_control.flight_action_server import (
    FLYING,
    LANDED,
    LANDING,
    TAKING_OFF,
    FlightActionServer,
)
from uav_control.px4_frames import (
    landing_alignment_gate, landing_deadline_expired, landing_setpoint_ned, map_enu_to_local_ned,
    map_reference_velocity_bias, map_velocity_to_ned, px4_position_is_ready,
    reference_from_positions, velocity_sample_is_fresh, deck_contact_is_stable,
)
from uav_control.px4_battery_bridge import battery_sample_fresh

try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLandDetected,
        VehicleLocalPosition,
        VehicleStatus,
        VehicleOdometry,
        EstimatorStatusFlags,
    )
except ImportError as error:  # pragma: no cover - checked by setup script
    raise RuntimeError(
        "PX4 backend requires px4_msgs v1.16. Run ./setup_px4_backend.sh "
        "and source ./setup_workspace.bash before launching."
    ) from error


class _PositionSetpointAdapter:
    def __init__(self, controller):
        self.controller = controller

    def publish(self, message: Twist):
        self.controller.set_map_target(
            message.linear.x,
            message.linear.y,
            message.linear.z,
        )


class Px4OffboardController(FlightActionServer):
    """Keep the project UAV facade while executing it through PX4 Offboard."""

    def __init__(self):
        super().__init__()
        self.declare_parameter("offboard_rate", 20.0)
        self.declare_parameter("offboard_warmup_seconds", 1.2)
        self.declare_parameter("takeoff_clearance", 1.0)
        self.declare_parameter("takeoff_altitude_tolerance", 0.15)
        self.declare_parameter("landing_disarm_delay", 1.0)
        self.declare_parameter("landing_descent_speed", 0.30)
        self.declare_parameter("landing_xy_tolerance", 0.08)
        self.declare_parameter("landing_horizontal_speed_tolerance", 0.15)
        self.declare_parameter("precision_landing_timeout_s", 60.0)
        self.declare_parameter("enable_sim_velocity_aiding", False)
        self.enable_sim_velocity_aiding = bool(self.get_parameter("enable_sim_velocity_aiding").value)
        self.velocity_aiding_sample_ns = None
        self.velocity_aiding_received_at = None
        self.velocity_fusion_at = None
        self.velocity_fusion_sim_ns = None
        self.velocity_fusion_active = False
        self.height_fusion_active = False
        self.target_landing_height = None
        self.deck_contact_started_ns = None
        self.native_deck_handoff = False
        self.native_land_request_ns = None
        self.create_subscription(Float64, "landing_height", self._landing_height_callback, 10)
        self.precision_landing_timeout_s = float(self.get_parameter("precision_landing_timeout_s").value)
        if not math.isfinite(self.precision_landing_timeout_s) or self.precision_landing_timeout_s <= 0:
            raise ValueError("precision_landing_timeout_s must be finite and positive")
        self.precision_landing_start_ns = None
        self.declare_parameter("require_energy_bridge", False)
        self.require_energy_bridge = bool(self.get_parameter("require_energy_bridge").value)
        self.energy_bridge_at = None
        self.energy_bridge_sim_ns = None
        self.energy_soc = None
        self.declare_parameter("minimum_takeoff_soc", 0.20)
        self.create_subscription(BatteryState, "battery_state",
            lambda message: setattr(self, "energy_soc", float(message.percentage)),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, "px4/battery_bridge_status",
                                 self._energy_bridge_callback, 10)
        self.declare_parameter(
            "vehicle_status_topic", "/fmu/out/vehicle_status_v1"
        )
        self.offboard_rate = float(self.get_parameter("offboard_rate").value)
        self.warmup_seconds = float(
            self.get_parameter("offboard_warmup_seconds").value
        )
        self.takeoff_clearance = float(
            self.get_parameter("takeoff_clearance").value
        )
        self.takeoff_altitude_tolerance = float(
            self.get_parameter("takeoff_altitude_tolerance").value
        )
        self.landing_disarm_delay = float(
            self.get_parameter("landing_disarm_delay").value
        )
        self.landing_descent_speed = float(self.get_parameter("landing_descent_speed").value)
        if not math.isfinite(self.landing_descent_speed) or not .30 <= self.landing_descent_speed <= .5:
            raise ValueError("landing_descent_speed must be in [0.3, 0.5] for the PX4 land detector")
        self.landing_xy_tolerance = float(self.get_parameter("landing_xy_tolerance").value)
        self.landing_speed_tolerance = float(self.get_parameter("landing_horizontal_speed_tolerance").value)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.landing_xy_tolerance, self.landing_speed_tolerance)):
            raise ValueError("Landing alignment tolerances must be finite and positive")
        self.vehicle_status_topic = str(
            self.get_parameter("vehicle_status_topic").value
        )

        self.local_position = None
        self.vehicle_status = None
        self.land_detected = True
        self.map_reference = None
        self.map_target = None
        self.pending_takeoff_at = None
        self.landing_started_at = None
        self.precision_landing = False
        self.completed_map_target = None
        self.landing_aligned = False
        self.landing_hold_height = None
        self.landing_diagnostic_ns = None
        self.last_disarm_request = None
        self.last_arm_request = None
        self._last_published_state = None

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            px4_qos,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            px4_qos,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            px4_qos,
        )
        self.velocity_aiding_pub = self.create_publisher(
            VehicleOdometry, "/fmu/in/vehicle_visual_odometry", px4_qos)
        self.create_subscription(EstimatorStatusFlags, "/fmu/out/estimator_status_flags",
                                 self._estimator_status_callback, px4_qos)
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_position_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            self.vehicle_status_topic,
            self._vehicle_status_callback,
            px4_qos,
        )
        self.create_subscription(
            VehicleLandDetected,
            "/fmu/out/vehicle_land_detected",
            self._land_detected_callback,
            px4_qos,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.px4_state_pub = self.create_publisher(Int8, "state", state_qos)
        self.backend_pub = self.create_publisher(
            String, "flight_backend", state_qos
        )
        self.connected_pub = self.create_publisher(
            Bool, "px4/connected", state_qos
        )
        self.velocity_aiding_status_pub = self.create_publisher(
            String, "px4/velocity_aiding_status", state_qos)
        self.landing_fault_pub = self.create_publisher(String, "px4/landing_fault", state_qos)
        self._set_landing_fault("")

        self.cmd_pub = _PositionSetpointAdapter(self)
        self.create_timer(
            1.0 / max(self.offboard_rate, 10.0), self._offboard_timer
        )
        self.create_timer(1.0, self._status_timer)
        self.backend_pub.publish(String(data="PX4_SITL_OFFBOARD"))
        self.get_logger().info(
            "PX4 SITL Offboard backend is ready; waiting for DDS telemetry"
        )

    def _timestamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _set_landing_fault(self, reason):
        self.landing_fault_pub.publish(String(data=reason))

    def _landing_height_callback(self, message):
        height = float(message.data)
        self.target_landing_height = height if math.isfinite(height) and .2 <= height <= 2 else None

    def _try_native_deck_handoff(self):
        if not (self.enable_sim_velocity_aiding and self.precision_landing
                and self._last_published_state == LANDING):
            return
        now_ns = self.get_clock().now().nanoseconds
        if not self.native_deck_handoff:
            odom, _ = self._snapshot()
            stable = False
            if odom is not None and self.target_landing_height is not None:
                p, v, q = odom.pose.pose.position, odom.twist.twist.linear, odom.pose.pose.orientation
                stable = (velocity_sample_is_fresh(self.velocity_aiding_sample_ns or 0, now_ns)
                          and deck_contact_is_stable((p.x, p.y, p.z), self.map_target,
                              (v.x, v.y, v.z), (q.x, q.y, q.z, q.w), self.target_landing_height))
            if not stable:
                self.deck_contact_started_ns = None
                return
            if self.deck_contact_started_ns is None:
                self.deck_contact_started_ns = now_ns
                return
            if not landing_deadline_expired(self.deck_contact_started_ns, now_ns, .5):
                return
            self.native_deck_handoff = True
            self.get_logger().info("Deck contact stable; handing final landing to PX4 NAV_LAND")
        if (self.vehicle_status is not None and self.vehicle_status.nav_state
                == getattr(VehicleStatus, "NAVIGATION_STATE_AUTO_LAND", 18)):
            self.precision_landing = False
        elif self.native_land_request_ns is None or now_ns-self.native_land_request_ns >= 500_000_000:
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.native_land_request_ns = now_ns

    def _odom_callback(self, message):
        super()._odom_callback(message)
        # The base constructor installs this subscription before these fields exist.
        if not getattr(self, "enable_sim_velocity_aiding", False):
            return
        if message.header.frame_id not in ("uav/odom", "uav_odom", "map", "world"):
            return
        sample_ns = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
        v = message.twist.twist.linear
        p = message.pose.pose.position
        try:
            velocity = map_velocity_to_ned((v.x, v.y, v.z))
            position = map_velocity_to_ned((p.x, p.y, p.z))
        except ValueError:
            return
        sample = VehicleOdometry()
        sample.timestamp = self._timestamp()
        sample.timestamp_sample = sample_ns//1000
        sample.pose_frame = VehicleOdometry.POSE_FRAME_NED
        sample.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        # PX4 requires a finite position vector to accept height observations;
        # EKF2_EV_CTRL=6 fuses Z and velocity only, never horizontal position.
        sample.position = position
        sample.q = [math.nan]*4
        sample.velocity = velocity
        sample.angular_velocity = [math.nan]*3
        sample.position_variance = [.0004]*3
        sample.orientation_variance = [math.nan]*3
        sample.velocity_variance = [.0001]*3
        sample.quality = 100
        with self.lock:
            if not velocity_sample_is_fresh(sample_ns, self.get_clock().now().nanoseconds,
                                            self.velocity_aiding_sample_ns):
                return
            self.velocity_aiding_pub.publish(sample)
            self.velocity_aiding_sample_ns = sample_ns
            self.velocity_aiding_received_at = time.monotonic()

    def _estimator_status_callback(self, message):
        with self.lock:
            self.velocity_fusion_active = bool(message.cs_ev_vel)
            self.height_fusion_active = bool(message.cs_ev_hgt)
            self.velocity_fusion_sim_ns = self.get_clock().now().nanoseconds
            self.velocity_fusion_at = time.monotonic()

    def _velocity_aiding_health(self):
        # Odometry callbacks are reentrant: freeze observations BEFORE reading
        # the clock so a concurrent receipt cannot produce a negative age.
        with self.lock:
            sample_ns = self.velocity_aiding_sample_ns
            received_wall = self.velocity_aiding_received_at
            fusion_ns, fusion_wall = self.velocity_fusion_sim_ns, self.velocity_fusion_at
            velocity_active, height_active = self.velocity_fusion_active, self.height_fusion_active
            now_ns, now_wall = self.get_clock().now().nanoseconds, time.monotonic()
        ready = (velocity_active and height_active
                 and velocity_sample_is_fresh(sample_ns or 0, now_ns)
                 and received_wall is not None and 0 <= now_wall-received_wall <= 30.0
                 and battery_sample_fresh(fusion_ns, fusion_wall, now_ns, now_wall, 2.0))
        return {"ready": bool(ready), "velocity_fused": velocity_active, "height_fused": height_active,
                "sample_sim_age_s": None if sample_ns is None else (now_ns-sample_ns)/1e9,
                "sample_wall_age_s": None if received_wall is None else now_wall-received_wall,
                "flags_sim_age_s": None if fusion_ns is None else (now_ns-fusion_ns)/1e9,
                "flags_wall_age_s": None if fusion_wall is None else now_wall-fusion_wall}

    def _velocity_aiding_ready(self):
        if not self.enable_sim_velocity_aiding:
            return True
        return self._velocity_aiding_health()["ready"]

    def _energy_bridge_callback(self, message):
        synchronized = message.data.startswith("SYNCED:")
        self.energy_bridge_at = time.monotonic() if synchronized else None
        self.energy_bridge_sim_ns = (
            self.get_clock().now().nanoseconds if synchronized else None)

    def _energy_bridge_ready(self):
        return not self.require_energy_bridge or battery_sample_fresh(
            self.energy_bridge_sim_ns, self.energy_bridge_at,
            self.get_clock().now().nanoseconds, time.monotonic(), 2.0)

    def _local_position_callback(self, message):
        self.local_position = message
        self._update_reference()

    def _vehicle_status_callback(self, message):
        self.vehicle_status = message

    def _land_detected_callback(self, message):
        self.land_detected = bool(message.landed)
        now = time.monotonic()
        if (
            self.landing_started_at is not None
            and bool(message.landed)
            and now - self.landing_started_at >= self.landing_disarm_delay
            and (
                self.last_disarm_request is None
                or now - self.last_disarm_request >= 1.0
            )
        ):
            self._send_vehicle_command(
                getattr(
                    VehicleCommand, "VEHICLE_CMD_COMPONENT_ARM_DISARM", 400
                ),
                0.0,
            )
            self.last_disarm_request = now

    def _update_reference(self):
        odom, _ = self._snapshot()
        local = self.local_position
        if odom is None or local is None:
            return
        if not getattr(local, "xy_valid", True) or not getattr(
            local, "z_valid", True
        ):
            return
        first_alignment = self.map_reference is None
        point = odom.pose.pose.position
        self.map_reference = reference_from_positions(
            (point.x, point.y, point.z),
            (local.x, local.y, local.z),
        )
        if self.map_target is None:
            self.map_target = (point.x, point.y, point.z)
        if first_alignment:
            self.get_logger().info(
                "PX4 local NED aligned with campus map ENU at "
                f"({point.x:.2f}, {point.y:.2f}, {point.z:.2f})"
            )

    def set_map_target(self, x, y, z):
        values = (float(x), float(y), float(z))
        if all(math.isfinite(value) for value in values):
            self.map_target = values

    def _execute_fly_to_pose(self, goal_handle):
        self.completed_map_target = None
        result = super()._execute_fly_to_pose(goal_handle)
        if result.success:
            point = goal_handle.request.target.pose.position
            self.completed_map_target = (float(point.x), float(point.y), float(point.z))
            self.map_target = self.completed_map_target
        return result

    def _publish_state(self, value):
        if value == self._last_published_state:
            return
        self._last_published_state = value
        self.px4_state_pub.publish(Int8(data=int(value)))

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        message = VehicleCommand()
        message.timestamp = self._timestamp()
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.command = int(command)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.vehicle_command_pub.publish(message)

    def _publish_offboard_setpoint(self):
        if self.map_reference is None or self.map_target is None:
            return
        north, east, down = map_enu_to_local_ned(
            self.map_target, self.map_reference
        )
        mode = OffboardControlMode()
        mode.timestamp = self._timestamp()
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        if hasattr(mode, "thrust_and_torque"):
            mode.thrust_and_torque = False
        if hasattr(mode, "direct_actuator"):
            mode.direct_actuator = False
        self.offboard_pub.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = mode.timestamp
        setpoint.position = [float(north), float(east), float(down)]
        setpoint.velocity = [math.nan, math.nan, math.nan]
        if self.precision_landing and self._last_published_state == LANDING:
            odom, _ = self._snapshot()
            aligned = False
            bias = None
            if odom is not None and self.local_position is not None:
                p, v = odom.pose.pose.position, odom.twist.twist.linear
                aligned = landing_alignment_gate(
                    (p.x, p.y), self.map_target, (v.x, v.y),
                    self.landing_aligned,
                    self.landing_xy_tolerance, self.landing_speed_tolerance)
                local = self.local_position
                try:
                    bias = map_reference_velocity_bias((v.x, v.y, v.z),
                                                       (local.vx, local.vy, local.vz))
                except ValueError:
                    aligned = False
                if self.landing_hold_height is None or (self.landing_aligned and not aligned):
                    self.landing_hold_height = float(p.z)
                if not aligned:
                    _, _, down = map_enu_to_local_ned(
                        (self.map_target[0], self.map_target[1], self.landing_hold_height),
                        self.map_reference)
                now_ns = self.get_clock().now().nanoseconds
                if self.landing_diagnostic_ns is None or now_ns-self.landing_diagnostic_ns >= 5_000_000_000:
                    self.get_logger().info(
                        f"Landing feedback: descend={aligned}, xy_error="
                        f"{math.hypot(p.x-self.map_target[0], p.y-self.map_target[1]):.3f}m, "
                        f"z={p.z:.3f}m, speed_xy={math.hypot(v.x,v.y):.3f}m/s, "
                        f"velocity_bias={bias}, px4_landed={self.land_detected}")
                    self.landing_diagnostic_ns = now_ns
            self.landing_aligned = aligned
            setpoint.position, setpoint.velocity = landing_setpoint_ned(
                (north, east, down), self.landing_descent_speed if aligned else 0.0,
                velocity_bias=bias, hold_altitude=not aligned)
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.yaw = math.nan
        setpoint.yawspeed = math.nan
        self.setpoint_pub.publish(setpoint)

    def _offboard_timer(self):
        self._update_reference()
        self._try_native_deck_handoff()
        aiding_health = self._velocity_aiding_health() if self.enable_sim_velocity_aiding else None
        if (aiding_health is not None and not aiding_health["ready"]
                and not self.native_deck_handoff
                and (self._last_published_state in (FLYING, TAKING_OFF) or self.precision_landing)
                and not self.land_detected):
            self.get_logger().error(
                "Simulation velocity fusion lost; requesting PX4 native landing: " + str(aiding_health))
            self._set_landing_fault("SIM_VELOCITY_FUSION_LOST")
            self.pending_takeoff_at = None
            self.precision_landing = False
            self.landing_started_at = time.monotonic()
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self._publish_state(LANDING)
        if self.precision_landing and landing_deadline_expired(
                self.precision_landing_start_ns, self.get_clock().now().nanoseconds,
                self.precision_landing_timeout_s):
            self.get_logger().error("Precision landing timed out; switching to PX4 native emergency land")
            self._set_landing_fault("PRECISION_LANDING_TIMEOUT")
            self.precision_landing = False
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if (self.require_energy_bridge and not self._energy_bridge_ready()
                and self._last_published_state == FLYING and not self.land_detected):
            self.get_logger().error("Battery synchronization lost; requesting PX4 landing")
            self._set_landing_fault("BATTERY_SYNC_LOST")
            self.precision_landing = False
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.landing_started_at = time.monotonic()
            self._publish_state(LANDING)
        if self.precision_landing and not self._energy_bridge_ready():
            self.get_logger().error("Battery synchronization lost during precision landing")
            self._set_landing_fault("BATTERY_SYNC_LOST")
            self.precision_landing = False
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        if self._last_published_state != LANDING or self.precision_landing:
            self._publish_offboard_setpoint()
        now = time.monotonic()
        if self.pending_takeoff_at is not None:
            if not self._energy_bridge_ready():
                self.get_logger().error("Battery synchronization lost while arming; requesting landing")
                self._set_landing_fault("BATTERY_SYNC_LOST")
                self.precision_landing = False
                self.pending_takeoff_at = None
                self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self._publish_state(LANDING if not self.land_detected else LANDED)
                return
            self._publish_state(TAKING_OFF)
            if now - self.pending_takeoff_at >= self.warmup_seconds:
                if self.last_arm_request is None or now - self.last_arm_request > 0.5:
                    self._send_vehicle_command(
                        getattr(VehicleCommand, "VEHICLE_CMD_DO_SET_MODE", 176),
                        1.0,
                        6.0,
                    )
                    self._send_vehicle_command(
                        getattr(
                            VehicleCommand, "VEHICLE_CMD_COMPONENT_ARM_DISARM", 400
                        ),
                        1.0,
                    )
                    self.last_arm_request = now
            odom, _ = self._snapshot()
            reached_takeoff_altitude = (
                odom is not None
                and self.map_target is not None
                and odom.pose.pose.position.z
                >= self.map_target[2] - self.takeoff_altitude_tolerance
            )
            if not self.land_detected and reached_takeoff_altitude:
                self.pending_takeoff_at = None
                self._publish_state(FLYING)
        elif (
            self.land_detected and self.vehicle_status is not None
            and self.vehicle_status.arming_state
            != getattr(VehicleStatus, "ARMING_STATE_ARMED", 2)
        ):
            self.landing_started_at = None
            self.precision_landing = False
            self.last_disarm_request = None
            self._publish_state(LANDED)
        elif self._last_published_state == LANDING:
            self._publish_state(LANDING)
        else:
            self._publish_state(FLYING)

    def _status_timer(self):
        connected = (self.map_reference is not None and self.vehicle_status is not None
                     and self._energy_bridge_ready() and self._position_ready()
                     and self._velocity_aiding_ready())
        self.connected_pub.publish(Bool(data=connected))
        status = ("DISABLED" if not self.enable_sim_velocity_aiding else
                  "ACTIVE:GAZEBO_VELOCITY_HEIGHT_EKF" if self._velocity_aiding_ready() else
                  "WAITING:GAZEBO_VELOCITY_EKF")
        self.velocity_aiding_status_pub.publish(String(data=status))
        self.backend_pub.publish(String(data="PX4_SITL_OFFBOARD"))

    def _position_ready(self):
        return px4_position_is_ready(self.local_position)

    def _takeoff_callback(self, _request, response):
        odom, state = self._snapshot()
        if state in (FLYING, TAKING_OFF):
            response.success = True
            response.message = "UAV is already airborne or taking off"
            return response
        if self.local_position is None or self.vehicle_status is None:
            response.success = False
            response.message = "PX4 DDS telemetry is not ready"
            return response
        if not self._position_ready():
            response.success = False
            response.message = "PX4 position/velocity estimate is not valid; keep UAV docked"
            return response
        if not self._velocity_aiding_ready():
            response.success = False
            response.message = "PX4 simulation velocity fusion is not ready; keep UAV docked"
            return response
        if not self._energy_bridge_ready():
            response.success = False
            response.message = "Project battery has not synchronized with PX4"
            return response
        if self.require_energy_bridge and (
                self.energy_soc is None or not math.isfinite(self.energy_soc)
                or self.energy_soc <= float(self.get_parameter("minimum_takeoff_soc").value)):
            response.success = False
            response.message = "UAV battery is below takeoff safety reserve"
            return response
        self._update_reference()
        if odom is None or self.map_reference is None:
            response.success = False
            response.message = "PX4/map frame alignment is not ready"
            return response
        point = odom.pose.pose.position
        self.map_target = (
            float(point.x),
            float(point.y),
            max(float(point.z) + self.takeoff_clearance, 1.0),
        )
        self.pending_takeoff_at = time.monotonic()
        self.precision_landing = False
        self.native_deck_handoff = False
        self.native_land_request_ns = None
        self._set_landing_fault("")
        self.completed_map_target = None
        self.last_arm_request = None
        self._publish_state(TAKING_OFF)
        response.success = True
        response.message = "PX4 Offboard takeoff sequence accepted"
        return response

    def _land_callback(self, _request, response):
        odom, state = self._snapshot()
        if state == LANDED and self.land_detected:
            response.success = True
            response.message = "UAV is already landed"
            return response
        if state == LANDING and self.landing_started_at is not None:
            response.success = True
            response.message = "PX4 landing is already in progress"
            return response
        self.pending_takeoff_at = None
        self.precision_landing_start_ns = self.get_clock().now().nanoseconds
        self.landing_started_at = time.monotonic()
        self.last_disarm_request = None
        self.landing_aligned = False
        self.landing_hold_height = None
        self.landing_diagnostic_ns = None
        self.native_deck_handoff = False
        self.native_land_request_ns = None
        self.deck_contact_started_ns = None
        self._publish_state(LANDING)
        self.precision_landing = (
            odom is not None and self.map_reference is not None
            and self._energy_bridge_ready())
        if self.precision_landing:
            point = odom.pose.pose.position
            target = self.completed_map_target
            if target is None or math.hypot(target[0] - point.x, target[1] - point.y) > .5:
                target = (float(point.x), float(point.y), float(point.z))
            self.map_target = (target[0], target[1], float(point.z))
            self._publish_offboard_setpoint()
            self._send_vehicle_command(
                getattr(VehicleCommand, "VEHICLE_CMD_DO_SET_MODE", 176), 1.0, 6.0)
            self.get_logger().info("PX4 precision landing: map-aligned XY hold and downward velocity")
        else:
            self._set_landing_fault("PRECISION_LANDING_UNAVAILABLE")
            self._send_vehicle_command(
                getattr(VehicleCommand, "VEHICLE_CMD_NAV_LAND", 21))
        response.success = True
        response.message = "PX4 landing command accepted"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Px4OffboardController()
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
