from datetime import datetime, timezone
import math
from pathlib import Path
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from campus_dynamic_obstacles.scenario import (
    load_obstacle_config,
    obstacle_instance_name,
    select_routes,
)
from cooperative_delivery_interfaces.action import ExecuteCooperativeDelivery
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Int32, String
from uav_interfaces.action import ExecuteDelivery

from delivery_evaluation.energy_estimator import (
    UgvEnergyIntegrator,
    UgvEnergyParameters,
)
from delivery_evaluation.metrics_collector import MissionMetricsCollector
from delivery_evaluation.models import RunRecord
from delivery_evaluation.report_generator import append_record
from delivery_evaluation.scenario_config import load_scenarios


VALID_MODES = {"ugv_only", "uav_only", "cooperative"}


class ExperimentRunner(Node):
    def __init__(self) -> None:
        super().__init__("experiment_runner")
        share = Path(get_package_share_directory("delivery_evaluation"))
        obstacle_share = Path(
            get_package_share_directory("campus_dynamic_obstacles")
        )
        self.declare_parameter("mode", "cooperative")
        self.declare_parameter("scenario", "teaching_building")
        self.declare_parameter("repetitions", 1)
        self.declare_parameter("obstacle_density", "none")
        self.declare_parameter("random_seed", 42)
        self.declare_parameter(
            "results_dir",
            "/home/wqi/design_final/wqi_final/simulation_ws/experiment_results",
        )
        self.declare_parameter(
            "scenario_file", str(share / "config" / "experiment_scenarios.yaml")
        )
        self.declare_parameter(
            "ugv_energy_file", str(share / "config" / "ugv_energy_model.yaml")
        )
        self.declare_parameter(
            "obstacle_config_file",
            str(obstacle_share / "config" / "obstacle_routes.yaml"),
        )
        self.declare_parameter("continue_on_failure", False)

        self.mode = str(self.get_parameter("mode").value)
        self.scenario_name = str(self.get_parameter("scenario").value)
        self.repetitions = int(self.get_parameter("repetitions").value)
        self.obstacle_density = str(
            self.get_parameter("obstacle_density").value
        )
        self.random_seed = int(self.get_parameter("random_seed").value)
        self.continue_on_failure = bool(
            self.get_parameter("continue_on_failure").value
        )
        if self.mode not in VALID_MODES:
            raise ValueError(f"Unknown experiment mode: {self.mode}")
        self.declare_parameter(
            "ugv_collision_radius_m",
            0.60 if self.mode == "cooperative" else 0.22,
        )
        self.declare_parameter("uav_collision_radius_m", 0.56)
        if self.repetitions <= 0:
            raise ValueError("repetitions must be positive")
        scenarios = load_scenarios(self.get_parameter("scenario_file").value)
        if self.scenario_name not in scenarios:
            raise ValueError(f"Unknown scenario: {self.scenario_name}")
        self.scenario = scenarios[self.scenario_name]
        self.delivery_by_name = {
            delivery.name: delivery
            for delivery in self.scenario.deliveries
        }

        root = Path(str(self.get_parameter("results_dir").value)).expanduser()
        batch_name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.output_directory = root / (
            f"{batch_name}_{self.mode}_{self.scenario_name}_{self.obstacle_density}"
        )
        self.output_directory.mkdir(parents=True, exist_ok=True)

        energy_parameters = UgvEnergyParameters.from_yaml(
            self.get_parameter("ugv_energy_file").value
        )
        self.metrics = MissionMetricsCollector(
            UgvEnergyIntegrator(energy_parameters),
            ugv_collision_radius_m=float(
                self.get_parameter("ugv_collision_radius_m").value
            ),
            uav_collision_radius_m=float(
                self.get_parameter("uav_collision_radius_m").value
            ),
        )
        self.callback_group = ReentrantCallbackGroup()
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
        self.cooperative_client = ActionClient(
            self,
            ExecuteCooperativeDelivery,
            "/cooperative_delivery/execute_mission",
            callback_group=self.callback_group,
        )

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Odometry,
            "/ground_truth/odom",
            self._ugv_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            "/uav/odom",
            self.metrics.update_uav,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        densities, obstacle_routes = load_obstacle_config(
            self.get_parameter("obstacle_config_file").value
        )
        self.selected_obstacles = select_routes(
            densities,
            obstacle_routes,
            self.obstacle_density,
            self.random_seed,
        )
        self.expected_dynamic_obstacle_names = set()
        for index, route in enumerate(self.selected_obstacles, start=1):
            obstacle_name = obstacle_instance_name(index, route)
            self.expected_dynamic_obstacle_names.add(obstacle_name)
            topic = f"/dynamic_obstacles/{obstacle_name}/odom"
            self.create_subscription(
                Odometry,
                topic,
                lambda message, name=obstacle_name, selected=route: (
                    self.metrics.update_dynamic_obstacle(
                        name,
                        selected.layer,
                        selected.radius_m,
                        message,
                    )
                ),
                qos_profile_sensor_data,
                callback_group=self.callback_group,
            )
        self.create_subscription(
            BatteryState,
            "/uav/battery_state",
            lambda message: self.metrics.update_battery(message.percentage),
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            "/uav/battery_consumed_wh",
            lambda message: self.metrics.update_consumed(message.data),
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            "/uav/battery_charged_wh",
            lambda message: self.metrics.update_charged(message.data),
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            BatteryState,
            "/ugv/drive_battery_state",
            lambda message: self.metrics.update_ugv_drive_battery(
                message.percentage
            ),
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            BatteryState,
            "/ugv/charging_battery_state",
            lambda message: self.metrics.update_ugv_charging_battery(
                message.percentage
            ),
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            "/ugv/drive_consumed_wh",
            lambda message: self.metrics.update_ugv_drive_consumed(
                message.data
            ),
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            "/ugv/charging_consumed_wh",
            lambda message: self.metrics.update_ugv_charging_consumed(
                message.data
            ),
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            "/uav/docked",
            lambda message: self.metrics.update_docked(message.data),
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Bool,
            "/uav/safety/blocked",
            lambda message: self.metrics.update_safety(
                message.data, self.get_clock().now().nanoseconds
            ),
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            "/uav/safety/min_distance",
            lambda message: self.metrics.update_clearance(message.data),
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "/uav/delivery_event",
            self._delivery_event_callback,
            transient_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Int32,
            "/uav/replan_count",
            lambda message: self.metrics.update_replans(message.data),
            transient_qos,
            callback_group=self.callback_group,
        )
        for topic in (
            "/uav/mission_status",
            "/cooperative_delivery/mission_status",
        ):
            self.create_subscription(
                String,
                topic,
                lambda message: self.metrics.set_phase(
                    message.data, self.get_clock().now().nanoseconds
                ),
                transient_qos,
                callback_group=self.callback_group,
            )
        self.status_pub = self.create_publisher(
            String, "/delivery_evaluation/status", transient_qos
        )
        self._payload_mass_kg = sum(
            delivery.payload_kg for delivery in self.scenario.deliveries
        )
        self._worker_started = False
        self.create_timer(1.0, self._start_worker, callback_group=self.callback_group)
        self.get_logger().info(
            f"Evaluation ready: mode={self.mode}, scenario={self.scenario_name}, "
            f"repetitions={self.repetitions}, obstacles={len(self.selected_obstacles)}, "
            f"output={self.output_directory}"
        )

    def _start_worker(self) -> None:
        if self._worker_started:
            return
        self._worker_started = True
        threading.Thread(target=self._run_batch, daemon=True).start()

    def _ugv_callback(self, message: Odometry) -> None:
        self.metrics.update_ugv(message, self._payload_mass_kg)

    def _delivery_event_callback(self, message: String) -> None:
        event = str(message.data)
        if not event.startswith("DELIVERED:"):
            return
        target_name = event.split(":", 1)[1]
        delivery = self.delivery_by_name.get(target_name)
        if delivery is None:
            self.get_logger().warning(
                f"Ignoring delivery event for unknown target '{target_name}'"
            )
            return
        self.metrics.mark_uav_endpoint(delivery.uav_pose, target_name)
        if self.mode == "cooperative":
            self.metrics.mark_ugv_endpoint(delivery.ugv_pose, target_name)

    @staticmethod
    def _wait_future(future, timeout: float) -> bool:
        deadline = time.monotonic() + float(timeout)
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return future.done()

    def _wait_server(self, client, timeout: float = 180.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_server(timeout_sec=0.5):
                return True
        return False

    @staticmethod
    def _pose(target) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(target.x)
        pose.pose.position.y = float(target.y)
        pose.pose.position.z = float(target.z)
        half_yaw = 0.5 * float(target.yaw)
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def _at_home(self) -> tuple[bool, str]:
        tolerance = self.scenario.precondition_position_tolerance_m
        speed_limit = self.scenario.precondition_speed_tolerance_mps
        with self.metrics.lock:
            ugv = self.metrics.latest_ugv
            uav = self.metrics.latest_uav
            docked = self.metrics.latest_docked
            observed_obstacles = set(
                self.metrics.latest_dynamic_obstacles
            )
            uav_energy_values = (
                self.metrics.latest_battery_soc,
                self.metrics.latest_consumed_wh,
                self.metrics.latest_charged_wh,
            )
            ugv_energy_values = (
                self.metrics.latest_ugv_drive_soc,
                self.metrics.latest_ugv_charging_soc,
                self.metrics.latest_ugv_drive_consumed_wh,
                self.metrics.latest_ugv_charging_consumed_wh,
            )
        missing_obstacles = (
            self.expected_dynamic_obstacle_names - observed_obstacles
        )
        if missing_obstacles:
            return False, (
                "Dynamic obstacle odometry is unavailable: "
                + ", ".join(sorted(missing_obstacles))
            )
        if (
            self.mode in ("uav_only", "cooperative")
            and any(value is None for value in uav_energy_values)
        ):
            return False, "UAV battery telemetry is unavailable"
        if (
            self.mode == "cooperative"
            and any(value is None for value in ugv_energy_values)
        ):
            return False, "UGV dual-battery telemetry is unavailable"
        if self.mode in ("ugv_only", "cooperative"):
            if ugv is None:
                return False, "UGV odometry is unavailable"
            position, speed = ugv
            error = math.hypot(
                position[0] - self.scenario.ugv_home.x,
                position[1] - self.scenario.ugv_home.y,
            )
            if error > tolerance or speed > speed_limit:
                return False, f"UGV is not settled at home (error={error:.2f}, speed={speed:.2f})"
        if self.mode in ("uav_only", "cooperative"):
            if uav is None:
                return False, "UAV odometry is unavailable"
            position, speed = uav
            horizontal = math.hypot(
                position[0] - self.scenario.uav_home.x,
                position[1] - self.scenario.uav_home.y,
            )
            if horizontal > tolerance or speed > speed_limit:
                return False, (
                    f"UAV is not settled at home (error={horizontal:.2f}, "
                    f"speed={speed:.2f})"
                )
            if self.mode == "cooperative" and docked is not True:
                return False, "UAV is not docked on the UGV"
        return True, "ready"

    def _wait_precondition(self, timeout: float = 180.0) -> tuple[bool, str]:
        deadline = time.monotonic() + timeout
        last_reason = "waiting for state"
        while rclpy.ok() and time.monotonic() < deadline:
            ready, last_reason = self._at_home()
            if ready:
                return True, last_reason
            time.sleep(0.5)
        return False, last_reason

    def _action_result(self, client, goal, feedback_callback=None):
        send_future = client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        if not self._wait_future(send_future, 30.0):
            return False, None, "goal submission timed out"
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, None, "goal was rejected"
        result_future = handle.get_result_async()
        if not self._wait_future(result_future, self.scenario.action_timeout_s):
            handle.cancel_goal_async()
            return False, None, "action timed out"
        wrapped = result_future.result()
        if wrapped is None:
            return False, None, "action returned no result"
        result = wrapped.result
        success = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        if hasattr(result, "success"):
            success = success and bool(result.success)
        message = str(getattr(result, "message", ""))
        return success, result, message or f"action status {wrapped.status}"

    def _nav_feedback(self, message) -> None:
        recoveries = int(message.feedback.number_of_recoveries)
        with self.metrics.lock:
            if self.metrics.active:
                self.metrics.record.nav_recovery_count = max(
                    self.metrics.record.nav_recovery_count, recoveries
                )

    def _run_ugv(self, record: RunRecord):
        for delivery in self.scenario.deliveries:
            self.metrics.set_phase(
                f"UGV_TRANSIT:{delivery.name}", self.get_clock().now().nanoseconds
            )
            goal = NavigateToPose.Goal()
            goal.pose = self._pose(delivery.ugv_pose)
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            success, _, message = self._action_result(
                self.nav_client, goal, self._nav_feedback
            )
            if not success:
                return (
                    False, f"UGV target {delivery.name}: {message}",
                    record.completed_targets,
                )
            self.metrics.mark_ugv_endpoint(delivery.ugv_pose, delivery.name)
            record.completed_targets += 1

        self.metrics.set_phase("RETURNING_HOME", self.get_clock().now().nanoseconds)
        home_goal = NavigateToPose.Goal()
        home_goal.pose = self._pose(self.scenario.ugv_home)
        home_goal.pose.header.stamp = self.get_clock().now().to_msg()
        success, _, message = self._action_result(
            self.nav_client, home_goal, self._nav_feedback
        )
        return success, message, record.completed_targets

    def _run_uav(self, record: RunRecord):
        goal = ExecuteDelivery.Goal()
        goal.targets = [item.name for item in self.scenario.deliveries]
        goal.return_home = True
        goal.home_name = ""
        goal.landing_height = 0.03
        goal.payload_masses_kg = [
            float(item.payload_kg) for item in self.scenario.deliveries
        ]
        goal.target_floors = [
            int(item.floor) for item in self.scenario.deliveries
        ]
        success, result, message = self._action_result(self.uav_client, goal)
        completed = int(getattr(result, "completed_targets", 0)) if result else 0
        record.completed_targets = completed
        return success, message, completed

    def _run_cooperative(self, record: RunRecord):
        goal = ExecuteCooperativeDelivery.Goal()
        goal.targets = [item.name for item in self.scenario.deliveries]
        goal.return_home = True
        goal.target_floors = [
            int(item.floor) for item in self.scenario.deliveries
        ]
        goal.payload_masses_kg = [
            float(item.payload_kg) for item in self.scenario.deliveries
        ]
        success, result, message = self._action_result(
            self.cooperative_client, goal
        )
        completed = int(getattr(result, "completed_targets", 0)) if result else 0
        record.completed_targets = completed
        return success, message, completed

    def _required_client(self):
        return {
            "ugv_only": self.nav_client,
            "uav_only": self.uav_client,
            "cooperative": self.cooperative_client,
        }[self.mode]

    def _run_batch(self) -> None:
        try:
            client = self._required_client()
            self.status_pub.publish(String(data="WAITING_FOR_ACTION_SERVER"))
            if not self._wait_server(client):
                self.get_logger().error("Required action server is unavailable")
                return

            for repetition in range(1, self.repetitions + 1):
                ready, reason = self._wait_precondition()
                run_id = (
                    f"{self.mode}-{self.scenario_name}-{self.obstacle_density}-"
                    f"{self.random_seed}-{repetition:03d}"
                )
                record = RunRecord(
                    run_id=run_id,
                    mode=self.mode,
                    scenario=self.scenario_name,
                    repetition=repetition,
                    obstacle_density=self.obstacle_density,
                    random_seed=self.random_seed,
                    targets=[item.name for item in self.scenario.deliveries],
                )
                self.metrics.start(record, self.get_clock().now().nanoseconds)
                self.status_pub.publish(String(data=f"RUNNING:{run_id}"))
                if not ready:
                    success = False
                    message = f"PRECONDITION_FAILED: {reason}"
                elif self.mode == "ugv_only":
                    success, message, _ = self._run_ugv(record)
                elif self.mode == "uav_only":
                    success, message, _ = self._run_uav(record)
                else:
                    success, message, _ = self._run_cooperative(record)
                record.success = bool(success)
                record.failure_reason = "" if success else str(message)
                finished = self.metrics.finish(self.get_clock().now().nanoseconds)
                if not finished.collision_free:
                    collision_reason = (
                        "Dynamic obstacle collision detected "
                        f"(UGV={finished.ugv_collision_count}, "
                        f"UAV={finished.uav_collision_count})"
                    )
                    finished.success = False
                    finished.failure_reason = "; ".join(
                        part for part in (
                            finished.failure_reason,
                            collision_reason,
                        ) if part
                    )
                append_record(self.output_directory, finished)
                state = "SUCCEEDED" if finished.success else "FAILED"
                result_message = finished.failure_reason or str(message)
                self.status_pub.publish(
                    String(data=f"{state}:{run_id}:{result_message}")
                )
                self.get_logger().info(
                    f"Run {run_id} {state}: {result_message}; "
                    f"results={self.output_directory}"
                )
                if not finished.success and not self.continue_on_failure:
                    break
                if repetition < self.repetitions:
                    time.sleep(self.scenario.settle_time_s)
        except Exception as error:  # Preserve a clear batch-level failure in logs.
            self.get_logger().error(
                f"Evaluation batch failed: {error!r}"
            )
        finally:
            self.status_pub.publish(
                String(data=f"BATCH_COMPLETED:{self.output_directory}")
            )
            time.sleep(0.5)
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
