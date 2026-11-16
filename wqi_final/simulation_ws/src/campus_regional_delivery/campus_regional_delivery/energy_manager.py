"""ROS adapter for one UAV and four independent dual-battery ground vehicles."""
import json
import math
from pathlib import Path
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Int8, String
from visualization_msgs.msg import Marker, MarkerArray

from campus_regional_delivery.energy_model import (
    FleetEnergyModel, NAMESPACES, charge_pose_safe, fresh_sample, load_energy_configuration,
)


def battery_message(stamp, frame, serial, energy, capacity, voltage, power, nominal_voltage=None):
    result = BatteryState()
    result.header.stamp, result.header.frame_id = stamp, frame
    result.percentage = float(max(0., min(1., energy/capacity)))
    result.voltage, result.current = float(voltage), float(-power/voltage)
    result.capacity = result.design_capacity = float(capacity/(nominal_voltage or voltage))
    result.charge = result.capacity*result.percentage
    result.temperature = math.nan
    result.present = True
    result.serial_number = result.location = serial
    result.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
    result.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
    result.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING if power < 0
        else BatteryState.POWER_SUPPLY_STATUS_FULL if result.percentage >= 1 and power == 0
        else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
    return result


class RegionalEnergyManager(Node):
    def __init__(self):
        super().__init__("regional_energy_manager")
        def share(name):
            return Path(get_package_share_directory(name))
        defaults = {"energy_file": str(share("campus_regional_delivery")/"config/regional_energy.yaml"),
            "uav_energy_file": str(share("uav_control")/"config/battery_model.yaml"),
            "ugv_energy_file": str(share("cooperative_delivery")/"config/ugv_energy_model.yaml"),
            "with_uav": True}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings, self.air_parameters, self.ground_parameters, self.config_digest = load_energy_configuration(
            *(self.get_parameter(name).value for name in ("energy_file", "uav_energy_file", "ugv_energy_file")))
        self.with_uav = bool(self.get_parameter("with_uav").value)
        self.model = FleetEnergyModel(self.settings, self.air_parameters, self.ground_parameters, self.with_uav)
        self.session_id, self.command_sequence, self.command_error = uuid.uuid4().hex, 0, ""
        self.samples, self.velocities, self.accelerations = {}, {}, {}
        self.state, self.state_sample = None, (None, None)
        self.docked, self.carrier, self.phase = False, "", "IDLE"
        self.last_update_ns = None
        self.shutdown_requested = False
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, "/regional_delivery/energy_status", qos)
        self.marker_pub = self.create_publisher(MarkerArray, "/regional_delivery/energy_markers", qos)
        self.packs, self.scalars = {}, {}
        for ns in NAMESPACES:
            self.create_subscription(Odometry, f"/{ns}/ground_truth/odom", lambda m,n=ns: self.odometry(n,m), 20)
            for kind in ("drive", "charging"):
                self.packs[f"{ns}_{kind}"] = self.create_publisher(BatteryState, f"/{ns}/{kind}_battery_state", qos)
            for name in ("drive_power_w", "charging_source_power_w", "drive_remaining_wh", "charging_remaining_wh",
                         "drive_consumed_wh", "charging_consumed_wh", "total_carried_mass_kg", "cargo_mass_kg"):
                self.scalars[ns+"/"+name] = self.create_publisher(Float32, f"/{ns}/{name}", qos)
        if self.with_uav:
            self.packs["uav"] = self.create_publisher(BatteryState, "/uav/battery_state", qos)
            for name in ("battery_percentage", "remaining_energy", "battery_power_w", "power_consumption",
                         "propulsion_power_w", "auxiliary_power_w", "total_mass_kg", "battery_consumed_wh",
                         "battery_charged_wh", "charging_input_power_w"):
                self.scalars["uav/"+name] = self.create_publisher(Float32, "/uav/"+name, qos)
            self.battery_status_pub = self.create_publisher(String, "/uav/battery_status", qos)
            self.create_subscription(Odometry, "/uav/odom", lambda m:self.odometry("uav",m), 20)
            self.create_subscription(Int8, "/uav/state", self.flight_state, qos)
            self.create_subscription(Bool, "/uav/docked", lambda m:setattr(self,"docked",m.data), qos)
            self.create_subscription(String, "/uav/docked_carrier", lambda m:setattr(self,"carrier",m.data), qos)
            self.create_subscription(String, "/uav/mission_status", lambda m:setattr(self,"phase",m.data), qos)
        self.create_subscription(String, "/regional_delivery/energy_command", self.command, 10)
        self.create_timer(1/self.settings["update_rate_hz"], self.update)
        self.create_timer(1/self.settings["publish_rate_hz"], self.publish)
        self.publish()
        self.get_logger().info(f"Regional finite energy model: {9 if self.with_uav else 8} packs; session {self.session_id}")

    def flight_state(self, message):
        if message.data in (0, 1, 2, 3):
            self.state = message.data
            self.state_sample = (self.get_clock().now().nanoseconds, time.monotonic())

    def odometry(self, name, message):
        p, v, w = message.pose.pose.position, message.twist.twist.linear, message.twist.twist.angular
        position, velocity = (p.x,p.y,p.z), (v.x,v.y,v.z)
        if not all(math.isfinite(x) for x in (*position, *velocity, w.z)):
            return
        stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        previous = self.samples.get(name)
        old_velocity = self.velocities.get(name)
        acceleration = self.accelerations.get(name, (0.,0.,0.))
        if previous and stamp > previous[0] and old_velocity is not None:
            dt = (stamp-previous[0])/1e9
            limit = self.settings["maximum_acceleration_mps2"]
            alpha = self.air_parameters.acceleration_filter_alpha
            acceleration = tuple(old+alpha*(max(-limit,min(limit,(new-before)/dt))-old)
                for old,new,before in zip(acceleration,velocity,old_velocity))
        elif previous and stamp < previous[0]:
            acceleration = (0.,0.,0.)
        self.samples[name] = (stamp, time.monotonic(), position, w.z)
        self.velocities[name], self.accelerations[name] = velocity, acceleration

    def fresh(self, name):
        sample = self.samples.get(name)
        return bool(sample and fresh_sample(sample[0],sample[1],self.get_clock().now().nanoseconds,
                                           time.monotonic(),self.settings))

    def readiness(self):
        missing = [name for name in NAMESPACES if not self.fresh(name)]
        if self.with_uav:
            if not self.fresh("uav"):
                missing.append("uav")
            # State is retained and only changes on transitions. Its retained
            # value is checked against continuously fresh aircraft odometry.
            if self.state is None:
                missing.append("uav_state")
        return missing

    def command(self, message):
        try:
            command = json.loads(message.data)
            if command["session_id"] != self.session_id:
                raise ValueError("Energy manager restarted; reject stale command")
            sequence = command["sequence"]
            if type(sequence) is not int or sequence <= 0:
                raise ValueError("Positive integer command sequence required")
            if sequence == self.command_sequence:
                return
            if command["previous_sequence"] != self.command_sequence or sequence != self.command_sequence+1:
                raise ValueError("Energy command sequence gap")
            target = command.get("charge_target_soc", self.model.charge_target)
            if not isinstance(target,(int,float)) or not math.isfinite(target) or not 0 <= target <= 1:
                raise ValueError("Invalid charge target")
            self.model.ledger.assign(command["cargo_kg"],command["payload_kg"],self.air_parameters.maximum_payload_mass_kg)
            self.model.charge_target = float(target)
            self.command_sequence, self.command_error = sequence, ""
        except (ValueError,KeyError,TypeError) as error:
            self.command_error = str(error)
            self.get_logger().error("Energy command rejected: "+str(error))
        self.publish()

    def update(self):
        now = self.get_clock().now().nanoseconds
        if now <= 0:
            return
        if self.last_update_ns is None or now <= self.last_update_ns:
            self.last_update_ns = now
            return
        remaining = (now-self.last_update_ns)/1e9
        self.last_update_ns = now
        carrying = ""
        if (self.with_uav and self.docked and self.carrier in NAMESPACES
                and self.fresh(self.carrier) and self.fresh("uav")
                and charge_pose_safe(self.samples[self.carrier][2],self.samples["uav"][2])):
            carrying = self.carrier
        motion = {}
        for ns in NAMESPACES:
            velocity = self.velocities.get(ns,(0.,0.,0.))
            acceleration = self.accelerations.get(ns,(0.,0.,0.))
            speed = math.hypot(*velocity[:2])
            parallel = sum(v*a for v,a in zip(velocity[:2],acceleration[:2]))/speed if speed > .01 else 0.
            motion[ns] = (speed,self.samples.get(ns,(0,0,0,0.))[3],parallel)
        while remaining > 1e-9:
            dt = min(remaining,self.settings["integration_step_s"])
            self.model.step(dt,motion,self.velocities.get("uav",(0.,0.,0.)),
                self.accelerations.get("uav",(0.,0.,0.)),self.phase,self.state in (1,2,3),
                carrying=carrying,charge_allowed=bool(carrying and self.state==0))
            remaining -= dt

    def publish(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        data = self.model.status()
        missing = self.readiness()
        if not data["energy_balance_ok"]:
            missing.append("energy_balance")
            self.get_logger().error("Nine-pack energy balance mismatch: " +
                str(data["energy_balance_error_wh"]) + " Wh", throttle_duration_sec=10.)
        data.update(session_id=self.session_id, command_sequence=self.command_sequence,
            command_error=self.command_error, config_digest=self.config_digest,
            sample_sim_ns=now.nanoseconds, ready=not missing, missing_telemetry=missing,
            with_uav=self.with_uav, carrier=self.carrier, flight_state=self.state)
        self.status_pub.publish(String(data=json.dumps(data,allow_nan=False)))
        markers = []
        for index,(ns,car) in enumerate(self.model.cars.items()):
            p = self.ground_parameters
            snap = car.snapshot()
            for kind,energy,capacity,power in (
                    ("drive",snap.drive_energy_wh,p.drive_capacity_wh,snap.drive_power_w),
                    ("charging",snap.charging_energy_wh,p.charging_capacity_wh,snap.charging_source_power_w)):
                self.packs[ns+"_"+kind].publish(battery_message(stamp,ns+"/base_link",ns+"_"+kind,
                    energy,capacity,p.drive_nominal_voltage if kind=="drive" else p.charging_nominal_voltage,power))
            values = {"drive_power_w":snap.drive_power_w,"charging_source_power_w":snap.charging_source_power_w,
                "drive_remaining_wh":snap.drive_energy_wh,"charging_remaining_wh":snap.charging_energy_wh,
                "drive_consumed_wh":snap.drive_consumed_wh,"charging_consumed_wh":snap.charging_consumed_wh,
                "total_carried_mass_kg":snap.total_carried_mass_kg,"cargo_mass_kg":self.model.ledger.cargo[ns]}
            for key,value in values.items():
                self.scalars[ns+"/"+key].publish(Float32(data=float(value)))
            markers.append(self.marker(index,ns+"/base_link",.75,
                f'{ns} Drive {snap.drive_soc:.0%} | Charger {snap.charging_soc:.0%}',
                min(snap.drive_soc-p.drive_reserve_percentage,snap.charging_soc-p.charging_reserve_percentage),stamp))
        if self.with_uav:
            b, snap = self.air_parameters, self.model.uav.snapshot()
            message = battery_message(stamp,"uav/base_link","SIM-UAV-BATTERY-001",snap.energy_wh,
                b.capacity_wh,snap.voltage,snap.battery_power_w,b.nominal_voltage)
            self.packs["uav"].publish(message)
            values = {"battery_percentage":snap.soc*100,"remaining_energy":snap.energy_wh,
                "battery_power_w":snap.battery_power_w,"power_consumption":snap.battery_power_w,
                "propulsion_power_w":snap.propulsion_power_w,"auxiliary_power_w":snap.auxiliary_power_w,
                "total_mass_kg":snap.total_mass_kg,"battery_consumed_wh":snap.consumed_energy_wh,
                "battery_charged_wh":snap.charged_energy_wh,"charging_input_power_w":self.model.input_power_w}
            for key,value in values.items():
                self.scalars["uav/"+key].publish(Float32(data=float(value)))
            self.battery_status_pub.publish(String(data=f'{snap.mode}:{snap.soc*100:.1f}%:{snap.energy_wh:.3f}Wh'))
            markers.append(self.marker(4,"uav/base_link",1.,f'UAV {snap.soc:.0%} | {snap.mode}',
                snap.soc-b.reserve_percentage,stamp))
        self.marker_pub.publish(MarkerArray(markers=markers))

    @staticmethod
    def marker(index,frame,height,text,margin,stamp):
        marker = Marker()
        marker.header.stamp, marker.header.frame_id = stamp, frame
        marker.ns, marker.id = "regional_energy", index
        marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        marker.pose.orientation.w, marker.pose.position.z = 1., float(height)
        marker.scale.z, marker.color.a = .22, 1.
        marker.color.r, marker.color.g, marker.color.b = (1.,.15,.05) if margin <= 0 else (.1,1.,.35)
        marker.text = text
        return marker


def main():
    from uav_control.runtime import run_node
    run_node(RegionalEnergyManager)
