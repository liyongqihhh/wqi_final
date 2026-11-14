"""Energy admission and measured charging gates for the regional state machine."""
import json
import math
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32, String

from campus_regional_delivery.energy_model import CargoLedger, NAMESPACES, fresh_sample, load_energy_configuration
from campus_regional_delivery.energy_plan import RegionalEnergyPlanner


class RegionalEnergyMixin:
    def init_energy(self, qos):
        def share(name):
            return Path(get_package_share_directory(name))
        defaults = {"enable_energy_constraints": False,
            "energy_file": str(share("campus_regional_delivery")/"config/regional_energy.yaml"),
            "uav_energy_file": str(share("uav_control")/"config/battery_model.yaml"),
            "ugv_energy_file": str(share("cooperative_delivery")/"config/ugv_energy_model.yaml")}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.enable_energy = bool(self.get_parameter("enable_energy_constraints").value)
        if not self.enable_energy:
            return
        self.energy_settings, self.uav_energy_parameters, self.ugv_energy_parameters, self.energy_digest = load_energy_configuration(
            *(self.get_parameter(name).value for name in ("energy_file", "uav_energy_file", "ugv_energy_file")))
        self.energy_planner = RegionalEnergyPlanner(self.layout,self.energy_settings,
            self.uav_energy_parameters,self.ugv_energy_parameters)
        self.energy_snapshot, self.energy_received, self.energy_session = None, 0., ""
        self.last_energy_guard = 0.
        self.cargo_ledger = CargoLedger()
        self.energy_command_pub = self.create_publisher(String,"/regional_delivery/energy_command",10)
        self.energy_plan_pub = self.create_publisher(String,"energy_plan",qos)
        self.energy_ready_pub = self.create_publisher(String,"energy_readiness",qos)
        self.create_subscription(String,"/regional_delivery/energy_status",self.receive_energy,qos,
                                 callback_group=self.group)
        self.create_timer(.5,self.publish_energy_readiness,callback_group=self.group)

    def publish_energy_readiness(self):
        try:
            self.energy_data()
            status="READY"
        except RuntimeError as error:
            status="WAITING:"+str(error)
        self.energy_ready_pub.publish(String(data=status))

    def receive_energy(self, message):
        try:
            data = json.loads(message.data)
            values = data["energy_wh"]
            expected = {f"{ns}_{kind}" for ns in NAMESPACES for kind in ("drive","charge")}
            if self.delivery_mode == "cooperative":
                expected.add("uav")
            if (set(values) != expected or data["config_digest"] != self.energy_digest
                    or any(not isinstance(v,(int,float)) or not math.isfinite(v) or v < 0 for v in values.values())):
                raise ValueError("Missing packs, nonfinite energy or inconsistent energy configuration")
            self.energy_snapshot, self.energy_received = data, time.monotonic()
        except (ValueError,KeyError,TypeError) as error:
            self.get_logger().error("Energy telemetry rejected: "+str(error))

    def energy_data(self):
        data = self.energy_snapshot
        if not data:
            raise RuntimeError("Energy telemetry not received by task manager yet; wait for energy_readiness=READY")
        if not fresh_sample(data.get("sample_sim_ns"),self.energy_received,
                self.get_clock().now().nanoseconds,time.monotonic(),self.energy_settings):
            sim_age=(self.get_clock().now().nanoseconds-data["sample_sim_ns"])/1e9
            wall_age=time.monotonic()-self.energy_received
            raise RuntimeError(f"Energy telemetry stale: sim_age={sim_age:.3f}s wall_age={wall_age:.3f}s")
        if not data.get("ready"):
            raise RuntimeError("Energy telemetry not ready: "+str(data.get("missing_telemetry")))
        if self.energy_session and data["session_id"] != self.energy_session:
            raise RuntimeError("Energy node restarted during a mission; battery continuity is not proven")
        return data

    def energy_admission(self, request):
        data = self.energy_data()
        if sum(data["cargo_kg"].values())+data["payload_kg"] > 1e-6:
            raise RuntimeError("Undelivered cargo remains from an earlier mission; recover it before accepting a new order")
        positions = {}
        for ns in NAMESPACES:
            if ns not in self.poses or time.monotonic()-self.received.get(ns,0.) > 3.:
                raise RuntimeError("Missing fresh parking position for "+ns)
            p = self.poses[ns].pose.pose.position
            positions[ns] = (p.x,p.y)
        assessment = self.energy_planner.assess(self.batches(request),request.return_home,
            positions,data["energy_wh"],self.delivery_mode=="ugv_only")
        self.energy_plan_pub.publish(String(data=json.dumps(assessment,allow_nan=False)))
        if not assessment["feasible"]:
            raise RuntimeError("Nine-pack energy admission rejected: "+assessment["reason"])
        return assessment

    def sync_energy_cargo(self, goal, charge_target=None):
        data = self.energy_data()
        command = {"session_id":data["session_id"],"previous_sequence":data["command_sequence"],
            "sequence":data["command_sequence"]+1,"cargo_kg":dict(self.cargo_ledger.cargo),
            "payload_kg":self.cargo_ledger.payload}
        if charge_target is not None:
            command["charge_target_soc"] = float(charge_target)
        self.energy_command_pub.publish(String(data=json.dumps(command,allow_nan=False)))
        def acknowledged():
            observed = self.energy_data()
            if observed["command_error"]:
                raise RuntimeError("Energy cargo/charging command rejected: "+observed["command_error"])
            return observed["command_sequence"] == command["sequence"]
        self.wait(acknowledged,goal,30.)
        self.payload_pub.publish(Float32(data=float(self.cargo_ledger.payload)))

    def energy_start(self, request, batches, goal):
        self.energy_session = ""
        self.energy_admission(request)
        self.energy_session = self.energy_data()["session_id"]
        self.cargo_ledger = CargoLedger()
        cargo = dict(self.cargo_ledger.cargo)
        cargo[self.layout.hub["namespace"]] = sum(o["mass"] for _, orders in batches for o in orders)
        self.cargo_ledger.assign(cargo,0.)
        self.sync_energy_cargo(goal)

    def energy_transfer(self, source, destination, mass, goal):
        self.cargo_ledger.transfer(source,destination,mass)
        self.sync_energy_cargo(goal)

    def energy_guard(self):
        if time.monotonic()-self.last_energy_guard < .4:
            return
        self.last_energy_guard = time.monotonic()
        data = self.energy_data()
        for ns in NAMESPACES:
            if data["energy_wh"][ns+"_drive"] < self.ugv_energy_parameters.drive_reserve_wh:
                raise RuntimeError(ns+" drive battery reached safety reserve; stop and recover")
        if (self.delivery_mode=="cooperative" and self.uav_state in (1,2)
                and data["energy_wh"]["uav"] < self.uav_energy_parameters.capacity_wh*self.uav_energy_parameters.reserve_percentage):
            raise RuntimeError("UAV reached safe-return reserve; interrupt order and recover")

    def energy_before_drive(self, namespace, pose):
        data = self.energy_data()
        p = self.poses[namespace].pose.pose.position
        spec = next(r for r in self.layout.vehicles.values() if r["namespace"]==namespace)
        home = spec["depot_pose"][:2]
        distance = self.energy_planner.road.distance((p.x,p.y),pose[:2])
        distance += self.energy_planner.road.distance(pose[:2],home)
        seconds = distance/self.ugv_energy_parameters.planning_speed_mps*self.ugv_energy_parameters.prediction_margin_factor
        seconds += self.energy_settings["drive_turn_allowance_s"]
        cargo = self.cargo_ledger.cargo[namespace] + (self.cargo_ledger.payload if self.carrier==namespace else 0.)
        from cooperative_delivery.ugv_energy_model import UgvEnergyModel
        power = UgvEnergyModel(self.ugv_energy_parameters).drive_power_w(
            self.ugv_energy_parameters.planning_speed_mps,0.,0.,cargo,self.carrier==namespace)
        if data["energy_wh"][namespace+"_drive"] < power*seconds/3600+self.ugv_energy_parameters.drive_reserve_wh:
            raise RuntimeError(namespace+" cannot reach target and its depot above drive reserve")

    def energy_before_flight(self, point, goal, floor=0):
        data = self.energy_data()
        source = self.carrier
        if source not in NAMESPACES or self.uav_state != 0:
            raise RuntimeError("Charging/takeoff requires a landed UAV locked to a known carrier")
        p = self.poses[source].pose.pose.position
        outbound,_ = self.energy_planner.flight((p.x,p.y),point,self.cargo_ledger.payload,floor)
        # Always retain an unloaded return-to-hub budget, even if return_home is false.
        hub = self.layout.hub["uav_pad"][:2]
        return_energy = 0. if math.dist(point[:2],hub)<.5 else self.energy_planner.flight(point,hub,0.)[0]
        required = outbound+return_energy+self.uav_energy_parameters.capacity_wh*self.uav_energy_parameters.reserve_percentage
        target = 1. if self.energy_settings["charging_policy"]=="full" else required/self.uav_energy_parameters.capacity_wh+.005
        if target > 1.:
            raise RuntimeError("Sortie plus safe return exceeds UAV battery capacity")
        b, g = self.uav_energy_parameters,self.ugv_energy_parameters
        deficit = max(0.,target*b.capacity_wh-data["energy_wh"]["uav"])
        seconds = deficit*3600/b.net_charge_power_w
        source_wh = (deficit+b.docked_idle_power_w*seconds/3600)/(b.charge_efficiency*g.charger_transfer_efficiency)
        source_wh += g.charger_idle_power_w*seconds/3600
        if data["energy_wh"][source+"_charge"] < source_wh+g.charging_reserve_wh:
            raise RuntimeError(source+" charging pack cannot supply the required sortie")
        for ns in NAMESPACES:
            if data["energy_wh"][ns+"_drive"] < g.idle_power_w*seconds/3600+g.drive_reserve_wh:
                raise RuntimeError(ns+" drive pack cannot support the charging wait")
        self.sync_energy_cargo(goal,target)
        if deficit <= 0:
            return
        self.phase_to("UAV_CHARGING",goal,source,"UAV")
        started = self.get_clock().now().nanoseconds
        def charged():
            status = self.energy_data()
            if status["energy_wh"]["uav"] >= target*b.capacity_wh-.02:
                return True
            if status["energy_wh"][source+"_charge"] <= g.charging_reserve_wh+1e-6:
                raise RuntimeError(source+" charging pack exhausted above its reserve")
            if (self.get_clock().now().nanoseconds-started)/1e9 > seconds+60.:
                raise TimeoutError("UAV charging made insufficient progress; inspect actual docking")
            return False
        self.wait(charged,goal,max(300.,seconds*20+120.))

    def energy_finish(self):
        if self.delivery_mode=="cooperative" and self.uav_state==0:
            self.sync_energy_cargo(None,self.energy_settings["idle_charge_target_soc"])
        self.energy_session = ""
