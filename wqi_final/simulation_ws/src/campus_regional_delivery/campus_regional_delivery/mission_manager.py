"""Sequential regional transport with measured stopped-platform capture gates."""
import json
import math
import os
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from cooperative_delivery.relay_plan import CaptureSettleWindow, CaptureVelocityFilter, capture_pose_is_safe
from cooperative_delivery_interfaces.action import ExecuteCooperativeDelivery
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32, Float64, Int8, String
from std_srvs.srv import Trigger
from uav_interfaces.action import FlyToPose
from uav_interfaces.srv import SelectCarrier
import yaml

from campus_regional_delivery.layout import RegionalLayout, partition_orders


class RegionalMissionManager(Node):
    def __init__(self):
        super().__init__("regional_mission_manager")
        share=Path(get_package_share_directory("campus_regional_delivery"))
        self.declare_parameter("layout_file",str(share/"config/campus_regions.yaml"))
        self.declare_parameter("delivery_mode","cooperative")
        self.delivery_mode=self.get_parameter("delivery_mode").value
        if self.delivery_mode not in ("cooperative","ugv_only"):
            raise ValueError("Unknown delivery mode")
        self.layout=RegionalLayout(self.get_parameter("layout_file").value)
        self.geometry=yaml.safe_load((share/"assets/building_geometry.yaml").read_text())
        self.group=ReentrantCallbackGroup()
        self.busy=False
        self.busy_lock=threading.Lock()
        self.carrier=""
        self.uav_state=None
        self.connected=False
        self.uav_pose=None
        self.uav_received=0.
        self.poses={}
        self.received={}
        self.fault=""
        self.phase="IDLE"
        self.waiting_on="STARTUP"
        self.target=""
        self.active_vehicle=""
        self.nav_goal=None
        self.fly_goal=None
        self.landing_goal=None
        self.landing_target=None
        self.landing_namespace=""
        self.last_landing_log=0.
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub=self.create_publisher(String,"mission_status",qos)
        self.event_pub=self.create_publisher(String,"cargo_events",qos)
        self.admission_pub=self.create_publisher(String,"task_admission",qos)
        self.readiness_pub=self.create_publisher(String,"readiness",qos)
        self.landing_pub=self.create_publisher(String,"landing_status",qos)
        self.capture_pub=self.create_publisher(String,"capture_status",qos)
        self.readiness_pub.publish(String(data="WAITING_FOR_VEHICLES_AND_PX4"
            if self.delivery_mode=="cooperative" else "WAITING_FOR_VEHICLES"))
        self.path_pub=self.create_publisher(NavPath,"/uav/planned_path",qos)
        self.height_pub=self.create_publisher(Float64,"/uav/landing_height",10)
        self.payload_pub=self.create_publisher(Float32,"/uav/payload_mass",qos)
        self.flight_phase_pub=self.create_publisher(String,"/uav/mission_status",qos)
        self.create_subscription(String,"/uav/docked_carrier",self._carrier,qos,callback_group=self.group)
        self.create_subscription(Int8,"/uav/state",lambda m:setattr(self,"uav_state",m.data),qos,callback_group=self.group)
        self.create_subscription(Bool,"/uav/px4/connected",lambda m:setattr(self,"connected",m.data),qos,callback_group=self.group)
        self.create_subscription(String,"/uav/px4/landing_fault",lambda m:setattr(self,"fault",m.data),qos,callback_group=self.group)
        self.create_subscription(Odometry,"/uav/odom",self._uav,20,callback_group=self.group)
        self.nav_clients={}
        self.stop_pubs={}
        self.lifecycle={}
        self.navigation_managers={}
        self.paused_navigation=set()
        for r in self.layout.vehicles.values():
            ns=r["namespace"]
            self.create_subscription(Odometry,f"/{ns}/ground_truth/odom",lambda m,n=ns:self._car(n,m),20,callback_group=self.group)
            self.nav_clients[ns]=ActionClient(self,NavigateToPose,f"/{ns}/navigate_to_pose",callback_group=self.group)
            self.stop_pubs[ns]=self.create_publisher(Twist,f"/{ns}/cmd_vel_safe",10)
            self.navigation_managers[ns]=self.create_client(ManageLifecycleNodes,
                f"/{ns}/lifecycle_manager_navigation/manage_nodes",callback_group=self.group)
            for node in ("controller_server","bt_navigator"):
                self.lifecycle[f"/{ns}/{node}"]=self.create_client(GetState,f"/{ns}/{node}/get_state",callback_group=self.group)
        self.fly_client=ActionClient(self,FlyToPose,"/uav/fly_to_pose",callback_group=self.group)
        self.takeoff=self.create_client(Trigger,"/uav/takeoff",callback_group=self.group)
        self.land=self.create_client(Trigger,"/uav/land",callback_group=self.group)
        self.attach=self.create_client(Trigger,"/uav/attach_uav",callback_group=self.group)
        self.detach=self.create_client(Trigger,"/uav/detach_uav",callback_group=self.group)
        self.select=self.create_client(SelectCarrier,"/uav/select_carrier",callback_group=self.group)
        self.action=ActionServer(self,ExecuteCooperativeDelivery,"execute_mission",execute_callback=self.execute,
            goal_callback=self.goal,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.create_timer(1.,self.publish_readiness,callback_group=self.group)
        self.create_timer(1.,self.publish_landing_progress,callback_group=self.group)
        self.status_pub.publish(String(data="IDLE"))
        self.get_logger().info(
            "Regional task server ready: /regional_delivery/execute_mission; "
            f"mode={self.delivery_mode}; ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID','0')}; "
            f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION','default')}. "
            "Waiting for a delivery goal; launching the map does not submit an order.")

    def publish_readiness(self):
        if self.phase!="PREPARING" or self.waiting_on=="READY":
            return
        details={"waiting_for":self.waiting_on,
            "missing_odometry":[n for n in self.nav_clients if n not in self.poses],
            "px4_connected":self.connected,"uav_pose_received":self.uav_pose is not None,
            "uav_state":self.uav_state,"fly_action_ready":self.fly_client.server_is_ready()}
        self.readiness_pub.publish(String(data=json.dumps(details)))

    def _carrier(self,message):
        self.carrier=message.data

    def _car(self,namespace,message):
        self.poses[namespace]=message
        self.received[namespace]=time.monotonic()

    def _uav(self,message):
        self.uav_pose=message
        self.uav_received=time.monotonic()

    def landing_observation(self,namespace):
        now=time.monotonic()
        car=self.poses.get(namespace)
        air=self.uav_pose
        if car is None or air is None:
            return {"fresh":False,"carrier":self.carrier,"uav_state":self.uav_state}
        cp,up=car.pose.pose.position,air.pose.pose.position
        height=cp.z+float(self.layout.data["flight"]["landing_height"])
        values=(cp.x,cp.y,cp.z,up.x,up.y,up.z,height)
        fresh=(all(math.isfinite(v) for v in values)
            and 0<=now-self.uav_received<=3.
            and 0<=now-self.received.get(namespace,-math.inf)<=3.)
        return {"fresh":fresh,"carrier":self.carrier,"expected_carrier":namespace,
            "uav_state":self.uav_state,"uav_z_m":up.z,"target_z_m":height,
            "height_error_m":up.z-height,"xy_error_m":math.hypot(up.x-cp.x,up.y-cp.y)}

    def verify_docked(self,namespace,goal):
        # A capture-service reply alone is not a measured final landing pose.
        window=CaptureSettleWindow(duration_s=.5)
        started=self.get_clock().now().nanoseconds/1e9
        def verified():
            observation=self.landing_observation(namespace)
            safe=(observation["fresh"] and observation["uav_state"]==0
                and observation["carrier"]==namespace and not self.fault
                and abs(observation.get("height_error_m",math.inf))<=.05
                and observation.get("xy_error_m",math.inf)<=.12)
            stamps=[o.header.stamp.sec+o.header.stamp.nanosec/1e9
                for o in (self.poses.get(namespace),self.uav_pose) if o is not None]
            stamp=min(stamps) if len(stamps)==2 else started-1.
            return window.observe(stamp,safe and stamp>=started)
        self.waiting_on="MEASURED_DOCKING:"+namespace
        self.wait(verified,goal,30.)
        observation=self.landing_observation(namespace)
        observation.update(phase="DOCKED_VERIFIED",verified=True)
        self.landing_pub.publish(String(data=json.dumps(observation)))
        self.get_logger().info("Measured docking verified: "+json.dumps(observation))

    def publish_landing_progress(self):
        goal=self.landing_goal
        if goal is None or not goal.is_active or self.landing_target is None:
            return
        if self.landing_namespace:
            observation=self.landing_observation(self.landing_namespace)
        else:
            observation={"fresh":False,"uav_state":self.uav_state,"carrier":self.carrier}
        observation.update(phase=self.phase,waiting_for=self.waiting_on,verified=False)
        self.landing_pub.publish(String(data=json.dumps(observation)))
        feedback=ExecuteCooperativeDelivery.Feedback()
        feedback.phase,feedback.current_target,feedback.active_vehicle=self.phase,self.target,"UAV"
        feedback.uav_docked=bool(self.carrier)
        if observation["fresh"]:
            feedback.distance_remaining=math.hypot(observation["height_error_m"],observation["xy_error_m"])
        goal.publish_feedback(feedback)
        now=time.monotonic()
        if now-self.last_landing_log>=5.:
            self.get_logger().info("Landing progress: "+json.dumps(observation))
            self.last_landing_log=now

    def batches(self,request):
        masses=list(request.payload_masses_kg) or [.3]*len(request.targets)
        if self.delivery_mode=="ugv_only" and request.target_floors:
            raise ValueError("UGV-only comparison delivers at building doors, not upper floors")
        batches=partition_orders(self.layout,list(request.targets),masses,list(request.target_floors),
            enforce_uav_capacity=self.delivery_mode!="ugv_only")
        for _,orders in batches:
            for order in orders:
                if order["floor"]>self.geometry[order["target"]]["floors"]:
                    raise ValueError("Requested floor exceeds the existing building height")
        return batches

    def goal(self,request):
        try:
            self.batches(request)
        except (ValueError,KeyError,StopIteration) as error:
            self.report_admission(request,False,str(error) or "Target has no assigned region")
            return GoalResponse.REJECT
        with self.busy_lock:
            if self.busy:
                self.report_admission(request,False,"A regional delivery mission is already active")
                return GoalResponse.REJECT
            self.busy=True
        self.report_admission(request,True,"Accepted; starting regional readiness checks")
        return GoalResponse.ACCEPT

    def report_admission(self,request,accepted,reason):
        details={"accepted":accepted,"reason":reason,"targets":list(request.targets),
            "action":"/regional_delivery/execute_mission","delivery_mode":self.delivery_mode,
            "valid_targets":list(self.layout.buildings)}
        self.admission_pub.publish(String(data=json.dumps(details)))
        message="Regional goal "+("accepted: " if accepted else "rejected: ")+json.dumps(details)
        if accepted:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

    def phase_to(self,phase,goal,target="",vehicle=""):
        self.phase,self.target,self.active_vehicle=phase,target,vehicle
        self.status_pub.publish(String(data=f"{phase}:{target}:{vehicle}"))
        if phase in ("UAV_TAKEOFF","UAV_FLOOR_DELIVERY","FAILED","CANCELED"):
            self.landing_pub.publish(String(data=json.dumps({"phase":phase,"verified":False})))
        if self.delivery_mode=="cooperative":
            self.flight_phase_pub.publish(String(data=phase))
        feedback=ExecuteCooperativeDelivery.Feedback()
        feedback.phase,feedback.current_target,feedback.active_vehicle=phase,target,vehicle
        feedback.uav_docked=bool(self.carrier)
        goal.publish_feedback(feedback)
        self.get_logger().info(f"Regional phase: {phase} {target} {vehicle}")

    def wait(self,predicate,goal,timeout=600.):
        deadline=time.monotonic()+timeout
        while rclpy.ok() and time.monotonic()<deadline:
            if goal is not None and goal.is_cancel_requested:
                raise InterruptedError("Regional mission canceled")
            if predicate():
                return
            time.sleep(.05)
        raise TimeoutError(f"Timeout during {self.phase}; waiting for {self.waiting_on}")

    def future(self,future,goal,timeout=30.):
        self.wait(future.done,goal,timeout)
        result=future.result()
        if result is None:
            raise RuntimeError("ROS request returned no result")
        return result

    def call(self,client,goal,request=None):
        self.wait(client.service_is_ready,goal,30)
        result=self.future(client.call_async(request or Trigger.Request()),goal)
        if not result.success:
            raise RuntimeError(result.message)
        return result

    def ready(self,goal):
        self.waiting_on="VEHICLE_ODOMETRY"
        self.wait(lambda: all(n in self.poses for n in self.nav_clients),goal,300)
        if self.delivery_mode=="cooperative":
            self.waiting_on="PX4_AND_FLIGHT_ACTION"
            self.wait(lambda: self.connected and self.uav_pose is not None and self.uav_state==0
                and self.fly_client.server_is_ready(),goal,300)
            self.set_navigation_paused(False,goal)
        for name,client in self.lifecycle.items():
            self.waiting_on="NAVIGATION:"+name
            self.wait(client.service_is_ready,goal,60)
            def active(client=client):
                return self.future(client.call_async(GetState.Request()),goal).current_state.id==3
            self.wait(active,goal,120)
        self.waiting_on="READY"
        self.readiness_pub.publish(String(data="READY"))

    def set_navigation_paused(self,paused,goal,namespace=None):
        names=[namespace] if namespace else list(self.nav_clients)
        for ns in names:
            if (ns in self.paused_navigation)==paused:
                continue
            client=self.navigation_managers[ns]
            self.wait(client.service_is_ready,goal,30)
            if paused:
                self.stop_pubs[ns].publish(Twist())
                # Record an in-flight pause request so cancellation can restore it.
                self.paused_navigation.add(ns)
            command=ManageLifecycleNodes.Request.PAUSE if paused else ManageLifecycleNodes.Request.RESUME
            # Finish this lifecycle RPC before honoring cancellation; otherwise
            # the manager could pause a stack after our cleanup had already run.
            self.call(client,None,ManageLifecycleNodes.Request(command=command))
            if not paused:
                self.paused_navigation.remove(ns)

    def settled(self,namespace,goal):
        since=[None]
        filtered=CaptureVelocityFilter()
        def check():
            if namespace not in self.poses or time.monotonic()-self.received[namespace]>3:
                since[0]=None
                return False
            odom=self.poses[namespace]
            twist=odom.twist.twist
            stamp=odom.header.stamp.sec+odom.header.stamp.nanosec/1e9
            # Contact impulses must not repeatedly reset a stationary-car
            # window. The docking plugin still checks its original raw limits.
            velocity=filtered.observe(stamp,[twist.linear.x,twist.linear.y,twist.linear.z,
                twist.angular.x,twist.angular.y,twist.angular.z])
            if velocity is None:
                since[0]=None
                return False
            linear=math.sqrt(sum(v*v for v in velocity[:3]))
            angular=math.sqrt(sum(v*v for v in velocity[3:]))
            now=self.get_clock().now().nanoseconds/1e9
            if not (math.isfinite(linear) and math.isfinite(angular) and linear<.04 and angular<.08):
                since[0]=None
                return False
            if since[0] is None or now<since[0]:
                since[0]=now
            return now-since[0]>=1.
        self.wait(check,goal,180)

    def drive(self,namespace,pose,goal):
        if self.delivery_mode=="cooperative" and self.uav_state!=0:
            raise RuntimeError("Ground transport is forbidden while the UAV is airborne")
        if namespace in self.paused_navigation:
            self.set_navigation_paused(False,goal,namespace)
        self.wait(lambda: namespace in self.poses and time.monotonic()-self.received[namespace]<3,goal,30)
        p=self.poses[namespace].pose.pose.position
        if math.hypot(p.x-pose[0],p.y-pose[1])>.3:
            request=NavigateToPose.Goal()
            request.pose.header.frame_id="map"
            request.pose.pose.position.x,request.pose.pose.position.y=float(pose[0]),float(pose[1])
            request.pose.pose.orientation.z,request.pose.pose.orientation.w=math.sin(pose[2]/2),math.cos(pose[2]/2)
            client=self.nav_clients[namespace]
            self.wait(client.server_is_ready,goal,30)
            handle=self.future(client.send_goal_async(request),goal)
            if not handle.accepted:
                raise RuntimeError("UGV navigation goal rejected")
            self.nav_goal=handle
            result=self.future(handle.get_result_async(),goal,7200)
            self.nav_goal=None
            if result.status!=4:
                raise RuntimeError(f"UGV navigation failed: {result.status}")
        self.stop_pubs[namespace].publish(Twist())
        self.settled(namespace,goal)
        p=self.poses[namespace].pose.pose.position
        if math.hypot(p.x-pose[0],p.y-pose[1])>.5:
            raise RuntimeError("UGV Action ended outside the physical arrival tolerance")

    def fly(self,point,goal,platform=False,tolerance=.35):
        request=FlyToPose.Goal()
        request.target.header.frame_id="map"
        request.target.pose.position.x,request.target.pose.position.y,request.target.pose.position.z=map(float,point)
        request.target.pose.orientation.w=1.
        request.position_tolerance=float(tolerance)
        request.timeout=600.
        request.allow_platform_proximity=platform
        handle=self.future(self.fly_client.send_goal_async(request),goal)
        if not handle.accepted:
            raise RuntimeError("UAV rejected a regional waypoint")
        self.fly_goal=handle
        result=self.future(handle.get_result_async(),goal,3600)
        self.fly_goal=None
        if result.status!=4 or not result.result.success:
            raise RuntimeError(result.result.message)

    def hover(self,goal,duration=None):
        duration=self.layout.data["flight"]["hover_seconds"] if duration is None else duration
        until=self.get_clock().now().nanoseconds+int(duration*1e9)
        self.wait(lambda: self.get_clock().now().nanoseconds>=until,goal,180)

    def depart(self,goal):
        self.wait(lambda: self.connected and self.uav_state==0,goal,300)
        if self.carrier:
            self.settled(self.carrier,goal)
        self.call(self.takeoff,goal)
        if self.carrier:
            self.call(self.detach,goal)
            self.wait(lambda: not self.carrier,goal,15)
        self.wait(lambda: self.uav_state==1,goal,300)

    def capture(self,namespace,goal):
        self.waiting_on="STOPPED_PLATFORM:"+namespace
        self.get_logger().info(f"Waiting for stopped receiving platform: {namespace}")
        self.settled(namespace,goal)
        self.waiting_on="CAPTURE_WINDOW:"+namespace
        self.get_logger().info(f"Waiting for combined aircraft/platform capture window: {namespace}")
        window,filtered=CaptureSettleWindow(),CaptureVelocityFilter()
        last_report=[0.]
        observation={}
        def ready():
            if self.uav_state!=0 or self.fault:
                raise RuntimeError("PX4 has not safely landed/disarmed: "+self.fault)
            observation.clear()
            observation.update(self.landing_observation(namespace))
            if not observation["fresh"]:
                window.observe(self.get_clock().now().nanoseconds/1e9,False)
                observation.update(ready=False,reason="STALE_OR_MISSING_ODOMETRY")
                report()
                return False
            car,air=self.poses[namespace],self.uav_pose
            vectors=[]
            for odom in (car,air):
                for v in (odom.twist.twist.linear,odom.twist.twist.angular):
                    vectors.extend([v.x,v.y,v.z])
            stamp=min(o.header.stamp.sec+o.header.stamp.nanosec/1e9 for o in (car,air))
            velocities=filtered.observe(stamp,vectors)
            safe=velocities is not None and capture_pose_is_safe(car,air,
                float(self.layout.data["flight"]["landing_height"]),velocities)
            distance=math.hypot(observation["height_error_m"],observation["xy_error_m"])
            speeds=[math.sqrt(sum(v*v for v in velocities[i:i+3]))
                for i in range(0,12,3)] if velocities is not None else []
            stable=window.observe(stamp,safe)
            observation.update(ready=stable,distance_m=distance,filtered_speeds=speeds,
                reason="OUTSIDE_CAPTURE_DISTANCE" if distance>.17 else
                    "SETTLING" if safe else "VELOCITY_LIMIT",
                sample_sim_s=stamp)
            report()
            return stable
        def report():
            now=time.monotonic()
            if now-last_report[0]>=5.:
                observation.update(phase=self.phase,expected_carrier=namespace)
                message=json.dumps(observation)
                self.capture_pub.publish(String(data=message))
                self.get_logger().info("Capture gate: "+message)
                last_report[0]=now
        for attempt in range(3):
            try:
                self.wait(ready,goal,180)
            except TimeoutError as error:
                raise TimeoutError(str(error)+"; last capture observation="+
                    json.dumps(observation)) from error
            self.waiting_on="PHYSICAL_CAPTURE:"+namespace
            response=self.future(self.attach.call_async(Trigger.Request()),goal)
            if response.success:
                self.wait(lambda: self.carrier==namespace,goal,15)
                self.verify_docked(namespace,goal)
                return
            if attempt==2 or not response.message.startswith("Both vehicles must be settled"):
                raise RuntimeError(response.message)

    def land_at(self,point,goal,namespace=None):
        self.landing_goal,self.landing_target,self.landing_namespace=goal,point,namespace or ""
        try:
            self.phase_to("UAV_APPROACH",goal,namespace or "central_hub","UAV")
            self.waiting_on="DESCENDING_TO_APPROACH_HEIGHT"
            self.fly((point[0],point[1],self.layout.data["flight"]["ground_approach_height"]),goal,True,.10)
            height=float(self.layout.data["flight"]["landing_height"]) if namespace else .15
            self.height_pub.publish(Float64(data=height))
            if namespace:
                self.settled(namespace,goal)
            self.phase_to("UAV_LANDING",goal,namespace or "central_hub","UAV")
            self.waiting_on="PX4_TOUCHDOWN_AND_DISARM"
            self.call(self.land,goal)
            self.wait(lambda: self.uav_state==0,goal,600)
            if self.fault:
                raise RuntimeError("PX4 landing fault: "+self.fault)
            if namespace:
                self.waiting_on="STOPPED_PLATFORM_CAPTURE"
                self.capture(namespace,goal)
            p=self.uav_pose.pose.pose.position
            if math.hypot(p.x-point[0],p.y-point[1])>.5:
                raise RuntimeError("Landing ended outside the requested XY region")
        finally:
            self.landing_goal=None

    def air_transport(self,point,goal,namespace=None):
        if namespace:
            # Nav2 may finish within its tolerance rather than at the nominal
            # parking coordinate. Land on the measured stationary deck centre.
            self.settled(namespace,goal)
            receiver=self.poses[namespace].pose.pose.position
            point=(receiver.x,receiver.y)
        self.set_navigation_paused(True,goal)
        self.phase_to("UAV_TAKEOFF",goal,namespace or "central_hub","UAV")
        self.depart(goal)
        if namespace:
            self.call(self.select,goal,SelectCarrier.Request(model_name=namespace,link_name=namespace+"/base_footprint"))
        current=self.uav_pose.pose.pose.position
        altitude=float(self.layout.data["flight"]["cruise_altitude"])
        path=NavPath()
        path.header.frame_id="map"
        for xyz in ((current.x,current.y,altitude),(point[0],point[1],altitude),(point[0],point[1],.8)):
            pose=PoseStamped();pose.header=path.header
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,xyz)
            pose.pose.orientation.w=1.;path.poses.append(pose)
        self.path_pub.publish(path)
        self.fly((current.x,current.y,altitude),goal,True)
        self.phase_to("UAV_HOVER",goal,namespace or "central_hub","UAV")
        self.hover(goal)
        self.phase_to("UAV_TRANSPORT",goal,namespace or "central_hub","UAV")
        self.fly((point[0],point[1],altitude),goal)
        self.land_at(point,goal,namespace)

    def floor_delivery(self,namespace,order,goal):
        self.phase_to("UAV_FLOOR_DELIVERY",goal,order["target"],"UAV")
        self.settled(namespace,goal)
        p=self.poses[namespace].pose.pose.position
        point=(p.x,p.y)
        self.set_navigation_paused(True,goal)
        self.depart(goal)
        height=1.6+3.2*(order["floor"]-1)
        self.fly((*point,height),goal,True)
        self.hover(goal)
        self.payload_pub.publish(Float32(data=0.))
        self.land_at(point,goal,namespace)

    def ground_delivery(self,batches,goal,result):
        """Single central carrier baseline: identical door targets, no UAV mass."""
        hub=self.layout.hub
        ns=hub["namespace"]
        start=hub["depot_pose"][:2]+[hub["depot_pose"][3]]
        self.phase_to("UGV_PARKING",goal,"central_hub",ns)
        self.drive(ns,start,goal)
        for _,orders in batches:
            for order in orders:
                self.phase_to("UGV_LAST_MILE",goal,order["target"],ns)
                self.drive(ns,self.layout.buildings[order["target"]]["delivery_pose"],goal)
                self.phase_to("GROUND_DELIVERY",goal,order["target"],ns)
                self.hover(goal,self.layout.data["flight"]["ground_service_seconds"])
                self.event_pub.publish(String(data=json.dumps({"event":"SIMULATED_DELIVERED",
                    "mode":"ugv_only","carrier":ns,"target":order["target"],
                    "mass_kg":order["mass"],"floor":0})))
                result.completed_targets+=1
        if goal.request.return_home:
            self.phase_to("UGV_RETURN_HUB",goal,"central_hub",ns)
            self.drive(ns,start,goal)

    def execute(self,goal):
        result=ExecuteCooperativeDelivery.Result()
        try:
            batches=self.batches(goal.request)
            self.phase_to("PREPARING",goal)
            self.ready(goal)
            if self.delivery_mode=="ugv_only":
                self.ground_delivery(batches,goal,result)
                self.phase_to("COMPLETED",goal)
                result.success=True
                result.message="Central UGV-only door-delivery comparison completed"
                goal.succeed()
                return result
            hub=self.layout.hub["uav_pad"][:2]
            hub_ns=self.layout.hub["namespace"]
            p=self.uav_pose.pose.pose.position
            if self.carrier not in ("",hub_ns) or math.hypot(p.x-hub[0],p.y-hub[1])>.5:
                raise RuntimeError("Start a regional task with UAV landed on the central carrier")
            self.set_navigation_paused(True,goal)
            self.phase_to("UGV_PARKING",goal,"central_hub",hub_ns)
            self.settled(hub_ns,goal)
            if not self.carrier:
                self.call(self.select,goal,SelectCarrier.Request(model_name=hub_ns,link_name=hub_ns+"/base_footprint"))
                self.capture(hub_ns,goal)
            else:
                self.verify_docked(hub_ns,goal)
            for index,(region,orders) in enumerate(batches):
                spec=self.layout.regions[region]
                ns=spec["namespace"]
                depot=spec["depot_pose"]
                self.phase_to("UGV_PARKING",goal,region,ns)
                self.drive(ns,depot[:2]+[depot[3]],goal)
                self.payload_pub.publish(Float32(data=float(sum(o["mass"] for o in orders))))
                self.air_transport(depot[:2],goal,ns)
                self.phase_to("CARGO_HANDOVER",goal,region,ns)
                self.event_pub.publish(String(data=json.dumps({"event":"HANDOVER","region":region,
                    "carrier":self.carrier,"parcels":orders,
                    "transfer_point":spec["cargo_transfer"],"cargo_operation":"simulated"})))
                self.payload_pub.publish(Float32(data=0.))
                for order in orders:
                    self.phase_to("UGV_LAST_MILE",goal,order["target"],ns)
                    self.drive(ns,self.layout.buildings[order["target"]]["delivery_pose"],goal)
                    if order["floor"]:
                        self.payload_pub.publish(Float32(data=float(order["mass"])))
                        self.floor_delivery(ns,order,goal)
                    else:
                        self.phase_to("GROUND_DELIVERY",goal,order["target"],ns)
                        self.hover(goal,self.layout.data["flight"]["ground_service_seconds"])
                    self.event_pub.publish(String(data=json.dumps({"event":"SIMULATED_DELIVERED",
                        "region":region,"target":order["target"],"floor":order["floor"],"mass_kg":order["mass"]})))
                    result.completed_targets+=1
                self.phase_to("UGV_RETURN_DEPOT",goal,region,ns)
                self.drive(ns,depot[:2]+[depot[3]],goal)
                if goal.request.return_home or index+1<len(batches):
                    self.air_transport(hub,goal,hub_ns)
            self.set_navigation_paused(False,goal)
            self.verify_docked(hub_ns if goal.request.return_home else ns,goal)
            self.phase_to("COMPLETED",goal)
            result.success=True
            result.message="Regional transport, simulated handover and last-mile delivery completed"
            goal.succeed()
        except Exception as error:
            if self.nav_goal:
                self.nav_goal.cancel_goal_async()
            if self.fly_goal:
                self.fly_goal.cancel_goal_async()
            for publisher in self.stop_pubs.values():
                publisher.publish(Twist())
            self.phase_to("CANCELED" if goal.is_cancel_requested else "FAILED",goal)
            result.success=False
            result.message=str(error)
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
            self.get_logger().error(result.message)
        finally:
            if self.delivery_mode=="cooperative" and self.uav_state==0 and self.paused_navigation:
                try:
                    self.set_navigation_paused(False,None)
                except Exception as error:
                    self.get_logger().error("Could not restore parked navigation: "+str(error))
            with self.busy_lock:
                self.busy=False
        return result


def main():
    rclpy.init()
    node=RegionalMissionManager()
    executor=MultiThreadedExecutor(num_threads=6)
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
