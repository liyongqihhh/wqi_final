"""Truth-based visualization/evidence only; never supplies navigation obstacles."""
import json
import math
import time

from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray


class RegionalDynamicMonitor(Node):
    def __init__(self):
        super().__init__("regional_dynamic_monitor")
        self.shutdown_requested=False
        self.manifest=None;self.actor_samples={};self.robot_samples={};self.actor_subs={}
        self.contact_subs={};self.contact_pairs=set()
        self.pairs=set();self.stats={}
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub=self.create_publisher(MarkerArray,"dynamic_markers",qos)
        self.status_pub=self.create_publisher(String,"dynamic_status",qos)
        self.create_subscription(String,"/dynamic_obstacles/manifest",self.configure,qos)
        for name in ("ugv_00","ugv_01","ugv_02","ugv_03","uav"):
            topic="/uav/odom" if name=="uav" else f"/{name}/ground_truth/odom"
            self.create_subscription(Odometry,topic,lambda m,n=name:self.sample(n,m,False),10)
            self.stats[name]={"moving_actor_encounters":0,"min_center_m":None,
                "min_bounding_clearance_m":None,"planner_messages":0,"last_planner_status":""}
            self.stats[name]["actors_with_physical_contact"]=0
            status="/uav/dynamic_replanning/status" if name=="uav" else f"/{name}/ugv/dstar_lite/status"
            self.create_subscription(String,status,lambda m,n=name:self.planner(n,m),10)
        self.create_timer(.5,self.publish)

    def sample(self,name,message,actor):
        p=message.pose.pose.position;v=message.twist.twist.linear
        if not all(math.isfinite(x) for x in (p.x,p.y,p.z,v.x,v.y,v.z)):
            return
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        (self.actor_samples if actor else self.robot_samples)[name]=(
            (p.x,p.y,p.z),math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z),stamp,time.monotonic())

    def configure(self,message):
        try:
            data=json.loads(message.data)
            if data["failures"] or data["spawned"]!=data["requested"]:
                self.get_logger().error("Dynamic models failed to spawn; no acceptance")
            self.manifest=data
            for route in data["routes"]:
                name=route["model"]
                if name not in self.actor_subs:
                    self.actor_subs[name]=self.create_subscription(Odometry,f"/dynamic_obstacles/{name}/odom",
                        lambda m,n=name:self.sample(n,m,True),10)
                    self.contact_subs[name]=self.create_subscription(ContactsState,f"/dynamic_obstacles/{name}/contacts",
                        lambda m,n=name:self.contact(n,m),10)
        except (ValueError,KeyError,TypeError) as error:
            self.get_logger().error(str(error))

    def planner(self,name,message):
        self.stats[name]["planner_messages"]+=1
        self.stats[name]["last_planner_status"]=message.data

    def contact(self,actor,message):
        for state in message.states:
            names=(state.collision1_name,state.collision2_name)
            for robot in self.stats:
                model="campus_uav" if robot=="uav" else robot
                pair=(robot,actor)
                if any(name.startswith(model+"::") for name in names) and pair not in self.contact_pairs:
                    self.contact_pairs.add(pair);self.stats[robot]["actors_with_physical_contact"]+=1
                    self.get_logger().error(f"DYNAMIC_PHYSICAL_CONTACT robot={robot} actor={actor}")

    def publish(self):
        if not self.manifest:
            return
        now=self.get_clock().now();seconds=now.nanoseconds/1e9
        def fresh(sample):
            return bool(sample and -.25<=seconds-sample[2]<=1.5 and time.monotonic()-sample[3]<30)
        markers=[]
        active_ground=active_air=0
        for index,route in enumerate(self.manifest["routes"]):
            name=route["model"];sample=self.actor_samples.get(name)
            if not fresh(sample):continue
            p,speed,_,_=sample
            active= speed>.05
            if active:
                if route["layer"]=="air":active_air+=1
                else:active_ground+=1
            marker=Marker();marker.header.frame_id="map";marker.header.stamp=now.to_msg()
            marker.ns="regional_dynamic_traffic";marker.id=index
            marker.action=Marker.ADD;marker.type=Marker.SPHERE if route["layer"]=="air" else Marker.CYLINDER
            marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=p
            marker.pose.orientation.w=1.
            marker.scale.x=marker.scale.y=2*route["radius"]
            marker.scale.z=2*route["radius"] if route["layer"]=="air" else 1.6
            marker.color.r,marker.color.g,marker.color.b=(1.,.15,.1) if route["layer"]=="air" else (1.,.7,.05)
            marker.color.a=.9 if active else .35
            markers.append(marker)
            if route["layer"]=="air":
                line=Marker();line.header=marker.header;line.ns="air_crossing_routes";line.id=index
                line.action=Marker.ADD;line.type=Marker.LINE_STRIP;line.pose.orientation.w=1.
                line.scale.x=.04;line.color.r=1.;line.color.b=.15;line.color.a=.35
                line.points=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in route["waypoints"]]
                markers.append(line)
            for robot,observation in self.robot_samples.items():
                if not fresh(observation) or (robot=="uav")!=(route["layer"]=="air"):
                    continue
                if abs(sample[2]-observation[2])>.15:
                    continue
                distance=math.dist(p,observation[0]) if robot=="uav" else math.dist(p[:2],observation[0][:2])
                stats=self.stats[robot]
                clearance=distance-route["radius"]-.60
                if stats["min_center_m"] is None or distance<stats["min_center_m"]:
                    stats["min_center_m"]=distance;stats["min_bounding_clearance_m"]=clearance
                pair=(robot,name)
                if active and distance<=(10. if robot=="uav" else 7.) and pair not in self.pairs:
                    self.pairs.add(pair);stats["moving_actor_encounters"]+=1
                    self.get_logger().info(f"DYNAMIC_ENCOUNTER robot={robot} actor={name} center_distance={distance:.3f}m")
        self.marker_pub.publish(MarkerArray(markers=markers))
        self.status_pub.publish(String(data=json.dumps({"density":self.manifest["density"],
            "spawned":self.manifest["spawned"],"moving_ground":active_ground,"moving_air":active_air,
            "robots":self.stats,"scope":"Truth diagnostics, not sensor input or collision/avoidance-success proof"})))


def main():
    from uav_control.runtime import run_node
    run_node(RegionalDynamicMonitor)
