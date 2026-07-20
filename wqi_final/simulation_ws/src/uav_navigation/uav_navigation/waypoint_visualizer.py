import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from uav_navigation.waypoint_navigator import WaypointMap


class WaypointVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_visualizer")
        default_path = os.path.join(
            get_package_share_directory("uav_navigation"),
            "config",
            "uav_delivery_waypoints.yaml",
        )
        self.declare_parameter("waypoint_file", default_path)
        waypoint_file = str(self.get_parameter("waypoint_file").value)
        self.waypoint_map = WaypointMap(waypoint_file)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MarkerArray, "delivery_points", qos)
        self.timer = self.create_timer(1.0, self.publish_markers)

    @staticmethod
    def _color(marker, red, green, blue, alpha=1.0):
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha

    def publish_markers(self) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        corridor = Marker()
        corridor.header.frame_id = "map"
        corridor.header.stamp = stamp
        corridor.ns = "uav_air_corridors"
        corridor.id = 0
        corridor.type = Marker.LINE_LIST
        corridor.action = Marker.ADD
        corridor.pose.orientation.w = 1.0
        corridor.scale.x = 0.18
        self._color(corridor, 0.05, 0.75, 1.0, 0.65)
        cruise_altitude = float(self.waypoint_map.flight["cruise_altitude"])
        for start_name, end_name in self.waypoint_map.corridor_edges:
            for node_name in (start_name, end_name):
                node = self.waypoint_map.corridor_nodes[node_name]
                corridor.points.append(
                    Point(x=node.x, y=node.y, z=cruise_altitude)
                )
        markers.markers.append(corridor)

        corridor_nodes = Marker()
        corridor_nodes.header = corridor.header
        corridor_nodes.ns = "uav_air_corridor_nodes"
        corridor_nodes.id = 1
        corridor_nodes.type = Marker.SPHERE_LIST
        corridor_nodes.action = Marker.ADD
        corridor_nodes.pose.orientation.w = 1.0
        corridor_nodes.scale.x = 0.45
        corridor_nodes.scale.y = 0.45
        corridor_nodes.scale.z = 0.45
        self._color(corridor_nodes, 0.1, 0.9, 1.0, 0.8)
        for node in self.waypoint_map.corridor_nodes.values():
            corridor_nodes.points.append(
                Point(x=node.x, y=node.y, z=cruise_altitude)
            )
        markers.markers.append(corridor_nodes)

        for index, waypoint in enumerate(self.waypoint_map.waypoints.values()):
            marker_id = index * 4
            pad = Marker()
            pad.header.frame_id = "map"
            pad.header.stamp = stamp
            pad.ns = "uav_delivery_pads"
            pad.id = marker_id
            pad.type = Marker.CYLINDER
            pad.action = Marker.ADD
            pad.pose.position.x = waypoint.x
            pad.pose.position.y = waypoint.y
            pad.pose.position.z = 0.06
            pad.pose.orientation.w = 1.0
            pad.scale.x = 2.0
            pad.scale.y = 2.0
            pad.scale.z = 0.12
            if waypoint.name == self.waypoint_map.home.name:
                self._color(pad, 0.1, 0.8, 0.2, 0.85)
            else:
                self._color(pad, 0.95, 0.45, 0.05, 0.85)
            markers.markers.append(pad)

            delivery_altitude = self.waypoint_map.delivery_altitude_for(waypoint)
            if waypoint.name != self.waypoint_map.home.name:
                vertical = Marker()
                vertical.header = pad.header
                vertical.ns = "uav_delivery_vertical_routes"
                vertical.id = marker_id + 1
                vertical.type = Marker.LINE_LIST
                vertical.action = Marker.ADD
                vertical.pose.orientation.w = 1.0
                vertical.scale.x = 0.12
                vertical.points = [
                    Point(x=waypoint.x, y=waypoint.y, z=0.12),
                    Point(x=waypoint.x, y=waypoint.y, z=delivery_altitude),
                ]
                self._color(vertical, 1.0, 0.55, 0.05, 0.75)
                markers.markers.append(vertical)

                delivery_point = Marker()
                delivery_point.header = pad.header
                delivery_point.ns = "uav_floor_delivery_points"
                delivery_point.id = marker_id + 2
                delivery_point.type = Marker.SPHERE
                delivery_point.action = Marker.ADD
                delivery_point.pose.position.x = waypoint.x
                delivery_point.pose.position.y = waypoint.y
                delivery_point.pose.position.z = delivery_altitude
                delivery_point.pose.orientation.w = 1.0
                delivery_point.scale.x = 0.8
                delivery_point.scale.y = 0.8
                delivery_point.scale.z = 0.8
                self._color(delivery_point, 1.0, 0.2, 0.05, 0.95)
                markers.markers.append(delivery_point)

            label = Marker()
            label.header = pad.header
            label.ns = "uav_delivery_labels"
            label.id = marker_id + 3
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = waypoint.x
            label.pose.position.y = waypoint.y
            label.pose.position.z = (
                1.8
                if waypoint.name == self.waypoint_map.home.name
                else delivery_altitude + 1.0
            )
            label.pose.orientation.w = 1.0
            label.scale.z = 0.9
            label.text = waypoint.label
            if waypoint.delivery_floor is not None:
                label.text += f" ({delivery_altitude:.1f} m)"
            self._color(label, 1.0, 1.0, 1.0, 1.0)
            markers.markers.append(label)
        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
