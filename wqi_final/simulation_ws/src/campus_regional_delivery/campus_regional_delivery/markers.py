"""Labels and actual layout coordinates shared by Gazebo, Nav2 and RViz."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from campus_regional_delivery.layout import RegionalLayout


class RegionalMarkers(Node):
    def __init__(self):
        super().__init__("regional_markers")
        default = Path(get_package_share_directory("campus_regional_delivery"))/"config/campus_regions.yaml"
        self.declare_parameter("layout_file", str(default))
        self.layout = RegionalLayout(self.get_parameter("layout_file").value)
        self.pub = self.create_publisher(MarkerArray, "/regional_delivery/markers",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(2.0, self.publish)
        self.publish()

    def publish(self):
        result = MarkerArray()
        def text(label, x, y, z, color, scale=1.0):
            item = Marker()
            item.header.frame_id = "map"
            item.header.stamp = self.get_clock().now().to_msg()
            item.ns, item.id = "regional_layout", len(result.markers)
            item.type, item.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            item.pose.position.x, item.pose.position.y, item.pose.position.z = float(x),float(y),float(z)
            item.pose.orientation.w = 1.
            item.scale.z = scale
            item.color.r,item.color.g,item.color.b,item.color.a = [float(v) for v in color]
            item.text = label
            result.markers.append(item)
        hub = self.layout.hub["uav_pad"]
        text("CENTRAL HUB / UGV-00 / UAV",hub[0],hub[1],2,[.95,.95,.95,1])
        for key,r in self.layout.regions.items():
            x,y=r["depot_pose"][:2]
            text(f"DEPOT {key} / {r['vehicle_label']}",x,y,2,r["color"])
            for target in r["targets"]:
                p=self.layout.buildings[target]["delivery_pose"]
                text(target,p[0],p[1],1.,r["color"],.8)
        self.pub.publish(result)


def main():
    rclpy.init()
    node=RegionalMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
