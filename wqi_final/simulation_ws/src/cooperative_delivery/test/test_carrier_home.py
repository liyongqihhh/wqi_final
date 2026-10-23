import threading
from types import SimpleNamespace
from unittest.mock import Mock

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry

from cooperative_delivery.cooperative_mission_manager import CooperativeMissionManager


def test_carrier_pose_uses_map_coordinates_and_current_simulation_clock():
    odom = Odometry()
    odom.header.stamp.sec = 1791200000
    odom.pose.pose.position.x = 29.9
    odom.pose.pose.position.y = 10.7
    clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=212)))
    manager = SimpleNamespace(lock=threading.Lock(), carrier_home_pub=Mock(),
                              get_clock=lambda: clock)
    CooperativeMissionManager._ugv_ground_truth_callback(manager, odom)
    pose = manager.carrier_home_pub.publish.call_args.args[0]
    assert pose.header.frame_id == "map" and pose.header.stamp.sec == 212
    assert pose.pose.position.x == 29.9 and pose.pose.position.y == 10.7
    assert odom.header.stamp.sec == 1791200000
    assert manager.ugv_ground_truth is odom
