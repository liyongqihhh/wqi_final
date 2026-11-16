from collections import deque
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformException

from uav_control.flight_action_server import FlightActionServer


def make_controller(ready_until):
    def cloud(nanos):
        header=Header(frame_id='uav/base_link')
        header.stamp.sec,header.stamp.nanosec=divmod(nanos,1_000_000_000)
        return point_cloud2.create_cloud_xyz32(header,[(2.,0.,0.)])
    entries=[(stamp,cloud(stamp),time.monotonic())
        for stamp in (10_100_000_000,10_200_000_000)]
    tracker=SimpleNamespace(update=Mock(),prediction_points=lambda:(),tracks=())
    def lookup(parent,child,stamp):
        assert parent=='map' and child=='uav/base_link'
        if stamp.nanoseconds>ready_until[0]:raise TransformException('future cloud')
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.,y=0.,z=0.),
            rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    return SimpleNamespace(lock=threading.Lock(),pending_cloud=deque(entries,maxlen=8),
        cloud_guard=None,cloud_tracker=tracker,
        cloud_tf=SimpleNamespace(lookup_transform=Mock(side_effect=lookup)),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_200_000_000)))


def test_controller_uses_exact_older_stamp_then_consumes_new_frame_when_tf_arrives():
    ready=[10_150_000_000]
    node=make_controller(ready)
    FlightActionServer._sync_cloud(node)
    assert node.cloud_guard.stamp_ns==10_100_000_000
    assert [e[0] for e in node.pending_cloud]==[10_200_000_000]
    assert node.cloud_tracker.update.call_count==1
    ready[0]=10_300_000_000
    FlightActionServer._sync_cloud(node)
    assert node.cloud_guard.stamp_ns==10_200_000_000
    assert not node.pending_cloud
    assert node.cloud_tracker.update.call_count==2


def test_no_matching_tf_never_falls_back_to_latest_or_discards_fresh_frames():
    node=make_controller([0])
    FlightActionServer._sync_cloud(node)
    assert node.cloud_guard is None
    assert len(node.pending_cloud)==2
    node.cloud_tracker.update.assert_not_called()
    assert [call.args[2].nanoseconds for call in node.cloud_tf.lookup_transform.call_args_list]==[
        10_200_000_000,10_100_000_000]
