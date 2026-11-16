import numpy as np
import pytest

from uav_control.cloud_motion_tracker import CloudMotionTracker
from uav_control.local_cloud_guard import LocalCloudGuard


def test_world_frame_motion_uses_timestamps_and_not_robot_relative_movement():
    tracker=CloudMotionTracker()
    for stamp in np.arange(0.,1.1,.1):
        tracker.update([(5.,2.-.8*stamp,10.),(5.1,2.-.8*stamp,10.)],float(stamp))
    assert tracker.tracks[0].velocity[1]==pytest.approx(-.8,abs=.08)
    assert np.min(tracker.prediction_points()[:,1])<-.8


def test_missed_scan_coasts_then_expires_without_infinite_ghost_occupancy():
    tracker=CloudMotionTracker()
    for stamp in np.arange(0.,1.1,.1):
        tracker.update([(5.,2.-.8*stamp,10.)],float(stamp))
    tracker.update([],1.5)
    assert len(tracker.prediction_points())>0
    tracker.update([],4.1)
    assert not tracker.tracks and len(tracker.prediction_points())==0


def test_stationary_compact_returns_do_not_create_future_motion():
    tracker=CloudMotionTracker()
    for stamp in np.arange(0.,1.1,.1):
        tracker.update([(5.,2.,10.)],float(stamp))
    assert np.linalg.norm(tracker.tracks[0].velocity)<1e-8
    assert len(tracker.prediction_points())==0


def test_large_wall_is_not_tracked_as_one_moving_actor():
    tracker=CloudMotionTracker()
    tracker.update([(4.,y,10.) for y in np.arange(-5.,5.,.5)],0.)
    assert not tracker.tracks


def test_out_of_order_scan_cannot_reverse_estimated_motion():
    tracker=CloudMotionTracker()
    tracker.update([(5.,2.,10.)],1.)
    tracker.update([(10.,8.,10.)],.5)
    assert tracker.stamp==1. and len(tracker.tracks)==1
    with pytest.raises(ValueError):CloudMotionTracker(speed_bound=float('nan'))


def test_crossing_prediction_blocks_a_route_clear_in_the_instantaneous_cloud():
    tracker=CloudMotionTracker()
    for stamp in np.arange(0.,1.1,.1):
        tracker.update([(5.,4.8-.8*stamp,10.)],float(stamp))
    raw=LocalCloudGuard([(5,4,10)],(0,0,0),(0,0,0,1),1,1.)
    assert raw.path_clear(((0,0,10),(10,0,10)),3.)
    raw.include_prediction(tracker.prediction_points())
    assert not raw.path_clear(((0,0,10),(10,0,10)),3.)
