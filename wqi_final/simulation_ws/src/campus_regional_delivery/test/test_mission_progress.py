from campus_regional_delivery.mission_progress import RouteProgressWatchdog
import pytest


def test_continued_oscillation_does_not_reset_route_progress():
    guard=RouteProgressWatchdog(0,60)
    for t,d in ((0,56),(10,55),(20,60),(30,56),(40,59),(60,55),(70,56)):
        guard.observe(d,t)
    assert guard.expired(71)


def test_road_progress_resets_deadline_even_when_away_from_building():
    guard=RouteProgressWatchdog(0,60)
    for t,d in ((0,56),(30,53),(60,50),(90,48)):
        guard.observe(d,t)
        assert not guard.expired(t+20)


def test_invalid_or_zero_feedback_does_not_invent_arrival_progress():
    guard=RouteProgressWatchdog(0,60)
    guard.observe(float("nan"),50)
    guard.observe(0.,59)
    assert guard.expired(61)
    disabled=RouteProgressWatchdog(0,0)
    assert not disabled.expired(10000)


@pytest.mark.parametrize("timeout",[-1.,float("nan"),float("inf")])
def test_invalid_progress_timeout_is_rejected(timeout):
    with pytest.raises(ValueError):RouteProgressWatchdog(0,timeout)


def test_previous_goal_short_path_does_not_poison_new_goal_progress():
    guard=RouteProgressWatchdog(0,60)
    assert not guard.observe(.505,0.,32.)
    assert not guard.observe(.305,.05,32.)
    assert guard.sample[0] is None
    for distance,t in ((45.56,.1),(42.,20.),(37.,40.),(30.,60.),(25.,80.)):
        assert guard.observe(distance,t,20.)
        assert not guard.expired(t+10.)


def test_invalid_prior_plan_feedback_does_not_hide_a_genuinely_stopped_vehicle():
    guard=RouteProgressWatchdog(0,60)
    for t in (0,20,40,60):
        assert not guard.observe(.3,t,32.)
    assert guard.expired(61.)


def test_projection_tolerance_allows_normal_small_cross_track_error():
    guard=RouteProgressWatchdog(0,60)
    assert guard.observe(5.,1.,5.4)
    assert not guard.observe(5.,2.,float('nan'))
