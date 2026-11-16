from uav_control.flight_timing import flight_deadline_expired,replan_clock,recent_cloud_candidates


def test_slow_simulation_does_not_consume_the_flight_simulation_deadline():
    assert not flight_deadline_expired(700,50,600,True,3600)
    assert flight_deadline_expired(700,50,600,False,3600)


def test_wall_watchdog_and_simulation_reset_still_fail_closed():
    assert flight_deadline_expired(3600,100,600,True,3600)
    assert flight_deadline_expired(700,600,600,True,3600)
    assert flight_deadline_expired(10,-1,600,True,3600)
    assert flight_deadline_expired(10,float("nan"),600,True,3600)


def test_slow_simulation_replanning_uses_sensor_time_not_repeated_wall_ticks():
    assert replan_clock(700.,50.,True)==50.
    assert replan_clock(704.,50.5,True)-replan_clock(700.,50.,True)==.5
    assert replan_clock(700.,50.,False)==700.


def test_late_tf_can_use_an_older_fresh_cloud_without_wrong_stamp_fallback():
    entries=[(10_100_000_000,'older',100.),(10_200_000_000,'newest',100.1)]
    candidates=recent_cloud_candidates(entries,0,10_200_000_000,100.2)
    assert [e[1] for e in candidates]==['newest','older']
    transform_ready_until=10_150_000_000
    usable=next(e for e in candidates if e[0]<=transform_ready_until)
    assert usable[1]=='older' and usable[0]==10_100_000_000


def test_cloud_candidates_keep_original_freshness_bounds_and_reject_zero_stamp():
    entries=[(0,'zero',100.),(8_000_000_000,'stale_sim',100.),
        (10_200_000_000,'stale_wall',60.),(11_000_000_000,'future',100.),
        (10_100_000_000,'used',100.),(10_200_000_000,'valid',100.)]
    assert [e[1] for e in recent_cloud_candidates(entries,10_100_000_000,10_300_000_000,100.2)]==['valid']


def test_cloud_candidates_sort_out_of_order_arrivals_but_do_not_reuse_consumed_cloud():
    entries=[(10_200_000_000,'new',100.),(10_100_000_000,'late_old',100.1)]
    assert recent_cloud_candidates(entries,10_200_000_000,10_300_000_000,100.2)==[]
