import math

import pytest

from uav_control.local_cloud_guard import LocalCloudGuard,filtered_cloud


def guard(points):
    return LocalCloudGuard(points,(0,0,0),(0,0,0,1),1_000_000_000,100.)


def test_filter_removes_ground_self_and_nonfinite_returns_without_inventing_centroids():
    points=[(0,0,-.45),(2,0,-1.36),(3.,0.,0.),(3.1,0.,0.),(float("nan"),0,0)]
    result=filtered_cloud(points,.45,.18,1.,.36,.05,.58)
    assert result==((3.,0.,.45),)


def test_sensor_time_transform_preserves_two_distinct_obstacles():
    g=LocalCloudGuard([(3,0,0),(6,1,1)],(10,20,5),(0,0,math.sqrt(.5),math.sqrt(.5)),1_000_000_000,100.)
    assert g.points[0]==pytest.approx((10,23,5))
    assert g.points[1]==pytest.approx((9,26,6))


def test_actual_humble_pointcloud_records_are_converted_without_a_dtype_cast_failure():
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    message=point_cloud2.create_cloud_xyz32(Header(),[(3.,0.,0.),(5.,1.,1.)])
    points=tuple(point_cloud2.read_points(message,field_names=("x","y","z"),skip_nans=True))
    assert guard(points).first_conflict((0,0,0),(8,0,0))==(3.,0.,0.)


def test_empty_measured_cloud_and_old_or_future_timestamps():
    g=guard([])
    assert g.path_clear(((0,0,0),(10,0,0)))
    assert g.fresh(1_100_000_000,101.)
    assert not g.fresh(2_000_000_000,101.)
    assert not g.fresh(0,101.)
    assert not g.fresh(1_100_000_000,140.)


def test_farther_forward_obstacle_is_seen_even_if_nearer_one_is_behind():
    g=guard([(-2.5,0,0),(5.,0,0)])
    assert g.first_conflict((0,0,0),(10,0,0))==(5.,0.,0.)


def test_multi_obstacle_check_rejects_a_detour_into_second_obstacle():
    g=guard([(5,0,0),(5,3,0)])
    assert not g.path_clear(((0,0,0),(5,3,0),(10,0,0)))
    assert g.path_clear(((0,0,0),(3,-3,0),(7,-3,0),(10,0,0)))


def test_moving_traffic_padding_rejects_a_static_only_close_pass():
    g=guard([(5,0,0)])
    route=((0,0,0),(3,-2.5,0),(7,-2.5,0),(10,0,0))
    assert g.path_clear(route,2.)
    assert not g.path_clear(route,3.)


def test_soft_envelope_allows_escape_but_never_deeper_penetration():
    g=guard([(0,1,0)])
    assert g.path_clear(((0,0,0),(0,-3,0)))
    assert not g.path_clear(((0,0,0),(0,.5,0)))


def test_scipy_voxel_dijkstra_routes_around_multiple_measured_obstacles():
    g=guard([(5,0,0),(5,3,0)])
    result=g.plan((0,0,0),(10,0,0),3.,-6.,6.)
    assert result.avoiding and result.reason=="voxel_dijkstra_3d"
    assert g.path_clear(result.points,3.)
    assert result.points[-1]==(10.,0.,0.)


def test_safe_existing_3d_route_is_retained_instead_of_changing_sides():
    g=guard([(5,0,0)])
    result=g.plan((0,0,0),(10,0,0),3.,-6.,6.)
    again=g.plan(result.points[1],(10,0,0),3.,-6.,6.,previous_path=result.points)
    assert again.reason=="retained_3d_sensor_path" and again.avoiding


def test_voxel_search_validates_a_measured_spherical_surface_not_only_its_center():
    import numpy as np
    points=[(5+.75*math.cos(a)*math.cos(b),.75*math.sin(a)*math.cos(b),.75*math.sin(b))
            for a in np.linspace(0,2*math.pi,24) for b in np.linspace(-math.pi/2,math.pi/2,12)]
    g=guard(points)
    result=g.plan((0,0,0),(10,0,0),3.,-6.,6.)
    assert result.avoiding and g.path_clear(result.points,3.)


def test_closed_local_wall_does_not_produce_an_unsafe_direct_fallback():
    import numpy as np
    g=guard([(2,y,z) for y in np.arange(-12,12.1,.5) for z in np.arange(-9,9.1,.5)])
    with pytest.raises(ValueError,match="3D_NO_COLLISION_FREE_LOCAL_PATH"):
        g.plan((0,0,0),(10,0,0),3.,-6.,6.)
