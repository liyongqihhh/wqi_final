from pathlib import Path
from types import SimpleNamespace
import time
from unittest.mock import Mock

import pytest

from campus_regional_delivery.layout import RegionalLayout,partition_orders
from campus_regional_delivery.mission_manager import RegionalMissionManager
from rclpy.action import GoalResponse


LAYOUT=Path(__file__).parents[1]/"config/campus_regions.yaml"


@pytest.mark.parametrize("reason",["Unknown target or invalid/excessive parcel mass",
    "Requested floor exceeds the existing building height"])
def test_invalid_goal_explains_its_rejection_without_reserving_the_server(reason):
    import threading
    manager=SimpleNamespace(batches=Mock(side_effect=ValueError(reason)),busy=False,
        busy_lock=threading.Lock(),report_admission=Mock())
    request=Mock()
    assert RegionalMissionManager.goal(manager,request)==GoalResponse.REJECT
    manager.report_admission.assert_called_once_with(request,False,reason)
    assert not manager.busy


def test_busy_goal_is_rejected_with_a_visible_reason():
    import threading
    manager=SimpleNamespace(batches=Mock(),busy=True,busy_lock=threading.Lock(),
        report_admission=Mock())
    request=Mock()
    assert RegionalMissionManager.goal(manager,request)==GoalResponse.REJECT
    manager.report_admission.assert_called_once_with(request,False,
        "A regional delivery mission is already active")


def test_valid_goal_is_logged_and_reserved_for_exactly_one_mission():
    import threading
    manager=SimpleNamespace(batches=Mock(),busy=False,busy_lock=threading.Lock(),
        report_admission=Mock())
    request=Mock()
    assert RegionalMissionManager.goal(manager,request)==GoalResponse.ACCEPT
    assert manager.busy
    manager.report_admission.assert_called_once_with(request,True,
        "Accepted; starting regional readiness checks")


def test_admission_topic_preserves_reason_endpoint_and_supported_buildings():
    import json
    logger=Mock()
    manager=SimpleNamespace(layout=RegionalLayout(LAYOUT),delivery_mode="cooperative",
        admission_pub=Mock(),get_logger=lambda:logger)
    request=SimpleNamespace(targets=["dormitory"])
    RegionalMissionManager.report_admission(manager,request,False,"Unknown target")
    details=json.loads(manager.admission_pub.publish.call_args.args[0].data)
    assert details["accepted"] is False and details["reason"]=="Unknown target"
    assert details["action"]=="/regional_delivery/execute_mission"
    assert "dormitory_1" in details["valid_targets"] and "dormitory" not in details["valid_targets"]
    logger.error.assert_called_once()


@pytest.mark.parametrize("target,mass,reason",[
    ("dormitory",.3,"Unknown target 'dormitory'"),
    ("library",2.,"UAV limit is 1 kg per parcel"),
    ("library",float("nan"),"finite nonnegative")])
def test_order_validation_distinguishes_wrong_building_from_bad_payload(target,mass,reason):
    import re
    with pytest.raises(ValueError,match=re.escape(reason)):
        partition_orders(RegionalLayout(LAYOUT),[target],[mass])


def test_regional_dispatch_keeps_each_parcels_mass_and_destination():
    layout=RegionalLayout(LAYOUT)
    batches=partition_orders(layout,["teaching_building","dormitory_4","cafeteria"],[.1,.2,.3],[3,4,2])
    assert [r for r,_ in batches]==["A","B","C"]
    assert [b[0]["mass"] for _,b in batches]==[.1,.2,.3]
    assert [b[0]["floor"] for _,b in batches]==[3,4,2]


def test_multiple_dormitories_are_grouped_on_one_vehicle():
    batches=partition_orders(RegionalLayout(LAYOUT),["dormitory_4","dormitory_2"],[.2,.3])
    assert len(batches)==1 and batches[0][0]=="B"
    assert len(batches[0][1])==2


def test_uav_capacity_splits_batches_without_dropping_parcels():
    batches=partition_orders(RegionalLayout(LAYOUT),["library","laboratory"],[.6,.6])
    assert len(batches)==2


@pytest.mark.parametrize("targets,masses,floors",[
    (["park"],[.3],None),(["library"],[float("nan")],None),(["library"],[1.1],None),
    (["library"],[-.1],None),(["library","cafeteria"],[.3],None),(["library"],[.3],[0])])
def test_invalid_orders_are_rejected(targets,masses,floors):
    with pytest.raises(ValueError):
        partition_orders(RegionalLayout(LAYOUT),targets,masses,floors)


def test_ground_vehicle_cannot_move_during_airborne_handoff():
    with pytest.raises(RuntimeError,match="airborne"):
        RegionalMissionManager.drive(SimpleNamespace(uav_state=1,delivery_mode="cooperative"),"ugv_01",[0,0,0],None)


def test_no_simulated_capture_by_just_setting_carrier_state():
    source=(Path(__file__).parents[1]/"campus_regional_delivery/mission_manager.py").read_text()
    assert "capture_pose_is_safe" in source and "CaptureSettleWindow" in source
    assert "self.attach.call_async" in source and "self.carrier==namespace" in source
    assert "set_entity_state" not in source


def test_landing_targets_the_measured_deck_not_the_nav2_nominal_goal():
    source=(Path(__file__).parents[1]/"campus_regional_delivery/mission_manager.py").read_text()
    transfer=source.split("def air_transport",1)[1].split("def floor_delivery",1)[0]
    assert "receiver=self.poses[namespace].pose.pose.position" in transfer
    assert "point=(receiver.x,receiver.y)" in transfer


def landing_manager(z=.425,carrier="ugv_00",age=0.):
    def odometry(x,y,z):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0)),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x,y=y,z=z))))
    now=time.monotonic()
    manager=SimpleNamespace(layout=RegionalLayout(LAYOUT),carrier=carrier,uav_state=0,fault="",
        poses={"ugv_00":odometry(-56.,-28.,.005)},uav_pose=odometry(-56.,-28.,z),
        uav_received=now-age,received={"ugv_00":now-age},landing_pub=Mock(),get_logger=Mock())
    manager.landing_observation=lambda ns:RegionalMissionManager.landing_observation(manager,ns)
    manager.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000))
    def wait(predicate,goal,timeout):
        for i in range(11):
            for odom in (manager.poses["ugv_00"],manager.uav_pose):
                odom.header.stamp.nanosec=i*100_000_000
            if predicate():
                return
        raise TimeoutError("Measured docking was not verified")
    manager.wait=wait
    return manager


@pytest.mark.parametrize("height,carrier,age",[
    (15.,"ugv_00",0.),(.8,"ugv_00",0.),(.425,"ugv_01",0.),(.425,"ugv_00",10.)])
def test_a_landed_flag_cannot_complete_a_hovering_wrong_carrier_or_stale_landing(height,carrier,age):
    manager=landing_manager(height,carrier,age)
    with pytest.raises(TimeoutError):
        RegionalMissionManager.verify_docked(manager,"ugv_00",None)
    manager.landing_pub.publish.assert_not_called()


def test_capture_requires_a_new_measured_stable_pose_before_reporting_verified():
    import json
    manager=landing_manager()
    RegionalMissionManager.verify_docked(manager,"ugv_00",None)
    status=json.loads(manager.landing_pub.publish.call_args.args[0].data)
    assert status["verified"] and status["phase"]=="DOCKED_VERIFIED"
    assert status["uav_z_m"]==pytest.approx(.425)
    assert status["target_z_m"]==pytest.approx(.425)


def test_missing_or_nonfinite_landing_odometry_is_not_fresh():
    manager=landing_manager(float("nan"))
    assert not manager.landing_observation("ugv_00")["fresh"]
    assert not manager.landing_observation("ugv_03")["fresh"]


def test_landing_feedback_reports_the_real_remaining_height():
    import json
    manager=landing_manager(.8,carrier="")
    manager.landing_goal=SimpleNamespace(is_active=True,publish_feedback=Mock())
    manager.landing_target=(-56.,-28.)
    manager.landing_namespace="ugv_00"
    manager.phase="UAV_LANDING"
    manager.target="ugv_00"
    manager.waiting_on="PX4_TOUCHDOWN_AND_DISARM"
    manager.last_landing_log=0.
    RegionalMissionManager.publish_landing_progress(manager)
    feedback=manager.landing_goal.publish_feedback.call_args.args[0]
    assert feedback.phase=="UAV_LANDING" and not feedback.uav_docked
    assert feedback.distance_remaining==pytest.approx(.375)
    status=json.loads(manager.landing_pub.publish.call_args.args[0].data)
    assert status["uav_z_m"]==pytest.approx(.8) and not status["verified"]


def test_docking_with_an_explicit_landing_fault_cannot_be_verified():
    manager=landing_manager()
    manager.fault="PRECISION_LANDING_TIMEOUT"
    with pytest.raises(TimeoutError):
        RegionalMissionManager.verify_docked(manager,"ugv_00",None)


@pytest.mark.parametrize("phase",["UAV_TAKEOFF","UAV_FLOOR_DELIVERY","FAILED","CANCELED"])
def test_departure_and_failure_invalidate_the_previous_landing_result(phase):
    import json
    manager=SimpleNamespace(delivery_mode="cooperative",status_pub=Mock(),landing_pub=Mock(),
        flight_phase_pub=Mock(),carrier="ugv_00",get_logger=Mock())
    RegionalMissionManager.phase_to(manager,phase,Mock())
    assert json.loads(manager.landing_pub.publish.call_args.args[0].data)=={
        "phase":phase,"verified":False}


def test_takeoff_waits_for_px4_estimation_not_just_an_advertised_action():
    source=(Path(__file__).parents[1]/"campus_regional_delivery/mission_manager.py").read_text()
    assert '"/uav/px4/connected"' in source
    departure=source.split("def depart",1)[1].split("def capture",1)[0]
    assert "self.connected and self.uav_state==0" in departure


@pytest.mark.parametrize("sustained_motion",[False,True])
def test_stopped_platform_window_filters_impulses_but_not_real_motion(sustained_motion):
    clock=SimpleNamespace(stamp=0.)
    message=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0,nanosec=0)),
        twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.,y=0.,z=0.),angular=SimpleNamespace(x=0.,y=0.,z=0.))))
    manager=SimpleNamespace(poses={"ugv_01":message},received={"ugv_01":time.monotonic()})
    manager.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock.stamp*1e9)))
    def wait(predicate,goal,timeout):
        for i in range(1,101):
            clock.stamp=i*.05
            message.header.stamp.sec=int(clock.stamp)
            message.header.stamp.nanosec=int((clock.stamp-int(clock.stamp))*1e9)
            message.twist.twist.angular.x=.22 if sustained_motion or i%5==0 else 0.
            if predicate():
                assert not sustained_motion
                return
        assert sustained_motion
        raise TimeoutError("Physical platform still moving")
    manager.wait=wait
    if sustained_motion:
        with pytest.raises(TimeoutError):
            RegionalMissionManager.settled(manager,"ugv_01",None)
    else:
        RegionalMissionManager.settled(manager,"ugv_01",None)


def test_ugv_only_orders_do_not_apply_the_aircraft_capacity_limit():
    batches=partition_orders(RegionalLayout(LAYOUT),["library","laboratory"],[1.2,.6],
        enforce_uav_capacity=False)
    assert len(batches)==1 and sum(o["mass"] for o in batches[0][1])==pytest.approx(1.8)


def test_ugv_only_comparison_does_not_accept_unreachable_floors():
    manager=SimpleNamespace(delivery_mode="ugv_only")
    request=SimpleNamespace(payload_masses_kg=[.3],targets=["library"],target_floors=[3])
    with pytest.raises(ValueError,match="building doors"):
        RegionalMissionManager.batches(manager,request)


def test_cooperation_starts_and_returns_on_the_central_carrier():
    source=(Path(__file__).parents[1]/"campus_regional_delivery/mission_manager.py").read_text()
    assert 'hub_ns=self.layout.hub["namespace"]' in source
    assert "self.capture(hub_ns,goal)" in source
    assert "self.air_transport(hub,goal,hub_ns)" in source


def test_ground_mode_becomes_ready_without_px4_or_uav_telemetry():
    names=[f"ugv_0{i}" for i in range(4)]
    manager=SimpleNamespace(delivery_mode="ugv_only",poses=dict.fromkeys(names),
        nav_clients=dict.fromkeys(names),lifecycle={},readiness_pub=Mock())
    def wait(predicate,goal,timeout):
        assert predicate()
    manager.wait=wait
    RegionalMissionManager.ready(manager,None)
    assert manager.readiness_pub.publish.call_args.args[0].data=="READY"


def test_ground_comparison_uses_only_the_central_car_and_returns_it():
    layout=RegionalLayout(LAYOUT)
    manager=SimpleNamespace(layout=layout,phase_to=Mock(),drive=Mock(),hover=Mock(),event_pub=Mock())
    orders=partition_orders(layout,["library","cafeteria"],[.3,.4],enforce_uav_capacity=False)
    result=SimpleNamespace(completed_targets=0)
    goal=SimpleNamespace(request=SimpleNamespace(return_home=True))
    RegionalMissionManager.ground_delivery(manager,orders,goal,result)
    assert result.completed_targets==2
    assert manager.drive.call_count==4
    assert all(call.args[0]=="ugv_00" for call in manager.drive.call_args_list)
    assert manager.drive.call_args_list[0].args[1]==manager.drive.call_args_list[-1].args[1]
    assert manager.event_pub.publish.call_count==2


def test_flight_phase_pauses_navigation_and_resumes_only_the_selected_car():
    from nav2_msgs.srv import ManageLifecycleNodes
    names=[f"ugv_0{i}" for i in range(4)]
    manager=SimpleNamespace(nav_clients=dict.fromkeys(names),paused_navigation=set(),
        navigation_managers={name:Mock(service_is_ready=lambda:True) for name in names},
        stop_pubs={name:Mock() for name in names},call=Mock())
    manager.wait=lambda predicate,*args: predicate() or pytest.fail("service missing")
    RegionalMissionManager.set_navigation_paused(manager,True,None)
    assert manager.paused_navigation==set(names)
    assert manager.call.call_count==4
    assert all(c.args[1] is None and c.args[2].command==ManageLifecycleNodes.Request.PAUSE
               for c in manager.call.call_args_list)
    assert all(p.publish.call_args.args[0].linear.x==0 for p in manager.stop_pubs.values())
    RegionalMissionManager.set_navigation_paused(manager,True,None)
    assert manager.call.call_count==4
    RegionalMissionManager.set_navigation_paused(manager,False,None,"ugv_01")
    assert manager.paused_navigation==set(names)-{"ugv_01"}
    assert manager.call.call_args.args[2].command==ManageLifecycleNodes.Request.RESUME
    RegionalMissionManager.set_navigation_paused(manager,False,None)
    assert not manager.paused_navigation and manager.call.call_count==8


def test_ground_drive_restores_its_stack_before_using_navigation():
    point=SimpleNamespace(x=0.,y=0.)
    manager=SimpleNamespace(delivery_mode="cooperative",uav_state=0,paused_navigation={"ugv_01"},
        set_navigation_paused=Mock(),poses={"ugv_01":SimpleNamespace(pose=SimpleNamespace(
            pose=SimpleNamespace(position=point)))},received={"ugv_01":time.monotonic()},
        stop_pubs={"ugv_01":Mock()},settled=Mock())
    manager.wait=lambda predicate,*args: predicate() or pytest.fail("pose missing")
    RegionalMissionManager.drive(manager,"ugv_01",[0.,0.,0.],None)
    manager.set_navigation_paused.assert_called_once_with(False,None,"ugv_01")


def test_uncertain_pause_is_recorded_so_failure_cleanup_can_restore_it():
    manager=SimpleNamespace(nav_clients={"ugv_00":Mock()},paused_navigation=set(),
        navigation_managers={"ugv_00":Mock(service_is_ready=lambda:True)},
        stop_pubs={"ugv_00":Mock()},call=Mock(side_effect=TimeoutError("pause reply missing")))
    manager.wait=lambda predicate,*args: predicate() or pytest.fail("service missing")
    with pytest.raises(TimeoutError):
        RegionalMissionManager.set_navigation_paused(manager,True,None)
    assert manager.paused_navigation=={"ugv_00"}


@pytest.mark.parametrize("initial_carrier",["","ugv_00"])
def test_initial_capture_pauses_stationary_navigation_before_the_contact_window(initial_carrier):
    import threading
    layout=RegionalLayout(LAYOUT)
    manager=Mock(spec=RegionalMissionManager)
    manager.layout=layout
    manager.delivery_mode="cooperative"
    manager.uav_state=0
    manager.carrier=initial_carrier
    manager.paused_navigation=set()
    manager.uav_pose=SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=
        SimpleNamespace(x=layout.hub["uav_pad"][0],y=layout.hub["uav_pad"][1]))))
    manager.select=Mock()
    manager.event_pub=Mock()
    manager.payload_pub=Mock()
    manager.nav_goal=None
    manager.fly_goal=None
    manager.busy_lock=threading.Lock()
    manager.busy=True
    order={"target":"innovation_center","mass":.3,"floor":0}
    manager.batches.return_value=[("A",[order])]
    goal=Mock(request=SimpleNamespace(return_home=True),is_cancel_requested=False)
    result=RegionalMissionManager.execute(manager,goal)
    assert result.success and result.completed_targets==1
    calls=[c[0] for c in manager.mock_calls]
    gate="verify_docked" if initial_carrier else "capture"
    assert calls.index("ready")<calls.index("set_navigation_paused")<calls.index(gate)<calls.index("air_transport")
    assert manager.set_navigation_paused.call_args_list[0].args==(True,goal)
    if initial_carrier:
        manager.capture.assert_not_called()
        assert manager.verify_docked.call_args_list[0].args==("ugv_00",goal)
    else:
        manager.capture.assert_called_once_with("ugv_00",goal)


@pytest.mark.parametrize("condition",["stable","outside","moving","stale","repeated_samples"])
def test_capture_uses_new_measured_samples_and_reports_the_blocking_condition(condition):
    import json
    manager=landing_manager(age=10. if condition=="stale" else 0.)
    manager.phase="UGV_PARKING"
    manager.settled=Mock()
    manager.capture_pub=Mock()
    manager.attach=Mock()
    manager.future=Mock(return_value=SimpleNamespace(success=True))
    manager.verify_docked=Mock()
    clock=SimpleNamespace(stamp=10.)
    manager.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock.stamp*1e9)))
    for odom in (manager.poses["ugv_00"],manager.uav_pose):
        odom.twist=SimpleNamespace(twist=SimpleNamespace(
            linear=SimpleNamespace(x=0.,y=0.,z=0.),angular=SimpleNamespace(x=0.,y=0.,z=0.)))
    if condition=="outside":manager.uav_pose.pose.pose.position.x+=.2
    if condition=="moving":manager.uav_pose.twist.twist.linear.x=.3
    def wait(predicate,goal,timeout):
        for i in range(21):
            clock.stamp=10.+i*.1
            if condition!="repeated_samples":
                for odom in (manager.poses["ugv_00"],manager.uav_pose):
                    odom.header.stamp.sec=int(clock.stamp)
                    odom.header.stamp.nanosec=int((clock.stamp-int(clock.stamp))*1e9)
            if predicate():return
        raise TimeoutError("Timeout during UGV_PARKING; waiting for "+manager.waiting_on)
    manager.wait=wait
    if condition=="stable":
        RegionalMissionManager.capture(manager,"ugv_00",None)
        manager.attach.call_async.assert_called_once()
        manager.verify_docked.assert_called_once_with("ugv_00",None)
    else:
        with pytest.raises(TimeoutError,match="last capture observation="):
            RegionalMissionManager.capture(manager,"ugv_00",None)
        manager.attach.call_async.assert_not_called()
        assert manager.waiting_on=="CAPTURE_WINDOW:ugv_00"
        report=json.loads(manager.capture_pub.publish.call_args.args[0].data)
        expected={"outside":"OUTSIDE_CAPTURE_DISTANCE","moving":"VELOCITY_LIMIT",
            "stale":"STALE_OR_MISSING_ODOMETRY","repeated_samples":"SETTLING"}
        assert report["reason"]==expected[condition]


def test_readiness_identifies_the_missing_gate_instead_of_silent_preparing():
    import json
    manager=SimpleNamespace(phase="PREPARING",waiting_on="VEHICLE_ODOMETRY",
        nav_clients=dict.fromkeys([f"ugv_0{i}" for i in range(4)]),poses={"ugv_00":Mock()},
        connected=False,uav_pose=None,uav_state=None,fly_client=Mock(server_is_ready=lambda:False),
        readiness_pub=Mock())
    RegionalMissionManager.publish_readiness(manager)
    detail=json.loads(manager.readiness_pub.publish.call_args.args[0].data)
    assert detail["waiting_for"]=="VEHICLE_ODOMETRY"
    assert detail["missing_odometry"]==["ugv_01","ugv_02","ugv_03"]
    assert not detail["px4_connected"] and not detail["fly_action_ready"]
    manager.waiting_on="READY"
    RegionalMissionManager.publish_readiness(manager)
    assert manager.readiness_pub.publish.call_count==1


def test_late_manager_receives_the_retained_landed_state_without_republishing():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy,QoSProfile
    from std_msgs.msg import Int8
    owned=not rclpy.ok()
    if owned:
        rclpy.init()
    publisher_node=Node("regional_late_state_test")
    manager=None
    try:
        publisher=publisher_node.create_publisher(Int8,"/uav/state",
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher.publish(Int8(data=0))
        time.sleep(.1)
        manager=RegionalMissionManager()
        deadline=time.monotonic()+5.
        while manager.uav_state is None and time.monotonic()<deadline:
            rclpy.spin_once(manager,timeout_sec=.05)
        assert manager.uav_state==0
    finally:
        if manager is not None:
            manager.action.destroy()
            manager.fly_client.destroy()
            for client in manager.nav_clients.values():
                client.destroy()
            manager.destroy_node()
        publisher_node.destroy_node()
        if owned:
            rclpy.shutdown()
