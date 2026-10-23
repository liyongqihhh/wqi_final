from types import SimpleNamespace

from composition_interfaces.srv import ListNodes,UnloadNode
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from std_srvs.srv import Trigger

from campus_regional_delivery.container_guard import shutdown_components


def test_shutdown_finalizes_navigation_before_reverse_component_unloading():
    calls=[]
    def request(kind,name,message):
        calls.append((kind,name,message))
        if kind is GetState:
            return SimpleNamespace(current_state=SimpleNamespace(id=3))
        return SimpleNamespace(unique_ids=[1,2,3]) if kind is ListNodes else SimpleNamespace(success=True)
    result=shutdown_components("ugv_02",request)
    kinds=[c[0] for c in calls]
    assert kinds.index(ManageLifecycleNodes)<kinds.index(Trigger)<kinds.index(UnloadNode)
    assert result=={"managed_shutdown":True,"unloaded":[3,2,1],"errors":[]}
    assert all(c[1].startswith("/ugv_02/") for c in calls)


def test_unload_failure_is_reported_not_hidden_as_clean_shutdown():
    def request(kind,name,message):
        if kind is GetState:
            return SimpleNamespace(current_state=SimpleNamespace(id=4))
        if kind is ListNodes:
            return SimpleNamespace(unique_ids=[1])
        if kind is UnloadNode:
            return SimpleNamespace(success=False,error_message="busy")
        return SimpleNamespace(success=True)
    result=shutdown_components("ugv_00",request)
    assert not result["unloaded"] and result["errors"]==["Component unload failed: busy"]


def test_unknown_state_does_not_allow_unloading_live_components():
    def request(kind,name,message):
        assert kind is GetState
        raise TimeoutError("State unavailable")
    result=shutdown_components("ugv_01",request)
    assert not result["managed_shutdown"] and not result["unloaded"]
    assert result["errors"]==["State unavailable"]


def test_already_finalized_navigation_never_receives_a_second_shutdown():
    def request(kind,name,message):
        assert kind is not ManageLifecycleNodes
        if kind is GetState:
            return SimpleNamespace(current_state=SimpleNamespace(id=4))
        return SimpleNamespace(unique_ids=[1]) if kind is ListNodes else SimpleNamespace(success=True)
    assert shutdown_components("ugv_00",request)=={"managed_shutdown":True,"unloaded":[1],"errors":[]}


def test_context_refuses_unloading_without_finalized_nodes():
    def request(kind,name,message):
        if kind is GetState:
            return SimpleNamespace(current_state=SimpleNamespace(id=4))
        assert kind is Trigger
        return SimpleNamespace(success=False,message="Node still active")
    result=shutdown_components("ugv_03",request)
    assert not result["unloaded"] and result["errors"]==["Unsafe component cleanup: Node still active"]
