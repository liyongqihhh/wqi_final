import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, Shutdown,
)
from launch.events.process import ProcessExited
from launch_ros.actions import Node


ROOT = Path(__file__).parents[1]


def load_launch():
    spec = importlib.util.spec_from_file_location(
        "campus_ugv_launch", ROOT / "launch" / "campus_ugv.launch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def exit_actions(description, node, context, returncode):
    event = ProcessExited(action=node, name="probe", cmd=[], cwd=None,
                          env=None, pid=1, returncode=returncode)
    handler = next(action.event_handler for action in description.entities
                   if isinstance(action, RegisterEventHandler)
                   and action.event_handler.matches(event))
    return list(handler.handle(event, context))


def test_standalone_launch_includes_physics_and_navigation(monkeypatch):
    module = load_launch()
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: f"/tmp/{name}")
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    groups = [action for action in description.entities if isinstance(action, GroupAction)]
    assert groups == []
    preflight = next(action for action in description.entities if isinstance(action, Node))
    startup = exit_actions(description, preflight, context, 0)
    assert isinstance(startup[0], GroupAction)
    probe = startup[1]
    groups = [startup[0]] + exit_actions(description, probe, context, 0)
    includes = [child for group in groups for child in group.get_sub_entities()
                if isinstance(child, IncludeLaunchDescription)]
    assert len(includes) == 2
    physics = dict(includes[0].launch_arguments)
    navigation = dict(includes[1].launch_arguments)
    assert physics["gui"].perform(context) == "false"
    assert navigation["rviz"].perform(context) == "true"
    assert navigation["rviz_software_rendering"].perform(context) == "true"
    assert navigation["localization_mode"] == "ground_truth"
    assert navigation["dynamic_obstacles"] == "false"
    for spawn_name, localization_name in [("x", "initial_x"), ("y", "initial_y"),
                                          ("yaw", "initial_yaw")]:
        context.launch_configurations[spawn_name] = "12.3"
        assert physics[spawn_name].perform(context) == "12.3"
        assert navigation[localization_name].perform(context) == "12.3"


def test_standalone_launch_does_not_include_uav():
    text = (ROOT / "launch" / "campus_ugv.launch.py").read_text()
    assert "uav_bringup" not in text
    assert "cooperative_delivery" not in text
    physics = (ROOT.parent / "ugvcar_description" / "launch"
               / "campus_delivery_sim.launch.py").read_text()
    assert '"server_required": "true"' in physics
    assert "event.returncode != 0" in physics


def test_failed_readiness_probe_stops_before_navigation(monkeypatch):
    module = load_launch()
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: f"/tmp/{name}")
    description = module.generate_launch_description()
    context = LaunchContext()
    preflight = next(action for action in description.entities if isinstance(action, Node))
    startup = exit_actions(description, preflight, context, 0)
    actions = exit_actions(description, startup[1], context, 1)
    assert len(actions) == 1
    assert isinstance(actions[0], Shutdown)


def test_preflight_failure_stops_before_gazebo(monkeypatch):
    module = load_launch()
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: f"/tmp/{name}")
    description = module.generate_launch_description()
    preflight = next(action for action in description.entities if isinstance(action, Node))
    actions = exit_actions(description, preflight, LaunchContext(), 1)
    assert len(actions) == 1
    assert isinstance(actions[0], Shutdown)


def test_duplicate_check_is_scoped_to_conflicting_robot_nodes():
    spec = importlib.util.spec_from_file_location(
        "ugv_startup_probe", ROOT / "scripts" / "wait_for_ugv_ready.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.conflicting_nodes([
        ("controller_server", "/"), ("controller_server", "/"),
        ("controller_server", "/another_robot"), ("unrelated_node", "/"),
        ("robot_state_publisher", "/"),
    ]) == ["controller_server", "robot_state_publisher"]
    assert module.conflicting_nodes([("unrelated_node", "/")]) == []
