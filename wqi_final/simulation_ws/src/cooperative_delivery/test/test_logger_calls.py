import ast
from pathlib import Path


LOGGER_METHODS = {"debug", "info", "warning", "error", "fatal"}


def _manager_tree():
    source = (
        Path(__file__).parents[1]
        / "cooperative_delivery"
        / "cooperative_mission_manager.py"
    )
    return ast.parse(source.read_text(encoding="utf-8"))


def test_ros_logger_calls_use_a_single_rendered_message():
    tree = _manager_tree()

    invalid_calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr not in LOGGER_METHODS:
            continue
        if len(node.args) != 1:
            invalid_calls.append((node.lineno, node.func.attr, len(node.args)))

    assert invalid_calls == []


def test_terminal_paths_cancel_nested_vehicle_goals():
    tree = _manager_tree()
    methods = {
        node.name: node
        for node in tree.body
        if isinstance(node, ast.ClassDef)
        for node in node.body
        if isinstance(node, ast.FunctionDef)
    }

    for method_name in ("_finish_failure", "_finish_canceled"):
        calls = {
            node.func.attr
            for node in ast.walk(methods[method_name])
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
        }
        assert "_cancel_active_goals" in calls
