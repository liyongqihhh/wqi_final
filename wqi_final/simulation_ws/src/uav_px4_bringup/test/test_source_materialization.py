import importlib.util
from pathlib import Path
import subprocess

import pytest


WORKSPACE = Path(__file__).parents[3]
spec = importlib.util.spec_from_file_location("prepare_px4_sources", WORKSPACE / "prepare_px4_sources.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def command(directory, *arguments):
    return subprocess.check_output(["git", "-C", str(directory), *arguments], text=True).strip()


def repository(directory):
    directory.mkdir()
    command(directory, "init", "--quiet")
    command(directory, "config", "user.name", "Test fixture")
    command(directory, "config", "user.email", "test@example.invalid")
    (directory / "source.txt").write_text("pinned source\n")
    command(directory, "add", "source.txt")
    command(directory, "commit", "--quiet", "-m", "Fixture")
    command(directory, "remote", "add", "origin", str(directory))
    return directory


def test_clone_preserves_commit_and_legal_empty_submodules_file(tmp_path):
    source = repository(tmp_path / "source")
    (source / ".gitmodules").write_text("")
    command(source, "add", ".gitmodules")
    command(source, "commit", "--quiet", "-m", "Empty modules file")
    destination = tmp_path / "build-copy"
    module.prepare(source, destination)
    assert command(source, "rev-parse", "HEAD") == command(destination, "rev-parse", "HEAD")
    assert (destination / "source.txt").read_text() == "pinned source\n"


def test_existing_dirty_build_copy_is_not_overwritten(tmp_path):
    source = repository(tmp_path / "source")
    destination = tmp_path / "build-copy"
    module.prepare(source, destination)
    (destination / "source.txt").write_text("local compatibility patch\n")
    module.prepare(source, destination)
    assert (destination / "source.txt").read_text() == "local compatibility patch\n"


def test_mismatched_version_is_rejected_without_reset(tmp_path):
    source = repository(tmp_path / "source")
    destination = tmp_path / "build-copy"
    module.prepare(source, destination)
    command(destination, "config", "user.name", "Test fixture")
    command(destination, "config", "user.email", "test@example.invalid")
    (destination / "source.txt").write_text("other commit\n")
    command(destination, "commit", "--quiet", "-am", "Other version")
    with pytest.raises(RuntimeError, match="differs from pinned"):
        module.prepare(source, destination)
    assert (destination / "source.txt").read_text() == "other commit\n"


def test_nested_source_is_materialized_without_network(tmp_path):
    child = repository(tmp_path / "child")
    source = repository(tmp_path / "source")
    command(source, "-c", "protocol.file.allow=always", "submodule", "add", str(child), "dependency")
    command(source, "commit", "--quiet", "-am", "Pin dependency")
    destination = tmp_path / "build-copy"
    module.prepare(source, destination)
    assert command(source / "dependency", "rev-parse", "HEAD") == command(
        destination / "dependency", "rev-parse", "HEAD"
    )


def test_uav_launch_defaults_to_px4_and_old_plugin_is_absent():
    for relative in ["uav_bringup/launch/uav_sim.launch.py", "uav_control/launch/uav_control.launch.py",
                     "uav_description/launch/uav_spawn.launch.py"]:
        text = (WORKSPACE / "src" / relative).read_text()
        assert 'default_value="px4"' in text
        assert 'choices=["px4"]' in text
    assert not (WORKSPACE / "src/vendor/sjtu_drone_description").exists()
    model = (WORKSPACE / "src/uav_description/urdf/uav.urdf.xacro").read_text()
    assert "libplugin_drone.so" not in model
