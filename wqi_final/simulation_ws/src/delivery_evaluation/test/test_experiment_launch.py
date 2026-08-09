from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_dynamic_experiments_enable_controller_and_single_spawner():
    launch_text = (
        PACKAGE_ROOT / "launch" / "experiment.launch.py"
    ).read_text(encoding="utf-8")

    assert '"dynamic_obstacles": "true"' in launch_text
    assert '"enable_dynamic_obstacles": "true"' in launch_text
    assert 'if mode != "cooperative":' in launch_text
    assert '"obstacle_density": LaunchConfiguration(' in launch_text
    assert '"random_seed": LaunchConfiguration(' in launch_text
