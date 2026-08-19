from pathlib import Path

from delivery_evaluation.paths import default_results_directory


def test_results_directory_uses_explicit_override():
    result = default_results_directory({
        "WQI_EXPERIMENT_RESULTS_DIR": "~/custom-results",
        "WQI_BUILD_ROOT": "/tmp/build-root",
    })

    assert result == Path("~/custom-results").expanduser()


def test_results_directory_uses_external_build_root():
    result = default_results_directory({
        "WQI_BUILD_ROOT": "/tmp/wqi-artifacts",
    })

    assert result == Path("/tmp/wqi-artifacts/experiment_results")


def test_results_directory_has_portable_fallback():
    result = default_results_directory({}, home=Path("/tmp/example-home"))

    assert result == Path(
        "/tmp/example-home/.ros/wqi_final/experiment_results"
    )
