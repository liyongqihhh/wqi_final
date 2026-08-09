from delivery_evaluation.models import RunRecord
from delivery_evaluation.report_generator import (
    append_record,
    load_records,
    summarize,
)


def record(run_id, success, duration, collision_free=True):
    return RunRecord(
        run_id=run_id,
        mode="cooperative",
        scenario="teaching_building",
        repetition=1,
        obstacle_density="low",
        random_seed=42,
        targets=["teaching_building"],
        success=success,
        collision_free=collision_free,
        sim_duration_s=duration,
        total_energy_wh=5.0,
    )


def test_report_preserves_failed_runs_and_summarizes_successes(tmp_path):
    append_record(tmp_path, record("run-1", True, 10.0))
    failed = record("run-2", False, 20.0, collision_free=False)
    failed.ugv_collision_count = 1
    failed.minimum_ugv_obstacle_clearance_m = -0.1
    append_record(tmp_path, failed)
    records = load_records(tmp_path)
    summary = summarize(records)[0]
    assert len(records) == 2
    assert summary["runs"] == 2
    assert summary["successes"] == 1
    assert summary["success_rate"] == 0.5
    assert summary["collision_free_runs"] == 1
    assert summary["avoidance_success_rate"] == 0.5
    assert summary["metrics"]["ugv_collision_count"]["mean"] == 0.5
    assert (
        summary["metrics"]["minimum_ugv_obstacle_clearance_m"]["minimum"]
        == -0.1
    )
    assert summary["metrics"]["sim_duration_s"]["mean"] == 10.0
    assert (tmp_path / "runs.csv").exists()
    assert (tmp_path / "summary.md").exists()
    assert "Avoidance rate" in (tmp_path / "summary.md").read_text(
        encoding="utf-8"
    )
