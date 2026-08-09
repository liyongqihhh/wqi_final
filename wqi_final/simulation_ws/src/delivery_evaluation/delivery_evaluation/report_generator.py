import argparse
import csv
from dataclasses import fields
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics

from delivery_evaluation.models import RunRecord


STRUCTURED_FIELDS = {"targets", "phase_durations_s"}
NUMERIC_METRICS = (
    "sim_duration_s",
    "wall_duration_s",
    "real_time_factor",
    "delivery_rate_per_min",
    "energy_per_completed_target_wh",
    "ugv_path_length_m",
    "uav_path_length_m",
    "ugv_endpoint_error_m",
    "uav_endpoint_error_m",
    "nav_recovery_count",
    "uav_replan_count",
    "uav_safety_hold_count",
    "uav_safety_hold_s",
    "minimum_uav_clearance_m",
    "ugv_energy_wh",
    "ugv_drive_energy_wh",
    "ugv_charging_energy_wh",
    "uav_energy_wh",
    "uav_charged_wh",
    "total_energy_wh",
)
ALL_RUN_METRICS = (
    "minimum_ugv_obstacle_clearance_m",
    "minimum_uav_dynamic_clearance_m",
    "ugv_collision_count",
    "uav_collision_count",
)


def _json_default(value):
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Unsupported JSON value: {type(value).__name__}")


def load_records(path) -> list[dict]:
    source = Path(path)
    if source.is_dir():
        source = source / "runs.json"
    if not source.exists():
        return []
    with source.open(encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, list):
        raise ValueError("runs.json must contain a list")
    return data


def write_records(output_directory, records: list[dict]) -> None:
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    json_path = output / "runs.json"
    temporary = output / ".runs.json.tmp"
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(records, stream, ensure_ascii=False, indent=2, default=_json_default)
        stream.write("\n")
    temporary.replace(json_path)

    field_names = [item.name for item in fields(RunRecord)]
    with (output / "runs.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=field_names)
        writer.writeheader()
        for record in records:
            row = dict(record)
            for name in STRUCTURED_FIELDS:
                row[name] = json.dumps(row.get(name), ensure_ascii=False, sort_keys=True)
            writer.writerow({name: row.get(name) for name in field_names})


def append_record(output_directory, record: RunRecord) -> list[dict]:
    records = load_records(output_directory)
    records.append(record.to_dict())
    write_records(output_directory, records)
    generate_summary(output_directory, records)
    return records


def _finite_values(records, field):
    values = []
    for record in records:
        value = record.get(field)
        if value is None:
            continue
        numeric = float(value)
        if math.isfinite(numeric):
            values.append(numeric)
    return values


def summarize(records: list[dict]) -> list[dict]:
    grouped = {}
    for record in records:
        key = (
            str(record.get("mode", "unknown")),
            str(record.get("scenario", "unknown")),
            str(record.get("obstacle_density", "none")),
        )
        grouped.setdefault(key, []).append(record)

    summaries = []
    for key in sorted(grouped):
        group = grouped[key]
        successful = [record for record in group if bool(record.get("success"))]
        collision_free = [
            record
            for record in group
            if bool(record.get("collision_free", True))
        ]
        summary = {
            "mode": key[0],
            "scenario": key[1],
            "obstacle_density": key[2],
            "runs": len(group),
            "successes": len(successful),
            "success_rate": len(successful) / len(group) if group else 0.0,
            "collision_free_runs": len(collision_free),
            "avoidance_success_rate": (
                len(collision_free) / len(group) if group else 0.0
            ),
            "metrics": {},
        }
        for field in NUMERIC_METRICS:
            values = _finite_values(successful, field)
            if not values:
                continue
            summary["metrics"][field] = {
                "mean": statistics.fmean(values),
                "stddev": statistics.stdev(values) if len(values) > 1 else 0.0,
                "minimum": min(values),
                "maximum": max(values),
            }
        for field in ALL_RUN_METRICS:
            values = _finite_values(group, field)
            if not values:
                continue
            summary["metrics"][field] = {
                "mean": statistics.fmean(values),
                "stddev": statistics.stdev(values) if len(values) > 1 else 0.0,
                "minimum": min(values),
                "maximum": max(values),
            }
        summaries.append(summary)
    return summaries


def _format(value, digits=3):
    return "-" if value is None else f"{float(value):.{digits}f}"


def _write_markdown(output: Path, records: list[dict], summaries: list[dict]) -> None:
    lines = [
        "# Campus Delivery Evaluation Results",
        "",
        f"Generated: {datetime.now(timezone.utc).isoformat()}",
        "",
        "All energy fields are simulation estimates, not physical battery telemetry.",
        "",
        "## Summary",
        "",
        "| Mode | Scenario | Obstacles | Runs | Success | Delivery rate | "
        "Collision-free | Avoidance rate | Sim time mean/std (s) | "
        "Total energy mean/std (Wh) |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in summaries:
        duration = item["metrics"].get("sim_duration_s", {})
        energy = item["metrics"].get("total_energy_wh", {})
        lines.append(
            "| {mode} | {scenario} | {density} | {runs} | {successes} | "
            "{rate:.1%} | {collision_free} | {avoidance_rate:.1%} | "
            "{duration} | {energy} |".format(
                mode=item["mode"],
                scenario=item["scenario"],
                density=item["obstacle_density"],
                runs=item["runs"],
                successes=item["successes"],
                rate=item["success_rate"],
                collision_free=item["collision_free_runs"],
                avoidance_rate=item["avoidance_success_rate"],
                duration=(
                    f"{_format(duration.get('mean'))} / "
                    f"{_format(duration.get('stddev'))}"
                    if duration else "-"
                ),
                energy=(
                    f"{_format(energy.get('mean'))} / "
                    f"{_format(energy.get('stddev'))}"
                    if energy else "-"
                ),
            )
        )
    lines.extend([
        "",
        "## Efficiency And Obstacle Avoidance",
        "",
        "| Mode | Scenario | Obstacles | Deliveries/min | Energy/target (Wh) | "
        "UGV error (m) | UAV error (m) | UAV sensor clearance (m) | "
        "UGV dynamic clearance (m) | UAV dynamic clearance (m) | "
        "UGV collisions | UAV collisions | Nav2 recoveries | UAV replans | "
        "Safety holds |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ])
    for item in summaries:
        metrics = item["metrics"]

        def mean(name):
            return metrics.get(name, {}).get("mean")

        lines.append(
            "| {mode} | {scenario} | {density} | {rate} | {energy} | "
            "{ugv_error} | {uav_error} | {clearance} | {ugv_clearance} | "
            "{uav_dynamic_clearance} | {ugv_collisions} | {uav_collisions} | "
            "{recoveries} | {replans} | {holds} |".format(
                mode=item["mode"],
                scenario=item["scenario"],
                density=item["obstacle_density"],
                rate=_format(mean("delivery_rate_per_min")),
                energy=_format(mean("energy_per_completed_target_wh")),
                ugv_error=_format(mean("ugv_endpoint_error_m")),
                uav_error=_format(mean("uav_endpoint_error_m")),
                clearance=_format(mean("minimum_uav_clearance_m")),
                ugv_clearance=_format(
                    mean("minimum_ugv_obstacle_clearance_m")
                ),
                uav_dynamic_clearance=_format(
                    mean("minimum_uav_dynamic_clearance_m")
                ),
                ugv_collisions=_format(mean("ugv_collision_count"), 2),
                uav_collisions=_format(mean("uav_collision_count"), 2),
                recoveries=_format(mean("nav_recovery_count"), 2),
                replans=_format(mean("uav_replan_count"), 2),
                holds=_format(mean("uav_safety_hold_count"), 2),
            )
        )

    lines.extend([
        "",
        "## Individual Runs",
        "",
        "| Run | Mode | Scenario | Density | Success | Collision-free | "
        "UGV/UAV collisions | UGV/UAV min clearance (m) | Sim time (s) | "
        "UGV path (m) | UAV path (m) | Total energy (Wh) | Failure reason |",
        "|---|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---|",
    ])
    for record in records:
        failure = str(record.get("failure_reason", "")).replace("|", "/")
        lines.append(
            "| {run} | {mode} | {scenario} | {density} | {success} | "
            "{collision_free} | {collisions} | {clearances} | {time} | "
            "{ugv} | {uav} | {energy} | {failure} |".format(
                run=record.get("run_id", ""),
                mode=record.get("mode", ""),
                scenario=record.get("scenario", ""),
                density=record.get("obstacle_density", ""),
                success="yes" if record.get("success") else "no",
                collision_free=(
                    "yes" if record.get("collision_free", True) else "no"
                ),
                collisions=(
                    f"{record.get('ugv_collision_count', 0)} / "
                    f"{record.get('uav_collision_count', 0)}"
                ),
                clearances=(
                    f"{_format(record.get('minimum_ugv_obstacle_clearance_m'))} / "
                    f"{_format(record.get('minimum_uav_dynamic_clearance_m'))}"
                ),
                time=_format(record.get("sim_duration_s")),
                ugv=_format(record.get("ugv_path_length_m")),
                uav=_format(record.get("uav_path_length_m")),
                energy=_format(record.get("total_energy_wh")),
                failure=failure,
            )
        )
    lines.append("")
    (output / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def _charts(output: Path, summaries: list[dict]) -> str | None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return "matplotlib is not installed; PNG charts were not generated"

    labels = [
        f"{item['mode']}\n{item['obstacle_density']}"
        for item in summaries
    ]
    chart_specs = (
        ("sim_duration_s", "Mission duration", "seconds", "phase_duration.png"),
        ("ugv_path_length_m", "UGV path length", "metres", "ugv_path_length.png"),
        ("uav_path_length_m", "UAV path length", "metres", "uav_path_length.png"),
        ("total_energy_wh", "Estimated total energy", "Wh", "energy_comparison.png"),
    )
    for field, title, unit, filename in chart_specs:
        means = [
            item["metrics"].get(field, {}).get("mean", 0.0)
            for item in summaries
        ]
        errors = [
            item["metrics"].get(field, {}).get("stddev", 0.0)
            for item in summaries
        ]
        figure, axis = plt.subplots(figsize=(max(7.0, len(labels) * 1.2), 4.8))
        axis.bar(range(len(labels)), means, yerr=errors, capsize=4, color="#1976a3")
        axis.set_title(title)
        axis.set_ylabel(unit)
        axis.set_xticks(range(len(labels)), labels, rotation=20, ha="right")
        axis.grid(axis="y", alpha=0.25)
        figure.tight_layout()
        figure.savefig(output / filename, dpi=180)
        plt.close(figure)

    rates = [item["success_rate"] * 100.0 for item in summaries]
    figure, axis = plt.subplots(figsize=(max(7.0, len(labels) * 1.2), 4.8))
    axis.bar(range(len(labels)), rates, color="#27864b")
    axis.set_title("Delivery success rate")
    axis.set_ylabel("percent")
    axis.set_ylim(0.0, 100.0)
    axis.set_xticks(range(len(labels)), labels, rotation=20, ha="right")
    axis.grid(axis="y", alpha=0.25)
    figure.tight_layout()
    figure.savefig(output / "success_rate.png", dpi=180)
    plt.close(figure)

    avoidance_rates = [
        item["avoidance_success_rate"] * 100.0 for item in summaries
    ]
    figure, axis = plt.subplots(figsize=(max(7.0, len(labels) * 1.2), 4.8))
    axis.bar(range(len(labels)), avoidance_rates, color="#b86418")
    axis.set_title("Dynamic obstacle avoidance success rate")
    axis.set_ylabel("percent")
    axis.set_ylim(0.0, 100.0)
    axis.set_xticks(range(len(labels)), labels, rotation=20, ha="right")
    axis.grid(axis="y", alpha=0.25)
    figure.tight_layout()
    figure.savefig(output / "avoidance_success_rate.png", dpi=180)
    plt.close(figure)
    return None


def generate_summary(output_directory, records=None) -> list[dict]:
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    values = load_records(output) if records is None else list(records)
    summaries = summarize(values)
    _write_markdown(output, values, summaries)
    warning = _charts(output, summaries)
    if warning:
        (output / "chart_generation_warning.txt").write_text(
            warning + "\n", encoding="utf-8"
        )
    return summaries


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Experiment result directory")
    options = parser.parse_args(args)
    summaries = generate_summary(options.input)
    print(f"Generated report for {len(summaries)} experiment groups in {options.input}")
