import argparse
from datetime import datetime, timezone
from pathlib import Path
import subprocess

from delivery_evaluation.report_generator import (
    generate_summary,
    load_records,
    write_records,
)
from delivery_evaluation.paths import default_results_directory


VALID_MODES = ("ugv_only", "uav_only", "cooperative")
VALID_DENSITIES = ("none", "low", "medium", "high")


def csv_values(raw: str) -> list[str]:
    return [value.strip() for value in str(raw).split(",") if value.strip()]


def launch_command(
    mode: str,
    scenario: str,
    density: str,
    repetitions: int,
    seed: int,
    output_directory: Path,
    initial_battery_percentage: float,
    continue_on_failure: bool,
    initial_ugv_drive_battery_percentage: float = 0.80,
    initial_ugv_charging_battery_percentage: float = 0.80,
) -> list[str]:
    return [
        "ros2",
        "launch",
        "delivery_evaluation",
        "experiment.launch.py",
        f"mode:={mode}",
        f"scenario:={scenario}",
        f"obstacle_density:={density}",
        f"repetitions:={int(repetitions)}",
        f"random_seed:={int(seed)}",
        f"results_dir:={output_directory}",
        f"initial_battery_percentage:={initial_battery_percentage:.3f}",
        "initial_ugv_drive_battery_percentage:="
        f"{initial_ugv_drive_battery_percentage:.3f}",
        "initial_ugv_charging_battery_percentage:="
        f"{initial_ugv_charging_battery_percentage:.3f}",
        f"continue_on_failure:={str(bool(continue_on_failure)).lower()}",
        "gui:=false",
        "rviz:=false",
        "visualize_sensor_rays:=false",
    ]


def collect_batch_records(output_directory: Path) -> list[dict]:
    records = []
    for path in sorted(output_directory.glob("*/runs.json")):
        records.extend(load_records(path))
    return records


def _validate(values, allowed, description):
    unknown = sorted(set(values) - set(allowed))
    if unknown:
        raise ValueError(
            f"Unknown {description}: {', '.join(unknown)}. "
            f"Available: {', '.join(allowed)}"
        )


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description="Run and aggregate the campus delivery experiment matrix."
    )
    parser.add_argument(
        "--modes", default=",".join(VALID_MODES),
        help="Comma-separated experiment modes.",
    )
    parser.add_argument(
        "--densities", default=",".join(VALID_DENSITIES),
        help="Comma-separated dynamic obstacle densities.",
    )
    parser.add_argument("--seeds", default="42,43,44")
    parser.add_argument("--scenario", default="teaching_building")
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--initial-battery", type=float, default=0.80)
    parser.add_argument("--initial-ugv-drive-battery", type=float, default=0.80)
    parser.add_argument(
        "--initial-ugv-charging-battery", type=float, default=0.80
    )
    parser.add_argument(
        "--results-dir",
        default=str(default_results_directory()),
    )
    parser.add_argument("--continue-on-failure", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    options = parser.parse_args(args)

    modes = csv_values(options.modes)
    densities = csv_values(options.densities)
    seeds = [int(value) for value in csv_values(options.seeds)]
    _validate(modes, VALID_MODES, "modes")
    _validate(densities, VALID_DENSITIES, "densities")
    if not seeds:
        raise ValueError("At least one random seed is required")
    if options.repetitions <= 0:
        raise ValueError("repetitions must be positive")
    if not 0.0 <= options.initial_battery <= 1.0:
        raise ValueError("initial battery must be in the range 0..1")
    if not 0.0 <= options.initial_ugv_drive_battery <= 1.0:
        raise ValueError("initial UGV drive battery must be in the range 0..1")
    if not 0.0 <= options.initial_ugv_charging_battery <= 1.0:
        raise ValueError("initial UGV charging battery must be in the range 0..1")

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = Path(options.results_dir).expanduser() / (
        f"{timestamp}_matrix_{options.scenario}"
    )
    commands = [
        launch_command(
            mode,
            options.scenario,
            density,
            options.repetitions,
            seed,
            output,
            options.initial_battery,
            options.continue_on_failure,
            options.initial_ugv_drive_battery,
            options.initial_ugv_charging_battery,
        )
        for mode in modes
        for density in densities
        for seed in seeds
    ]
    print(
        f"Experiment matrix: {len(commands)} simulator batches, "
        f"{len(commands) * options.repetitions} runs, output={output}"
    )
    for index, command in enumerate(commands, start=1):
        print(f"[{index}/{len(commands)}] {' '.join(command)}", flush=True)
        if options.dry_run:
            continue
        completed = subprocess.run(command, check=False)
        if completed.returncode != 0 and not options.continue_on_failure:
            raise SystemExit(completed.returncode)

    if options.dry_run:
        return
    records = collect_batch_records(output)
    write_records(output, records)
    summaries = generate_summary(output, records)
    print(
        f"Aggregated {len(records)} runs into {len(summaries)} groups at {output}"
    )
