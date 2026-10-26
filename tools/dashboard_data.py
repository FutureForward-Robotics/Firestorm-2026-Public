#!/usr/bin/env python3
"""Builds the data files behind the Robot Runtime Logs dashboard from every run in logs/.

    python3 tools/dashboard_data.py                  # newest 10 runs into logs/dashboard/
    python3 tools/dashboard_data.py --last 0         # every run in logs/
    python3 tools/dashboard_data.py --bin 0.1        # finer time bins (bigger signals.csv)

Writes three files, which replace the dashboard's attached sources of the same names:

    runs.json      one summary row per run (length, loop rate, battery sag, shooter at speed)
    signals.csv    every run's signals averaged into fixed time bins, tagged with its run
    commands.json  commands that span time (InstantCommand left out), with start and end seconds

Exit codes: 0 ok, 5 no runs found.
Standard library only.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from datetime import datetime
from pathlib import Path

import runlog

DEFAULT_BIN_S = 0.2
DEFAULT_LAST_RUNS = 10

# Dashboard column -> RunLog column. A bin takes the mean of these.
AVERAGED_COLUMNS = {
    "loop_hz": "loop.hz",
    "inputs_ms": "loop.inputsMs",
    "sense_ms": "loop.senseMs",
    "scheduler_ms": "loop.schedulerMs",
    "act_ms": "loop.actMs",
    "pollen_rpm": "pollenShooter.rpm",
    "pollen_target_rpm": "pollenShooter.targetRpm",
    "pollen_current_a": "pollenShooter.currentA",
    "nectar_rpm": "nectarShooter.rpm",
    "nectar_target_rpm": "nectarShooter.targetRpm",
    "battery_v": "battery.volts",
}

# Dashboard column -> RunLog column. A bin takes the last value, so pose, headings and states stay
# real values.
LAST_VALUE_COLUMNS = {
    "x": "drive.x",
    "y": "drive.y",
    "heading_deg": "drive.headingDeg",
    "intake_on": "intake.state",
    "pollen_at_speed": "pollenShooter.atSpeed",
    "assists": "drive.assists",
    # Aiming: the shot calculator's wanted heading, how far off the robot is, and which CELL.
    # Headings are last values because a mean across the 0/360 wrap is meaningless.
    "aim_heading_deg": "pollenShot.headingDeg",
    "aim_error_deg": "pollenShot.headingErrorDeg",
    "aim_cell_y": "pollenShot.targetCellY",
    "aim_distance_in": "pollenShot.distance",
    "can_shoot": "pollenShot.canShoot",
    "aim_bearing_deg": "pollenShot.bearingDeg",
    "aim_shift_in": "pollenShot.aimShiftIn",
    "aim_entrance_shift_in": "pollenShot.entranceShiftIn",
}

# Indexer.State.FEED_POLLEN's ordinal: the driver is holding the shoot button.
INDEXER_FEED_POLLEN = 2

# Shooter.State.SHOOT's ordinal: in TeleOp, only the aim trigger puts the shooter here.
SHOOTER_SPUN_UP_TO_SHOOT = 2

# Battery readings below this are sensor dropouts, not real voltage.
MIN_REAL_BATTERY_V = 5.0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="dashboard_data.py", description="Build the runtime logs dashboard's data files."
    )
    parser.add_argument("--logs", type=Path, default=runlog.repo_root() / "logs")
    parser.add_argument("--out", type=Path, help="output folder, default <logs>/dashboard")
    parser.add_argument("--bin", type=float, default=DEFAULT_BIN_S, help="time bin in seconds")
    parser.add_argument(
        "--last", type=int, default=DEFAULT_LAST_RUNS, help="newest N runs only; 0 for every run"
    )
    args = parser.parse_args(argv)

    run_dirs = runlog.find_runs(args.logs)
    if args.last > 0:
        run_dirs = run_dirs[-args.last :]
    if not run_dirs:
        print(f"no runs in {args.logs}", file=sys.stderr)
        return 5
    out_dir = args.out or args.logs / "dashboard"
    out_dir.mkdir(parents=True, exist_ok=True)

    run_summaries: list[dict] = []
    signal_rows: list[dict] = []
    command_spans: list[dict] = []
    for run_dir in run_dirs:
        run = runlog.load_run(run_dir)
        if not run.t:
            print(f"skipped {run.name}: no signals", file=sys.stderr)
            continue
        drop_zero_pose_at_stop(run)
        run_summaries.append(summarize_run(run))
        signal_rows.extend(binned_signals(run, args.bin))
        command_spans.extend(spanning_commands(run))

    (out_dir / "runs.json").write_text(json.dumps(run_summaries), encoding="utf-8")
    (out_dir / "commands.json").write_text(json.dumps(command_spans), encoding="utf-8")
    write_signals_csv(out_dir / "signals.csv", signal_rows)

    print(
        f"{len(run_summaries)} runs, {len(signal_rows)} signal rows, "
        f"{len(command_spans)} command spans -> {out_dir}"
    )
    return 0


def drop_zero_pose_at_stop(run: runlog.Run) -> None:
    """Removes the last loop when stop cut off its odometry read and the pose came back as zeros.

    The SDK hands back zero bytes for an I2C read interrupted by stop, so the final loop can show
    the robot at (0, 0, 0) though it never moved. Drive.isTrustworthyReading rejects these on the robot;
    logs from before that fix still have them.
    """
    pose_columns = ("drive.x", "drive.y", "drive.headingDeg")
    if len(run.t) < 2 or any(column not in run.series for column in pose_columns):
        return
    last_pose = [run.series[column][-1] for column in pose_columns]
    previous_pose = [run.series[column][-2] for column in pose_columns]
    if last_pose == [0, 0, 0] and previous_pose != [0, 0, 0]:
        run.t.pop()
        run.loops.pop()
        for values in run.series.values():
            values.pop()


def summarize_run(run: runlog.Run) -> dict:
    # The first loop's rate is measured from start-up, not from a previous loop.
    loop_rates = present(run.series.get("loop.hz", [])[1:])
    loop_rates = [rate for rate in loop_rates if rate > 0]
    battery_volts = [v for v in present(run.series.get("battery.volts", [])) if v > MIN_REAL_BATTERY_V]
    shooter_current = present(run.series.get("pollenShooter.currentA", []))

    return {
        "run": run.name,
        "op_mode": run.op_mode,
        "label": label_from_name(run.name, run.op_mode),
        "started": started_from_name(run.name),
        "duration_s": round(run_end_s(run), 1),
        "loops": len(run.t),
        "avg_hz": round_or_none(statistics.fmean(loop_rates) if loop_rates else None, 1),
        "p5_hz": round_or_none(percentile(loop_rates, 0.05), 1),
        "min_battery_v": round_or_none(min(battery_volts) if battery_volts else None, 2),
        "start_battery_v": round_or_none(battery_volts[0] if battery_volts else None, 2),
        "peak_shooter_a": round_or_none(max(shooter_current) if shooter_current else None, 2),
        "shooter_at_speed_share": round_or_none(shooter_at_speed_share(run), 3),
        "commands": sum(1 for e in run.events if e.get("type") == "command" and e.get("event") == "start"),
    }


def shooter_at_speed_share(run: runlog.Run) -> float | None:
    """Share of loops at speed among loops where the pollen shooter had a target RPM."""
    targets = run.series.get("pollenShooter.targetRpm", [])
    at_speed = run.series.get("pollenShooter.atSpeed", [])
    if not targets or not at_speed:
        return None
    loops_with_target = [i for i, target in enumerate(targets) if (target or 0) > 0]
    if not loops_with_target:
        return None
    loops_at_speed = [i for i in loops_with_target if at_speed[i] == 1]
    return len(loops_at_speed) / len(loops_with_target)


def binned_signals(run: runlog.Run, bin_s: float) -> list[dict]:
    loop_indexes_by_bin: dict[int, list[int]] = {}
    for loop_index, t in enumerate(run.t):
        loop_indexes_by_bin.setdefault(int(t / bin_s), []).append(loop_index)

    rows = []
    for bin_number, loop_indexes in sorted(loop_indexes_by_bin.items()):
        row = {"run": run.name, "t": fmt(bin_number * bin_s)}
        for name, column in AVERAGED_COLUMNS.items():
            values = present(values_at(run, column, loop_indexes))
            row[name] = fmt(statistics.fmean(values)) if values else ""
        loop_rates = [rate for rate in present(values_at(run, "loop.hz", loop_indexes)) if rate > 0]
        row["loop_hz_min"] = fmt(min(loop_rates)) if loop_rates else ""
        # Any loop, not the last, so a tap of the shoot button shorter than a bin still shows.
        indexer_states = values_at(run, "indexer.state", loop_indexes)
        row["feeding_pollen"] = "1" if INDEXER_FEED_POLLEN in indexer_states else "0"
        shooter_states = values_at(run, "pollenShooter.state", loop_indexes)
        row["aim_trigger_held"] = "1" if SHOOTER_SPUN_UP_TO_SHOOT in shooter_states else "0"
        for name, column in LAST_VALUE_COLUMNS.items():
            row[name] = fmt(values_at(run, column, loop_indexes[-1:])[0])
        rows.append(row)
    return rows


def spanning_commands(run: runlog.Run) -> list[dict]:
    """Pairs each command start with its finish or interrupt. Unfinished ones end with the run."""
    spans = []
    started_by_id: dict = {}
    for event in run.events:
        if event.get("type") != "command":
            continue
        if event.get("event") == "start":
            started_by_id[event.get("id")] = event
            continue
        start_event = started_by_id.pop(event.get("id"), None)
        if start_event is not None:
            spans.append(command_span(run, start_event, event["t"], event.get("event")))
    for start_event in started_by_id.values():
        spans.append(command_span(run, start_event, run_end_s(run), "run end"))
    return [span for span in spans if span["name"] != "InstantCommand"]


def command_span(run: runlog.Run, start_event: dict, end_s: float, ended_by: str) -> dict:
    return {
        "run": run.name,
        "name": start_event.get("name"),
        "start": start_event["t"],
        "end": end_s,
        "ended_by": ended_by,
    }


def write_signals_csv(path: Path, rows: list[dict]) -> None:
    columns = ["run", "t", *AVERAGED_COLUMNS, "loop_hz_min", "feeding_pollen", "aim_trigger_held", *LAST_VALUE_COLUMNS]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def label_from_name(run_name: str, op_mode: str) -> str:
    """Text added to a run folder after the op mode, e.g. MATCH4_auto. Empty when there is none."""
    prefix = f"{run_name[:15]}-{op_mode.replace(' ', '_')}-"
    return run_name[len(prefix):] if run_name.startswith(prefix) else ""


def run_end_s(run: runlog.Run) -> float:
    for event in run.events:
        if event.get("type") == "run" and event.get("event") == "end":
            return float(event["t"])
    return run.duration_s


def started_from_name(run_name: str) -> str:
    """Run folders start with the laptop's local time, e.g. 20261014-145916-TeleOp_Red."""
    return datetime.strptime(run_name[:15], "%Y%m%d-%H%M%S").strftime("%Y-%m-%dT%H:%M:%S")


def values_at(run: runlog.Run, column: str, loop_indexes: list[int]) -> list[float | None]:
    series = run.series.get(column)
    if series is None:
        return [None] * len(loop_indexes)
    return [series[i] for i in loop_indexes]


def present(values: list[float | None]) -> list[float]:
    return [v for v in values if v is not None]


def percentile(values: list[float], share: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[int(share * (len(ordered) - 1))]


def round_or_none(value: float | None, digits: int) -> float | None:
    return None if value is None else round(value, digits)


def fmt(value: float | None) -> str:
    """Two decimals at most, trailing zeros dropped, to keep signals.csv small."""
    if value is None:
        return ""
    return f"{value:.2f}".rstrip("0").rstrip(".")


if __name__ == "__main__":
    sys.exit(main())
