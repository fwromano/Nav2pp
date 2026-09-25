#!/usr/bin/env python3
"""Replay measured vehicle CSV telemetry through comfort/head-neck metrics."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

from neck_model import NeckModel, response_metrics, simulate_head_response


def load_csv(path: Path) -> list[dict[str, float]]:
    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        required = {"time_s", "ay_mps2"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path}: missing columns {sorted(missing)}")
        for raw in reader:
            if not raw.get("time_s") or not raw.get("ay_mps2"):
                continue
            row = {}
            for key, value in raw.items():
                if value not in (None, ""):
                    try:
                        row[key] = float(value)
                    except ValueError:
                        pass
            rows.append(row)
    if len(rows) < 4:
        raise ValueError(f"{path}: need at least four valid samples")
    rows.sort(key=lambda row: row["time_s"])
    return rows


def median_dt(rows: list[dict[str, float]]) -> float:
    dts = [
        b["time_s"] - a["time_s"]
        for a, b in zip(rows[:-1], rows[1:])
        if b["time_s"] > a["time_s"]
    ]
    if not dts:
        raise ValueError("timestamps are not increasing")
    return statistics.median(dts)


def interp(rows: list[dict[str, float]], key: str, t: float) -> float:
    if t <= rows[0]["time_s"]:
        return rows[0][key]
    if t >= rows[-1]["time_s"]:
        return rows[-1][key]

    lo = 0
    hi = len(rows) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if rows[mid]["time_s"] <= t:
            lo = mid
        else:
            hi = mid

    a, b = rows[lo], rows[hi]
    fraction = (t - a["time_s"]) / (b["time_s"] - a["time_s"])
    return a[key] + fraction * (b[key] - a[key])


def resample(rows: list[dict[str, float]]) -> tuple[float, list[dict[str, float]]]:
    dt = median_dt(rows)
    # Reject extreme timing pathologies rather than silently inventing data.
    if dt <= 0.0 or dt > 0.5:
        raise ValueError(f"implausible median sample period {dt:.6f} s")

    keys = [key for key in rows[0].keys() if key != "time_s"]
    t0, t1 = rows[0]["time_s"], rows[-1]["time_s"]
    n = int(math.floor((t1 - t0) / dt)) + 1
    out = []
    for i in range(n):
        t = t0 + i * dt
        row = {"time_s": t - t0}
        for key in keys:
            if all(key in r for r in rows):
                row[key] = interp(rows, key, t)
        out.append(row)
    return dt, out


def rms(values: list[float]) -> float:
    return math.sqrt(sum(x * x for x in values) / len(values)) if values else 0.0


def analyze(path: Path, model: NeckModel) -> dict:
    raw = load_csv(path)
    dt, rows = resample(raw)
    ay = [row["ay_mps2"] for row in rows]
    jerk = [(b - a) / dt for a, b in zip(ay[:-1], ay[1:])]

    result = {
        "file": str(path),
        "samples": len(rows),
        "sample_period_s": dt,
        "duration_s": rows[-1]["time_s"],
        "peak_abs_lateral_accel_mps2": max(abs(x) for x in ay),
        "rms_lateral_accel_mps2": rms(ay),
        "peak_abs_lateral_jerk_mps3": max((abs(x) for x in jerk), default=0.0),
        "rms_lateral_jerk_mps3": rms(jerk),
    }

    if all("vx_mps" in row for row in rows):
        vx = [row["vx_mps"] for row in rows]
        result["mean_speed_mps"] = statistics.fmean(vx)
        result["peak_speed_mps"] = max(abs(x) for x in vx)

    if all("wz_rad_s" in row for row in rows):
        wz = [row["wz_rad_s"] for row in rows]
        result["peak_abs_yaw_rate_rad_s"] = max(abs(x) for x in wz)

    result.update(response_metrics(simulate_head_response(ay, dt, model)))
    return result


def compare(baseline: dict, chauffeur: dict) -> dict:
    comparable = [
        "peak_abs_lateral_accel_mps2",
        "rms_lateral_accel_mps2",
        "peak_abs_lateral_jerk_mps3",
        "rms_lateral_jerk_mps3",
        "peak_angle_deg",
        "rms_angle_deg",
        "peak_angular_velocity_deg_s",
        "peak_angular_accel_deg_s2",
    ]
    changes = {}
    for key in comparable:
        b = baseline.get(key, 0.0)
        c = chauffeur.get(key, 0.0)
        changes[key + "_reduction_pct"] = (1.0 - c / b) * 100.0 if b else None
    return {"baseline": baseline, "chauffeur": chauffeur, "changes": changes}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--chauffeur", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--natural-frequency-hz", type=float, default=2.0)
    parser.add_argument("--damping-ratio", type=float, default=0.35)
    args = parser.parse_args()

    model = NeckModel(
        natural_frequency_hz=args.natural_frequency_hz,
        damping_ratio=args.damping_ratio,
    )
    result = compare(analyze(args.baseline, model), analyze(args.chauffeur, model))
    rendered = json.dumps(result, indent=2)
    print(rendered)

    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered + "\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
