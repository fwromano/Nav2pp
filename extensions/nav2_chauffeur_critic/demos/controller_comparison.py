#!/usr/bin/env python3
"""Toy with/without-ChauffeurCritic controller-selection experiment.

Every candidate executes the same nominal 90-degree bend. Candidates vary
speed and steering-ramp duration. A baseline objective selects the fastest
candidate. The comfort-enabled objective adds log(1 + ChauffeurCritic cost).

The selected vehicle lateral acceleration is then passed into the independent
head/neck dynamic proxy.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from comfort_model import chauffeur_cost
from neck_model import NeckModel, response_metrics, simulate_head_response

DT = 0.05
TURN_CURVATURE = 0.10  # 10 m nominal radius
MIN_TURNING_RADIUS = 5.0
SPEEDS = [2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]
RAMPS = [0.10, 0.25, 0.50, 0.75, 1.00, 1.25]
LAMBDAS = [0.0, 0.05, 0.10, 0.20, 0.50, 1.0, 2.0, 5.0]


def raised_cosine(x: float) -> float:
    return 0.5 * (1.0 - math.cos(math.pi * x))


def make_corner_candidate(speed: float, ramp_s: float) -> dict:
    # For a 90 degree heading change, the integral of yaw rate must be pi/2.
    # Two half-area ramps together contribute one ramp_s at full yaw rate.
    equivalent_turn_time = (math.pi / 2.0) / (speed * TURN_CURVATURE)
    hold_s = max(0.0, equivalent_turn_time - ramp_s)
    before_s = 1.0
    after_s = 1.0
    total_s = before_s + 2.0 * ramp_s + hold_s + after_s

    n = int(round(total_s / DT)) + 1
    vx: list[float] = []
    wz: list[float] = []

    for i in range(n):
        t = i * DT
        if t < before_s:
            curvature = 0.0
        elif t < before_s + ramp_s:
            x = (t - before_s) / ramp_s
            curvature = TURN_CURVATURE * raised_cosine(x)
        elif t < before_s + ramp_s + hold_s:
            curvature = TURN_CURVATURE
        elif t < before_s + 2.0 * ramp_s + hold_s:
            x = (t - (before_s + ramp_s + hold_s)) / ramp_s
            curvature = TURN_CURVATURE * (1.0 - raised_cosine(x))
        else:
            curvature = 0.0

        vx.append(speed)
        wz.append(speed * curvature)

    comfort, parts = chauffeur_cost(vx, wz, DT, MIN_TURNING_RADIUS)
    ay = [v * w for v, w in zip(vx, wz)]
    head = response_metrics(simulate_head_response(ay, DT, NeckModel()))

    return {
        "speed_mps": speed,
        "ramp_s": ramp_s,
        "traversal_time_s": total_s,
        "critic_cost": comfort,
        "critic_parts": parts,
        **head,
    }


def candidates() -> list[dict]:
    return [make_corner_candidate(v, r) for v in SPEEDS for r in RAMPS]


def objective(row: dict, comfort_lambda: float) -> float:
    # log1p keeps one very abrupt trajectory from numerically dominating the
    # time term while preserving the ordering of comfort cost.
    return row["traversal_time_s"] + comfort_lambda * math.log1p(row["critic_cost"])


def select(rows: list[dict], comfort_lambda: float) -> dict:
    return min(rows, key=lambda row: objective(row, comfort_lambda))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    rows = candidates()
    selected = []
    for lam in LAMBDAS:
        row = dict(select(rows, lam))
        row["comfort_lambda"] = lam
        row["objective"] = objective(row, lam)
        selected.append(row)

    baseline = selected[0]
    chauffeur = next(row for row in selected if row["comfort_lambda"] == 1.0)

    print("baseline (lambda=0)")
    print(baseline)
    print("\nchauffeur (lambda=1)")
    print(chauffeur)

    angle_reduction = 1.0 - chauffeur["peak_angle_deg"] / baseline["peak_angle_deg"]
    accel_reduction = (
        1.0
        - chauffeur["peak_angular_accel_deg_s2"]
        / baseline["peak_angular_accel_deg_s2"]
    )
    time_increase = (
        chauffeur["traversal_time_s"] / baseline["traversal_time_s"] - 1.0
    )

    print(
        "\nchanges: "
        f"peak angle {angle_reduction*100:.1f}% lower, "
        f"peak head angular acceleration {accel_reduction*100:.1f}% lower, "
        f"traversal time {time_increase*100:.1f}% higher"
    )

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        fields = [
            "comfort_lambda",
            "speed_mps",
            "ramp_s",
            "traversal_time_s",
            "critic_cost",
            "peak_angle_deg",
            "rms_angle_deg",
            "peak_angular_velocity_deg_s",
            "peak_angular_accel_deg_s2",
            "objective",
        ]
        with args.csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(selected)

    if args.check:
        assert chauffeur["speed_mps"] > 0.0
        assert chauffeur["peak_angle_deg"] < baseline["peak_angle_deg"]
        assert (
            chauffeur["peak_angular_accel_deg_s2"]
            < baseline["peak_angular_accel_deg_s2"]
        )
        assert chauffeur["traversal_time_s"] < 2.0 * baseline["traversal_time_s"]

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
