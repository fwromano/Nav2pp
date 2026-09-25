#!/usr/bin/env python3
"""Repeated measured-like surrogate experiment for robustness analysis.

This does NOT replace a physical Jeep trial. It perturbs vehicle response,
accelerometer behavior, and head/neck proxy parameters to test whether the
baseline-vs-Chauffeur conclusion is fragile to plausible nuisance variation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
from pathlib import Path

from neck_model import NeckModel, response_metrics, simulate_head_response

DT = 0.05
TURN_CURVATURE = 0.10


def raised_cosine(x: float) -> float:
    return 0.5 * (1.0 - math.cos(math.pi * x))


def candidate(speed: float, ramp_s: float) -> tuple[list[float], list[float]]:
    turn_time = (math.pi / 2.0) / (speed * TURN_CURVATURE)
    hold_s = max(0.0, turn_time - ramp_s)
    before_s = 1.0
    after_s = 1.0
    total_s = before_s + 2.0 * ramp_s + hold_s + after_s
    n = int(round(total_s / DT)) + 1

    vx, wz = [], []
    for i in range(n):
        t = i * DT
        if t < before_s:
            curvature = 0.0
        elif t < before_s + ramp_s:
            curvature = TURN_CURVATURE * raised_cosine((t - before_s) / ramp_s)
        elif t < before_s + ramp_s + hold_s:
            curvature = TURN_CURVATURE
        elif t < before_s + 2.0 * ramp_s + hold_s:
            x = (t - (before_s + ramp_s + hold_s)) / ramp_s
            curvature = TURN_CURVATURE * (1.0 - raised_cosine(x))
        else:
            curvature = 0.0
        vx.append(speed)
        wz.append(speed * curvature)
    return vx, wz


def lag(signal: list[float], tau_s: float) -> list[float]:
    out = [0.0] * len(signal)
    alpha = min(1.0, DT / tau_s)
    for i in range(1, len(signal)):
        out[i] = out[i - 1] + alpha * (signal[i] - out[i - 1])
    return out


def measured_like(
    vx: list[float],
    wz: list[float],
    tau_s: float,
    gain: float,
    bias: float,
    noise_std: float,
    noise_seed: int,
) -> list[float]:
    ideal = [v * w for v, w in zip(vx, wz)]
    filtered = lag(ideal, tau_s)
    rng = random.Random(noise_seed)
    return [gain * x + bias + rng.gauss(0.0, noise_std) for x in filtered]


def sample_model(rng: random.Random) -> NeckModel:
    return NeckModel(
        head_mass_kg=max(3.8, rng.gauss(4.5, 0.35)),
        com_lever_arm_m=max(0.07, rng.gauss(0.10, 0.01)),
        rotational_inertia_kg_m2=max(0.015, rng.gauss(0.025, 0.003)),
        natural_frequency_hz=max(1.3, rng.gauss(2.0, 0.2)),
        damping_ratio=min(0.65, max(0.20, rng.gauss(0.35, 0.05))),
    )


def percentile(xs: list[float], q: float) -> float:
    values = sorted(xs)
    if not values:
        return float("nan")
    pos = (len(values) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return values[lo]
    f = pos - lo
    return values[lo] * (1.0 - f) + values[hi] * f


def median(xs: list[float]) -> float:
    return percentile(xs, 0.5)


def run(n: int) -> tuple[list[dict], dict]:
    base_v, base_w = candidate(6.0, 0.10)
    chauffeur_v, chauffeur_w = candidate(4.0, 1.25)

    paired = []
    for trial in range(n):
        rng = random.Random(1000 + trial)
        tau_s = rng.uniform(0.08, 0.25)
        gain = rng.gauss(1.0, 0.04)
        bias = rng.gauss(0.0, 0.03)
        noise_std = rng.uniform(0.03, 0.10)
        model = sample_model(rng)

        b_ay = measured_like(
            base_v, base_w, tau_s, gain, bias, noise_std, 20000 + trial
        )
        c_ay = measured_like(
            chauffeur_v, chauffeur_w, tau_s, gain, bias, noise_std, 30000 + trial
        )

        b = response_metrics(simulate_head_response(b_ay, DT, model))
        c = response_metrics(simulate_head_response(c_ay, DT, model))

        row = {
            "trial": trial,
            "vehicle_tau_s": tau_s,
            "accel_gain": gain,
            "accel_bias_mps2": bias,
            "accel_noise_std_mps2": noise_std,
            "neck_frequency_hz": model.natural_frequency_hz,
            "neck_damping_ratio": model.damping_ratio,
        }
        for key in b:
            row["baseline_" + key] = b[key]
            row["chauffeur_" + key] = c[key]
            row[key + "_reduction_pct"] = (1.0 - c[key] / b[key]) * 100.0
        paired.append(row)

    metrics = [
        "peak_angle_deg",
        "rms_angle_deg",
        "peak_angular_velocity_deg_s",
        "peak_angular_accel_deg_s2",
    ]
    summary = {"trials": n, "metrics": {}}
    for key in metrics:
        b = [r["baseline_" + key] for r in paired]
        c = [r["chauffeur_" + key] for r in paired]
        reductions = [r[key + "_reduction_pct"] for r in paired]
        summary["metrics"][key] = {
            "baseline_median": median(b),
            "chauffeur_median": median(c),
            "median_reduction_pct": median(reductions),
            "monte_carlo_95_interval_reduction_pct": [
                percentile(reductions, 0.025),
                percentile(reductions, 0.975),
            ],
            "fraction_improved": sum(ci < bi for bi, ci in zip(b, c)) / n,
        }

    return paired, summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=500)
    parser.add_argument("--output-csv", type=Path)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()

    paired, summary = run(args.trials)
    print(json.dumps(summary, indent=2))

    if args.output_csv:
        args.output_csv.parent.mkdir(parents=True, exist_ok=True)
        with args.output_csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(paired[0].keys()))
            writer.writeheader()
            writer.writerows(paired)

    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(summary, indent=2) + "\n")

    if args.check:
        for metric in summary["metrics"].values():
            assert metric["fraction_improved"] >= 0.95
            assert metric["monte_carlo_95_interval_reduction_pct"][0] > 0.0

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
