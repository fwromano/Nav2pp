#!/usr/bin/env python3
"""Pure-Python mirror of ChauffeurCritic for fast toy experiments."""

from dataclasses import dataclass
from math import fabs
from typing import Iterable, Sequence


@dataclass(frozen=True)
class ComfortWeights:
    longitudinal_jerk: float = 1.0
    lateral_accel: float = 0.35
    lateral_jerk: float = 1.0
    steering_effort: float = 0.10
    steering_rate: float = 0.80
    steering_accel: float = 0.35


@dataclass(frozen=True)
class ComfortDeadbands:
    longitudinal_jerk: float = 0.75
    lateral_accel: float = 1.50
    lateral_jerk: float = 1.00
    steering_effort: float = 0.75
    steering_rate: float = 0.80
    steering_accel: float = 2.00


def diff(xs: Sequence[float], dt: float) -> list[float]:
    return [(b - a) / dt for a, b in zip(xs[:-1], xs[1:])]


def deadband_sq(xs: Iterable[float], limit: float) -> list[float]:
    return [max(0.0, fabs(x) - limit) ** 2 for x in xs]


def integrate(xs: Iterable[float], dt: float) -> float:
    return sum(xs) * dt


def trajectory_metrics(
    vx: Sequence[float],
    wz: Sequence[float],
    dt: float,
    min_turning_radius: float,
    min_steering_speed: float = 0.25,
    max_normalized_steering: float = 1.5,
) -> dict[str, list[float]]:
    if len(vx) != len(wz):
        raise ValueError("vx and wz must have equal length")
    if dt <= 0:
        raise ValueError("dt must be positive")

    ax = diff(vx, dt)
    jx = diff(ax, dt)
    ay = [v * w for v, w in zip(vx, wz)]
    jy = diff(ay, dt)

    steering = []
    for v, w in zip(vx, wz):
        denom = max(abs(v), min_steering_speed)
        s = min_turning_radius * w / denom
        steering.append(max(-max_normalized_steering, min(max_normalized_steering, s)))

    steering_rate = diff(steering, dt)
    steering_accel = diff(steering_rate, dt)

    return {
        "longitudinal_jerk": jx,
        "lateral_accel": ay,
        "lateral_jerk": jy,
        "steering_effort": steering,
        "steering_rate": steering_rate,
        "steering_accel": steering_accel,
    }


def chauffeur_cost(
    vx: Sequence[float],
    wz: Sequence[float],
    dt: float,
    min_turning_radius: float,
    weights: ComfortWeights = ComfortWeights(),
    deadbands: ComfortDeadbands = ComfortDeadbands(),
) -> tuple[float, dict[str, float]]:
    metrics = trajectory_metrics(vx, wz, dt, min_turning_radius)
    parts = {
        name: getattr(weights, name)
        * integrate(deadband_sq(values, getattr(deadbands, name)), dt)
        for name, values in metrics.items()
    }
    return sum(parts.values()), parts


def summarize(
    name: str,
    vx: Sequence[float],
    wz: Sequence[float],
    dt: float,
    radius: float,
) -> float:
    total, parts = chauffeur_cost(vx, wz, dt, radius)
    print(f"{name:>24}: total={total:10.4f}")
    for key, value in parts.items():
        print(f"  {key:>20}: {value:10.4f}")
    return total
