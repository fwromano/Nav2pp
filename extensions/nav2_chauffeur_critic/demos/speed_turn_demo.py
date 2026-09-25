#!/usr/bin/env python3
"""Show why MPPI can prefer a slower candidate on the same-curvature bend."""

from comfort_model import summarize

DT = 0.1
RADIUS = 5.0
N = 60
CURVATURE = 1.0 / 10.0  # 10 m path radius


def candidate(speed: float) -> tuple[list[float], list[float]]:
    vx = [speed] * N
    wz = [speed * CURVATURE] * N
    return vx, wz


fast_v, fast_w = candidate(6.0)
slow_v, slow_w = candidate(3.0)

fast = summarize("6 m/s through bend", fast_v, fast_w, DT, RADIUS)
slow = summarize("3 m/s through bend", slow_v, slow_w, DT, RADIUS)
print(f"\ncomfort improvement: {(1.0 - slow / fast) * 100.0:.1f}% lower critic cost")
assert slow < fast
