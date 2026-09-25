#!/usr/bin/env python3
"""Compare an abrupt steering command with a chauffeur-like ramp."""

from comfort_model import summarize

DT = 0.1
RADIUS = 4.5
N = 50
VX = [4.0] * N

abrupt = [0.0] * 10 + [0.55] * 20 + [0.0] * 20
smooth = (
    [0.0] * 8
    + [0.08, 0.16, 0.25, 0.34, 0.43, 0.50, 0.55]
    + [0.55] * 12
    + [0.50, 0.43, 0.34, 0.25, 0.16, 0.08]
    + [0.0] * 15
)
smooth = smooth[:N] + [0.0] * max(0, N - len(smooth))

bad = summarize("abrupt steering", VX, abrupt, DT, RADIUS)
good = summarize("smooth steering", VX, smooth, DT, RADIUS)
print(f"\ncomfort improvement: {(1.0 - good / bad) * 100.0:.1f}% lower critic cost")
assert good < bad
