#!/usr/bin/env python3
"""Compare an abrupt left/right transition with a smooth S-curve."""

from math import pi, sin

from comfort_model import summarize

DT = 0.1
RADIUS = 4.5
N = 70
VX = [3.5] * N

abrupt = [0.0] * 10 + [0.45] * 18 + [-0.45] * 18 + [0.0] * 24
smooth = [0.0] * 8
for i in range(46):
    smooth.append(0.45 * sin(2.0 * pi * i / 45.0))
smooth += [0.0] * (N - len(smooth))
smooth = smooth[:N]

bad = summarize("abrupt S transition", VX, abrupt, DT, RADIUS)
good = summarize("smooth S transition", VX, smooth, DT, RADIUS)
print(f"\ncomfort improvement: {(1.0 - good / bad) * 100.0:.1f}% lower critic cost")
assert good < bad
