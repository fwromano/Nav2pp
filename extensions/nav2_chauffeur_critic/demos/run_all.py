#!/usr/bin/env python3
"""Run all no-ROS toy demos and fail if the critic ranks them incorrectly."""

import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
DEMOS = ["steering_step_demo.py", "s_curve_demo.py", "speed_turn_demo.py"]

for demo in DEMOS:
    print(f"\n=== {demo} ===")
    subprocess.run([sys.executable, str(HERE / demo)], cwd=HERE, check=True)

print("\nAll toy demonstrations passed.")
