# Jeep physical validation protocol

## Purpose

Compare Nav2 MPPI with `ChauffeurCritic` disabled versus enabled using measured
vehicle motion. The head/neck proxy remains an offline evaluation metric.

## Required signals

At minimum:

- monotonic timestamp;
- longitudinal speed `vx`;
- yaw rate `wz`;
- body-frame lateral acceleration `ay`.

Recommended:

- steering-wheel or road-wheel angle;
- MPPI selected trajectory;
- per-critic costs;
- commanded `vx/wz`;
- IMU angular velocity;
- RTK/GNSS or local pose for path-error reconstruction.

## Recorder

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source install/setup.bash

python3 extensions/nav2_chauffeur_critic/scripts/record_comfort_trial.py \
  --ros-args \
  -p odom_topic:=/odom \
  -p imu_topic:=/imu/data \
  -p output_csv:=baseline_trial_01.csv
```

If the IMU's lateral axis is opposite vehicle-left:

```bash
-p lateral_accel_sign:=-1.0
```

## Course

Use a closed, controlled test area. Suggested repeatable course:

1. straight acceleration segment;
2. constant-radius 90 degree left;
3. constant-radius 90 degree right;
4. three-cone slalom;
5. controlled stop.

Do not change obstacle/safety critics between conditions.

## Conditions

- A: `ChauffeurCritic.enabled=false`
- B: `ChauffeurCritic.enabled=true`

Recommended minimum: 10 valid repetitions per condition, interleaved ABBA/BAAB
to reduce drift from battery state, tires, terrain, or operator setup.

Keep fixed:

- global path/course;
- controller frequency;
- MPPI horizon/batch size;
- speed limits;
- tire pressure;
- payload;
- test direction where feasible.

## Offline replay

```bash
cd extensions/nav2_chauffeur_critic/demos

python3 replay_vehicle_log.py \
  --baseline /path/to/baseline_trial_01.csv \
  --chauffeur /path/to/chauffeur_trial_01.csv \
  --output-json ../analysis/physical_pair_01.json
```

Primary endpoints:

- peak and RMS measured lateral acceleration;
- peak and RMS measured lateral jerk;
- peak and RMS head-angle proxy;
- peak head angular velocity proxy;
- peak head angular acceleration proxy;
- traversal time and path error.

## Interpretation rule

Do not call the neck-model outputs injury risk, whiplash probability, or a
clinical threshold. They are comparative dynamical response metrics.

A physical result is considered supportive only if comfort improves without a
material increase in collision/path violations and the result repeats across
trials rather than depending on a single run.
