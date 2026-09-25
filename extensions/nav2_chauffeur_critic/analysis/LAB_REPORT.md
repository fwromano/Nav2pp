# Laboratory Report: Nav2 MPPI ChauffeurCritic Passenger-Comfort Evaluation

**Repository:** fwromano/Nav2pp  
**Branch:** feat/chauffeur-critic  
**Status:** simulation/replay-pipeline validation complete; physical Jeep trial pending real telemetry  
**Date:** 2026-09-24

## Abstract

This study evaluates a stackable Nav2 Model Predictive Path Integral (MPPI) critic intended to prefer passenger-comfortable Ackermann trajectories without weakening collision, constraint, path-following, or goal-reaching objectives. The critic penalizes longitudinal jerk, lateral acceleration, lateral jerk, normalized steering effort, steering rate, and steering acceleration outside configurable deadbands.

Three levels of evidence were produced. First, deterministic toy experiments verify that the critic ranks ramped steering, smooth S-curves, and lower-speed traversal of a fixed-curvature bend as more comfortable than abrupt alternatives. Second, an external second-order head/neck dynamics proxy was used as an out-of-objective consequence metric. On the nominal 90 degree / 10 m-radius turn, the comfort-weighted selector reduced peak head-angle proxy by 65.2%, RMS head-angle proxy by 58.0%, peak head angular velocity by 92.7%, and peak head angular acceleration by 96.1%, while increasing traversal time by 52.1%. Third, a 500-pair measured-like Monte Carlo experiment perturbed vehicle response lag, accelerometer gain/bias/noise, and head/neck parameters. Median reductions remained 56.0% for peak head angle, 57.0% for RMS head angle, 84.0% for peak head angular velocity, and 80.0% for peak head angular acceleration; every paired trial improved all four proxy metrics.

No real Jeep telemetry was found in the connected files or repository, so this report does **not** claim physical vehicle validation. A ROS2 telemetry recorder, CSV replay analyzer, and controlled physical-test protocol are included so the same analysis can be applied directly to measured vehicle data.

## 1. Objective

Evaluate whether a stackable MPPI comfort critic can reduce aggressive steering and acceleration behavior in an Ackermann ground vehicle while preserving the existing Nav2 architecture.

Primary research question:

> Among safe and task-valid MPPI trajectories, does adding ChauffeurCritic select trajectories that produce lower passenger-dynamics excitation?

Secondary questions:

1. Does the result extend beyond the critic's own cost function to an independent dynamical proxy?
2. Does the conclusion persist under uncertainty in vehicle response, sensor measurement, and head/neck model parameters?
3. Can the evaluation be reproduced on real Jeep telemetry without changing the analysis method?

## 2. Hypotheses

**H1.** Smooth steering profiles receive lower ChauffeurCritic cost than step-like steering profiles that execute comparable maneuvers.

**H2.** For a fixed path curvature, reducing speed reduces passenger-dynamics excitation because lateral acceleration scales approximately with squared speed:

```
a_y = v * omega = v^2 * kappa
```

**H3.** A controller-selection objective containing ChauffeurCritic cost reduces an independent head/neck response metric relative to a traversal-time-only baseline.

**H4.** H3 remains directionally true under reasonable perturbations to vehicle lag, accelerometer error, neck natural frequency, and damping.

## 3. System Under Test

The plugin is implemented as an independent Nav2 MPPI critic and is loaded through pluginlib. It does not replace or modify the MPPI optimizer.

The comfort cost is a weighted sum of deadbanded penalties for:

- longitudinal jerk;
- lateral acceleration;
- lateral jerk;
- normalized Ackermann steering effort;
- steering rate;
- steering acceleration.

For Ackermann motion, steering is represented by the normalized curvature proxy

```
s = R_min * omega / max(|v|, v_floor)
```

where `R_min` is the configured minimum turning radius. Values near `|s| = 1` correspond approximately to the maximum curvature admitted by the motion model.

Deadbanding is intentional. The critic does not continuously punish normal steering or acceleration; it begins contributing cost when motion exceeds a configured comfort envelope.

## 4. Independent Passenger-Dynamics Proxy

The controller is not evaluated solely against the quantities it optimizes. Vehicle lateral acceleration is also passed to a separate rotational spring-damper model:

```
I * theta_ddot + c * theta_dot + k * theta = -m * h * a_y
```

where:

- `theta` is head angle relative to the torso;
- `m` is effective head mass;
- `h` is a head-COM lever arm;
- `I` is effective rotational inertia;
- `k` is rotational stiffness;
- `c` is rotational damping.

The default illustrative parameters are:

| Parameter | Value |
|---|---:|
| Effective head mass | 4.5 kg |
| COM lever arm | 0.10 m |
| Rotational inertia | 0.025 kg m^2 |
| Natural frequency | 2.0 Hz |
| Damping ratio | 0.35 |

The equations are integrated with fourth-order Runge-Kutta at 20 Hz in the deterministic experiments.

This model is a **comparative dynamics proxy**, not a clinical whiplash model or injury predictor. Human head-neck dynamics can behave approximately as a second-order underdamped passive system over limited operating ranges, but the system is not strictly linear and changes with loading and neural activation. Tangorra, Jones, and Hunter reported second-order underdamped behavior over 0.5-10 Hz under their test conditions. Keshner and Peterson documented frequency-dependent mechanical, reflex, and voluntary contributions to head stabilization, including resonance behavior at higher perturbation frequencies. More detailed lumped-parameter models use multiple cervical bodies and intervertebral joints.

## 5. Experimental Design

### 5.1 Deterministic unit demonstrations

Three no-ROS experiments test local behavior:

1. abrupt steering step versus a gradual steering ramp;
2. abrupt left-right transition versus a smooth S-curve;
3. 6 m/s versus 3 m/s on the same fixed-curvature bend.

Each executable asserts that the smoother candidate receives lower comfort cost.

### 5.2 Controller-selection experiment

All candidate trajectories execute the same nominal 90 degree, 10 m-radius turn.

Candidate variables:

- speed: 2.5-6.0 m/s;
- steering transition duration: 0.10-1.25 s.

Baseline objective:

```
J_baseline = traversal_time
```

Comfort-weighted objective:

```
J = traversal_time + lambda * log(1 + J_chauffeur)
```

The logarithm preserves comfort-cost ordering while preventing a discontinuous trajectory from numerically overwhelming traversal time.

The reported comparison uses `lambda = 1`.

### 5.3 Measured-like Monte Carlo robustness experiment

Five hundred paired trials were generated. Within each pair, baseline and Chauffeur conditions used the same sampled vehicle/sensor and neck-model nuisance parameters.

Perturbed quantities:

| Quantity | Distribution / range |
|---|---|
| Vehicle lateral-response time constant | uniform 0.08-0.25 s |
| Accelerometer gain | Gaussian, mean 1.0, sigma 0.04 |
| Accelerometer bias | Gaussian, mean 0, sigma 0.03 m/s^2 |
| Accelerometer white-noise sigma | uniform 0.03-0.10 m/s^2 |
| Effective head mass | Gaussian around 4.5 kg, floor 3.8 kg |
| COM lever arm | Gaussian around 0.10 m, floor 0.07 m |
| Rotational inertia | Gaussian around 0.025 kg m^2, floor 0.015 |
| Natural frequency | Gaussian around 2.0 Hz, floor 1.3 Hz |
| Damping ratio | Gaussian around 0.35, clipped 0.20-0.65 |

The vehicle acceleration command was passed through a first-order lag and then perturbed by gain, bias, and measurement noise. The resulting measured-like acceleration trace drove the independently parameterized head/neck proxy.

The reported 95% intervals are empirical Monte Carlo percentile intervals, not confidence intervals from physical repeated trials.

### 5.4 Neck-model sensitivity grid

A deterministic 3 x 3 grid was evaluated across:

- natural frequency: 1.5, 2.0, 2.5 Hz;
- damping ratio: 0.25, 0.35, 0.50.

This tests whether the conclusion depends strongly on the nominal 2.0 Hz / 0.35 assumption.

## 6. Results

### 6.1 Deterministic baseline versus Chauffeur

| Metric | Baseline | Chauffeur | Change |
|---|---:|---:|---:|
| Selected speed | 6.0 m/s | 4.0 m/s | -33.3% |
| Steering ramp | 0.10 s | 1.25 s | +1150% |
| Traversal time | 4.718 s | 7.177 s | +52.1% |
| Peak head-angle proxy | 30.45 deg | 10.61 deg | **-65.2%** |
| RMS head-angle proxy | 17.61 deg | 7.40 deg | **-58.0%** |
| Peak head angular velocity | 177.87 deg/s | 13.00 deg/s | **-92.7%** |
| Peak head angular acceleration | 1422.87 deg/s^2 | 55.11 deg/s^2 | **-96.1%** |

The result is not merely lower critic cost. The independently calculated passenger-dynamics proxy is also substantially lower.

### 6.2 Comfort-weight sweep

As `lambda` increases, the selected strategy changes in stages:

- low weights retain 6 m/s but begin lengthening steering transitions;
- intermediate weights strongly smooth steering;
- higher weights reduce speed through the bend.

This is the expected qualitative behavior from `a_y = v^2 * kappa`: smoothing steering transitions reduces transient excitation, but reducing steady lateral acceleration eventually requires lower speed or lower curvature.

### 6.3 Monte Carlo robustness

| Passenger proxy metric | Baseline median | Chauffeur median | Median reduction | 95% Monte Carlo interval | Fraction improved |
|---|---:|---:|---:|---:|---:|
| Peak head angle | 25.36 deg | 10.98 deg | **56.0%** | 52.7-63.0% | 1.000 |
| RMS head angle | 17.09 deg | 7.33 deg | **57.0%** | 55.8-58.4% | 1.000 |
| Peak angular velocity | 112.38 deg/s | 18.13 deg/s | **84.0%** | 75.8-88.9% | 1.000 |
| Peak angular acceleration | 814.66 deg/s^2 | 164.81 deg/s^2 | **80.0%** | 60.5-89.6% | 1.000 |

All 500 paired surrogate trials improved all four passenger-dynamics proxy metrics.

### 6.4 Neck-model sensitivity

Across all nine natural-frequency/damping combinations:

- peak head-angle reduction ranged from **60.9% to 68.1%**;
- peak head-angular-acceleration reduction ranged from **95.0% to 97.0%**.

Thus the deterministic conclusion is not specific to the nominal 2.0 Hz / 0.35 neck parameters over this tested grid.

## 7. Physical Replay Pipeline

No existing Jeep telemetry or rosbag containing the required signals was found in the connected files or `fwromano/Nav2pp`. A physical run therefore was not fabricated or inferred.

The branch now contains:

- `scripts/record_comfort_trial.py`: ROS2 recorder for odometry and IMU;
- `demos/replay_vehicle_log.py`: offline CSV resampling and metric computation;
- `docs/PHYSICAL_TEST_PROTOCOL.md`: controlled OFF/ON test procedure.

The recorder captures:

```
time_s
ros_time_s
vx_mps
vy_mps
wz_rad_s
imu_ax_mps2
ay_mps2
imu_az_mps2
imu_wz_rad_s
```

The replay analyzer uses measured `ay_mps2` as the passenger-model input and reports measured lateral acceleration/jerk plus head/neck proxy metrics.

## 8. Proposed Physical Test

Recommended minimum experiment:

- 10 valid baseline trials;
- 10 valid Chauffeur trials;
- interleaved ABBA/BAAB condition order;
- fixed course, payload, controller rate, MPPI horizon, speed caps, tire pressure, and safety critics;
- straight, left/right constant-radius turns, slalom, and controlled stop.

Primary outcomes:

1. peak and RMS measured lateral acceleration;
2. peak and RMS measured lateral jerk;
3. peak/RMS head-angle proxy;
4. peak head angular velocity/acceleration proxy;
5. traversal time;
6. path error;
7. collision/constraint events.

A supportive physical result requires repeated improvement in comfort metrics without a material increase in path violations or safety events.

## 9. Threats to Validity

### 9.1 The head/neck model is intentionally low order

Real cervical dynamics are multi-body, nonlinear, posture-dependent, and influenced by muscle activation and vestibular/proprioceptive control. The proxy should therefore be used for relative controller comparison, not injury prediction.

### 9.2 The deterministic candidate selector is not full MPPI

The candidate-set experiment isolates the intended objective trade but is not a complete Gazebo or vehicle-level reproduction of the MPPI optimizer. The actual plugin still requires a full Nav2 integration test.

### 9.3 Surrogate sensor perturbations are not measured Jeep noise distributions

The Monte Carlo distributions are engineering stress tests, not empirically fitted sensor models. Their purpose is to test fragility, not to estimate real-world probability.

### 9.4 Comfort is broader than lateral head motion

Whole-body vibration includes multiple axes and frequencies. ISO 2631-1 addresses measurement/evaluation of whole-body vibration for health, comfort/perception, and motion sickness; the current published 1997 edition remains current while a third-edition draft is under development in 2026. ChauffeurCritic currently targets planar driving smoothness, not a complete ISO 2631 exposure metric.

### 9.5 Time/comfort trade is tunable, not universally optimal

The `lambda=1` comparison is an illustrative operating point. The correct deployed weight depends on mission urgency, terrain, vehicle limits, and acceptable passenger motion.

## 10. Reproducibility

From the repository branch:

```bash
cd extensions/nav2_chauffeur_critic/demos

python3 run_all.py
python3 controller_comparison.py --check
python3 monte_carlo_surrogate.py --trials 500 --check
```

Physical replay:

```bash
python3 replay_vehicle_log.py \
  --baseline /path/to/baseline.csv \
  --chauffeur /path/to/chauffeur.csv \
  --output-json ../analysis/physical_pair.json
```

GitHub Actions executes the toy demos, head/neck unit tests, deterministic controller comparison, and 500-trial Monte Carlo robustness assertion.

## 11. Conclusions

The present evidence supports the narrow engineering claim that ChauffeurCritic encodes the intended preference for smooth Ackermann motion and that its selected trajectories reduce an independent low-order passenger-dynamics proxy in simulation.

The evidence is strongest for **direction of effect**, not absolute human comfort magnitude. The conclusion remains stable across the tested neck-model sensitivity grid and 500 measured-like perturbation trials.

The principal remaining falsification opportunity is the physical Jeep experiment. Measured vehicle acceleration may expose actuator nonlinearities, terrain impulses, suspension/body roll, timing artifacts, or planner-controller interactions absent from the current model. The repository now contains the instrumentation and replay path required to perform that test without changing the primary analysis.

## 12. Peer-Review Questions

1. Is normalized curvature `R_min * omega / |v|` an adequate steering proxy, or should road-wheel angle be required for Ackermann platforms?
2. Should lateral acceleration and lateral jerk remain separate critic terms, or does that double-count the same discomfort mechanism?
3. Are deadband-squared penalties preferable to Huber, softplus, or asymmetric penalties?
4. Is the use of a single-DOF passenger proxy appropriate for controller validation, provided no injury claim is made?
5. Should the physical experiment use ISO 2631-style frequency-weighted whole-body acceleration as an additional endpoint?
6. Is the time/comfort Pareto sweep sufficient, or should path error and obstacle-clearance margins form additional Pareto dimensions?
7. Does the Monte Carlo nuisance model omit a likely failure mode, especially suspension roll, terrain shocks, steering actuator saturation, or time synchronization?
8. What physical effect size and repeatability criterion should be required before treating the critic as validated on the Jeep?

## References

1. Tangorra JL, Jones LA, Hunter IW. Dynamics of the human head-neck system in the horizontal plane: joint properties with respect to a static torque. *Ann Biomed Eng.* 2003;31(5):606-620. PMID 12757204. DOI 10.1114/1.1566772.
2. Keshner EA, Peterson BW. Mechanisms controlling human head stabilization. I. Head-neck dynamics during random rotations in the horizontal plane. *J Neurophysiol.* 1995;73(6):2293-2301. PMID 7666139. DOI 10.1152/jn.1995.73.6.2293.
3. Deng YC, Goldsmith W. Response of a human head/neck/upper-torso replica to dynamic loading--II. Analytical/numerical model. *J Biomech.* 1987;20(5):487-497. PMID 3611123. DOI 10.1016/0021-9290(87)90249-1.
4. McGill SM, Jones K, Bennett G, Bishop PJ. Passive stiffness of the human neck in flexion, extension, and lateral bending. *Clin Biomech.* 1994;9(3):193-198. PMID 23916181. DOI 10.1016/0268-0033(94)90021-3.
5. ISO 2631-1:1997. Mechanical vibration and shock - Evaluation of human exposure to whole-body vibration - Part 1: General requirements. Current published edition reviewed/confirmed in 2021; ISO/DIS 2631-1 Edition 3 under development in 2026.
