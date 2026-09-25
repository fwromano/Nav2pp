# Nav2 MPPI ChauffeurCritic

`ChauffeurCritic` is a soft MPPI objective that prefers passenger-comfortable trajectories without replacing Nav2's safety, obstacle, path, or goal critics.

It is intended for Ackermann vehicles such as a UGV/Jeep where two trajectories may both be safe and path-valid, but one would whip the steering, enter a bend too fast, or abruptly change acceleration.

## What it scores

For each sampled MPPI trajectory, the critic adds deadbanded cost for:

- longitudinal jerk: `d²(vx)/dt²`
- lateral acceleration: `vx * wz`
- lateral jerk: `d(vx*wz)/dt`
- steering effort near full lock
- steering rate
- steering acceleration

For Ackermann motion, steering is represented by a wheelbase-independent normalized curvature proxy:

```text
curvature            = wz / |vx|
normalized_steering  = curvature * min_turning_radius
```

A normalized magnitude near `1.0` is approximately the maximum curvature permitted by the configured MPPI Ackermann motion model. This lets the critic express "avoid snapping toward full lock unless another critic makes that maneuver worthwhile" without requiring steering-rack geometry.

All terms use deadbands. Normal driving inside the comfort envelope contributes zero cost; the critic only starts influencing the optimizer after a configured threshold is exceeded.

## Build

From the Nav2++ repository root:

```bash
mkdir -p workspace/src
ln -sfn "$(pwd)/extensions/nav2_chauffeur_critic" workspace/src/nav2_chauffeur_critic

source /opt/ros/${ROS_DISTRO}/setup.bash
rosdep install --from-paths workspace/src --ignore-src -r -y
colcon build --base-paths workspace/src --packages-select nav2_chauffeur_critic
source install/setup.bash
```

The package exports its plugin description against `nav2_mppi_controller`, so an unmodified MPPI controller can discover it through pluginlib.

## Configure MPPI

Append `ChauffeurCritic` to the controller's existing critic list:

```yaml
FollowPath:
  motion_model: Ackermann
  critics: [..., ChauffeurCritic]

  ChauffeurCritic:
    enabled: true
    cost_power: 1
    cost_weight: 1.0

    longitudinal_jerk_weight: 1.0
    lateral_accel_weight: 0.35
    lateral_jerk_weight: 1.0
    steering_effort_weight: 0.10
    steering_rate_weight: 0.80
    steering_accel_weight: 0.35

    longitudinal_jerk_deadband: 0.75
    lateral_accel_deadband: 1.50
    lateral_jerk_deadband: 1.00
    steering_effort_deadband: 0.75
    steering_rate_deadband: 0.80
    steering_accel_deadband: 2.00
    min_steering_speed: 0.25
```

A complete mergeable example is in [`config/chauffeur_mppi.yaml`](config/chauffeur_mppi.yaml).

The numerical defaults are simulation starting points, not certified passenger-comfort or vehicle-safety limits. Tune against measured vehicle response, steering actuator limits, terrain, tire behavior, and the controller period.

## Recommended tuning order

1. Keep collision, constraint, and path-following behavior unchanged.
2. Set `cost_weight` low enough that the vehicle still completes tight valid maneuvers.
3. Tune `lateral_accel_deadband` to make MPPI slow before uncomfortable bends.
4. Tune `steering_rate_deadband` and `steering_accel_deadband` to remove wheel-snapping behavior.
5. Increase `steering_effort_weight` only if MPPI unnecessarily uses near-full-lock trajectories.
6. Tune longitudinal jerk last, after the platform's acceleration limits are correct.

If the robot refuses legitimate tight turns, reduce `steering_effort_weight` before relaxing safety or obstacle critics.

## Toy demos

These use the same equations as the plugin and require only Python 3:

```bash
cd extensions/nav2_chauffeur_critic/demos
python3 run_all.py
```

The demos verify three expected preferences:

- a ramped steering maneuver scores lower than a step input;
- a smooth S-curve scores lower than an abrupt left/right transition;
- on the same geometric bend, a slower trajectory scores lower than an unnecessarily fast one.

Each demo fails with a non-zero exit code if the ranking reverses.

## Relationship to existing Nav2 behavior

This is deliberately a critic, not a command filter. MPPI sees the comfort cost while it is choosing the trajectory, so it can trade path geometry and velocity together. A downstream velocity smoother can still be used as a final actuator-facing guard, but it cannot make the same look-ahead decision to slow before a future bend.
