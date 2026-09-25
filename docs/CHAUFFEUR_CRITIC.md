# ChauffeurCritic design note

## Goal

Add a stackable Nav2 MPPI critic whose only job is to prefer trajectories that feel deliberate rather than abrupt on a passenger-carrying Ackermann vehicle.

The critic is intentionally subordinate to collision avoidance and feasibility. It should answer:

> Among trajectories that already accomplish the navigation task, which one drives most like a careful chauffeur?

## Cost model

For a sampled trajectory with longitudinal velocity `v` and yaw rate `w`:

```text
longitudinal acceleration  ax = dv/dt
longitudinal jerk          jx = d(ax)/dt
lateral acceleration       ay = v*w
lateral jerk               jy = d(ay)/dt
curvature                  k  = w/|v|
normalized steering        s  = k*Rmin
```

`Rmin` is the Ackermann motion model's configured minimum turning radius. Therefore `|s| ~= 1` means approximately full-lock curvature.

The total cost is the weighted integral of squared deadband exceedance:

```text
rho(x, b) = max(0, |x| - b)^2

J = W * integral(
      w_jx   rho(jx,   b_jx)
    + w_ay   rho(ay,   b_ay)
    + w_jy   rho(jy,   b_jy)
    + w_s    rho(s,    b_s)
    + w_ds   rho(ds/dt,b_ds)
    + w_dds  rho(d2s/dt2,b_dds)
) dt
```

The deadband is important. Ordinary steering and acceleration are not continuously punished; only behavior outside the chosen comfort envelope starts competing with other MPPI objectives.

## Why normalized curvature instead of steering-wheel angle

Nav2 MPPI exposes `vx` and `wz`, not steering-rack state. Using `Rmin*wz/|vx|` gives a vehicle-independent proxy for how much of the allowed steering envelope the rollout is consuming. It does not require wheelbase or steering-ratio calibration, yet still distinguishes a mild arc from a trajectory that jumps toward full lock.

A future vehicle-specific extension can replace the proxy with measured or modeled road-wheel angle if the platform exposes that state.

## Failure modes to watch

- **Overweight comfort:** the controller may avoid required tight maneuvers or progress too slowly.
- **Very low speed:** curvature from `w/v` is numerically unstable. The implementation floors the speed denominator with `min_steering_speed`.
- **Loose acceleration constraints:** the critic can prefer smooth samples, but platform feasibility should still be enforced by the motion model and constraint critic.
- **Short prediction horizon:** MPPI cannot slow gracefully for a bend it cannot yet see. Comfort tuning and horizon length are coupled.
- **Terrain transients:** off-road body motion is not captured by planar `vx/wz`. IMU-derived roll/pitch acceleration could become a later critic term.

## Next vehicle-level validation

For the Jeep, log at minimum:

- commanded and measured `vx`, `wz`;
- steering actuator angle or CAN steering signal if available;
- IMU longitudinal/lateral acceleration;
- controller-cycle timestamps;
- selected MPPI trajectory and per-critic cost statistics.

Then tune to measured response rather than assuming the default deadbands represent the real platform.
