#!/usr/bin/env python3
"""Simple lateral head/neck dynamic proxy for controller experiments.

This is a deliberately low-order second-order model:
    I*theta_ddot + c*theta_dot + k*theta = -m*h*a_y

It is useful for comparing candidate vehicle trajectories. It is not a
whiplash injury model and must not be interpreted as an injury threshold.
"""

from dataclasses import dataclass
from math import pi, sqrt
from typing import Sequence


@dataclass(frozen=True)
class NeckModel:
    # Illustrative defaults; expose them so experiments can sweep uncertainty.
    head_mass_kg: float = 4.5
    com_lever_arm_m: float = 0.10
    rotational_inertia_kg_m2: float = 0.025
    natural_frequency_hz: float = 2.0
    damping_ratio: float = 0.35

    @property
    def omega_n(self) -> float:
        return 2.0 * pi * self.natural_frequency_hz

    @property
    def stiffness_nm_per_rad(self) -> float:
        return self.rotational_inertia_kg_m2 * self.omega_n**2

    @property
    def damping_nms_per_rad(self) -> float:
        return (
            2.0
            * self.damping_ratio
            * self.rotational_inertia_kg_m2
            * self.omega_n
        )


def simulate_head_response(
    lateral_accel_mps2: Sequence[float],
    dt: float,
    model: NeckModel = NeckModel(),
) -> list[dict[str, float]]:
    """Integrate the second-order model with RK4 and piecewise-constant forcing."""
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    theta = 0.0
    omega = 0.0
    out: list[dict[str, float]] = []

    def alpha(th: float, om: float, ay: float) -> float:
        torque = -model.head_mass_kg * model.com_lever_arm_m * ay
        return (
            torque
            - model.damping_nms_per_rad * om
            - model.stiffness_nm_per_rad * th
        ) / model.rotational_inertia_kg_m2

    for ay in lateral_accel_mps2:
        def deriv(th: float, om: float) -> tuple[float, float]:
            return om, alpha(th, om, ay)

        k1_t, k1_w = deriv(theta, omega)
        k2_t, k2_w = deriv(
            theta + 0.5 * dt * k1_t,
            omega + 0.5 * dt * k1_w,
        )
        k3_t, k3_w = deriv(
            theta + 0.5 * dt * k2_t,
            omega + 0.5 * dt * k2_w,
        )
        k4_t, k4_w = deriv(
            theta + dt * k3_t,
            omega + dt * k3_w,
        )

        theta += dt * (k1_t + 2*k2_t + 2*k3_t + k4_t) / 6.0
        omega += dt * (k1_w + 2*k2_w + 2*k3_w + k4_w) / 6.0
        angular_accel = alpha(theta, omega, ay)

        out.append({
            "theta_rad": theta,
            "omega_rad_s": omega,
            "alpha_rad_s2": angular_accel,
            "lateral_accel_mps2": ay,
        })

    return out


def response_metrics(response: Sequence[dict[str, float]]) -> dict[str, float]:
    if not response:
        return {
            "peak_angle_deg": 0.0,
            "rms_angle_deg": 0.0,
            "peak_angular_velocity_deg_s": 0.0,
            "peak_angular_accel_deg_s2": 0.0,
        }

    rad_to_deg = 180.0 / pi
    angles = [abs(s["theta_rad"]) * rad_to_deg for s in response]
    velocities = [abs(s["omega_rad_s"]) * rad_to_deg for s in response]
    accelerations = [abs(s["alpha_rad_s2"]) * rad_to_deg for s in response]

    return {
        "peak_angle_deg": max(angles),
        "rms_angle_deg": sqrt(sum(x*x for x in angles) / len(angles)),
        "peak_angular_velocity_deg_s": max(velocities),
        "peak_angular_accel_deg_s2": max(accelerations),
    }
