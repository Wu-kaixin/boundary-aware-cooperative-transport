"""Task-progress feedback and anti-windup pressure regulation.

This module is deliberately independent of cargo truth.  Its inputs are the
locally estimated signed task progress and object velocity; its outputs are a
convoy velocity reference and a signed contact-effort command.  The latter is
translated into robot boundary-normal velocities by :mod:`dbact.controller`.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ProgressPIParams:
    target: float
    progress_kp: float = 0.8
    max_reference_speed: float = 0.18
    position_effort_gain: float = 0.25
    velocity_kp: float = 0.9
    velocity_ki: float = 0.35
    pressure_bias: float = 0.025
    effort_limit: float = 0.20
    integral_limit: float = 0.50
    anti_windup_gain: float = 1.0
    brake_position_gain: float = 0.15

    def assert_valid(self) -> None:
        if self.target <= 0.0:
            raise ValueError("progress target must be positive")
        for name in (
            "progress_kp",
            "max_reference_speed",
            "position_effort_gain",
            "velocity_kp",
            "velocity_ki",
            "effort_limit",
            "integral_limit",
            "anti_windup_gain",
            "brake_position_gain",
        ):
            if float(getattr(self, name)) < 0.0:
                raise ValueError(f"{name} cannot be negative")
        if self.max_reference_speed <= 0.0 or self.effort_limit <= 0.0:
            raise ValueError("reference speed and effort limits must be positive")


@dataclass(frozen=True)
class ProgressPIOutput:
    progress: float
    parallel_velocity: float
    position_error: float
    velocity_reference: float
    velocity_error: float
    effort: float
    integral: float
    saturated: bool
    braking: bool


class ProgressPIController:
    """PI contact-effort regulator with saturation and back-calculation."""

    def __init__(self, params: ProgressPIParams):
        params.assert_valid()
        self.params = params
        self.integral = 0.0
        self.last_output = ProgressPIOutput(
            progress=0.0,
            parallel_velocity=0.0,
            position_error=params.target,
            velocity_reference=0.0,
            velocity_error=0.0,
            effort=0.0,
            integral=0.0,
            saturated=False,
            braking=False,
        )

    def reset(self) -> None:
        self.integral = 0.0

    def update(
        self,
        progress: float,
        parallel_velocity: float,
        dt: float,
        *,
        braking: bool = False,
    ) -> ProgressPIOutput:
        p = self.params
        step = max(float(dt), 0.0)
        e_j = float(p.target - progress)
        v_ref = 0.0 if braking else float(
            np.clip(p.progress_kp * e_j, -p.max_reference_speed, p.max_reference_speed)
        )
        e_v = float(v_ref - parallel_velocity)

        candidate_integral = float(
            np.clip(self.integral + e_v * step, -p.integral_limit, p.integral_limit)
        )
        position_gain = p.brake_position_gain if braking else p.position_effort_gain
        bias = 0.0
        if not braking and abs(e_j) > 1e-12:
            bias = float(np.copysign(p.pressure_bias, e_j))
        raw = position_gain * e_j + p.velocity_kp * e_v + p.velocity_ki * candidate_integral + bias
        effort = float(np.clip(raw, -p.effort_limit, p.effort_limit))
        saturated = not bool(np.isclose(raw, effort, rtol=0.0, atol=1e-12))

        # Back-calculation drains the integrator whenever saturation prevents the
        # commanded pressure from following the unsaturated PI output.
        if saturated and p.velocity_ki > 1e-12:
            candidate_integral += p.anti_windup_gain * (effort - raw) * step / p.velocity_ki
            candidate_integral = float(
                np.clip(candidate_integral, -p.integral_limit, p.integral_limit)
            )
        self.integral = candidate_integral
        self.last_output = ProgressPIOutput(
            progress=float(progress),
            parallel_velocity=float(parallel_velocity),
            position_error=e_j,
            velocity_reference=v_ref,
            velocity_error=e_v,
            effort=effort,
            integral=self.integral,
            saturated=saturated,
            braking=bool(braking),
        )
        return self.last_output


__all__ = ["ProgressPIParams", "ProgressPIOutput", "ProgressPIController"]
