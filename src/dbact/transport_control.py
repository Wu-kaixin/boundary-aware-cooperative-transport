"""D3/D4 - the transport outer loop: feedback on the task, not on position.

Why the previous law could not work
-----------------------------------
The A-branch nominal input was

    u = kp_cage (c_i - p_i)  +  kp_transport (-n.u_goal) (-n)

-- a coverage term that pulls the robot out to the cage ring and a constant
transport term that presses it in. Both are functions of *position*. They balance
at some penetration ``delta*``, the team applies the constant force
``k_p delta*``, and if that force is below the Coulomb breakaway ``mu_g m g`` the
cargo does not move. Nothing in that loop is a function of whether the cargo is
moving, so the equilibrium cannot notice that the task is failing: measured,
``J`` was flat at 0.0561 m from frame 97 to frame 500, and raising the gain moved
``delta*`` without removing the equilibrium.

The loop this module closes
---------------------------
    v_par  = v_obj_hat . d_goal        speed along the task direction
    e_v    = v_ref - v_par             tracking error
    s     += dt e_v                    integral state, per robot
    effort = clip(kp e_v + ki s, 0, effort_max)
    u_bias = a_i effort (-n_hat_i),    a_i = clip(-n_hat_i . d_goal, 0, 1)

The integral is the part that matters. While the cargo is stuck, ``e_v`` sits at
``v_ref`` and ``s`` grows, so the press deepens every step until either the cargo
breaks loose -- at which point ``e_v`` collapses and the integral stops growing --
or the safety filter refuses to go deeper. Integral action against a static
friction dead zone is the textbook remedy for exactly this equilibrium; it is used
here because it is standard, not because it is new.

Two things are deliberately *not* done. The bias is not applied along ``d_goal``:
that direction is inward only at the centre of the trailing face and tangential
everywhere else, so it slides robots off the arc they were holding. Each robot
presses along its own locally observed ``-n_hat``, and scaling by ``a_i`` makes
the press strongest where it helps most. And ``v_obj_hat`` is the robot's own
map-registration estimate, not a simulator velocity, so the loop is closed on
something the robot can actually measure.

Saturation and wind-up
----------------------
``effort_max`` is the robot's speed limit: the safety filter caps ``||u||`` there,
so a nominal press larger than that is discarded by the QP and would only inflate
the integral. The integral bound follows from it rather than from tuning,

    s_max = (effort_max - kp v_ref) / ki ,

the state at which the loop already commands everything the actuator can deliver.
On top of that the integral is frozen whenever the safety filter is actively
pushing the robot back along its press direction -- the robot is at the barrier,
not short of effort -- and back-calculation retracts ``s`` to the value that
reproduces the clipped output, so the loop leaves saturation immediately when the
cargo moves instead of unwinding for the seconds it took to accumulate.

Braking
-------
``v_ref`` is not constant. With ``r`` the remaining distance from the robot's own
integrated registration estimate,

    v_ref = clip(brake_gain * r, 0, reference_speed) .

The reference falls linearly inside ``reference_speed / brake_gain`` of the
target, the error changes sign if the cargo is still running, the effort is
released, and the Coulomb floor absorbs the rest. Past the target the loop latches
into hold: reference zero, integral zero, no press. The cage stays; the pushing
stops. Nothing here reads the true cargo pose, so the stopping condition is a
property of the team's own estimate, and the overshoot that estimate costs is
reported rather than hidden.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class TransportControlParams:
    reference_speed: float = 0.055
    kp: float = 2.4
    ki: float = 3.0
    effort_max: float = 0.30
    deadband_fraction: float = 0.05
    brake_gain: float = 0.55
    convoy_gain: float = 1.0
    convoy_max: float = 0.12

    @property
    def integral_max(self) -> float:
        """``s_max`` from the actuator bound, not from tuning. See module docstring."""
        if self.ki <= 0.0:
            return 0.0
        return max(0.0, (self.effort_max - self.kp * self.reference_speed) / self.ki)


@dataclass
class TransportEffort:
    effort: float
    reference_speed: float
    parallel_speed: float
    error: float
    integral: float
    remaining: float
    holding: bool
    saturated: bool


class DirectionalProgressController:
    """One robot's copy of the outer loop. State is the integral and the latch."""

    def __init__(self, params: TransportControlParams):
        self.params = params
        self.integral = 0.0
        self.holding = False

    def reset(self) -> None:
        self.integral = 0.0
        self.holding = False

    def reference_for(self, remaining: float) -> float:
        p = self.params
        if self.holding:
            return 0.0
        return float(np.clip(p.brake_gain * remaining, 0.0, p.reference_speed))

    def update(
        self,
        direction: np.ndarray,
        object_velocity: np.ndarray,
        progress: float,
        target_distance: float,
        dt: float,
        blocked: bool = False,
        active: bool = True,
    ) -> TransportEffort:
        p = self.params
        d = np.asarray(direction, dtype=float).reshape(2)
        v_par = float(np.dot(np.asarray(object_velocity, dtype=float).reshape(2), d))
        remaining = float(target_distance) - float(progress)

        if remaining <= 0.0:
            self.holding = True
        if not active:
            self.integral = 0.0
            return TransportEffort(0.0, 0.0, v_par, 0.0, 0.0, remaining, self.holding, False)
        if self.holding:
            self.integral = 0.0
            return TransportEffort(0.0, 0.0, v_par, -v_par, 0.0, remaining, True, False)

        v_ref = self.reference_for(remaining)
        error = v_ref - v_par

        # Conditional integration: do not accumulate while the safety filter is
        # already pushing back along the press direction, and do not accumulate on
        # noise inside the dead band.
        if not blocked and abs(error) > p.deadband_fraction * max(p.reference_speed, 1e-9):
            self.integral = float(np.clip(self.integral + dt * error, 0.0, p.integral_max))

        raw = p.kp * error + p.ki * self.integral
        effort = float(np.clip(raw, 0.0, p.effort_max))
        saturated = raw > p.effort_max or raw < 0.0
        if p.ki > 0.0 and raw > p.effort_max:
            # Back-calculation, upper clip only: retract the integral to the value
            # that reproduces the clipped output, so leaving saturation costs one
            # step rather than the seconds the integral took to build.
            #
            # Deliberately *not* applied at the lower clip. There the same formula
            # solves ``0 = kp e + ki s`` for a large negative ``e``, which asks for
            # a large positive ``s`` -- so a cargo that suddenly ran away would
            # have re-armed the integral to its bound at the exact moment the loop
            # wanted to let go. At the lower clip ordinary integration is already
            # the right behaviour: the error is negative, so the state falls.
            self.integral = float(np.clip((effort - p.kp * error) / p.ki, 0.0, p.integral_max))

        return TransportEffort(
            effort=effort,
            reference_speed=v_ref,
            parallel_speed=v_par,
            error=error,
            integral=self.integral,
            remaining=remaining,
            holding=False,
            saturated=saturated,
        )

    def convoy_velocity(self, object_velocity: np.ndarray) -> np.ndarray:
        """Station-keeping feed-forward: travel with the cargo, not after it.

        Without it the pushing arc advances and the rest of the ring does not, so
        the enclosure tears open behind the object; the coverage law then spends
        the next steps closing it again, and the team oscillates between pressing
        and re-forming instead of transporting. The command is the robot's own
        estimate of how fast the boundary it is holding is moving, which is the
        velocity that keeps its cage target stationary in the object frame -- a
        feed-forward term with a derivation, not a tuned nudge.
        """
        v = self.params.convoy_gain * np.asarray(object_velocity, dtype=float).reshape(2)
        speed = float(np.linalg.norm(v))
        if speed > self.params.convoy_max:
            v = v * (self.params.convoy_max / speed)
        return v


__all__ = ["TransportControlParams", "TransportEffort", "DirectionalProgressController"]
