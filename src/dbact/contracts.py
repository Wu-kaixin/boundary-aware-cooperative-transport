"""Cross-layer numerical contracts.

A contract here is not a note in a design document. It is a numerical relation
between quantities owned by *different* layers, asserted at construction time or
at run time, whose violation fails the experiment instead of silently degrading
it.

The three contracts are motivated by a specific failure: every module passed its
own unit test, yet the closed loop produced a net cargo displacement pointing
away from the goal. That class of failure cannot originate inside a single
module -- it comes from relations between modules that nobody wrote down.

C1  contact/safety   the cage radius must land inside the contact band that the
                     object-boundary CBF still admits
C2  solver provenance a requested QP backend must never silently degrade to the
                     projection fallback
C3  success criterion directional progress, not displacement magnitude, decides
                     whether a transport run succeeded
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


class ContractViolation(RuntimeError):
    """Raised when a cross-layer numerical contract does not hold."""


# --------------------------------------------------------------------------- #
# C1 - contact/safety contract
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class ContactSafetyContract:
    """Ties the cage radius to the robot radius and the CBF safety distance.

    Under a penalty contact model the normal force a robot applies to the cargo
    is ``f = k_p * delta`` with penetration ``delta = r_robot - s``, where ``s``
    is the signed distance from the robot centre to the cargo boundary. The
    object-boundary CBF enforces ``s >= r_safe``, hence
    ``delta <= r_robot - r_safe = delta_max``.

    Two failure modes follow directly:

    * ``d_c >= r_robot``  the CVT converges to ``s ~ d_c >= r_robot``, so
      ``delta <= 0`` and the contact force is identically zero. The team can
      never move the cargo, yet every safety metric reports a perfect run --
      this is the dangerous case, because an acceptance script that only checks
      coverage and clearance will grade "never touches the cargo" as excellent.
    * ``d_c <= r_safe``  the cage target sits inside the region the CBF forbids,
      so the QP and the coverage law fight each other indefinitely: chattering
      or deadlock.

    The admissible band is therefore ``r_safe < d_c < r_robot``. The second
    inequality ``gamma_obj * (d_c - r_safe) > rho`` additionally requires the
    class-K term to dominate the ISSf robustness margin at the cage radius,
    without which a robot is pushed back out of the contact band before it ever
    reaches it.
    """

    robot_radius: float
    cage_offset: float
    delta_max: float
    gamma_obj: float
    rho: float
    boundary_error_bound: float = 0.0
    d_min: float | None = None
    lead_offset: float | None = None

    @property
    def r_safe(self) -> float:
        """Signed distance the object-boundary CBF keeps the robot centre at."""
        return self.robot_radius - self.delta_max

    @property
    def contact_band(self) -> tuple[float, float]:
        return (self.r_safe, self.robot_radius)

    @property
    def barrier_margin(self) -> float:
        """``gamma_obj * (d_c - r_safe) - rho``; must be strictly positive."""
        return (
            self.gamma_obj
            * (self.cage_offset - self.r_safe - self.boundary_error_bound)
            - self.rho
        )

    def violations(self) -> list[str]:
        out: list[str] = []
        if not self.robot_radius > 0.0:
            out.append(f"robot_radius must be positive, got {self.robot_radius:.4f}")
        if not 0.0 < self.delta_max < self.robot_radius:
            out.append(
                f"delta_max must lie in (0, robot_radius); got delta_max={self.delta_max:.4f}, "
                f"robot_radius={self.robot_radius:.4f}"
            )
        if not self.cage_offset > self.r_safe:
            out.append(
                f"C1 lower bound violated: cage_offset={self.cage_offset:.4f} <= r_safe={self.r_safe:.4f}; "
                "the cage ring sits inside the CBF-forbidden region, expect chattering or deadlock"
            )
        if not self.cage_offset < self.robot_radius:
            out.append(
                f"C1 upper bound violated: cage_offset={self.cage_offset:.4f} >= robot_radius="
                f"{self.robot_radius:.4f}; penetration is non-positive at the cage ring so the contact "
                "force is identically zero and the cargo can never move"
            )
        if self.lead_offset is not None and self.lead_offset <= self.robot_radius:
            out.append(
                f"lead_offset={self.lead_offset:.4f} <= robot_radius={self.robot_radius:.4f}; the leading arc "
                "would be in contact and would resist the very motion the pushing arc is producing, so net "
                "progress is structurally impossible regardless of gains"
            )
        if self.d_min is not None and self.d_min < 2.0 * self.robot_radius:
            out.append(
                f"inter-robot d_min={self.d_min:.4f} < 2*robot_radius={2.0 * self.robot_radius:.4f}; "
                "robots modelled as discs would overlap while the inter-robot barrier reports safety"
            )
        if self.rho < 0.0:
            out.append(f"rho must be non-negative, got {self.rho:.4f}")
        if self.boundary_error_bound < 0.0:
            out.append(
                f"boundary_error_bound must be non-negative, got {self.boundary_error_bound:.4f}"
            )
        if self.cage_offset <= self.r_safe + self.boundary_error_bound:
            out.append(
                "C1 perception-robust lower bound violated: cage_offset must exceed "
                "r_safe + boundary_error_bound"
            )
        if self.gamma_obj <= 0.0:
            out.append(f"gamma_obj must be positive, got {self.gamma_obj:.4f}")
        elif self.barrier_margin <= 0.0:
            out.append(
                f"C1 barrier margin violated: gamma_obj*(d_c - r_safe - epsilon_b) = "
                f"{self.gamma_obj * (self.cage_offset - self.r_safe - self.boundary_error_bound):.4f} "
                f"<= rho = {self.rho:.4f}; "
                "robots are repelled before reaching the cage ring"
            )
        return out

    def assert_valid(self) -> None:
        problems = self.violations()
        if problems:
            raise ContractViolation("C1 contact/safety contract violated:\n  - " + "\n  - ".join(problems))

    def as_dict(self) -> dict:
        return {
            "robot_radius": self.robot_radius,
            "cage_offset": self.cage_offset,
            "delta_max": self.delta_max,
            "gamma_obj": self.gamma_obj,
            "rho": self.rho,
            "boundary_error_bound": self.boundary_error_bound,
            "d_min": self.d_min,
            "lead_offset": self.lead_offset,
            "r_safe": self.r_safe,
            "effective_r_safe": self.r_safe + self.boundary_error_bound,
            "barrier_margin": self.barrier_margin,
        }


# --------------------------------------------------------------------------- #
# C2 - solver provenance contract
# --------------------------------------------------------------------------- #

VALID_BACKENDS = ("qp", "cvxpy", "projection")
EXACT_BACKENDS = ("qp", "cvxpy")


@dataclass(frozen=True)
class SolverContract:
    """Fail-closed backend selection for the safety filter.

    There is deliberately no ``auto`` backend. Under ``auto`` a missing solver
    turns into a silent projection fallback, and every downstream claim of
    "hard QP" becomes unsupported without anything in the logs saying so.

    ``qp``          exact analytic planar QP solver, no external dependency
    ``cvxpy``       the same problem through cvxpy; used to cross-check ``qp``
    ``projection``  iterated half-plane projection; inexact, must be asked for
    """

    backend: str

    def __post_init__(self) -> None:
        if self.backend not in VALID_BACKENDS:
            raise ContractViolation(
                f"C2 solver contract: backend must be one of {VALID_BACKENDS!r}, got {self.backend!r}. "
                "There is no 'auto' backend -- a missing solver must fail, not degrade silently."
            )

    @property
    def requires_solver(self) -> bool:
        return self.backend in EXACT_BACKENDS

    def on_solver_failure(self, detail: str) -> None:
        """Called by the safety filter when a QP solve does not return a point."""
        if self.requires_solver:
            raise ContractViolation(
                f"C2 solver contract: backend={self.backend!r} was requested but the solve failed "
                f"({detail}). Install cvxpy/osqp or set backend='projection' explicitly in the config."
            )


# --------------------------------------------------------------------------- #
# C3 - success criterion contract
# --------------------------------------------------------------------------- #


@dataclass
class SuccessVerdict:
    """Outcome of the C3 evaluation.

    ``reasons`` is empty exactly when ``success`` is true. Failures are recorded
    as strings rather than a bare boolean so that a summary file says *why* a run
    failed without anyone having to re-run it.
    """

    success: bool
    reasons: list[str] = field(default_factory=list)
    metrics: dict = field(default_factory=dict)

    def as_dict(self) -> dict:
        return {"success": self.success, "reasons": list(self.reasons), "metrics": dict(self.metrics)}


@dataclass(frozen=True)
class DirectionalProgressContract:
    """Directional progress replaces displacement magnitude in the success test.

    ``J = (x_T - x_0) . u_goal`` is a signed scalar projection, so a run that ends
    up behind where it started scores negative and cannot pass. The efficiency
    ratio ``J / ||dx||`` suppresses large lateral drift, but is only meaningful
    once the cargo has actually moved, hence the ``displacement_gate``: at small
    displacement the direction is dominated by transient jitter and is close to
    undefined.

    Safety is part of the success test, not a separate report. A run that reaches
    the goal by driving robots through the cargo has not succeeded.
    """

    j_min: float = 0.15
    j_max: float = float("inf")
    efficiency_min: float = 0.7
    displacement_gate: float = 0.1
    coverage_min: float = 0.0
    max_rotation_deg: float = float("inf")
    discrete_overshoot: float = 0.0

    def evaluate(
        self,
        start: np.ndarray,
        end: np.ndarray,
        goal_direction: np.ndarray,
        *,
        min_signed_clearance: float | None = None,
        max_penetration: float | None = None,
        delta_max: float | None = None,
        solver_fallbacks: int | None = None,
        min_inter_agent_distance: float | None = None,
        d_min: float | None = None,
        final_strict_coverage: float | None = None,
        rotation_deg: float | None = None,
    ) -> SuccessVerdict:
        x0 = np.asarray(start, dtype=float).reshape(2)
        xt = np.asarray(end, dtype=float).reshape(2)
        u = np.asarray(goal_direction, dtype=float).reshape(2)
        norm_u = float(np.linalg.norm(u))
        if norm_u < 1e-9:
            return SuccessVerdict(False, ["C3: goal direction is degenerate (zero vector)"])
        u = u / norm_u

        dx = xt - x0
        displacement = float(np.linalg.norm(dx))
        j = float(np.dot(dx, u))
        efficiency = j / displacement if displacement > 1e-12 else 0.0

        reasons: list[str] = []
        if j < self.j_min:
            reasons.append(f"C3: directional progress J={j:.4f} m < J_min={self.j_min:.4f} m")
        if j > self.j_max:
            reasons.append(f"C3: directional progress J={j:.4f} m > J_max={self.j_max:.4f} m")
        if displacement >= self.displacement_gate and efficiency < self.efficiency_min:
            reasons.append(
                f"C3: progress efficiency J/||dx||={efficiency:.4f} < {self.efficiency_min:.2f} "
                f"(||dx||={displacement:.4f} m)"
            )
        if min_signed_clearance is not None and min_signed_clearance < 0.0:
            reasons.append(f"C3: min signed clearance {min_signed_clearance:.4f} m < 0 (robot entered the cargo)")
        if max_penetration is not None and delta_max is not None:
            # The barrier condition holds in continuous time; a fixed-step
            # integrator can overshoot the boundary of the safe set by at most one
            # step of relative motion. That bound is stated rather than absorbed
            # into delta_max, so the invariant stays exactly "penetration never
            # exceeds the budget the CBF leaves open, up to the discretisation".
            budget = delta_max + self.discrete_overshoot
            if max_penetration > budget:
                reasons.append(
                    f"C3: max penetration {max_penetration:.4f} m > delta_max {delta_max:.4f} m "
                    f"+ discrete overshoot {self.discrete_overshoot:.4f} m"
                )
        if solver_fallbacks:
            reasons.append(f"C3: {solver_fallbacks} solver fallback(s) occurred (violates C2)")
        if min_inter_agent_distance is not None and d_min is not None and min_inter_agent_distance < d_min:
            reasons.append(
                f"C3: min inter-agent distance {min_inter_agent_distance:.4f} m < d_min {d_min:.4f} m"
            )
        if final_strict_coverage is not None and final_strict_coverage < self.coverage_min:
            reasons.append(
                f"C3: final strict coverage {final_strict_coverage:.4f} < {self.coverage_min:.4f}"
            )
        if rotation_deg is not None and abs(rotation_deg) > self.max_rotation_deg:
            reasons.append(
                f"C3: |cargo rotation|={abs(rotation_deg):.4f} deg > {self.max_rotation_deg:.4f} deg"
            )

        metrics = {
            "directional_progress_J": j,
            "displacement": displacement,
            "progress_efficiency": efficiency,
            "goal_direction": u.tolist(),
            "final_strict_coverage": final_strict_coverage,
            "rotation_deg": rotation_deg,
        }
        return SuccessVerdict(success=not reasons, reasons=reasons, metrics=metrics)


# --------------------------------------------------------------------------- #
# coverage-layer contract
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class CoverageContract:
    """``R_l <= R_comm / 2`` makes the decentralised Voronoi cell exact.

    For any ``q`` in ``B(p_i, R_l)`` that is closer to ``p_j`` than to ``p_i`` we
    get ``||p_i - p_j|| <= ||p_i - q|| + ||q - p_j|| <= 2 R_l <= R_comm``, so
    ``j`` is already a communication neighbour. The cell computed from neighbours
    alone therefore *equals* the true Voronoi cell restricted to the disk; it is
    not an approximation. Outside this regime the equality silently fails, which
    is why it is asserted rather than assumed.
    """

    local_radius: float
    comm_range: float

    def violations(self) -> list[str]:
        if self.local_radius > 0.5 * self.comm_range + 1e-12:
            return [
                f"neighbour completeness violated: R_l={self.local_radius:.4f} > R_comm/2="
                f"{0.5 * self.comm_range:.4f}; the local Voronoi cell is no longer exact"
            ]
        return []

    def assert_valid(self) -> None:
        problems = self.violations()
        if problems:
            raise ContractViolation("Coverage contract violated:\n  - " + "\n  - ".join(problems))


__all__ = [
    "ContractViolation",
    "ContactSafetyContract",
    "SolverContract",
    "DirectionalProgressContract",
    "SuccessVerdict",
    "CoverageContract",
    "VALID_BACKENDS",
    "EXACT_BACKENDS",
]
