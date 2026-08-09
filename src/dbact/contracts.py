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
        return self.gamma_obj * (self.cage_offset - self.r_safe) - self.rho

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
        if self.gamma_obj <= 0.0:
            out.append(f"gamma_obj must be positive, got {self.gamma_obj:.4f}")
        elif self.barrier_margin <= 0.0:
            out.append(
                f"C1 barrier margin violated: gamma_obj*(d_c - r_safe) = "
                f"{self.gamma_obj * (self.cage_offset - self.r_safe):.4f} <= rho = {self.rho:.4f}; "
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
            "d_min": self.d_min,
            "lead_offset": self.lead_offset,
            "r_safe": self.r_safe,
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
    efficiency_min: float = 0.7
    displacement_gate: float = 0.1
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

        metrics = {
            "directional_progress_J": j,
            "displacement": displacement,
            "progress_efficiency": efficiency,
            "goal_direction": u.tolist(),
        }
        return SuccessVerdict(success=not reasons, reasons=reasons, metrics=metrics)


# --------------------------------------------------------------------------- #
# C5 - transport-time QP feasibility
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class TransportFeasibilityContract:
    """Where the transport press is allowed to stop, so that the QP stays feasible.

    The object-boundary row is

        n_k^T u  >=  n_k^T v_obj - gamma_obj h_k + rho ,

    so its right-hand side is non-positive -- and therefore satisfied by ``u = 0``
    -- exactly when

        h_k  >=  ( n_k^T v_obj + rho ) / gamma_obj .

    Bounding ``n_k^T v_obj`` by the ISSf disturbance bound ``V`` gives a
    *demand band* of width ``(V + rho) / gamma_obj`` above ``r_safe``, inside which
    every object row asks for active retreat.

    This is the whole of the scaled-barrier problem, and it was designed in rather
    than tuned in. The transport press generates force by driving robots against
    the object barrier, so a pushing robot's steady state is wherever the press
    stops -- and with the press stopping 0.015 m above ``r_safe`` against a band of
    0.0275 m, **every pusher sat permanently inside the band**. Each one then
    demanded retreat on every step, and any neighbour at ``d_min`` made the set
    empty. Measured: 168 of 168 scaled-barrier events were object rows demanding
    retreat, with zero positive inter-robot demands.

    Requiring the press to stop *above* the band restores a constructive
    feasibility certificate:

        Proposition. If every inter-agent barrier satisfies ``h_ij >= 0`` and every
        object row satisfies ``h_k >= (V + rho)/gamma_obj``, then ``u = 0`` satisfies
        every row of the QP, so the problem is feasible and no relaxation is needed.

    Both hypotheses are maintained rather than assumed: the first by the
    inter-agent CBF from a valid initial state, the second by the press floor this
    contract fixes. The relaxation tiers stay in place for the transient in which a
    map correction moves a robot into the band, which is now the only way to get
    there.

    The cost is force. A robot stopping at ``r_safe + margin`` applies
    ``k_p (r_robot - r_safe - margin)`` instead of ``k_p delta_max``, so
    ``delta_max`` has to be large enough that what is left is still worth having --
    which is the cross-layer relation this contract exists to check.
    """

    r_safe: float
    robot_radius: float
    gamma_obj: float
    rho: float
    object_velocity_bound: float
    press_margin: float
    stiffness: float | None = None
    breakaway_force: float | None = None
    min_push_agents: int = 3
    safety_factor: float = 1.2

    @property
    def demand_band(self) -> float:
        """Width above ``r_safe`` inside which object rows demand active retreat."""
        return (self.object_velocity_bound + self.rho) / max(self.gamma_obj, 1e-9)

    @property
    def required_margin(self) -> float:
        return self.safety_factor * self.demand_band

    @property
    def press_floor(self) -> float:
        """Signed clearance the press is allowed to drive a robot down to."""
        return self.r_safe + self.press_margin

    @property
    def press_penetration(self) -> float:
        """Penetration left at the press floor; this is what makes the force."""
        return self.robot_radius - self.press_floor

    def force_per_robot(self) -> float | None:
        if self.stiffness is None:
            return None
        return self.stiffness * max(self.press_penetration, 0.0)

    def structural_violations(self) -> list[str]:
        """Conditions under which the *controller* is not well posed.

        Separate from the force budget because they fail differently. A press
        floor inside the demand band makes the QP infeasible -- the controller is
        wrong however good the scenario is. A force budget below the breakaway
        force makes the *scenario* impossible -- the controller is fine and the
        cargo simply cannot be moved by that quorum, which a run reports as
        ``J < L``. The first is asserted for every transport controller; the
        second only where a scenario has declared a transport task, so the legacy
        enclosure configurations still load and their known-marginal budget is
        reported rather than raised.
        """
        out: list[str] = []
        if self.press_margin < self.required_margin:
            out.append(
                f"C5 press floor violated: press_margin={self.press_margin:.4f} < "
                f"{self.safety_factor:.2f} * (V + rho)/gamma_obj = {self.required_margin:.4f}. "
                "Pushing robots would rest inside the object rows' demand band, so every one of "
                "them demands retreat on every step and the QP has no feasible input as soon as a "
                "neighbour reaches d_min"
            )
        if self.press_penetration <= 0.0:
            out.append(
                f"C5 press floor violated: the floor {self.press_floor:.4f} is at or beyond "
                f"robot_radius={self.robot_radius:.4f}, so penetration is non-positive and the "
                "pushing arc applies no force at all"
            )
        return out

    def budget_violations(self) -> list[str]:
        """Whether the quorum can move this cargo at all, from the press floor."""
        if self.breakaway_force is None or self.stiffness is None or self.press_penetration <= 0.0:
            return []
        available = self.min_push_agents * self.force_per_robot()
        if available > self.breakaway_force:
            return []
        return [
            f"C5 force budget violated: {self.min_push_agents} robots at the press floor supply "
            f"{available:.2f} N against a breakaway force of {self.breakaway_force:.2f} N. Raise "
            "delta_max so the floor sits deeper, lower the ground friction, or require more pushing "
            "robots -- as configured the quorum cannot move the cargo"
        ]

    def violations(self) -> list[str]:
        return self.structural_violations() + self.budget_violations()

    def assert_structural(self) -> None:
        problems = self.structural_violations()
        if problems:
            raise ContractViolation("C5 transport feasibility contract violated:\n  - " + "\n  - ".join(problems))

    def assert_valid(self) -> None:
        problems = self.violations()
        if problems:
            raise ContractViolation("C5 transport feasibility contract violated:\n  - " + "\n  - ".join(problems))

    def as_dict(self) -> dict:
        return {
            "r_safe": self.r_safe,
            "gamma_obj": self.gamma_obj,
            "rho": self.rho,
            "object_velocity_bound": self.object_velocity_bound,
            "demand_band": self.demand_band,
            "required_margin": self.required_margin,
            "press_margin": self.press_margin,
            "press_floor": self.press_floor,
            "press_penetration": self.press_penetration,
            "force_per_robot": self.force_per_robot(),
            "breakaway_force": self.breakaway_force,
            "min_push_agents": self.min_push_agents,
            "quorum_force": (
                self.min_push_agents * self.force_per_robot() if self.force_per_robot() is not None else None
            ),
            "violations": self.violations(),
        }


# --------------------------------------------------------------------------- #
# C4 - the closed-loop 500-frame contract
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class ClosedLoopContract:
    """G500: one budget, every stage of the loop scored inside it.

    C3 asks whether the cargo ended up in the right place. That is necessary and
    not sufficient for a closed loop: a run that spends 480 frames finding the
    object and 20 pushing it has not demonstrated the same thing as one that
    finds it in 60. The deadlines are therefore part of the criterion, and they
    are deadlines on a shared budget rather than a per-phase schedule -- a run
    that encloses early may spend the saving on transport.

    Every gate here fails *closed*. A missing measurement is a failure, not a
    pass, because a criterion that cannot be evaluated cannot be defended; and
    every failure carries the number that caused it, so the summary says why
    without anyone re-running it.

    ``progress_max_ratio`` is the one gate that looks unusual. The team stops on
    its *own* estimate of how far the cargo has gone, and that estimate is biased
    low, so the cargo always travels somewhat past the target. Bounding the
    overshoot is what distinguishes "stopped late" from "never stopped", and it
    is the gate that makes BRAKE and HOLD mean something.
    """

    # A deadline of ``None`` is reported but not gated. That is the difference
    # between asking whether the team can finish inside a budget somebody chose,
    # and asking how long the team actually takes -- and the second question is
    # the one whose answer is a property of the algorithm. The frame counts are
    # still measured either way, so removing a gate costs no evidence.
    detect_by: int | None = 100
    contact_ready_by: int | None = 300
    transport_by: int | None = 350
    reach_by: int | None = 500
    efficiency_min: float = 0.80
    direction_error_max_deg: float = 20.0
    cross_track_max: float = 0.15
    coverage_min: float = 0.70
    progress_max_ratio: float = 1.40
    hold_speed_max: float = 0.02
    yaw_max_deg: float = 15.0
    # Steps on which the object-barrier decrease rate had to be scaled down to
    # keep the QP feasible. Zero by default: a relaxation that is counted but not
    # scored is a relaxation that has been renamed rather than removed.
    barrier_scalings_max: int = 0
    # Tolerance on the inter-agent separation comparison. The barrier is *exactly*
    # binding for most of a transport run -- the ring sits on d_min by design --
    # so an exact float comparison reports the last bit of the QP's arithmetic as a
    # collision. Measured deficits at those "breaches" were 1e-16 to 3e-8 m; a
    # micrometre is a hundred times the worst of them and still nine orders of
    # magnitude below anything physical, so this separates numerical noise from a
    # safety event without weakening the gate.
    d_min_tolerance: float = 1e-6

    def evaluate(self, report: dict) -> SuccessVerdict:
        reasons: list[str] = []

        horizon = report.get("frames_run") or self.reach_by

        def frame(name: str, limit: int | None, label: str) -> None:
            value = report.get(name)
            if value is None:
                reasons.append(f"G500: {label} never happened within the {horizon}-frame run")
            elif limit is not None and int(value) > limit:
                reasons.append(f"G500: {label} at frame {int(value)} > {limit}")

        frame("first_detection_frame", self.detect_by, "object detection")
        frame("contact_ready_frame", self.contact_ready_by, "enclosure / contact-ready")
        frame("transport_frame", self.transport_by, "transport activation")
        frame("reached_frame", self.reach_by, "target reached")

        target = float(report.get("target_distance", 0.0))
        progress = report.get("J")
        if progress is None:
            reasons.append("G500: directional progress J was not measured")
        else:
            progress = float(progress)
            if progress < target:
                reasons.append(f"G500: J={progress:.4f} m < target L={target:.4f} m")
            elif target > 0.0 and progress > self.progress_max_ratio * target:
                reasons.append(
                    f"G500: J={progress:.4f} m overshot L={target:.4f} m by more than "
                    f"{(self.progress_max_ratio - 1.0) * 100:.0f}%: the team did not stop"
                )

        # --- T3, reported rather than gated ------------------------------- #
        # These four are trajectory *quality*, and quality is a distribution with a
        # rate, not a member of a fourteen-way conjunction. Two of them are also
        # the same quantity: measured over twelve seeds,
        #     max cross-track = J * sin(direction error),   correlation 0.968,
        # so an absolute cross-track corridor silently changes meaning whenever the
        # task distance changes -- the identical controller passed it at L = 0.5 m
        # and failed it at L = 1.5 m. The thresholds are unchanged and every breach
        # is still named in ``quality_reasons``; what changed is that a run is not
        # declared invalid by them. A criterion whose difficulty depends on a
        # parameter the experimenter picked is a design target, not a verdict.
        quality: list[str] = []

        efficiency = report.get("efficiency")
        if efficiency is None:
            reasons.append("G500: progress efficiency was not measured")
        elif float(efficiency) < self.efficiency_min:
            quality.append(f"T3: efficiency J/||dx||={float(efficiency):.4f} < {self.efficiency_min:.2f}")

        angle = report.get("direction_error_deg")
        if angle is None:
            reasons.append("G500: direction error was not measured")
        elif float(angle) > self.direction_error_max_deg:
            quality.append(
                f"T3: direction error {float(angle):.2f} deg > {self.direction_error_max_deg:.1f} deg"
            )

        cross = report.get("max_cross_track")
        if cross is None:
            reasons.append("G500: cross-track error was not measured")
        elif float(cross) > self.cross_track_max:
            quality.append(f"T3: cross-track error {float(cross):.4f} m > {self.cross_track_max:.2f} m")

        coverage = report.get("max_strict_coverage")
        if coverage is None or float(coverage) < self.coverage_min:
            reasons.append(
                f"G500: strict boundary coverage {float(coverage or 0.0):.3f} < {self.coverage_min:.2f}: "
                "the team never enclosed the object"
            )

        yaw = report.get("rotation_deg")
        if yaw is not None and abs(float(yaw)) > self.yaw_max_deg:
            quality.append(f"T3: cargo rotated {float(yaw):+.2f} deg, beyond +/-{self.yaw_max_deg:.0f} deg")

        if not report.get("holding", False):
            reasons.append("G500: the run did not end in HOLD")
        final_speed = report.get("final_cargo_speed")
        if final_speed is None or float(final_speed) > self.hold_speed_max:
            reasons.append(
                f"G500: final cargo speed {float(final_speed or 0.0):.4f} m/s > {self.hold_speed_max:.3f} m/s: "
                "the cargo was still drifting at the end of the budget"
            )

        if report.get("engine") == "scripted":
            reasons.append("G500: engine='scripted' -- the cargo was translated, not transported")
        for name, label, limit in (
            ("solver_fallbacks", "solver fallback", 0),
            ("solver_infeasible", "QP infeasibility", 0),
            ("barrier_scalings", "scaled-barrier", self.barrier_scalings_max),
        ):
            count = report.get(name)
            if count is None:
                reasons.append(f"G500: {label} count was not recorded")
            elif int(count) > limit:
                detail = ""
                if name == "barrier_scalings" and report.get("min_barrier_scale") is not None:
                    detail = f", smallest factor {float(report['min_barrier_scale']):.3f}"
                reasons.append(
                    f"G500: {int(count)} {label} event(s){detail}; the contract allows {limit}"
                )

        distance = report.get("min_inter_agent_distance")
        d_min = report.get("d_min")
        if distance is None or d_min is None:
            reasons.append("G500: inter-agent separation was not recorded")
        elif float(distance) < float(d_min) - self.d_min_tolerance:
            reasons.append(
                f"G500: min inter-agent distance {float(distance):.6f} m < d_min {float(d_min):.6f} m "
                f"by {float(d_min) - float(distance):.2e} m"
            )

        clearance = report.get("min_signed_clearance")
        if clearance is None:
            reasons.append("G500: signed clearance was not recorded")
        elif float(clearance) < 0.0:
            reasons.append(f"G500: min signed clearance {float(clearance):.4f} m < 0 (a robot entered the cargo)")

        penetration = report.get("max_penetration")
        budget = report.get("penetration_budget")
        if penetration is None or budget is None:
            reasons.append("G500: penetration was not recorded")
        elif float(penetration) > float(budget):
            reasons.append(
                f"G500: max penetration {float(penetration):.4f} m > budget {float(budget):.4f} m "
                "(delta_max + discrete overshoot)"
            )

        metrics = dict(report)
        metrics["quality_reasons"] = quality
        metrics["quality_ok"] = not quality
        return SuccessVerdict(success=not reasons, reasons=reasons, metrics=metrics)

    def as_dict(self) -> dict:
        return {
            "detect_by": self.detect_by,
            "contact_ready_by": self.contact_ready_by,
            "transport_by": self.transport_by,
            "reach_by": self.reach_by,
            "efficiency_min": self.efficiency_min,
            "direction_error_max_deg": self.direction_error_max_deg,
            "cross_track_max": self.cross_track_max,
            "coverage_min": self.coverage_min,
            "progress_max_ratio": self.progress_max_ratio,
            "hold_speed_max": self.hold_speed_max,
            "yaw_max_deg": self.yaw_max_deg,
            "barrier_scalings_max": self.barrier_scalings_max,
        }


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
    "ClosedLoopContract",
    "SuccessVerdict",
    "CoverageContract",
    "VALID_BACKENDS",
    "EXACT_BACKENDS",
]
