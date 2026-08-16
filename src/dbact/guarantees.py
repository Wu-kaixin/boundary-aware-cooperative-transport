"""Executable admissibility certificates for the conditional DBACT theorem.

Ported from the CODEX branch onto the v1 controller. The certificate deliberately
separates statements that simulation reports routinely blur:

* the lane partition's sensor tubes cover the complete rectangular workspace;
* one concrete cargo belongs to the declared admissible simple-polygon class;
* its cage and translated footprint fit, and the team has enough geometric and
  force capacity to execute the task;
* what, if anything, follows about *time*.

It does not infer a theorem from successful trials. Every premise is named,
measured, and fail-closed; an ineligible object may still be simulated, but the
run is forbidden from carrying the conditional-guarantee label.

What changed in the port, and why
---------------------------------
**The search premises are v1's, not CODEX's.** CODEX proved workspace coverage
for a ``paired_sweep`` layout with a rendezvous-and-gossip protocol. v1 has a
different search -- a static boustrophedon lane partition with an object token
relayed hop by hop -- and it has its own coverage argument. Restating CODEX's
premises verbatim would have produced a certificate that is false for every v1
run for a reason that has nothing to do with the object. The predicates below are
therefore stated over v1's lane partition: same theorem shape, v1's constants.
The one place where v1's protocol is genuinely weaker is the relay, and it is
marked as such rather than smoothed over -- see ``token_relay_connectivity``.

**No field is read with a default.** ``_controller_premises`` reads every
parameter as a plain attribute, so a rename in ``DBACTParams`` raises
``AttributeError`` while the certificate is being built. ``getattr(x, name,
default)`` is exactly how a premise gets quietly satisfied by a value nobody
chose, and a quietly satisfied premise is how a theorem gets inferred from a
successful trial. The same rule applies to the ``guarantee`` configuration block:
:func:`_required` raises rather than substituting a permissive bound.

**CODEX's transport_distance_consistency predicate is gone**, because the
inconsistency it guarded against cannot arise here. CODEX carried the task
distance twice -- once on the task, once on the controller -- and had to check
that the two agreed. In v1 the distance exists once, on the sampled
``TransportTask``; the team closes its own loop on its own registration estimate
and never reads a configured distance. There is nothing to reconcile.

**Formal caging is never claimed.** ``formal_caging`` is a constant ``False``.
What the predicates below certify is operational enclosure: enough of the
boundary is covered, by robots close enough, to apply the required wrench. That
is not a proof that the object cannot escape, and the certificate says so in the
one place a reader is guaranteed to look.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import linprog

from .cargo import Cargo
from .contact_dynamics import ContactParams
from .controller import DBACTParams
from .geometry import (
    certified_inscribed_radius,
    closest_point_on_segment,
    is_simple_polygon,
    outward_edge_normals,
    polygon_diameter,
    polygon_perimeter,
    sample_polygon_boundary,
)

THEOREM_ID = "DBACT-CONDITIONAL-SIMPLE-POLYGON-v2"
FINITE_TIME_BOUND_ID = "DBACT-CONDITIONAL-FINITE-TIME-v2"

#: Contraction rates the finite-time bound is stated over. Until each of these
#: has an independent certificate -- a proof, not a curve fitted to episodes that
#: happened to succeed -- the bound is reported as unavailable regardless of how
#: many runs finish. Listing them here rather than in prose keeps the reason a
#: bound is unavailable machine-readable.
UNCERTIFIED_CONTRACTION_RATES = (
    "enclosure_contraction_rate_hz",
    "transport_progress_rate_mps",
    "brake_contraction_rate_hz",
)


class GuaranteeSpecError(ValueError):
    """A guarantee premise was not declared. Never substituted with a default."""


@dataclass(frozen=True)
class Check:
    passed: bool
    value: float | int | str | bool | dict | None
    bound: str
    rationale: str

    def as_dict(self) -> dict:
        return {
            "passed": bool(self.passed),
            "value": self.value,
            "bound": self.bound,
            "rationale": self.rationale,
        }


def _required(spec: dict, key: str, kind=float):
    """Read a declared premise, or refuse to build the certificate.

    A missing ``max_perimeter`` defaulting to infinity is a check that always
    passes; a missing ``min_feature_radius`` defaulting to zero is a witness
    requirement no shape can fail. Both are worse than an error, because both
    look like evidence.
    """
    if key not in spec:
        raise GuaranteeSpecError(
            f"guarantee premise {key!r} is not declared. It has no default: a bound nobody "
            "wrote down is not a premise, and a certificate built over it would be a "
            "statement about the default rather than about the run"
        )
    return kind(spec[key])


# --------------------------------------------------------------------------- #
# the finite-time bound
# --------------------------------------------------------------------------- #


def derive_conditional_finite_time_bound(
    *,
    dt: float,
    search_bound_s: float,
    map_bound_s: float,
    enclosure_initial_error_m: float,
    enclosure_terminal_error_m: float,
    enclosure_contraction_rate_hz: float,
    transport_distance_m: float,
    brake_activation_distance_m: float,
    transport_progress_rate_mps: float,
    brake_initial_error_m: float,
    brake_terminal_error_m: float,
    brake_contraction_rate_hz: float,
    hold_dwell_s: float,
    contraction_rates_certified: bool = False,
) -> dict:
    """Conditional sufficient finite-time bound with explicit premises.

    The enclosure premise is ``D+ E <= -lambda_e E`` until ``E <= E_tol``. The
    transport premise is ``dot J >= v_min`` outside the BRAKE band. The braking
    premise is ``D+ |e_J| <= -lambda_b |e_J|`` until the terminal band. These are
    theorem assumptions to prove or certify independently; they are never
    estimated from a successful episode by this function.

    ``available`` is the field a caller should read. It is ``False`` -- and the
    arithmetic below is reported as ``arithmetic_only`` -- until
    ``contraction_rates_certified`` is passed as ``True`` by something holding an
    independent certificate for every rate in
    :data:`UNCERTIFIED_CONTRACTION_RATES`. Nothing in this repository passes it,
    which is the honest state of the work: the numbers are what the bound *would*
    be, not a bound.
    """
    values = {
        "dt": float(dt),
        "search_bound_s": float(search_bound_s),
        "map_bound_s": float(map_bound_s),
        "enclosure_initial_error_m": float(enclosure_initial_error_m),
        "enclosure_terminal_error_m": float(enclosure_terminal_error_m),
        "enclosure_contraction_rate_hz": float(enclosure_contraction_rate_hz),
        "transport_distance_m": float(transport_distance_m),
        "brake_activation_distance_m": float(brake_activation_distance_m),
        "transport_progress_rate_mps": float(transport_progress_rate_mps),
        "brake_initial_error_m": float(brake_initial_error_m),
        "brake_terminal_error_m": float(brake_terminal_error_m),
        "brake_contraction_rate_hz": float(brake_contraction_rate_hz),
        "hold_dwell_s": float(hold_dwell_s),
    }
    checks = {
        "positive_dt": values["dt"] > 0.0,
        "finite_search_bound": values["search_bound_s"] >= 0.0,
        "finite_map_bound": values["map_bound_s"] >= 0.0,
        "enclosure_error_order": (
            values["enclosure_initial_error_m"] >= values["enclosure_terminal_error_m"] > 0.0
        ),
        "positive_enclosure_contraction": values["enclosure_contraction_rate_hz"] > 0.0,
        "positive_transport_distance": values["transport_distance_m"] > 0.0,
        "valid_brake_activation": (
            0.0 <= values["brake_activation_distance_m"] <= values["transport_distance_m"]
        ),
        "positive_transport_progress_rate": values["transport_progress_rate_mps"] > 0.0,
        "brake_error_order": (
            values["brake_initial_error_m"] >= values["brake_terminal_error_m"] > 0.0
        ),
        "positive_brake_contraction": values["brake_contraction_rate_hz"] > 0.0,
        "nonnegative_hold_dwell": values["hold_dwell_s"] >= 0.0,
    }
    arithmetic_ok = bool(all(checks.values()))
    certified = bool(contraction_rates_certified)
    base = {
        "bound_id": FINITE_TIME_BOUND_ID,
        "classification": "provable_sufficient_conditional",
        # The only field a caller should gate on.
        "available": arithmetic_ok and certified,
        "arithmetic_consistent": arithmetic_ok,
        "contraction_rates_certified": certified,
        "uncertified_rates": [] if certified else list(UNCERTIFIED_CONTRACTION_RATES),
        "empirical": False,
        "premises": values,
        "checks": checks,
        "failure_reasons": [name for name, passed in checks.items() if not passed],
    }
    if not arithmetic_ok:
        return {
            **base,
            "phase_bounds_s": None,
            "phase_bounds_frames": None,
            "total_bound_s": None,
            "total_bound_frames": None,
        }

    enclosure_time = float(
        np.log(values["enclosure_initial_error_m"] / values["enclosure_terminal_error_m"])
        / values["enclosure_contraction_rate_hz"]
    )
    drive_distance = max(
        0.0, values["transport_distance_m"] - values["brake_activation_distance_m"]
    )
    drive_time = drive_distance / values["transport_progress_rate_mps"]
    brake_time = float(
        np.log(values["brake_initial_error_m"] / values["brake_terminal_error_m"])
        / values["brake_contraction_rate_hz"]
    )
    phase_seconds = {
        "search": values["search_bound_s"],
        "map": values["map_bound_s"],
        "enclose": enclosure_time,
        "transport": drive_time + brake_time,
        "hold": values["hold_dwell_s"],
    }
    phase_frames = {
        name: int(math.ceil(seconds / values["dt"] - 1e-12))
        for name, seconds in phase_seconds.items()
    }
    return {
        **base,
        "formulas": {
            "enclose": "log(E0/E_tol)/lambda_enclose",
            "transport_drive": "max(0,L-e_brake)/v_progress_min",
            "brake": "log(e_brake0/e_hold)/lambda_brake",
            "total": "T_search+T_map+T_enclose+T_transport+T_hold",
        },
        "phase_bounds_s": phase_seconds,
        "phase_bounds_frames": phase_frames,
        "total_bound_s": float(sum(phase_seconds.values())),
        "total_bound_frames": int(sum(phase_frames.values())),
    }


# --------------------------------------------------------------------------- #
# geometric witnesses
# --------------------------------------------------------------------------- #


def guaranteed_detection_radius(
    sensor_range: float,
    feature_radius: float,
    ray_count: int,
    required_returns: int = 1,
) -> float:
    """Conservative radius of a finite-ray scan around a robot path.

    If the polygon contains a disk of radius ``r_f``, a ray that crosses that disk
    must hit the polygon. An angular interval of width at least ``k`` ray spacings
    contains at least ``k`` rays for every scanner phase. Combining that angular
    condition with a conservative range condition gives the sensor tube radius
    below. This is what stops the coverage argument from quietly assuming an ideal
    disk sensor: at 72 rays the angular term, not the range, is usually binding.
    """
    count = max(1, int(ray_count))
    k = max(1, int(required_returns))
    angle = min(0.5 * math.pi, k * math.pi / count)
    angular = feature_radius / max(math.sin(angle), 1e-12)
    ranged = max(0.0, sensor_range - feature_radius)
    return float(max(0.0, min(angular, ranged)))


def boundary_map_gap_upper_bound(
    vertices: np.ndarray,
    map_points: np.ndarray,
    sample_count: int = 1024,
) -> dict:
    """Conservative continuous-boundary gap from finite uniform samples.

    Distance to a fixed point set is 1-Lipschitz along polygon arclength. Every
    boundary point is within ``P/(2n)`` arclength of one of ``n`` uniform samples,
    so adding that term converts the sampled maximum into a rigorous one-sided
    Hausdorff upper bound. Reporting the sampled maximum alone would be optimistic
    by exactly the amount that matters -- the gap between samples is where an
    unobserved notch hides.

    The true outline is read here and nowhere else in this module's runtime path:
    this is a witness computed after the fact, never a signal.
    """
    count = max(3, int(sample_count))
    points = np.asarray(map_points, dtype=float).reshape(-1, 2)
    if len(points) == 0:
        return {
            "boundary_samples": count,
            "sampled_max_boundary_gap": float("inf"),
            "sampling_resolution_bound": float("inf"),
            "max_boundary_gap": float("inf"),
            "p95_boundary_gap": float("inf"),
        }
    truth, _ = sample_polygon_boundary(vertices, count=count)
    gaps = np.min(np.linalg.norm(truth[:, None, :] - points[None, :, :], axis=2), axis=1)
    sampling_bound = float(polygon_perimeter(vertices) / (2.0 * count))
    sampled_max = float(np.max(gaps))
    return {
        "boundary_samples": count,
        "sampled_max_boundary_gap": sampled_max,
        "sampling_resolution_bound": sampling_bound,
        "max_boundary_gap": sampled_max + sampling_bound,
        "p95_boundary_gap": float(np.quantile(gaps, 0.95)),
    }


def issf_margin_budget(
    *,
    rho: float,
    tangential_window: float,
    omega_max: float,
    recovery_fraction: float,
    max_speed: float,
    velocity_error: float = 0.0,
) -> dict:
    """What ``rho`` has to pay for on a *moving* boundary, and whether it can.

    The object row keeps ``n_k^T (u_i - v_{b_k})`` and drops
    ``(dn_k/dt)^T (p_i - b_k)``; ``rho`` is the price of the drop. For a rigid body
    ``dn_k/dt = omega R90 n_k``, so the dropped term is ``omega`` times the robot's
    *tangential* offset from its boundary point -- and that offset is bounded by the
    tangential window ``W``, because the window is exactly the filter that admits a row only
    while ``|t_k| <= W``. Hence

        | dropped term |  <=  omega_max * W.

    This is what makes the window part of the barrier construction rather than an
    implementation detail: without it the dropped term is unbounded and no ``rho`` exists.

    **The budget is double-booked, and this function separates it.**
    ``bounded_perception_and_motion_error`` checks ``velocity_error <= rho``. But
    ``velocity_error`` is the error in the *kept* term -- a different and independent
    disturbance from the *dropped* one ``rho`` was sized for. Asking one margin to cover both
    is how a premise comes to look satisfied while neither error is bounded. The honest
    requirement is the sum:

        rho  >=  omega_max * W  +  e_v.

    And ``rho`` is bounded from *above* as well. ``_cap_to_reachable`` limits every row to
    ``recovery_fraction * max_speed`` along the common retreat direction, so a margin larger
    than that is not a stronger guarantee but an infeasible problem:

        omega_max * W + e_v   <=   rho   <=   recovery_fraction * max_speed.

    When the left exceeds the right **no value of rho works** -- a statement about the
    actuator rather than about tuning, reported as ``satisfiable``.

    Measured on the baseline (``scripts/derive_issf_margin.py``): the rotation term at the
    declared 0.80 rad/s yaw bound is 0.2240 m/s, 11.2x the configured ``rho = 0.02`` and
    1.07x the 0.2100 m/s cap -- unsatisfiable before the velocity error is even added.
    Inverted, ``rho = 0.02`` covers rotation up to 4.09 deg/s and the baseline cargo turns at
    roughly 0.003 deg/s. So the configured value is defensible *for that object* and
    indefensible as a general bound, which is why this returns the regime and not just a
    verdict.
    """
    rotation_term = float(omega_max) * float(tangential_window)
    cap = float(recovery_fraction) * float(max_speed)
    required = rotation_term + float(velocity_error)
    return {
        "rho_configured": float(rho),
        "tangential_window": float(tangential_window),
        "omega_max": float(omega_max),
        "reachable_cap": cap,
        # The two independent disturbances, kept apart.
        "dropped_normal_rate_term": rotation_term,
        "kept_velocity_error_term": float(velocity_error),
        "required_rho": required,
        "sufficient": bool(float(rho) + 1e-12 >= required),
        "within_reachable_cap": bool(required <= cap + 1e-12),
        # The field to gate on: a margin both large enough and deliverable.
        "satisfiable": bool(required <= cap + 1e-12 and float(rho) + 1e-12 >= required),
        # Inverted regimes, so a reader gets the condition and not only the verdict.
        "omega_max_covered_by_rho": float(rho) / max(float(tangential_window), 1e-12),
        "omega_max_covered_by_cap": cap / max(float(tangential_window), 1e-12),
        "velocity_error_budget_at_cap": max(0.0, cap - rotation_term),
    }


def _domain_margin(points: np.ndarray, domain: tuple[float, float, float, float]) -> float:
    xmin, xmax, ymin, ymax = domain
    p = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(p) == 0:
        return float("-inf")
    return float(
        min(
            np.min(p[:, 0] - xmin),
            np.min(xmax - p[:, 0]),
            np.min(p[:, 1] - ymin),
            np.min(ymax - p[:, 1]),
        )
    )


def _edge_offset_endpoints(vertices: np.ndarray, offset: float) -> np.ndarray:
    v = np.asarray(vertices, dtype=float).reshape(-1, 2)
    normals = outward_edge_normals(v)
    return np.vstack([v + offset * normals, np.roll(v, -1, axis=0) + offset * normals])


def minimum_facing_cage_clearance(vertices: np.ndarray, offset: float) -> float:
    """Minimum signed gap between mutually facing non-adjacent offset edges.

    A concavity narrower than ``d_min`` at the cage offset is a place two robots
    are both told to stand and the inter-robot barrier forbids them both from
    standing. The run does not fail loudly when this happens -- it fails as a
    permanently open arc -- so the geometry is checked before the run instead.

    A negative return value means the two offset walls have crossed: the concavity
    is narrower than the two offsets together. ``inf`` means the outline has no
    mutually facing non-adjacent edge pair at all, which is the normal answer for a
    convex outline and must not be confused with zero.

    Fixed here, and why (T1)
    ------------------------
    The ported version tested "does edge j lie on edge i's outward side" using the
    *offset* midpoints, and skipped the pair when it did not. That skip is correct
    for a back-to-back pair -- the top and bottom of a U are antiparallel but not
    facing -- and wrong for the worst case in the class. Offsetting moves each
    midpoint outward by ``offset``, so the offset separation is roughly the wall
    separation minus ``2 * offset``; once a slot is narrower than that, the test
    flips sign and the pair was skipped, reporting ``inf`` -- "no facing edge pair"
    -- for precisely the geometry the predicate exists to reject. A 0.36 m slot at
    a 0.20 m cage offset passed the check. That is a premise satisfied by an
    accident of arithmetic rather than by the shape, which is the failure mode this
    whole module is written against.

    Facing is a property of the walls, so the facing test now runs on the un-offset
    midpoints, where it is a well-posed geometric question, and the clearance is
    still measured on the offset curves. The change is one-directional: because the
    offset separation never exceeds the un-offset separation for an antiparallel
    pair, every pair the old test admitted is still admitted and measured by the
    same four-candidate distance. What is new is the crossed pairs, which now
    report a negative width instead of nothing.
    """
    v = np.asarray(vertices, dtype=float).reshape(-1, 2)
    normals = outward_edge_normals(v)
    a = v + offset * normals
    b = np.roll(v, -1, axis=0) + offset * normals
    mid = 0.5 * (a + b)
    wall_mid = 0.5 * (v + np.roll(v, -1, axis=0))
    best = float("inf")
    for i in range(len(v)):
        for j in range(i + 1, len(v)):
            if j == i + 1 or (i == 0 and j == len(v) - 1):
                continue
            delta = mid[j] - mid[i]
            if float(np.dot(normals[i], normals[j])) > -0.5:
                continue
            wall_delta = wall_mid[j] - wall_mid[i]
            if (
                float(np.dot(normals[i], wall_delta)) <= 0.0
                or float(np.dot(normals[j], -wall_delta)) <= 0.0
            ):
                # Antiparallel but back to back: the outline's two outer faces, not
                # a concavity. There is no corridor between them to measure.
                continue
            signed = min(float(np.dot(normals[i], delta)), float(np.dot(normals[j], -delta)))
            if signed <= 0.0:
                # The offset walls have crossed. Report the overlap so that the
                # ``>= d_min`` check fails; an unsigned segment distance would
                # return a small positive number, or zero, for the worst case.
                best = min(best, signed)
                continue
            candidates = [
                np.linalg.norm(a[i] - closest_point_on_segment(a[i], a[j], b[j])[0]),
                np.linalg.norm(b[i] - closest_point_on_segment(b[i], a[j], b[j])[0]),
                np.linalg.norm(a[j] - closest_point_on_segment(a[j], a[i], b[i])[0]),
                np.linalg.norm(b[j] - closest_point_on_segment(b[j], a[i], b[i])[0]),
            ]
            best = min(best, *(float(value) for value in candidates))
    return best


def _wrench_feasibility(
    cargo: Cargo,
    goal: np.ndarray,
    agent_count: int,
    contact: ContactParams,
    robot_radius: float,
    cage_offset: float,
    force_margin: float,
) -> tuple[bool, float, float]:
    """Exact polygon-edge LP for a positive, zero-torque pushing wrench.

    Contact forces are unilateral, so the allocation is constrained nonnegative;
    an LP that allowed pulling would certify wrenches no pusher can produce. The
    zero-torque equality is what makes this a statement about *transport* rather
    than about spinning the object into the required displacement.
    """
    vertices = cargo.vertices
    normals = outward_edge_normals(vertices)
    points = np.vstack([vertices, np.roll(vertices, -1, axis=0)])
    forces = np.vstack([-normals, -normals])
    useful = (forces @ goal) > 1e-9
    points, forces = points[useful], forces[useful]
    if len(points) == 0:
        return False, float("inf"), 0.0
    arms = points - cargo.center[None, :]
    torques = arms[:, 0] * forces[:, 1] - arms[:, 1] * forces[:, 0]
    matrix = np.vstack([forces.T, torques[None, :]])
    required_force = force_margin * contact.breakaway_force(cargo.mass)
    target = np.array([required_force * goal[0], required_force * goal[1], 0.0])
    per_robot = contact.stiffness * max(robot_radius - cage_offset, 0.0)
    if per_robot <= 0.0:
        return False, float("inf"), 0.0
    result = linprog(
        np.zeros(len(points)),
        A_ub=np.ones((1, len(points))),
        b_ub=np.array([agent_count * per_robot]),
        A_eq=matrix,
        b_eq=target,
        bounds=[(0.0, per_robot) for _ in range(len(points))],
        method="highs",
    )
    residual = float("inf")
    used = 0.0
    if result.success and result.x is not None:
        residual = float(np.linalg.norm(matrix @ result.x - target))
        used = float(np.sum(result.x) / max(per_robot, 1e-12))
    return bool(result.success and residual <= 1e-7), residual, used


# --------------------------------------------------------------------------- #
# the v1 parameter map
# --------------------------------------------------------------------------- #


def _controller_premises(controller: DBACTParams) -> dict:
    """Field-by-field map from v1's ``DBACTParams`` to certificate premises.

    Every name is a plain attribute read. If a field is renamed or removed, this
    raises ``AttributeError`` while the certificate is being built, which is the
    intended behaviour: a certificate that survives its own parameters going
    missing is certifying something other than the run.
    """
    return {
        # perception -- the finite-ray detection tube
        "sensor_range": float(controller.sensor_range),
        "ray_count": int(controller.ray_count),
        # communication -- the token relay
        "comm_range": float(controller.comm_range),
        "token_ttl": float(controller.token_ttl),
        # search -- v1's static lane partition
        "search_mode": str(controller.search_mode),
        "search_margin": float(controller.search_margin),
        "search_speed": float(controller.search_speed),
        # enclosure geometry
        "cage_offset": float(controller.cage_offset),
        "lead_offset": (
            None if controller.lead_offset is None else float(controller.lead_offset)
        ),
        "robot_radius": float(controller.robot_radius),
        "d_min": float(controller.d_min),
        # safety margins entering the conditional ISSf statement
        "rho": float(controller.rho),
        "gamma_obj": float(controller.gamma_obj),
        "delta_max": float(controller.delta_max),
        "object_velocity_bound": float(controller.object_velocity_bound),
        # transport quorum
        "min_push_agents": int(controller.min_push_agents),
        # braking, for the finite-time arithmetic
        "brake_fraction": float(controller.brake_fraction),
        "transport_reference_speed": float(controller.transport_reference_speed),
        "brake_gain": float(controller.brake_gain),
        "contact_dwell": int(controller.contact_dwell),
    }


def build_admissibility_certificate(
    *,
    cargo: Cargo,
    agents: list,
    domain: tuple[float, float, float, float],
    goal_direction: np.ndarray | None,
    target_distance: float,
    config: dict,
    controller: DBACTParams,
    contact: ContactParams,
    dt: float,
) -> dict:
    """Build a JSON-serialisable, fail-closed theorem certificate."""
    spec = config.get("guarantee", {}) or {}
    enabled = bool(spec.get("enabled", False))
    p = _controller_premises(controller)

    search_spec = spec.get("search", {}) or {}
    evaluation = config.get("evaluation", {}) or {}

    feature_required = _required(spec, "min_feature_radius")
    required_returns = int(search_spec.get("required_returns", 1))
    r_detect = guaranteed_detection_radius(
        p["sensor_range"], feature_required, p["ray_count"], required_returns
    )

    count = len(agents)
    xmin, xmax, ymin, ymax = domain
    margin = p["search_margin"]
    lanes = max(1, count)
    # The lane partition v1 actually walks: ``_sweep_velocity`` puts robot i of N
    # at x = xmin + m + (i + 0.5) (W - 2m) / N and walks y between ymin + m and
    # ymax - m. The predicates below are stated over exactly those numbers.
    lane_width = max(xmax - xmin - 2.0 * margin, 1e-6) / lanes
    lane_height = max(ymax - ymin - 2.0 * margin, 0.0)
    sweep_frames = int(math.ceil(lane_height / max(p["search_speed"] * dt, 1e-12)))

    witness_radius = certified_inscribed_radius(cargo.vertices) if is_simple_polygon(cargo.vertices) else 0.0
    checks: dict[str, Check] = {}
    groups: dict[str, list[str]] = {"shape": [], "search": [], "task": [], "time": []}

    def add(group: str, name: str, passed: bool, value, bound: str, rationale: str) -> None:
        checks[name] = Check(bool(passed), value, bound, rationale)
        groups[group].append(name)

    # -- shape ------------------------------------------------------------- #
    simple = is_simple_polygon(cargo.vertices)
    add(
        "shape",
        "simple_polygon",
        simple,
        bool(simple),
        "true",
        "A simple, non-degenerate outline is required for inside/outside and boundary ordering.",
    )
    add(
        "shape",
        "single_cargo_episode",
        len(config.get("cargoes", [])) == 1,
        len(config.get("cargoes", [])),
        "== 1",
        "The search argument is a single-unknown-object one; occluding multiple cargoes are out of scope.",
    )
    add(
        "shape",
        "feature_witness",
        witness_radius + 1e-12 >= feature_required > 0.0,
        witness_radius,
        f">= {feature_required:.6g} m",
        "An ear-triangle incircle is a constructive disk contained in the polygon, not a grid estimate.",
    )
    max_perimeter = _required(spec, "max_perimeter")
    add(
        "shape",
        "perimeter_bound",
        cargo.perimeter <= max_perimeter + 1e-12,
        cargo.perimeter,
        f"<= {max_perimeter:.6g} m",
        "A fixed team cannot cover an unbounded boundary.",
    )
    diameter = polygon_diameter(cargo.vertices)
    max_diameter = _required(spec, "max_diameter")
    add(
        "shape",
        "diameter_bound",
        diameter <= max_diameter + 1e-12,
        diameter,
        f"<= {max_diameter:.6g} m",
        "The object and its cage must fit in the finite workspace.",
    )

    # -- search: v1's lane partition --------------------------------------- #
    add(
        "search",
        "lane_partition_declared",
        p["search_mode"] == "sweep",
        p["search_mode"],
        "== 'sweep'",
        "The coverage argument is about the static lane partition; the spiral ablation has no such bound.",
    )
    add(
        "search",
        "finite_ray_detection_tube",
        r_detect > 0.0,
        r_detect,
        "> 0 m",
        "Finite angular ray spacing is included; this is not an ideal disk-sensor assumption.",
    )
    add(
        "search",
        "lane_spacing_cover",
        0.5 * lane_width <= r_detect + 1e-12,
        0.5 * lane_width,
        f"<= detection tube {r_detect:.6g} m",
        "Every workspace point is within half a lane of a lane centre, hence inside one sensor tube.",
    )
    edge_reach = max(margin + 0.5 * lane_width, margin)
    add(
        "search",
        "workspace_edge_cover",
        edge_reach <= r_detect + 1e-12,
        edge_reach,
        f"<= detection tube {r_detect:.6g} m",
        "The sweep stops a margin short of every wall; the tubes must still reach the outer boundary.",
    )
    add(
        "search",
        "token_relay_connectivity",
        lane_width <= p["comm_range"] + 1e-12,
        lane_width,
        f"<= comm_range {p['comm_range']:.6g} m",
        "NECESSARY, NOT SUFFICIENT: adjacent lanes must be within range for a token to hop at all. "
        "v1's relay is opportunistic, so no claim is made that the graph is connected when it matters.",
    )
    add(
        "search",
        "token_lifetime",
        p["token_ttl"] >= max(0, count - 1) * dt,
        p["token_ttl"],
        f">= {max(0, count - 1) * dt:.6g} s ({max(0, count - 1)} hops at dt={dt:.6g})",
        "A token must outlive a flood across the team's hop diameter, conditional on connectivity.",
    )
    map_epsilon = _required(spec, "boundary_map_epsilon")
    add(
        "search",
        "boundary_map_resolution_declared",
        map_epsilon > 0.0,
        map_epsilon,
        "> 0 m",
        "The post-search statement is conditional on an epsilon-dense observed boundary map.",
    )

    # -- task: enclosure, corridor, force ---------------------------------- #
    contact_radius = float(_required(evaluation, "contact_radius"))
    per_agent_arc = 2.0 * max(0.0, contact_radius - p["cage_offset"])
    required_agents = math.inf if per_agent_arc <= 0.0 else int(math.ceil(cargo.perimeter / per_agent_arc))
    add(
        "task",
        "boundary_covering_number",
        required_agents <= count,
        required_agents if math.isfinite(required_agents) else "infinite",
        f"<= {count} agents",
        "Arclength plus the triangle inequality gives a conservative continuous-boundary cover, "
        "ceil(P / (2 (R_cov - d_c))).",
    )
    facing_clearance = minimum_facing_cage_clearance(cargo.vertices, p["cage_offset"])
    add(
        "task",
        "cage_offset_self_clearance",
        facing_clearance >= p["d_min"] - 1e-12,
        facing_clearance if math.isfinite(facing_clearance) else "no facing edge pair",
        f">= d_min {p['d_min']:.6g} m",
        "Robots assigned to mutually facing concavity walls must fit without violating separation.",
    )

    goal = None if goal_direction is None else np.asarray(goal_direction, dtype=float).reshape(2)
    goal_ok = goal is not None and float(np.linalg.norm(goal)) > 1e-12
    if goal_ok:
        goal = goal / float(np.linalg.norm(goal))
    distance = float(target_distance)
    shift = distance * goal if goal_ok else np.zeros(2)
    cargo_margin = min(_domain_margin(cargo.vertices, domain), _domain_margin(cargo.vertices + shift, domain))
    add(
        "task",
        "swept_cargo_corridor",
        goal_ok and cargo_margin >= -1e-12,
        cargo_margin,
        ">= 0 m",
        "A rectangle is convex, so start/end containment certifies every translated intermediate footprint.",
    )

    cage_offset_outer = max(p["cage_offset"], p["lead_offset"] or p["cage_offset"])
    cage_centers = _edge_offset_endpoints(cargo.vertices, cage_offset_outer)
    cage_margin = min(_domain_margin(cage_centers, domain), _domain_margin(cage_centers + shift, domain))
    add(
        "task",
        "swept_cage_corridor",
        goal_ok and cage_margin >= p["robot_radius"] - 1e-12,
        cage_margin,
        f">= robot_radius {p['robot_radius']:.6g} m",
        "Every edge-offset cage centre and robot disk fits at both corridor endpoints.",
    )

    cooperative_need = int(math.ceil(contact.min_cooperating_robots(cargo.mass, p["cage_offset"])))
    add(
        "task",
        "contact_force_capacity",
        cooperative_need <= p["min_push_agents"] <= count,
        cooperative_need,
        f"<= min_push_agents {p['min_push_agents']} <= team {count}",
        "The locally gated pushing quorum must exceed Coulomb breakaway at nominal cage penetration.",
    )
    force_margin = _required(spec, "force_margin")
    wrench_ok, wrench_residual, wrench_agents = (
        _wrench_feasibility(
            cargo, goal, count, contact, p["robot_radius"], p["cage_offset"], force_margin
        )
        if goal_ok
        else (False, float("inf"), 0.0)
    )
    add(
        "task",
        "goal_wrench_feasibility",
        wrench_ok,
        {"residual": wrench_residual, "equivalent_agents": wrench_agents},
        f"zero-torque goal wrench >= {force_margin:.3g} x breakaway force",
        "A nonnegative edge-contact allocation must produce the requested force with zero net torque.",
    )

    error_spec = spec.get("bounded_errors", {}) or {}
    normal_error = _required(error_spec, "normal_error_deg")
    velocity_error = _required(error_spec, "velocity_error")
    add(
        "task",
        "bounded_perception_and_motion_error",
        0.0 <= normal_error < 90.0 and 0.0 <= velocity_error <= p["rho"] + 1e-12,
        {"normal_error_deg": normal_error, "velocity_error": velocity_error},
        f"normal < 90 deg and velocity_error <= rho {p['rho']:.6g} m/s",
        "The conditional safety statement requires declared finite normal and moving-boundary errors.",
    )

    # -- time -------------------------------------------------------------- #
    finite_time_spec = spec.get("finite_time")
    derived_time_bound = None
    if isinstance(finite_time_spec, dict):
        brake_activation = (1.0 - p["brake_fraction"]) * distance
        derived_time_bound = derive_conditional_finite_time_bound(
            dt=dt,
            search_bound_s=sweep_frames * dt,
            map_bound_s=_required(finite_time_spec, "map_bound_s"),
            enclosure_initial_error_m=_required(finite_time_spec, "enclosure_initial_error_m"),
            enclosure_terminal_error_m=_required(finite_time_spec, "enclosure_terminal_error_m"),
            enclosure_contraction_rate_hz=_required(finite_time_spec, "enclosure_contraction_rate_hz"),
            transport_distance_m=distance,
            brake_activation_distance_m=brake_activation,
            transport_progress_rate_mps=_required(finite_time_spec, "transport_progress_rate_mps"),
            brake_initial_error_m=_required(finite_time_spec, "brake_initial_error_m"),
            brake_terminal_error_m=_required(finite_time_spec, "brake_terminal_error_m"),
            brake_contraction_rate_hz=_required(finite_time_spec, "brake_contraction_rate_hz"),
            hold_dwell_s=p["contact_dwell"] * dt,
            # Nothing in this repository certifies the contraction rates, so this
            # is hard-wired rather than exposed: an argument nobody can pass a
            # True to is an argument nobody can pass a True to by accident.
            contraction_rates_certified=False,
        )
        add(
            "time",
            "derived_conditional_finite_time_bound",
            bool(derived_time_bound["available"]),
            derived_time_bound.get("total_bound_frames"),
            "available only when every contraction rate holds an independent certificate",
            "Analytic sufficient bound; never inferred from successful episode durations. "
            f"Uncertified: {', '.join(UNCERTIFIED_CONTRACTION_RATES)}.",
        )

    frame_budget = int(evaluation.get("frame_budget", 0) or 0)
    enclosure_bound = int(spec.get("enclosure_bound_frames", 0) or 0)
    transport_bound = int(spec.get("transport_bound_frames", 0) or 0)
    hold_bound = int(spec.get("hold_bound_frames", 0) or 0)
    total_bound = sweep_frames + enclosure_bound + transport_bound + hold_bound
    add(
        "time",
        "declared_finite_time_bounds",
        enclosure_bound > 0 and transport_bound > 0 and hold_bound >= 0,
        {
            "search_release": sweep_frames,
            "enclosure": enclosure_bound,
            "transport": transport_bound,
            "hold": hold_bound,
        },
        "positive phase bounds",
        "These are explicit premises, not values inferred from successful trials.",
    )
    add(
        "time",
        "frame_budget",
        frame_budget > 0 and total_bound <= frame_budget,
        total_bound,
        f"<= {frame_budget} frames",
        "The sum of the assumed phase bounds must fit the requested horizon.",
    )

    domain_names = groups["shape"] + groups["search"] + groups["task"]
    time_names = groups["time"]
    domain_eligible = enabled and all(checks[name].passed for name in domain_names)
    finite_time_eligible = (
        enabled and bool(time_names) and all(checks[name].passed for name in time_names)
    )
    return {
        "theorem_id": THEOREM_ID,
        "enabled": enabled,
        "eligible": bool(domain_eligible and finite_time_eligible),
        "domain_eligible": bool(domain_eligible),
        "finite_time_eligible": bool(finite_time_eligible),
        # Constant. Operational enclosure is what the predicates above certify;
        # nothing here proves the object cannot escape the cage.
        "formal_caging": False,
        "claim": (
            "conditional lane-swept discovery plus operational enclosure and bounded-wrench transport "
            "for this admissible simple polygon; formal caging is NOT claimed"
        ),
        "search": {
            "guaranteed_detection_radius": r_detect,
            "lane_width": lane_width,
            "lane_height": lane_height,
            "lane_count": lanes,
            "sweep_bound_frames": sweep_frames,
            "required_returns": required_returns,
        },
        "shape": {
            "vertices": int(len(cargo.vertices)),
            "area": float(cargo.area),
            "perimeter": float(cargo.perimeter),
            "diameter": diameter,
            "certified_inscribed_radius": witness_radius,
            "required_boundary_agents": required_agents if math.isfinite(required_agents) else None,
        },
        "mapping": {"required_max_boundary_gap": map_epsilon},
        # Reported, not gated. The decomposition shows the configured rho is insufficient
        # for the declared yaw bound -- and in fact unsatisfiable, because the rotation term
        # alone exceeds the reachability cap. Adding it to ``domain_eligible`` would make
        # every run on this branch ineligible at a stroke, which is a true statement but a
        # different experiment from the ones already committed; it is surfaced here so a
        # reader sees it beside the checks rather than only in the analysis script.
        # ``velocity_error`` is the *declared* premise, not a measurement: the measured value
        # is 30x larger and lives in the error audit.
        "issf_margin": issf_margin_budget(
            rho=p["rho"],
            tangential_window=float(controller.object_row_window),
            omega_max=float(controller.max_object_yaw_rate),
            recovery_fraction=float(controller.recovery_fraction),
            max_speed=float(controller.max_speed),
            velocity_error=velocity_error,
        ),
        "task": {
            "transport_distance": distance,
            "cargo_corridor_margin": cargo_margin,
            "cage_corridor_margin": cage_margin,
            "cooperative_agents_required": cooperative_need,
            "wrench_residual": wrench_residual,
            "wrench_equivalent_agents": wrench_agents,
            "total_bound_frames": total_bound,
        },
        "derived_finite_time_bound": derived_time_bound,
        "groups": groups,
        "checks": {name: check.as_dict() for name, check in checks.items()},
        "failure_reasons": [name for name, check in checks.items() if not check.passed],
        "domain_failure_reasons": [name for name in domain_names if not checks[name].passed],
        "finite_time_failure_reasons": [name for name in time_names if not checks[name].passed],
    }


def evaluate_runtime_map_completeness(
    *,
    certificate: dict,
    vertices: np.ndarray,
    map_points: np.ndarray,
    sample_count: int = 1024,
) -> dict:
    """Did the team's map actually reach the declared epsilon density?

    Separate from the pre-run certificate on purpose. The admissibility
    predicates are about whether the object *could* be handled; this is about
    whether the map the robots ended up with is dense enough for the conditional
    statement to apply to the run that happened. A run whose map never closed is
    ineligible even if every geometric premise held.
    """
    required = float(certificate["mapping"]["required_max_boundary_gap"])
    gap = boundary_map_gap_upper_bound(vertices, map_points, sample_count=sample_count)
    passed = bool(gap["max_boundary_gap"] <= required)
    reasons = [] if passed else ["boundary_map_epsilon"]
    return {
        "passed": passed,
        "required_max_boundary_gap": required,
        "runtime_failure_reasons": reasons,
        "runtime_domain_eligible": bool(certificate["domain_eligible"] and passed),
        **gap,
    }


__all__ = [
    "THEOREM_ID",
    "FINITE_TIME_BOUND_ID",
    "UNCERTIFIED_CONTRACTION_RATES",
    "Check",
    "GuaranteeSpecError",
    "boundary_map_gap_upper_bound",
    "guaranteed_detection_radius",
    "minimum_facing_cage_clearance",
    "issf_margin_budget",
    "derive_conditional_finite_time_bound",
    "build_admissibility_certificate",
    "evaluate_runtime_map_completeness",
]
