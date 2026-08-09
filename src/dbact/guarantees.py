"""Executable admissibility certificates for the conditional DBACT theorem.

The certificate deliberately separates three statements that are often blurred
in simulation reports:

* the predetermined sensor tubes cover the complete rectangular workspace;
* one concrete cargo belongs to the declared admissible simple-polygon class;
* its cage and translated footprint fit, and the team has enough geometric and
  force capacity to execute the task within the declared finite-time bounds.

It does not infer a theorem from successful trials.  Every premise is named,
measured, and fail-closed; an ineligible object may still be simulated, but the
run is forbidden from carrying the conditional-guarantee label.
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


THEOREM_ID = "DBACT-CONDITIONAL-SIMPLE-POLYGON-v1"
FINITE_TIME_BOUND_ID = "DBACT-CONDITIONAL-FINITE-TIME-v1"


@dataclass(frozen=True)
class Check:
    passed: bool
    value: float | int | str | bool | None
    bound: str
    rationale: str

    def as_dict(self) -> dict:
        return {
            "passed": bool(self.passed),
            "value": self.value,
            "bound": self.bound,
            "rationale": self.rationale,
        }


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
) -> dict:
    """Conditional sufficient finite-time bound with explicit premises.

    The enclosure premise is ``D+ E <= -lambda_e E`` until ``E <= E_tol``.
    The transport premise is ``dot J >= v_min`` outside the BRAKE band.  The
    braking premise is ``D+ |e_J| <= -lambda_b |e_J|`` until the terminal band.
    These are theorem assumptions to prove or certify independently; they are
    never estimated from a successful episode by this function.
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
            values["enclosure_initial_error_m"]
            >= values["enclosure_terminal_error_m"]
            > 0.0
        ),
        "positive_enclosure_contraction": values["enclosure_contraction_rate_hz"] > 0.0,
        "positive_transport_distance": values["transport_distance_m"] > 0.0,
        "valid_brake_activation": (
            0.0
            <= values["brake_activation_distance_m"]
            <= values["transport_distance_m"]
        ),
        "positive_transport_progress_rate": values["transport_progress_rate_mps"] > 0.0,
        "brake_error_order": (
            values["brake_initial_error_m"]
            >= values["brake_terminal_error_m"]
            > 0.0
        ),
        "positive_brake_contraction": values["brake_contraction_rate_hz"] > 0.0,
        "nonnegative_hold_dwell": values["hold_dwell_s"] >= 0.0,
    }
    eligible = bool(all(checks.values()))
    if not eligible:
        return {
            "bound_id": FINITE_TIME_BOUND_ID,
            "classification": "provable_sufficient_conditional",
            "eligible": False,
            "empirical": False,
            "premises": values,
            "checks": checks,
            "failure_reasons": [name for name, passed in checks.items() if not passed],
            "phase_bounds_s": None,
            "phase_bounds_frames": None,
            "total_bound_s": None,
            "total_bound_frames": None,
        }

    enclosure_time = float(
        np.log(
            values["enclosure_initial_error_m"] / values["enclosure_terminal_error_m"]
        )
        / values["enclosure_contraction_rate_hz"]
    )
    drive_distance = max(
        0.0,
        values["transport_distance_m"] - values["brake_activation_distance_m"],
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
        "bound_id": FINITE_TIME_BOUND_ID,
        "classification": "provable_sufficient_conditional",
        "eligible": True,
        "empirical": False,
        "premises": values,
        "checks": checks,
        "failure_reasons": [],
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


def guaranteed_detection_radius(
    sensor_range: float,
    feature_radius: float,
    ray_count: int,
    required_returns: int = 1,
) -> float:
    """Conservative radius of a finite-ray scan around a robot path.

    If the polygon contains a disk of radius ``r_f``, a ray that crosses that
    disk must hit the polygon.  An angular interval of width at least ``k`` ray
    spacings contains at least ``k`` rays for every scanner phase.  Combining
    that angular condition with a conservative range condition gives the sensor
    tube radius below.
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
    boundary point is within ``P/(2n)`` arclength of one of ``n`` uniform
    samples, so adding that term converts the sampled maximum into a rigorous
    one-sided Hausdorff upper bound.
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
    """Minimum gap between mutually facing non-adjacent offset edges."""
    v = np.asarray(vertices, dtype=float).reshape(-1, 2)
    normals = outward_edge_normals(v)
    a = v + offset * normals
    b = np.roll(v, -1, axis=0) + offset * normals
    mid = 0.5 * (a + b)
    best = float("inf")
    for i in range(len(v)):
        for j in range(i + 1, len(v)):
            if j == i + 1 or (i == 0 and j == len(v) - 1):
                continue
            delta = mid[j] - mid[i]
            if float(np.dot(normals[i], normals[j])) > -0.5:
                continue
            if float(np.dot(normals[i], delta)) <= 0.0 or float(np.dot(normals[j], -delta)) <= 0.0:
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
    controller: DBACTParams,
    force_margin: float,
) -> tuple[bool, float, float]:
    """Exact polygon-edge LP for a positive, zero-torque pushing wrench."""
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
    per_robot = contact.stiffness * max(controller.robot_radius - controller.cage_offset, 0.0)
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


def build_admissibility_certificate(
    *,
    cargo: Cargo,
    agents: list,
    domain: tuple[float, float, float, float],
    goal_direction: np.ndarray | None,
    config: dict,
    controller: DBACTParams,
    contact: ContactParams,
    dt: float,
) -> dict:
    """Build a JSON-serialisable, fail-closed theorem certificate."""
    spec = config.get("guarantee", {}) or {}
    search_spec = spec.get("search", {}) or {}
    evaluation = config.get("evaluation", {}) or {}
    task = config.get("task", {}) or {}
    random_goal = task.get("random_goal", {}) or {}

    feature_required = float(spec.get("min_feature_radius", 0.0))
    required_returns = int(search_spec.get("required_returns", 1))
    r_detect = guaranteed_detection_radius(
        controller.sensor_range,
        feature_required,
        controller.ray_count,
        required_returns,
    )
    count = len(agents)
    xmin, xmax, ymin, ymax = domain
    width, height = xmax - xmin, ymax - ymin
    paired = count >= 2 and count % 2 == 0
    per_side = count // 2 if paired else 0
    lane_spacing = height / per_side if per_side else float("inf")
    edge_padding = min(
        (min(float(a.position[0] - xmin), float(xmax - a.position[0])) for a in agents),
        default=float("inf"),
    )

    configured_tube = float(controller.search_detection_radius)
    sweep_distance = max(0.0, 0.5 * width - edge_padding - configured_tube)
    rendezvous_distance = max(0.0, configured_tube - 0.5 * controller.search_meeting_gap)
    sweep_frames = int(math.ceil(sweep_distance / max(controller.search_speed * dt, 1e-12)))
    rendezvous_frames = int(math.ceil(rendezvous_distance / max(controller.search_speed * dt, 1e-12)))
    gossip_frames = int(math.ceil(controller.search_gossip_time / max(dt, 1e-12)))
    release_frames = sweep_frames + rendezvous_frames + gossip_frames
    relay_hops = per_side if per_side else 0
    relay_frames = relay_hops * int(controller.perception_every)

    witness_radius = certified_inscribed_radius(cargo.vertices) if is_simple_polygon(cargo.vertices) else 0.0
    checks: dict[str, Check] = {}

    def add(name: str, passed: bool, value, bound: str, rationale: str) -> None:
        checks[name] = Check(bool(passed), value, bound, rationale)

    add(
        "simple_polygon",
        is_simple_polygon(cargo.vertices),
        bool(is_simple_polygon(cargo.vertices)),
        "true",
        "A simple, non-degenerate outline is required for inside/outside and boundary ordering.",
    )
    add(
        "single_cargo_episode",
        len(config.get("cargoes", [])) == 1,
        len(config.get("cargoes", [])),
        "== 1",
        "The current search proof is a single-unknown-object theorem; occluding multiple cargoes are out of scope.",
    )
    add(
        "feature_witness",
        witness_radius + 1e-12 >= feature_required > 0.0,
        witness_radius,
        f">= {feature_required:.6g} m",
        "An ear-triangle incircle is a constructive disk contained in the polygon.",
    )
    max_perimeter = float(spec.get("max_perimeter", float("inf")))
    add(
        "perimeter_bound",
        cargo.perimeter <= max_perimeter + 1e-12,
        cargo.perimeter,
        f"<= {max_perimeter:.6g} m",
        "A fixed team cannot cover an unbounded boundary.",
    )
    diameter = polygon_diameter(cargo.vertices)
    max_diameter = float(spec.get("max_diameter", float("inf")))
    add(
        "diameter_bound",
        diameter <= max_diameter + 1e-12,
        diameter,
        f"<= {max_diameter:.6g} m",
        "The object and its cage must fit in the finite workspace.",
    )

    add(
        "paired_lane_layout",
        paired and str(config.get("agents", {}).get("layout")) == "paired_sweep",
        str(config.get("agents", {}).get("layout")),
        "paired_sweep with an even team",
        "Each half-workspace receives an independent lane cover.",
    )
    add(
        "paired_lane_controller",
        controller.search_pattern == "paired_lanes" and controller.map_gossip,
        f"{controller.search_pattern}, gossip={controller.map_gossip}",
        "paired_lanes and map_gossip=true",
        "The proof applies to the predetermined sweep/rendezvous protocol.",
    )
    add(
        "finite_ray_detection_tube",
        configured_tube <= r_detect + 1e-12,
        r_detect,
        f">= configured tube {configured_tube:.6g} m",
        "Finite angular ray spacing is included; this is not an ideal disk-sensor assumption.",
    )
    add(
        "vertical_workspace_cover",
        0.5 * lane_spacing <= configured_tube + 1e-12,
        0.5 * lane_spacing,
        f"<= tube {configured_tube:.6g} m",
        "Every point is within one sensor tube of a lane on its half-workspace.",
    )
    add(
        "outer_edge_cover",
        edge_padding <= configured_tube + 1e-12,
        edge_padding,
        f"<= tube {configured_tube:.6g} m",
        "The sensor tubes reach the complete outer workspace boundary.",
    )
    add(
        "rendezvous_connectivity",
        lane_spacing <= controller.comm_range + 1e-12
        and controller.search_meeting_gap <= controller.comm_range + 1e-12,
        max(lane_spacing, controller.search_meeting_gap),
        f"<= comm_range {controller.comm_range:.6g} m",
        "At rendezvous the two lane chains form one connected ladder graph.",
    )
    add(
        "gossip_dissemination_time",
        gossip_frames >= relay_frames,
        gossip_frames,
        f">= {relay_frames} frames ({relay_hops} hops at perception_every={controller.perception_every})",
        "Flooding a finite map over a connected graph takes at most its hop diameter.",
    )
    map_epsilon = float(spec.get("boundary_map_epsilon", 0.0))
    add(
        "boundary_map_resolution_declared",
        map_epsilon > 0.0,
        map_epsilon,
        "> 0 m",
        "The post-search theorem is conditional on an epsilon-dense observed boundary map.",
    )

    contact_radius = float(evaluation.get("contact_radius", 0.0))
    per_agent_arc = 2.0 * max(0.0, contact_radius - controller.cage_offset)
    required_agents = math.inf if per_agent_arc <= 0.0 else int(math.ceil(cargo.perimeter / per_agent_arc))
    add(
        "boundary_covering_number",
        required_agents <= count,
        required_agents if math.isfinite(required_agents) else "infinite",
        f"<= {count} agents",
        "Arclength plus triangle inequality gives a conservative continuous-boundary cover.",
    )
    facing_clearance = minimum_facing_cage_clearance(cargo.vertices, controller.cage_offset)
    add(
        "cage_offset_self_clearance",
        facing_clearance >= controller.d_min - 1e-12,
        facing_clearance if math.isfinite(facing_clearance) else "no facing edge pair",
        f">= d_min {controller.d_min:.6g} m",
        "Robots assigned to mutually facing concavity walls must fit without violating inter-agent separation.",
    )

    goal = None if goal_direction is None else np.asarray(goal_direction, dtype=float).reshape(2)
    goal_ok = goal is not None and float(np.linalg.norm(goal)) > 1e-12
    if goal_ok:
        goal = goal / float(np.linalg.norm(goal))
    distance = float(random_goal.get("target_distance", task.get("target_distance", 0.0)))
    target_reserve = float(spec.get("transport_target_reserve_m", 0.0))
    add(
        "transport_distance_consistency",
        distance > 0.0
        and abs(controller.transport_distance - distance - target_reserve) <= 1e-12,
        controller.transport_distance,
        f"== task distance {distance:.6g} m + declared reserve {target_reserve:.6g} m",
        "The local HOLD latch and swept corridor include only an explicitly declared estimator reserve.",
    )
    certified_distance = distance + max(0.0, target_reserve)
    shift = certified_distance * goal if goal_ok else np.zeros(2)
    cargo_margin = min(_domain_margin(cargo.vertices, domain), _domain_margin(cargo.vertices + shift, domain))
    add(
        "swept_cargo_corridor",
        goal_ok and cargo_margin >= -1e-12,
        cargo_margin,
        ">= 0 m",
        "A rectangle is convex, so start/end containment certifies every translated intermediate footprint.",
    )

    cage_offset = max(controller.cage_offset, controller.lead_offset or controller.cage_offset)
    cage_centers = _edge_offset_endpoints(cargo.vertices, cage_offset)
    cage_margin = min(_domain_margin(cage_centers, domain), _domain_margin(cage_centers + shift, domain))
    add(
        "swept_cage_corridor",
        goal_ok and cage_margin >= controller.robot_radius - 1e-12,
        cage_margin,
        f">= robot_radius {controller.robot_radius:.6g} m",
        "Every edge-offset cage centre and robot disk fits at both corridor endpoints.",
    )

    cooperative_need = int(math.ceil(contact.min_cooperating_robots(cargo.mass, controller.cage_offset)))
    add(
        "contact_force_capacity",
        cooperative_need <= controller.min_push_agents <= count,
        cooperative_need,
        f"<= min_push_agents {controller.min_push_agents} <= team {count}",
        "The locally gated pushing quorum exceeds Coulomb breakaway force at nominal cage penetration.",
    )
    force_margin = float(spec.get("force_margin", 1.05))
    wrench_ok, wrench_residual, wrench_agents = (
        _wrench_feasibility(cargo, goal, count, contact, controller, force_margin)
        if goal_ok
        else (False, float("inf"), 0.0)
    )
    add(
        "goal_wrench_feasibility",
        wrench_ok,
        {"residual": wrench_residual, "equivalent_agents": wrench_agents},
        f"zero-torque goal wrench >= {force_margin:.3g} x breakaway force",
        "A nonnegative edge-contact force allocation must produce the requested force with zero net torque.",
    )

    error_spec = spec.get("bounded_errors", {}) or {}
    normal_error = float(error_spec.get("normal_error_deg", -1.0))
    velocity_error = float(error_spec.get("velocity_error", -1.0))
    add(
        "bounded_perception_and_motion_error",
        0.0 <= normal_error < 90.0 and 0.0 <= velocity_error <= controller.rho + 1e-12,
        {"normal_error_deg": normal_error, "velocity_error": velocity_error},
        f"normal < 90 deg and velocity_error <= rho {controller.rho:.6g} m/s",
        "The conditional safety proof requires declared finite normal and moving-boundary estimation errors.",
    )

    finite_time_spec = spec.get("finite_time")
    derived_time_bound = None
    if isinstance(finite_time_spec, dict):
        derived_time_bound = derive_conditional_finite_time_bound(
            dt=dt,
            search_bound_s=sweep_frames * dt,
            # A conservative serial accounting: rendezvous, map gossip and
            # local boundary completion are not credited for overlap.
            map_bound_s=(rendezvous_frames + gossip_frames) * dt
            + float(controller.boundary_mapping_time),
            enclosure_initial_error_m=float(
                finite_time_spec.get("enclosure_initial_error_m", 0.0)
            ),
            enclosure_terminal_error_m=float(
                finite_time_spec.get("enclosure_terminal_error_m", 0.0)
            ),
            enclosure_contraction_rate_hz=float(
                finite_time_spec.get("enclosure_contraction_rate_hz", 0.0)
            ),
            transport_distance_m=distance,
            brake_activation_distance_m=controller.brake_activation_distance,
            transport_progress_rate_mps=float(
                finite_time_spec.get("transport_progress_rate_mps", 0.0)
            ),
            brake_initial_error_m=float(
                finite_time_spec.get(
                    "brake_initial_error_m",
                    controller.brake_activation_distance,
                )
            ),
            brake_terminal_error_m=float(
                finite_time_spec.get(
                    "brake_terminal_error_m",
                    controller.brake_position_tolerance,
                )
            ),
            brake_contraction_rate_hz=float(
                finite_time_spec.get("brake_contraction_rate_hz", 0.0)
            ),
            hold_dwell_s=float(
                finite_time_spec.get(
                    "hold_dwell_s",
                    controller.brake_dwell_steps * dt,
                )
            ),
        )
        add(
            "derived_conditional_finite_time_bound",
            bool(derived_time_bound["eligible"]),
            derived_time_bound.get("total_bound_frames"),
            "finite under declared contraction/progress premises",
            "Analytic sufficient bound; never inferred from successful episode durations.",
        )

    frame_budget = int(evaluation.get("frame_budget", 0) or 0)
    enclosure_bound = int(spec.get("enclosure_bound_frames", 0) or 0)
    transport_bound = int(spec.get("transport_bound_frames", 0) or 0)
    hold_bound = int(spec.get("hold_bound_frames", 0) or 0)
    total_bound = release_frames + enclosure_bound + transport_bound + hold_bound
    add(
        "declared_finite_time_bounds",
        enclosure_bound > 0 and transport_bound > 0 and hold_bound >= 0,
        {
            "search_release": release_frames,
            "enclosure": enclosure_bound,
            "transport": transport_bound,
            "hold": hold_bound,
        },
        "positive phase bounds",
        "These are explicit theorem premises, not values inferred from successful trials.",
    )
    add(
        "frame_budget",
        frame_budget > 0 and total_bound <= frame_budget,
        total_bound,
        f"<= {frame_budget} frames",
        "The sum of the certified/assumed phase bounds must fit the requested horizon.",
    )

    time_check_names = {
        "declared_finite_time_bounds",
        "frame_budget",
        "derived_conditional_finite_time_bound",
    }
    domain_checks = {
        name: check for name, check in checks.items() if name not in time_check_names
    }
    time_checks = {
        name: check for name, check in checks.items() if name in time_check_names
    }
    enabled = bool(spec.get("enabled", False))
    domain_eligible = enabled and all(check.passed for check in domain_checks.values())
    finite_time_eligible = enabled and bool(time_checks) and all(
        check.passed for check in time_checks.values()
    )
    eligible = domain_eligible and finite_time_eligible
    return {
        "theorem_id": THEOREM_ID,
        "enabled": enabled,
        "eligible": eligible,
        "domain_eligible": domain_eligible,
        "finite_time_eligible": finite_time_eligible,
        "claim": (
            "complete rectangular-workspace discovery plus conditional enclosure and bounded transport "
            "for this admissible simple polygon"
        ),
        "search": {
            "guaranteed_detection_radius": r_detect,
            "configured_detection_tube": configured_tube,
            "lane_spacing": lane_spacing,
            "sweep_bound_frames": sweep_frames,
            "rendezvous_bound_frames": rendezvous_frames,
            "gossip_bound_frames": gossip_frames,
            "release_bound_frames": release_frames,
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
        "task": {
            "transport_distance": distance,
            "transport_target_reserve": target_reserve,
            "certified_corridor_distance": certified_distance,
            "cargo_corridor_margin": cargo_margin,
            "cage_corridor_margin": cage_margin,
            "cooperative_agents_required": cooperative_need,
            "wrench_residual": wrench_residual,
            "wrench_equivalent_agents": wrench_agents,
            "total_bound_frames": total_bound,
        },
        "time_bounds": {
            "first_detection": sweep_frames,
            "first_enclosure": release_frames + enclosure_bound,
            "first_transport": release_frames + enclosure_bound + transport_bound,
            "first_hold": total_bound,
        },
        "derived_finite_time_bound": derived_time_bound,
        "checks": {name: check.as_dict() for name, check in checks.items()},
        "failure_reasons": [name for name, check in checks.items() if not check.passed],
        "domain_failure_reasons": [
            name for name, check in domain_checks.items() if not check.passed
        ],
        "finite_time_failure_reasons": [
            name for name, check in time_checks.items() if not check.passed
        ],
    }


__all__ = [
    "THEOREM_ID",
    "FINITE_TIME_BOUND_ID",
    "Check",
    "boundary_map_gap_upper_bound",
    "guaranteed_detection_radius",
    "minimum_facing_cage_clearance",
    "derive_conditional_finite_time_bound",
    "build_admissibility_certificate",
]
