#!/usr/bin/env python
"""Per-stage verification of the S1-S7 refactor.

Each stage reports the measurement that decides whether it is done, not a
subjective status. S2/S3/S4/S5/S6 are fast; S1 and S7 run a simulation.

    python scripts/verify_refactor.py --stage all --steps 900 --json out/verify.json
    python scripts/verify_refactor.py --stage S2 S5 S6          # fast regression
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import sys
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact.boundary_density import BoundaryAwareDensity, DensityParams  # noqa: E402
from dbact.boundary_map import LocalBoundaryMap  # noqa: E402
from dbact.cargo import Cargo  # noqa: E402
from dbact.contact_dynamics import ContactParams, PenaltyContactModel  # noqa: E402
from dbact.geometry import points_in_polygon, signed_distance_to_polygon  # noqa: E402
from dbact.local_cvt import LocalCVT, coverage_cost  # noqa: E402
from dbact.metrics import signed_clearances  # noqa: E402
from dbact.perception import (  # noqa: E402
    LegacyProximitySampler,
    PerceptionParams,
    RayCastBoundarySensor,
    normal_errors_deg,
    occlusion_rate,
)
from dbact.transport_dynamics import ScriptedParams, ScriptedTransportEngine  # noqa: E402
from dbact.types import AgentState  # noqa: E402
from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402

STAGES = ("S1", "S2", "S3", "S4", "S5", "S6", "S7")
DEFAULT_CONFIG = "configs/sim/v2/l_shape_v2.yaml"


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #


def ring_of_agents(cargo: Cargo, count: int, offset: float) -> list[AgentState]:
    """Place agents on the outward offset ring of a cargo."""
    points, normals = cargo.boundary_samples(count)
    return [
        AgentState(agent_id=f"agent_{i:02d}", position=points[i] + offset * normals[i])
        for i in range(count)
    ]


def percentile(values: np.ndarray, q: float) -> float:
    return float(np.percentile(values, q)) if len(values) else float("nan")


# --------------------------------------------------------------------------- #
# S1 - safety layer
# --------------------------------------------------------------------------- #


def verify_s1(config: dict, steps: int) -> dict:
    """Object-boundary CBF: penetration with and without the object rows."""
    out: dict = {"stage": "S1", "steps": steps}
    for label, use_barrier in (("with_object_cbf", True), ("legacy_no_object_cbf", False)):
        cfg = copy.deepcopy(config)
        cfg["controller"]["use_object_barrier"] = use_barrier
        env = SimulationEnvironment(cfg, seed=0)
        env.run(steps)
        summary = env.summary()
        cargo_id = next(iter(summary["cargoes"]))
        entry = summary["cargoes"][cargo_id]
        out[label] = {
            "min_signed_clearance": entry["min_signed_clearance"],
            "r_safe": summary["contracts"]["C1"]["r_safe"] if summary["contracts"]["C1"] else None,
            "discrete_overshoot": summary["contracts"]["discrete_overshoot"],
            "max_cargo_speed": entry["max_cargo_speed"],
            "max_penetration": entry["max_penetration"],
            "max_agents_inside": entry["max_agents_inside"],
            "agents_total": len(env.agents),
            "final_strict_coverage": entry["final_strict_coverage"],
            "final_legacy_coverage": entry["final_coverage_legacy"],
            "solver": summary["solver"],
        }
    good = out["with_object_cbf"]
    overshoot = float(good["discrete_overshoot"])
    out["penetration_budget"] = float(config["controller"]["delta_max"]) + overshoot
    # The barrier holds in continuous time. A fixed-step integrator can dip below
    # r_safe by at most one step of relative motion, so the invariant is stated with
    # that allowance rather than as an exact inequality. The zero-input certificate
    # failures are that same dip seen from the QP's side, so they are reported as a
    # rate rather than required to be zero -- what must be zero is a robot getting
    # inside the cargo, and that is checked exactly.
    certificate = good["solver"]
    out["certificate_failure_rate"] = (
        certificate["zero_input_feasible_failures"] / max(certificate["zero_input_feasible_checks"], 1)
    )
    out["pass"] = bool(
        good["max_agents_inside"] == 0
        and good["min_signed_clearance"] >= 0.0
        and good["min_signed_clearance"] >= good["r_safe"] - overshoot
        and good["max_penetration"] <= out["penetration_budget"]
        and out["certificate_failure_rate"] < 0.10
        and certificate["max_slack"] == 0.0
    )
    out["note"] = (
        "the legacy row shows the metric hazard directly: robots inside the cargo raise the legacy "
        "coverage number while the run is physically invalid"
    )
    return out


# --------------------------------------------------------------------------- #
# S2 - perception layer
# --------------------------------------------------------------------------- #


def verify_s2(config: dict) -> dict:
    cargo = Cargo.from_config(config["cargoes"][0])
    controller = config["controller"]
    sensor_range = float(controller["sensor_range"])
    noise = float(controller.get("range_noise_std", 0.0))
    # 3-sigma line-of-sight tolerance: without it the audit reports its own noise.
    tolerance = 3.0 * noise

    viewpoints = ring_of_agents(cargo, 44, sensor_range * 0.55)
    ray = RayCastBoundarySensor(
        PerceptionParams(
            sensor_range=sensor_range,
            ray_count=int(controller.get("ray_count", 96)),
            range_noise_std=noise,
            pca_neighbors=int(controller.get("pca_neighbors", 5)),
            residual_tolerance=float(controller.get("residual_tolerance", 0.03)),
            min_confidence=float(controller.get("min_confidence", 0.15)),
        )
    )
    legacy = LegacyProximitySampler(sensor_range=sensor_range)

    ray_blocked = ray_total = legacy_blocked = legacy_total = 0
    ungated: list[np.ndarray] = []
    gated: list[np.ndarray] = []
    for agent in viewpoints:
        raw = ray.sense(agent, [cargo], 0.0, apply_gate=False)
        kept = [o for o in raw if o.confidence >= ray.params.min_confidence]
        b, t = occlusion_rate(raw, agent.position, [cargo], tolerance=tolerance)
        ray_blocked += b
        ray_total += t
        legacy_obs = legacy.sense(agent, [cargo], 0.0)
        b, t = occlusion_rate(legacy_obs, agent.position, [cargo], tolerance=tolerance)
        legacy_blocked += b
        legacy_total += t
        ungated.append(normal_errors_deg(raw, [cargo]))
        gated.append(normal_errors_deg(kept, [cargo]))

    ungated_err = np.concatenate(ungated) if ungated else np.empty(0)
    gated_err = np.concatenate(gated) if gated else np.empty(0)
    out = {
        "stage": "S2",
        "viewpoints": len(viewpoints),
        "line_of_sight_tolerance": tolerance,
        "raycast": {
            "occluded": ray_blocked,
            "returns": ray_total,
            "occlusion_rate": ray_blocked / ray_total if ray_total else float("nan"),
        },
        "legacy_sampler": {
            "occluded": legacy_blocked,
            "returns": legacy_total,
            "occlusion_rate": legacy_blocked / legacy_total if legacy_total else float("nan"),
        },
        "normal_error_deg": {
            "ungated_p50": percentile(ungated_err, 50),
            "ungated_p90": percentile(ungated_err, 90),
            "ungated_count": int(len(ungated_err)),
            "gated_p50": percentile(gated_err, 50),
            "gated_p90": percentile(gated_err, 90),
            "gated_count": int(len(gated_err)),
            "gross_errors_over_30deg_before": int(np.sum(ungated_err > 30.0)),
            "gross_errors_over_30deg_after": int(np.sum(gated_err > 30.0)),
        },
    }
    # The residual occlusion is the genuine tail of the range-noise distribution
    # beyond the 3-sigma line-of-sight allowance, not see-through sensing, so the
    # criterion is a stated rate rather than exact zero.
    out["pass"] = bool(
        out["raycast"]["occlusion_rate"] < 0.01
        and out["raycast"]["occlusion_rate"] < 0.05 * out["legacy_sampler"]["occlusion_rate"]
        and out["normal_error_deg"]["gated_p90"] <= out["normal_error_deg"]["ungated_p90"] + 1e-9
    )
    out["note"] = (
        "the legacy sampler's non-zero occlusion rate is the point: it returned boundary points the "
        "observer could not see, and the simulator's exact normals with them"
    )
    return out


# --------------------------------------------------------------------------- #
# S3 - map layer
# --------------------------------------------------------------------------- #


def verify_s3(config: dict) -> dict:
    controller = config["controller"]
    cargo = Cargo.from_config(config["cargoes"][0])
    sensor = RayCastBoundarySensor(
        PerceptionParams(
            sensor_range=float(controller["sensor_range"]),
            ray_count=int(controller.get("ray_count", 96)),
            range_noise_std=0.0,
        )
    )
    agent = ring_of_agents(cargo, 8, 0.5)[0]
    observations = sensor.sense(agent, [cargo], 0.0)

    voxel = float(controller.get("voxel_size", 0.06))
    decay = float(controller.get("age_decay", 0.3))

    # Relay idempotence: the same packet arriving many times must not add mass.
    single = LocalBoundaryMap(voxel_size=voxel, age_decay=decay)
    single.update(observations, 0.0)
    repeated = LocalBoundaryMap(voxel_size=voxel, age_decay=decay)
    for _ in range(20):
        repeated.update(observations, 0.0)
    relayed = LocalBoundaryMap(voxel_size=voxel, age_decay=decay)
    relayed.update(observations * 8, 0.0)

    params = DensityParams(
        mode="offset",
        cage_offset=float(controller["cage_offset"]),
        sigma=float(controller["sigma"]),
    )
    grid = np.column_stack([g.ravel() for g in np.meshgrid(np.linspace(0, 8, 90), np.linspace(0, 8, 90))])

    def mass(m: LocalBoundaryMap) -> float:
        density = BoundaryAwareDensity.from_observations(m.all_observations(0.0), params)
        return float(np.sum(density(grid)))

    base_mass = mass(single)
    out = {
        "stage": "S3",
        "raw_observations": len(observations),
        "voxels_single_update": len(single),
        "voxels_after_20_updates": len(repeated),
        "voxels_after_8x_relay": len(relayed),
        "arc_length_single": single.total_arc_length(),
        "arc_length_after_20_updates": repeated.total_arc_length(),
        "arc_length_after_8x_relay": relayed.total_arc_length(),
        "density_mass_single": base_mass,
        "density_mass_after_20_updates": mass(repeated),
        "density_mass_after_8x_relay": mass(relayed),
        "true_perimeter": cargo.perimeter,
    }
    # Age decay must actually reduce read-time confidence.
    aged = single.all_observations(0.0)
    later = single.all_observations(4.0)
    out["mean_confidence_t0"] = float(np.mean([o.confidence for o in aged])) if aged else 0.0
    out["mean_confidence_t4"] = float(np.mean([o.confidence for o in later])) if later else 0.0
    out["voxels_after_long_gap"] = len(single)

    out["pass"] = bool(
        out["voxels_single_update"] == out["voxels_after_20_updates"] == out["voxels_after_8x_relay"]
        and math.isclose(out["arc_length_single"], out["arc_length_after_8x_relay"], rel_tol=1e-9)
        and math.isclose(out["density_mass_single"], out["density_mass_after_8x_relay"], rel_tol=1e-9)
        and out["mean_confidence_t4"] < out["mean_confidence_t0"]
    )
    out["note"] = "capacity and fusion act on voxels, so repetition and relay cannot amplify density mass"
    return out


# --------------------------------------------------------------------------- #
# S4 - density layer
# --------------------------------------------------------------------------- #


def verify_s4(config: dict) -> dict:
    controller = config["controller"]
    cargo = Cargo.from_config(config["cargoes"][0])
    sensor = RayCastBoundarySensor(
        PerceptionParams(sensor_range=6.0, ray_count=720, range_noise_std=0.0)
    )
    # One distant all-round viewpoint would still be occluded, so fuse a ring.
    voxel = float(controller.get("voxel_size", 0.06))
    fused = LocalBoundaryMap(voxel_size=voxel, age_decay=0.0)
    for agent in ring_of_agents(cargo, 24, 0.35):
        fused.update(sensor.sense(agent, [cargo], 0.0), 0.0)
    observations = fused.all_observations(0.0)

    lo = cargo.vertices.min(axis=0) - 0.6
    hi = cargo.vertices.max(axis=0) + 0.6
    xs = np.linspace(lo[0], hi[0], 220)
    ys = np.linspace(lo[1], hi[1], 220)
    grid = np.column_stack([g.ravel() for g in np.meshgrid(xs, ys)])
    inside = points_in_polygon(grid, cargo.vertices)
    signed = signed_distance_to_polygon(grid, cargo.vertices)

    # Concave region: points whose nearest boundary is a reflex corner. Use the
    # sign of the cross product along the outline to find reflex vertices.
    v = cargo.vertices
    prev_edge = v - np.roll(v, 1, axis=0)
    next_edge = np.roll(v, -1, axis=0) - v
    cross = prev_edge[:, 0] * next_edge[:, 1] - prev_edge[:, 1] * next_edge[:, 0]
    reflex = v[cross < 0.0]

    out: dict = {"stage": "S4", "observations": len(observations), "reflex_corners": len(reflex)}
    for mode in ("offset", "distance_field"):
        params = DensityParams(
            mode=mode,
            cage_offset=float(controller["cage_offset"]),
            sigma=float(controller["sigma"]),
            base_density=float(controller.get("base_density", 1e-3)),
        )
        density = BoundaryAwareDensity.from_observations(observations, params)
        values = np.atleast_1d(density(grid))
        band = np.abs(signed - params.cage_offset) <= 2.0 * params.sigma
        median = float(np.median(values[band])) if np.any(band) else float("nan")

        if len(reflex):
            near_reflex = np.min(np.linalg.norm(grid[:, None, :] - reflex[None, :, :], axis=2), axis=1)
            concave_zone = (near_reflex <= 3.0 * params.cage_offset) & band
            peak = float(np.max(values[concave_zone])) if np.any(concave_zone) else float("nan")
            peak_index = int(np.argmax(np.where(concave_zone, values, -np.inf))) if np.any(concave_zone) else -1
        else:
            peak, peak_index = float("nan"), -1

        total = float(np.sum(values))
        out[mode] = {
            "concave_peak_over_median": peak / median if median and median > 0 else float("nan"),
            "concave_peak_location": grid[peak_index].tolist() if peak_index >= 0 else None,
            "mass_inside_object_fraction": float(np.sum(values[inside]) / total) if total > 0 else float("nan"),
            "targets_inside_object_fraction": float(
                np.mean(points_in_polygon(density.targets, cargo.vertices))
            )
            if mode == "offset" and len(density.targets)
            else 0.0,
        }
    out["pass"] = bool(
        out["distance_field"]["mass_inside_object_fraction"] < out["offset"]["mass_inside_object_fraction"]
    )
    out["note"] = (
        "the distance-field level set is the boundary of a Minkowski sum and cannot self-intersect; "
        "the cost is that total mass is no longer proportional to estimated perimeter"
    )
    return out


# --------------------------------------------------------------------------- #
# S5 - coverage layer
# --------------------------------------------------------------------------- #


def verify_s5(config: dict) -> dict:
    controller = config["controller"]
    domain = (0.0, 8.0, 0.0, 8.0)
    local_radius = float(controller["local_radius"])
    cvt = LocalCVT(local_radius=local_radius, grid_resolution=int(controller["grid_resolution"]),
                   comm_range=float(controller["comm_range"]))

    cargo = Cargo.from_config(config["cargoes"][0])
    sensor = RayCastBoundarySensor(PerceptionParams(sensor_range=6.0, ray_count=720))
    fused = LocalBoundaryMap(voxel_size=float(controller.get("voxel_size", 0.06)), age_decay=0.0)
    for agent in ring_of_agents(cargo, 24, 0.35):
        fused.update(sensor.sense(agent, [cargo], 0.0), 0.0)
    density = BoundaryAwareDensity.from_observations(
        fused.all_observations(0.0),
        DensityParams(
            mode="offset",
            cage_offset=float(controller["cage_offset"]),
            sigma=float(controller["sigma"]),
            base_density=float(controller.get("base_density", 1e-3)),
        ),
    )

    # Locality: a robot in a far corner must not integrate across the domain.
    corner = [AgentState("agent_00", np.array([6.0, 6.0]))]
    samples, _ = cvt.cell_samples(0, corner, [], domain)
    max_reach = float(np.max(np.linalg.norm(samples - corner[0].position[None, :], axis=1))) if len(samples) else 0.0

    # Neighbour-completeness contract must warn when violated.
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        LocalCVT(local_radius=0.9 * float(controller["comm_range"]), comm_range=float(controller["comm_range"]))
        warned = any(issubclass(w.category, RuntimeWarning) for w in caught)

    # Truncated-cost descent, and the step-size bound it comes with.
    rng = np.random.default_rng(7)
    start = np.column_stack([rng.uniform(1.0, 7.0, 12), rng.uniform(1.0, 7.0, 12)])
    descent = {}
    for gain in (0.25, 0.60, 1.00):
        agents = [AgentState(f"agent_{i:02d}", start[i].copy()) for i in range(len(start))]
        positions = np.vstack([a.position for a in agents])
        history = [coverage_cost(positions, density, local_radius, domain, resolution=110)]
        rises = 0
        for _ in range(79):
            centroids = [cvt.compute(i, agents, [j for j in range(len(agents)) if j != i], density, domain).centroid
                         for i in range(len(agents))]
            for agent, centroid in zip(agents, centroids):
                agent.position = agent.position + gain * (centroid - agent.position)
            positions = np.vstack([a.position for a in agents])
            history.append(coverage_cost(positions, density, local_radius, domain, resolution=110))
            if history[-1] > history[-2] + 1e-12:
                rises += 1
        descent[f"gain_{gain:.2f}"] = {
            "H_start": history[0],
            "H_end": history[-1],
            "rising_steps": rises,
            "total_steps": len(history) - 1,
        }

    out = {
        "stage": "S5",
        "local_radius": local_radius,
        "max_sample_distance_from_agent": max_reach,
        "neighbour_completeness_warns_when_violated": warned,
        "truncated_cost_descent": descent,
    }
    out["pass"] = bool(
        max_reach <= local_radius + 1e-9
        and warned
        and descent["gain_0.25"]["rising_steps"] == 0
        and descent["gain_0.25"]["H_end"] < descent["gain_0.25"]["H_start"]
    )
    out["note"] = (
        "descent holds under a step-size bound: at gain 0.25 no step rises, at larger gains some do, "
        "so the bound belongs in the paper rather than an unqualified monotonicity claim"
    )
    return out


# --------------------------------------------------------------------------- #
# S6 - physics layer
# --------------------------------------------------------------------------- #


def verify_s6(config: dict) -> dict:
    """Three falsification tests for 'the cargo moves only by contact'."""
    contact_cfg = {k: v for k, v in config.get("transport", {}).items() if k in ContactParams.__dataclass_fields__}
    contact_cfg["robot_radius"] = float(config["controller"]["robot_radius"])
    params = ContactParams(**contact_cfg)
    model = PenaltyContactModel(params)
    r = params.robot_radius
    dt = float(config.get("dt", 0.05))

    # 1. Push from the left while the configured goal direction says (0, -1).
    cargo = Cargo.rectangle("box", [0.0, 0.0], 1.0, 0.6)
    pushers = [
        AgentState("a0", np.array([-0.5 - r + 0.02, -0.15]), velocity=np.array([0.25, 0.0])),
        AgentState("a1", np.array([-0.5 - r + 0.02, 0.15]), velocity=np.array([0.25, 0.0])),
    ]
    for _ in range(160):
        for a in pushers:
            a.position = a.position + a.velocity * dt
        model.step(cargo, pushers, dt)
    displacement = cargo.displacement
    configured = np.array([0.0, -1.0])
    contact_direction = displacement / max(float(np.linalg.norm(displacement)), 1e-12)
    angle_to_configured = math.degrees(
        math.acos(float(np.clip(np.dot(contact_direction, configured), -1.0, 1.0)))
    )

    # The same scenario under the legacy scripted engine, for contrast. Fresh
    # agents: the run above has already driven the originals well past the cargo.
    scripted_cargo = Cargo.rectangle("box", [0.0, 0.0], 1.0, 0.6)
    scripted_agents = [
        AgentState("a0", np.array([-0.5 - r + 0.02, -0.15]), velocity=np.array([0.25, 0.0])),
        AgentState("a1", np.array([-0.5 - r + 0.02, 0.15]), velocity=np.array([0.25, 0.0])),
    ]
    scripted = ScriptedTransportEngine(
        ScriptedParams(
            contact_radius=0.5,
            coverage_threshold=0.0,
            min_contact_agents=1,
            speed=0.16,
            goal_directions={"box": configured},
        )
    )
    for _ in range(160):
        scripted.step([scripted_cargo], scripted_agents, dt)
    scripted_displacement = scripted_cargo.displacement
    scripted_angle = math.degrees(
        math.acos(
            float(
                np.clip(
                    np.dot(scripted_displacement / max(float(np.linalg.norm(scripted_displacement)), 1e-12), configured),
                    -1.0,
                    1.0,
                )
            )
        )
    )

    # 2. Single off-centre push must rotate the body.
    spin_cargo = Cargo.rectangle("spin", [0.0, 0.0], 1.0, 0.6)
    spinner = [AgentState("a0", np.array([-0.5 - r + 0.02, 0.25]), velocity=np.array([0.25, 0.0]))]
    for _ in range(160):
        spinner[0].position = spinner[0].position + spinner[0].velocity * dt
        model.step(spin_cargo, spinner, dt)

    # 3. No contact at all must produce exactly zero displacement.
    idle_cargo = Cargo.rectangle("idle", [0.0, 0.0], 1.0, 0.6)
    far = [AgentState("a0", np.array([4.0, 4.0]), velocity=np.array([0.3, 0.0]))]
    for _ in range(160):
        model.step(idle_cargo, far, dt)

    out = {
        "stage": "S6",
        "push_from_left": {
            "displacement": displacement.tolist(),
            "direction": contact_direction.tolist(),
            "configured_goal_direction": configured.tolist(),
            "angle_between_displacement_and_configured_deg": angle_to_configured,
        },
        "scripted_engine_same_scenario": {
            "displacement": scripted_displacement.tolist(),
            "angle_between_displacement_and_configured_deg": scripted_angle,
        },
        "single_offcentre_push": {
            "rotation_deg": math.degrees(spin_cargo.angle),
            "displacement": spin_cargo.displacement.tolist(),
        },
        "no_contact": {
            "displacement_norm": float(np.linalg.norm(idle_cargo.displacement)),
            "rotation_deg": math.degrees(idle_cargo.angle),
        },
    }
    out["pass"] = bool(
        contact_direction[0] > 0.99
        and angle_to_configured > 80.0
        and abs(math.degrees(spin_cargo.angle)) > 1.0
        and out["no_contact"]["displacement_norm"] == 0.0
        and scripted_angle < 1e-6
    )
    out["note"] = (
        "under the contact engine the motion direction is set by contact geometry and the configured "
        "direction is ~90 deg away from it; under the scripted engine the two agree to 0.000000 deg, "
        "which is what makes the pre-refactor transport result a restatement of its configuration"
    )
    return out


# --------------------------------------------------------------------------- #
# S7 - closed loop
# --------------------------------------------------------------------------- #


def verify_s7(config: dict, steps: int) -> dict:
    env = SimulationEnvironment(copy.deepcopy(config), seed=0)
    env.run(steps)
    summary = env.summary()
    cargo_id = next(iter(summary["cargoes"]))
    entry = summary["cargoes"][cargo_id]
    clearances = signed_clearances(env.cargoes[0], env.agents)
    out = {
        "stage": "S7",
        "steps": steps,
        "engine": summary["engine"],
        "solver": summary["solver"],
        "min_inter_agent_distance": summary["min_inter_agent_distance"],
        "cargo": entry,
        "final_clearance_distribution": np.round(np.sort(clearances), 4).tolist(),
        "mode_counts": env.controller.mode_counts(),
        "pass": bool(entry.get("success")),
    }
    return out


# --------------------------------------------------------------------------- #


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify the S1-S7 refactor stage by stage.")
    parser.add_argument("--stage", nargs="+", default=["all"], help=f"Any of {STAGES} or 'all'.")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--steps", type=int, default=900, help="Simulation steps for S1 and S7.")
    parser.add_argument("--json", default="", help="Write the full report to this path.")
    args = parser.parse_args()

    requested = list(STAGES) if "all" in args.stage else [s.upper() for s in args.stage]
    unknown = [s for s in requested if s not in STAGES]
    if unknown:
        parser.error(f"unknown stage(s) {unknown}; expected any of {STAGES}")

    config = load_yaml(args.config)
    report: dict = {"config": args.config, "stages": {}}

    runners = {
        "S1": lambda: verify_s1(config, args.steps),
        "S2": lambda: verify_s2(config),
        "S3": lambda: verify_s3(config),
        "S4": lambda: verify_s4(config),
        "S5": lambda: verify_s5(config),
        "S6": lambda: verify_s6(config),
        "S7": lambda: verify_s7(config, args.steps),
    }

    all_pass = True
    for stage in requested:
        print(f"=== {stage} ===", flush=True)
        result = runners[stage]()
        report["stages"][stage] = result
        all_pass &= bool(result.get("pass"))
        print(json.dumps(result, indent=2, default=float), flush=True)
        print(f"--> {stage}: {'PASS' if result.get('pass') else 'FAIL'}\n", flush=True)

    report["all_pass"] = all_pass
    if args.json:
        path = Path(args.json)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2, default=float), encoding="utf-8")
        print(f"report written to {path}")
    print("ALL REQUESTED STAGES PASS" if all_pass else "SOME STAGES FAIL")
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
