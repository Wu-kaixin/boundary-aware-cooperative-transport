"""Paired Gate 5/6 experiments with isolated control and truth-only evaluation.

Gate 5: oracle vs locally sensed boundary, no true-geometry veto in local mode.
Gate 6: boundary CVT vs nearest observed offset target, before CONTACT_READY only.
All other controller, sensing, safety, initial-state and task settings are paired.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import numpy as np
import scipy
import yaml

from dbact.geometry import sample_polygon_boundary
from dbact.phase import Phase
from dbact.static_deployment_diagnostics import UniformOffsetObserver
from dbact.theorem_mode import TheoremModeAbort, hold_segment_object_clearance
from dbact_sim.environment import SimulationEnvironment


def encode(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def dump(path, value):
    def finite(item):
        if isinstance(item, dict):
            return {key: finite(val) for key, val in item.items()}
        if isinstance(item, (list, tuple)):
            return [finite(val) for val in item]
        if isinstance(item, (float, np.floating)) and not np.isfinite(item):
            return None  # Undefined direction/no observations, never a zero.
        return item
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(finite(value), indent=2, allow_nan=False), encoding="utf-8")


class NearestObservedDeployment:
    """Change only the pre-contact deployment target; preserve all other layers."""

    def __init__(self, controller):
        self.controller = controller
        self.base = controller.cvt

    def compute(self, i, agents, neighbors, density, domain):
        cell = self.base.compute(i, agents, neighbors, density, domain)
        if self.controller.phase_monitor.reached(Phase.CONTACT_READY) or not len(density.points):
            return cell
        targets = density.points + density.offsets[:, None] * density.normals
        nearest = np.argmin(np.linalg.norm(targets - agents[i].position, axis=1))
        return replace(cell, centroid=targets[nearest].copy())


def config_for(gate, shape):
    path = (ROOT / f"configs/sim/theorem/static_{shape}_n16_oracle.yaml" if gate == 5
            else ROOT / "configs/sim/d/l_shape_closed_loop.yaml")
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    if gate == 5:
        cfg["controller"]["grid_resolution"] = 20
    else:
        geometry = yaml.safe_load((ROOT / f"configs/sim/theorem/static_{shape}_n16_oracle.yaml").read_text(encoding="utf-8"))["cargoes"][0]
        geometry["movable"] = True
        geometry["surface_density"] = cfg["cargoes"][0].get("surface_density", 2.0)
        cfg["cargoes"] = [geometry]
    return cfg


def static_arm(cfg, seed, variant, frames, out):
    cfg["controller"]["theorem_map_source"] = variant
    env = SimulationEnvironment(cfg, seed=seed)
    params = env.controller.params
    vertices = env.cargoes[0].vertices.copy()
    boundary, _ = sample_polygon_boundary(vertices, count=256)
    observer = UniformOffsetObserver(env.domain, params.local_radius, params.sigma,
                                     params.base_density, params.cage_offset, ntheta=64, nradial=8)
    initial = np.vstack([a.position for a in env.agents])
    rows, positions, costs = [], [initial.copy()], []
    abort = None
    for frame in range(frames):
        p = np.vstack([a.position for a in env.agents])
        try:
            commands = env.controller.step(env.agents, env.cargoes, frame * env.dt, env.dt)
        except TheoremModeAbort as exc:
            abort = exc.as_dict()
            break
        rec = env.controller.last_theorem_record
        # Observe after command commitment. This value never changes execution.
        clearance, sampled, _, _ = hold_segment_object_clearance(p, rec.u_command, [vertices], env.dt)
        pooled = [v.points for v in env.controller._views.values() if len(v)]
        coverage = 0.0
        if pooled:
            from scipy.spatial import cKDTree
            distance, _ = cKDTree(np.vstack(pooled)).query(boundary)
            coverage = float(np.mean(distance <= 0.08))
        rows.append({"frame": frame, "true_hold_clearance_lower_bound": float(clearance),
                     "true_hold_clearance_sampled": float(sampled),
                     "safety_margin": float(clearance - params.r_safe),
                     "min_pair_distance": rec.hold_min_pair_distance,
                     "union_map_coverage_at_0_08m": coverage,
                     "qp_intervention_fraction": float(np.mean([r.modification > 1e-8 for r in env.controller._last_filter_results])),
                     "empty_map_holds": sum(m == "local_empty_map_hold" for m in rec.modes),
                     "solver_status": rec.solver_status})
        if frame % 10 == 0:
            measured = observer.evaluate(p, vertices)
            costs.append({"frame": frame, "H": measured["H"], "centroid_residual_squared": measured["centroid2"]})
        env.controller.apply_commands(env.agents, commands, env.dt)
        positions.append(np.vstack([a.position for a in env.agents]))
    final = observer.evaluate(positions[-1], vertices)
    result = {"variant": variant, "seed": seed, "frames_completed": len(rows), "frames_requested": frames,
              "abort": abort, "initial_state_sha256": encode(initial.tolist()), "config_sha256": encode(cfg),
              "H_initial": observer.evaluate(initial, vertices)["H"], "H_final": final["H"],
              "centroid_residual_squared_final": final["centroid2"],
              "minimum_true_clearance": min((r["true_hold_clearance_lower_bound"] for r in rows), default=None),
              "minimum_safety_margin": min((r["safety_margin"] for r in rows), default=None),
              "minimum_pair_distance": min((r["min_pair_distance"] for r in rows), default=None),
              "final_union_map_coverage": rows[-1]["union_map_coverage_at_0_08m"] if rows else 0,
              "qp_intervention_fraction": float(np.mean([r["qp_intervention_fraction"] for r in rows])) if rows else None,
              "object_safety_violation_frames": sum(r["safety_margin"] < -1e-9 for r in rows),
              "label": "empirical local-boundary comparison; no local-map object-safety theorem",
              "observer": {"angular_nodes": 64, "radial_nodes": 8, "cost_stride": 10, "safety_stride": 1}}
    dump(out / "config.json", cfg)
    dump(out / "metrics.json", {"steps": rows, "cost": costs})
    dump(out / "summary.json", result)
    np.savez_compressed(out / "trajectory.npz", positions=positions, vertices=vertices)
    return result


def transport_arm(cfg, seed, variant, frames, out):
    env = SimulationEnvironment(cfg, seed=seed)
    if variant == "nearest_observed_target":
        env.controller.cvt = NearestObservedDeployment(env.controller)
    initial = np.vstack([a.position for a in env.agents])
    cid = env.cargoes[0].object_id
    goal = env.goal_directions[cid]
    hist = np.zeros(12, dtype=int)
    rows = []

    def observe(frame, current):
        if frame == 0:
            return
        cargo = current.cargoes[0]
        report = current.engine.last_reports[cid]
        fmax = current.contact_params.stiffness * current.controller.params.delta_max
        max_forward, torque_positive, torque_negative = 0.0, 0.0, 0.0
        for contact in report.contacts:
            relative = contact.point - cargo.position
            angle = float(np.arctan2(relative[1], relative[0]) % (2 * np.pi))
            hist[min(11, int(angle * 12 / (2 * np.pi)))] += 1
            direction = -contact.normal
            max_forward += fmax * max(0.0, float(direction @ goal))
            tau = fmax * float(relative[0] * direction[1] - relative[1] * direction[0])
            torque_positive += max(tau, 0.0)
            torque_negative += min(tau, 0.0)
        rows.append({"frame": frame, "contacts": report.contact_count,
                     "normal_only_forward_capacity": max_forward,
                     "normal_only_torque_interval": [torque_negative, torque_positive],
                     "actual_net_force": report.net_force.tolist(), "actual_net_torque": report.net_torque,
                     "qp_intervention_fraction": float(np.mean([d.modification > 1e-8 for d in current.controller.diagnostics]))})

    termination = env.run_until_settled(max_frames=frames, on_frame=observe)
    summary = env.save_outputs(out)
    cargo = summary["cargoes"][cid]
    gate = cargo["g500"]
    metrics = gate["metrics"]
    force = np.asarray([r["actual_net_force"] for r in rows])
    result = {"variant": variant, "seed": seed, "initial_state_sha256": encode(initial.tolist()),
              "config_sha256": encode(cfg), "goal_direction": goal.tolist(),
              "task": summary["tasks"], "termination": termination,
              "contact_ready_frame": metrics["contact_ready_frame"],
              "contact_ready_seconds": None if metrics["contact_ready_frame"] is None else metrics["contact_ready_frame"] * env.dt,
              "mean_contacts": cargo["mean_contacts"], "max_contacts": cargo["max_contacts"],
              "contact_azimuth_histogram_12_bins": hist.tolist(),
              "peak_normal_only_forward_capacity": max(r["normal_only_forward_capacity"] for r in rows),
              "torque_capacity_min": min(r["normal_only_torque_interval"][0] for r in rows),
              "torque_capacity_max": max(r["normal_only_torque_interval"][1] for r in rows),
              "peak_actual_net_force": float(np.linalg.norm(force, axis=1).max()),
              "peak_absolute_actual_torque": max(abs(r["actual_net_torque"]) for r in rows),
              "direction_error_deg": metrics["direction_error_deg"], "max_cross_track": metrics["max_cross_track"],
              "final_distance_error": float(metrics["J"] - cargo["task"]["target_distance"]),
              "qp_intervention_fraction": float(np.mean([r["qp_intervention_fraction"] for r in rows])),
              "min_inter_agent_distance": metrics["min_inter_agent_distance"],
              "min_signed_clearance": metrics["min_signed_clearance"],
              "g500_success": gate["success"], "failure_reasons": gate["failure_reasons"],
              "capacity_label": "normal-only instantaneous contact wrench box, 0 <= f_i <= stiffness*delta_max; not dynamically guaranteed",
              "changed_mechanism": "Only centroid target before CONTACT_READY; CVT mass, redeployment guards, sensing, safety and subsequent transport identical"}
    dump(out / "paired_metrics.json", {"steps": rows, "summary": result})
    dump(out / "config.json", cfg)
    return result


def run_pair(payload):
    from dbact.cpu_budget import limit_blas_threads
    limit_blas_threads(1)
    os.environ["DBACT_CVT_WORKERS"] = "1"
    gate, shape, seed, frames, out = payload
    variants = ("oracle", "local") if gate == 5 else ("boundary_cvt", "nearest_observed_target")
    results = []
    for variant in variants:
        cfg = config_for(gate, shape)
        folder = Path(out) / shape / f"seed_{seed}" / variant
        folder.mkdir(parents=True, exist_ok=True)
        fn = static_arm if gate == 5 else transport_arm
        result = fn(cfg, seed, variant, frames, folder)
        result["shape"] = shape
        results.append(result)
    assert results[0]["initial_state_sha256"] == results[1]["initial_state_sha256"]
    if gate == 6:
        assert results[0]["config_sha256"] == results[1]["config_sha256"]
        assert results[0]["task"] == results[1]["task"]
        assert results[0]["goal_direction"] == results[1]["goal_direction"]
    dump(Path(out) / shape / f"seed_{seed}" / "pair.json", results)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gate", type=int, choices=(5, 6), required=True)
    parser.add_argument("--shapes", nargs="+", default=["l_shape", "rectangle", "c_shape"])
    parser.add_argument("--seeds", nargs="+", type=int, default=[2, 5, 8])
    parser.add_argument("--frames", type=int, default=None)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    frames = args.frames or (600 if args.gate == 5 else 3000)
    manifest = {"commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
                "python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__,
                "gate": args.gate, "shapes": args.shapes, "seeds": args.seeds, "frames": frames,
                "source_sha256": {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in [*sorted((ROOT / "src").rglob("*.py")), Path(__file__).resolve()]}}
    dump(args.out / "manifest.json", manifest)
    jobs = [(args.gate, shape, seed, frames, str(args.out)) for shape in args.shapes for seed in args.seeds]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        pairs = list(pool.map(run_pair, jobs))
    dump(args.out / "summary.json", {"manifest": manifest, "pairs": pairs})
    print(json.dumps({"gate": args.gate, "pairs": len(pairs), "out": str(args.out)}), flush=True)


if __name__ == "__main__":
    main()
