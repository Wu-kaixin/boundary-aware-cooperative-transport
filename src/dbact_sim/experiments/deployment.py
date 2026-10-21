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
import tempfile

from dbact_sim.experiments.paths import repository_root

ROOT = repository_root()
import numpy as np
import scipy
import yaml

from dbact.geometry import sample_polygon_boundary
from dbact.control.phase import Phase
from dbact.static_deployment_diagnostics import UniformOffsetObserver
from dbact.deployment.theorem import TheoremModeAbort, hold_segment_object_clearance
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.paper_evaluation import SCHEMA_VERSION, evaluate_static_hold, progress_errors, event_record


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
    payload = json.dumps(finite(value), indent=2, allow_nan=False)
    # Readers see either the previous receipt or a complete new one.
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                     prefix=path.name + ".", suffix=".tmp", delete=False) as stream:
        temporary = Path(stream.name)
        stream.write(payload)
    try:
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


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
    first_violation = None
    held_commands = []
    for frame in range(frames):
        p = np.vstack([a.position for a in env.agents])
        try:
            commands = env.controller.step(env.agents, env.cargoes, frame * env.dt, env.dt)
        except TheoremModeAbort as exc:
            abort = exc.as_dict()
            break
        rec = env.controller.last_theorem_record
        # Observe after command commitment. This value never changes execution.
        clearance, sampled, lipschitz_margin, sample_details = hold_segment_object_clearance(p, rec.u_command, [vertices], env.dt)
        independent = evaluate_static_hold(p, rec.u_command, vertices, env.dt, params.r_safe)
        held_commands.append(rec.u_command.copy())
        if first_violation is None and independent['status'] == 'OBSERVED_VIOLATION':
            witness = next(r for r in independent['robots'] if r['status'] == 'OBSERVED_VIOLATION')
            idx = witness['robot_index']; agent = env.agents[idx]
            view = env.controller._views.get(agent.agent_id)
            filt = env.controller._last_filter_results[idx]
            first_violation = {**witness, 'frame':frame, 'control_time':frame*env.dt,
                'robot_id':agent.agent_id, 'hold_start':p[idx].tolist(), 'command':rec.u_command[idx].tolist(),
                'observed_points':view.points.tolist() if view is not None else [],
                'observed_normals':view.normals.tolist() if view is not None else [],
                'object_residual_min':float(filt.object_residual_min),
                'active_object_rows':filt.object_rows_active,
                'A_rows':None if filt.A_rows is None else filt.A_rows.tolist(),
                'b_effective':None if filt.b_effective is None else filt.b_effective.tolist(),
                'solver_status':filt.status}
        pooled = [v.points for v in env.controller._views.values() if len(v)]
        coverage = 0.0
        if pooled:
            from scipy.spatial import cKDTree
            distance, _ = cKDTree(np.vstack(pooled)).query(boundary)
            coverage = float(np.mean(distance <= 0.08))
        guard = getattr(env.controller, "last_gap_guard", {"enabled": False, "status": "NOT_APPLIED", "truncated_samples": 0, "max_half_gap_m": 0.0, "cap_m": 0.04})
        rows.append({"frame": frame, "true_hold_clearance_lower_bound": float(clearance),
                     "gap_guard_status": guard.get("status"),
                     "gap_truncated_samples": int(guard.get("truncated_samples") or 0),
                     "gap_half_max_m": float(guard.get("max_half_gap_m") or 0.0),
                     "true_hold_clearance_sampled": float(sampled),
                     "safety_margin": float(clearance - params.r_safe),
                     "min_pair_distance": rec.hold_min_pair_distance,
                     "union_map_coverage_at_0_08m": coverage,
                     "qp_intervention_fraction": float(np.mean([r.modification > 1e-8 for r in env.controller._last_filter_results])),
                     "empty_map_holds": sum(m == "local_empty_map_hold" for m in rec.modes),
                     "solver_status": rec.solver_status,
                     "independent_status": independent['status'],
                     "clearance_path_min": independent['clearance_path_min'],
                     "actual_violation_count": independent['actual_violation_count'],
                     "sample_count":sample_details["n_samples"], "sample_h_seconds":sample_details["h"], "lipschitz_margin":lipschitz_margin})
        if frame % 10 == 0:
            measured = observer.evaluate(p, vertices)
            costs.append({"frame": frame, "H": measured["H"], "centroid_residual_squared": measured["centroid2"]})
        env.controller.apply_commands(env.agents, commands, env.dt)
        positions.append(np.vstack([a.position for a in env.agents]))
    final = observer.evaluate(positions[-1], vertices)
    result = {"variant": variant, "seed": seed, "frames_completed": len(rows), "frames_requested": frames,
              "r_safe":params.r_safe, "d_min":params.d_min, "solver_statistics":env.controller.safety.stats.as_dict(),
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
    result.update({"schema_version":SCHEMA_VERSION,
        "clearance_lower_bound_min":result['minimum_true_clearance'],
        "clearance_sampled_min":min((r['true_hold_clearance_sampled'] for r in rows),default=None),
        "clearance_path_min":min((r['clearance_path_min'] for r in rows if r['clearance_path_min'] is not None),default=None),
        "lower_bound_check_failed_frames":result['object_safety_violation_frames'],
        "actual_violation_observed":any(r['independent_status']=='OBSERVED_VIOLATION' for r in rows) if rows else None,
        "actual_violation_count":sum(r['actual_violation_count'] or 0 for r in rows) if rows and all(r['actual_violation_count'] is not None for r in rows) else None,
        "safety_evaluation_status":'OBSERVED_VIOLATION' if first_violation else ('CERTIFIED_SAFE' if len(rows)==frames and all(r['independent_status']=='CERTIFIED_SAFE' for r in rows) else 'UNRESOLVED'),
        "offline_fixed_polygon_status":'OBSERVED_VIOLATION' if first_violation else ('CERTIFIED_SAFE' if len(rows)==frames and all(r['independent_status']=='CERTIFIED_SAFE' for r in rows) else 'UNRESOLVED'),
        "offline_check":"every completed control frame; full commanded linear hold against the fixed polygon; truth is evaluation only",
        "online_gap_guard_status":(
            'NOT_APPLIED' if not cfg['controller'].get('theorem_local_segments') else
            'UNAVAILABLE' if (not rows or any(r.get('gap_guard_status') != 'HALF_GAP_WITHIN_CAP' for r in rows)) else
            'WITHIN_CAP_NOT_A_BOUND'),
        "online_gap_guard_truncated_frames":sum(r.get('gap_guard_status') == 'CAP_TRUNCATED' for r in rows),
        "online_gap_guard_max_half_gap_m":max((r.get('gap_half_max_m') or 0.0 for r in rows), default=0.0),
        "provable_online_error_bound":False,
        "first_violation_trace":first_violation,
        "evaluation_tolerance_m":1e-9,
        "mean_robot_displacement":float(np.mean(np.linalg.norm(positions[-1]-initial,axis=1))),
        "mean_robot_path_length":float(np.mean(np.sum(np.linalg.norm(np.diff(np.asarray(positions),axis=0),axis=2),axis=0))) if rows else 0.0})
    dump(out / "config.json", cfg)
    dump(out / "metrics.json", {"steps": rows, "cost": costs})
    dump(out / "summary.json", result)
    np.savez_compressed(out / "trajectory.npz", positions=positions, vertices=vertices, commands=held_commands)
    return result


def transport_arm(cfg, seed, variant, frames, out):
    env = SimulationEnvironment(cfg, seed=seed)
    penalty_capacity = env.engine.name == "penalty"
    from dbact.deployment.variants import DeploymentVariant
    if variant != "boundary_cvt":
        env.controller.cvt = DeploymentVariant(env.controller,variant)
    initial = np.vstack([a.position for a in env.agents])
    cid = env.cargoes[0].object_id
    goal = env.goal_directions[cid]
    if hasattr(env.engine, 'model'):
        env.engine.model.record_substeps = True
    physical_streak = 0
    physical_ready = None
    substep_rows = []
    env.controller.trace_enabled = True
    hist = np.zeros(12, dtype=int)
    rows = []

    def observe(frame, current):
        nonlocal physical_streak, physical_ready
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
        positive_contacts = sum(float(c.force @ goal)>1e-9 for c in report.contacts)
        gates = current.controller.phase_monitor.gates
        physical_streak = physical_streak + 1 if positive_contacts >= gates.contact_quorum else 0
        if physical_ready is None and physical_streak >= gates.contact_dwell:
            physical_ready = frame
        if hasattr(current.engine, 'model'):
            substep_rows.extend({'frame':frame, **r} for r in current.engine.model.last_substeps)
        rows.append({"frame": frame, "contacts": report.contact_count,
                     "positive_contacts":positive_contacts, "physical_contact_streak":physical_streak,
                     "contact_ids":[c.agent_id for c in report.contacts],
                     "contact_normals":[c.normal.tolist() for c in report.contacts],
                     "contact_forces":[c.force.tolist() for c in report.contacts],
                     "forward_force":float(report.net_force @ goal),
                     "positive_force_sum":sum(max(0.,float(c.force @ goal)) for c in report.contacts),
                     "reverse_force_sum":sum(min(0.,float(c.force @ goal)) for c in report.contacts),
                     "true_progress":current.tasks[cid].progress(cargo.position),
                     "estimated_progress":float(current.controller.team_progress.get(cid,0.)),
                     "true_velocity":cargo.linear_velocity.tolist(),
                     "phase":current.controller.phase.name,
                     "phase_guards":vars(current.controller.phase_signals).copy(),
                     "robot_positions":[a.position.tolist() for a in current.agents],
                     "commands":[a.velocity.tolist() for a in current.agents],
                     "rotation":float(cargo.angle),
                     "estimated_velocities":{a.agent_id:current.controller.maps[a.agent_id].object_velocity(cid).tolist() for a in current.agents},
                     "integrals":{key:float(loop.integral) for key,loop in current.controller.transport_loops.items()},
                     "diagnostics":[vars(d).copy() for d in current.controller.diagnostics],
                     "solver_statistics":current.controller.safety.stats.as_dict(),
                     "normal_only_forward_capacity": max_forward if penalty_capacity else None,
                     "normal_only_torque_interval": [torque_negative, torque_positive] if penalty_capacity else [None, None],
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
              "peak_normal_only_forward_capacity": max(r["normal_only_forward_capacity"] for r in rows) if penalty_capacity else None,
              "torque_capacity_min": min(r["normal_only_torque_interval"][0] for r in rows) if penalty_capacity else None,
              "torque_capacity_max": max(r["normal_only_torque_interval"][1] for r in rows) if penalty_capacity else None,
              "peak_actual_net_force": float(np.linalg.norm(force, axis=1).max()),
              "peak_absolute_actual_torque": max(abs(r["actual_net_torque"]) for r in rows),
              "direction_error_deg": metrics["direction_error_deg"], "max_cross_track": metrics["max_cross_track"],
              "final_distance_error": float(metrics["J"] - cargo["task"]["target_distance"]),
              "qp_intervention_fraction": float(np.mean([r["qp_intervention_fraction"] for r in rows])),
              "min_inter_agent_distance": metrics["min_inter_agent_distance"],
              "min_signed_clearance": metrics["min_signed_clearance"],
              "g500_success": gate["success"], "failure_reasons": gate["failure_reasons"],
              "capacity_label": "normal-only instantaneous contact wrench box, 0 <= f_i <= stiffness*delta_max; not dynamically guaranteed" if penalty_capacity else "NOT_APPLICABLE: a penalty spring force bound is not a rigid-contact impulse bound",
              "changed_mechanism": "Only centroid target before CONTACT_READY; CVT mass, redeployment guards, sensing, safety and subsequent transport identical"}
    errors=progress_errors(env.tasks[cid].start, env.cargoes[0].position, goal, env.tasks[cid].distance)
    events={name:event_record(summary['phases']['entered'].get(name),env.dt,termination['frames_run']) for name in ('SEARCH','DISCOVER','ENCLOSE','CONTACT_READY','TRANSPORT','BRAKE','HOLD')}
    phases=list(events)
    durations={a+'->'+b:None if events[a]['seconds'] is None or events[b]['seconds'] is None else events[b]['seconds']-events[a]['seconds'] for a,b in zip(phases,phases[1:])}
    result.update({"schema_version":SCHEMA_VERSION, **errors, "events":events,"phase_durations":durations,
        "physical_contact_ready":event_record(physical_ready,env.dt,termination['frames_run']),
        "physical_readiness_definition":"positive projected contact force, existing controller contact_quorum and contact_dwell; control-frame observations",
        "task_completed":bool(termination['settled'] and errors['euclidean_endpoint_error'] <= cfg['task']['tolerance']),
        "task_completed_definition":"settled plus final Euclidean endpoint error <= existing task tolerance",
        "settled":termination['settled'],"g500_pass":gate['success'],"watchdog":termination['terminated_by']=='watchdog',
        "final_speed":float(np.linalg.norm(env.cargoes[0].linear_velocity)),
        "rotation_deg":cargo['rotation_deg'],
        "moving_safety_status":"UNRESOLVED: discrete physics substeps, no continuous certificate"})
    result["contact_reporting"] = getattr(env.engine, "contact_reporting", "start_of_control_hold_penalty_force")
    dump(out / "physics_substeps.json", substep_rows)
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gate", type=int, choices=(5, 6), required=True)
    parser.add_argument("--shapes", nargs="+", choices=("l_shape", "rectangle", "c_shape"), default=["l_shape", "rectangle", "c_shape"])
    parser.add_argument("--seeds", nargs="+", type=int, default=[2, 5, 8])
    parser.add_argument("--frames", type=int, default=None)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.workers <= 0 or (args.frames is not None and args.frames <= 0):
        parser.error("workers and frames must be positive")
    if len(set(args.seeds)) != len(args.seeds) or len(set(args.shapes)) != len(args.shapes):
        parser.error("duplicate seeds or shapes would share an output directory")
    if args.out.exists() and any(args.out.iterdir()):
        parser.error("output is nonempty; use an independent directory or study --resume")
    frames = args.frames if args.frames is not None else (600 if args.gate == 5 else 3000)
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
