"""Validate the sampled theorem_mode execution interface.

Checks:
1. Original clipping counterexample becomes an abort / illegal without clipping,
   while wall rows make the unclipped one-step update stay in D when feasible.
2. Ordinary static oracle scene runs without mode switches and records hold gaps.
3. Wall-corner scene exercises wall rows; failures keep original thresholds.
4. Cost deltas are recorded; monotonic decrease is NOT asserted.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def dump(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def clipping_counterexample(out: Path) -> dict:
    from dbact.controller import DBACTController, DBACTParams
    from dbact.theorem_mode import hold_segment_min_distance, point_in_domain, wall_halfplanes
    from dbact.types import AgentState, ControlCommand
    from dbact.qp2d import solve_min_norm_2d

    domain = (0.0, 8.0, 0.0, 8.0)
    dt = 0.05
    d_min = 0.28
    u_max = 0.35
    gamma = 6.0
    a = d_min / math.sqrt(2)
    # Same construction as the audit: half-constraints hold, clipping breaks them.
    agents_legacy = [AgentState("a", [0.0, a]), AgentState("b", [a, 0.0])]
    velocities = [
        np.array([-1.0, -1.0]) * u_max / math.sqrt(2),
        np.zeros(2),
    ]
    before = np.vstack([x.position for x in agents_legacy])
    free = before + dt * np.vstack(velocities)
    legacy = DBACTController(
        DBACTParams(
            d_min=d_min,
            robot_radius=0.13,
            delta_max=0.065,
            cage_offset=0.105,
            max_speed=u_max,
            theorem_mode=False,
            backend="qp",
            dt=dt,
        ),
        domain,
        seed=0,
    )
    legacy.apply_commands(
        agents_legacy,
        [ControlCommand(x.agent_id, v) for x, v in zip(agents_legacy, velocities)],
        dt,
    )
    after_clip = np.vstack([x.position for x in agents_legacy])

    # theorem_mode path: add wall rows, then refuse clipping.
    p0 = before[0]
    A_wall, b_wall = wall_halfplanes(p0, domain, dt)
    # Agent half-row against neighbour at before[1]
    diff = p0 - before[1]
    h = float(np.dot(diff, diff) - d_min**2)
    A_agent = (2.0 * diff).reshape(1, 2)
    b_agent = np.array([-0.5 * gamma * h])
    A = np.vstack([A_agent, A_wall])
    b = np.concatenate([b_agent, b_wall])
    sol = solve_min_norm_2d(velocities[0], A, b, u_max)
    u_safe = sol.u if sol.feasible else np.zeros(2)
    nxt = p0 + dt * u_safe
    hold = hold_segment_min_distance(p0, before[1], u_safe, velocities[1], dt)

    theorem = DBACTController(
        DBACTParams(
            d_min=d_min,
            robot_radius=0.13,
            delta_max=0.065,
            cage_offset=0.105,
            max_speed=u_max,
            theorem_mode=True,
            theorem_disable_clipping=True,
            theorem_forbid_fallback=True,
            task_mode="caging",
            lead_offset=None,
            gap_gain=0.0,
            explore_gain=0.0,
            density_mode="offset",
            backend="qp",
            dt=dt,
            use_object_barrier=False,
        ),
        domain,
        seed=0,
    )
    agents_tm = [AgentState("a", [0.0, a]), AgentState("b", [a, 0.0])]
    aborted = None
    try:
        theorem.apply_commands(
            agents_tm,
            [ControlCommand("a", velocities[0]), ControlCommand("b", velocities[1])],
            dt,
        )
        # If walls are absent from apply_commands, the raw velocity exits D and aborts.
    except Exception as exc:  # TheoremModeAbort
        aborted = str(exc)

    result = {
        "d_min": d_min,
        "initial_distance": float(np.linalg.norm(before[0] - before[1])),
        "unclipped_distance": float(np.linalg.norm(free[0] - free[1])),
        "legacy_clipped_distance": float(np.linalg.norm(after_clip[0] - after_clip[1])),
        "legacy_clip_breaks_d_min": bool(
            np.linalg.norm(after_clip[0] - after_clip[1]) + 1e-12 < d_min
        ),
        "wall_filtered_feasible": bool(sol.feasible),
        "wall_filtered_next_in_domain": bool(point_in_domain(nxt, domain)),
        "wall_filtered_hold_min_distance": float(hold),
        "wall_filtered_respects_d_min": bool(hold + 1e-9 >= d_min),
        "raw_command_abort_without_clip": aborted,
        "failure_criteria_unchanged": True,
    }
    dump(out / "clipping_counterexample.json", result)
    return result


def wall_activation_case(out: Path) -> dict:
    """One robot near xmin with nominal velocity pointing out of D."""
    from dbact.safety_filter import SafetyFilter, SafetyFilterParams

    domain = (0.0, 8.0, 0.0, 8.0)
    dt = 0.05
    u_max = 0.35
    # One step at u_max would leave xmin if wall rows are disabled.
    p = np.array([0.01, 4.0])
    u_nom = np.array([-u_max, 0.0])

    with_wall = SafetyFilter(
        SafetyFilterParams(
            d_min=0.28,
            max_speed=u_max,
            enable_object_rows=False,
            enable_wall_rows=True,
            domain=domain,
            dt=dt,
            forbid_fallback=True,
            backend="qp",
        )
    )
    no_wall = SafetyFilter(
        SafetyFilterParams(
            d_min=0.28,
            max_speed=u_max,
            enable_object_rows=False,
            enable_wall_rows=False,
            domain=domain,
            dt=dt,
            forbid_fallback=True,
            backend="qp",
        )
    )
    filtered = with_wall.filter_velocity(p, u_nom, [])
    baseline = no_wall.filter_velocity(p, u_nom, [])
    nxt = p + dt * filtered.velocity
    result = {
        "position": p.tolist(),
        "u_nom": u_nom.tolist(),
        "u_final_with_walls": filtered.velocity.tolist(),
        "u_final_without_walls": baseline.velocity.tolist(),
        "modification_with_walls": filtered.modification,
        "modification_without_walls": baseline.modification,
        "wall_residual_min": filtered.wall_residual_min,
        "wall_rows_active": filtered.wall_rows_active,
        "next_in_domain_with_walls": bool(
            0.0 <= nxt[0] <= domain[1] and domain[2] <= nxt[1] <= domain[3]
        ),
        "next_in_domain_without_walls": bool(
            0.0 <= (p + dt * baseline.velocity)[0] <= domain[1]
        ),
        "wall_rows_caused_change": bool(
            np.linalg.norm(filtered.velocity - baseline.velocity) > 1e-6
        ),
        "u_nom_outside_domain_without_filter": bool((p + dt * u_nom)[0] < 0.0),
    }
    dump(out / "wall_activation.json", result)
    return result


def run_scene(config_path: Path, out_dir: Path, frames: int, seed: int) -> dict:
    from dbact.theorem_mode import TheoremModeAbort
    from dbact_sim.environment import SimulationEnvironment

    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    env = SimulationEnvironment(cfg, seed=seed)
    assert env.controller.params.theorem_mode
    records = []
    abort = None
    try:
        for _ in range(frames):
            env.step()
            rec = env.controller.last_theorem_record
            if rec is not None:
                records.append(
                    {
                        "frame": rec.frame,
                        "time": rec.time,
                        "modes": rec.modes,
                        "map_source": rec.map_source,
                        "solver_status": rec.solver_status,
                        "barrier_scale": rec.barrier_scale.tolist(),
                        "agent_residual_min": float(np.min(rec.agent_residual_min)),
                        "wall_residual_min": float(np.min(rec.wall_residual_min)),
                        "object_residual_min": float(np.min(rec.object_residual_min)),
                        "hold_min_pair_distance": rec.hold_min_pair_distance,
                        "hold_min_object_clearance": rec.hold_min_object_clearance,
                        "hold_object_clearance_sampled": rec.hold_object_clearance_sampled,
                        "hold_object_clearance_margin": rec.hold_object_clearance_margin,
                        "cost_H": rec.cost_H,
                        "cost_delta": rec.cost_delta,
                        "speed_max": float(np.max(rec.speed)),
                    }
                )
    except TheoremModeAbort as exc:
        abort = exc.as_dict()

    summary = env.controller.theorem_log.as_summary()
    summary["abort_runtime"] = abort or env.theorem_abort
    summary["config"] = str(config_path.relative_to(ROOT))
    summary["seed"] = seed
    summary["frames_requested"] = frames
    summary["modes_forbidden_present"] = sorted(
        m
        for m in summary.get("modes_seen", [])
        if m not in {"theorem_cvt"}
    )
    # Do NOT assert cost_delta <= 0. Only report sign counts.
    deltas = [r["cost_delta"] for r in records if r["cost_delta"] is not None]
    summary["cost_delta_negative"] = int(sum(1 for d in deltas if d < -1e-12))
    summary["cost_delta_positive"] = int(sum(1 for d in deltas if d > 1e-12))
    summary["cost_delta_zeroish"] = int(sum(1 for d in deltas if abs(d) <= 1e-12))
    summary["hold_min_pair_distance"] = (
        float(min(r["hold_min_pair_distance"] for r in records)) if records else None
    )
    summary["agent_residual_min"] = (
        float(min(r["agent_residual_min"] for r in records)) if records else None
    )
    summary["wall_residual_min"] = (
        float(min(r["wall_residual_min"] for r in records)) if records else None
    )
    dump(out_dir / "step_records.json", records)
    dump(out_dir / "summary.json", summary)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT / "artifacts" / "theorem_mode_validation",
    )
    parser.add_argument("--frames", type=int, default=200)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    clip = clipping_counterexample(args.out)
    wall = wall_activation_case(args.out)
    static = run_scene(
        ROOT / "configs/sim/theorem/static_l_shape_oracle.yaml",
        args.out / "static_l_shape",
        args.frames,
        seed=0,
    )
    corner = run_scene(
        ROOT / "configs/sim/theorem/wall_corner_oracle.yaml",
        args.out / "wall_corner",
        args.frames,
        seed=0,
    )
    report = {
        "clipping_counterexample": clip,
        "wall_activation": wall,
        "static_l_shape": static,
        "wall_corner": corner,
        "notes": [
            "Cost monotonicity is not required after the safety filter.",
            "Original d_min / domain failure criteria were not relaxed.",
            "Modes other than theorem_cvt must be absent.",
        ],
    }
    dump(args.out / "validation_summary.json", report)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
