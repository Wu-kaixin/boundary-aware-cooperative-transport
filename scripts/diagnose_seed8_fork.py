"""Seed-8 fork diagnostics on the unmodified frozen controller.

Distinguishes:
- Confirmed same-machine repeatability of seed 8.
- Instrumented logs around the previously observed divergence window
  (grid ownership, map choice, control mode, QP active rows, object scaling).
- Cross-platform float hypothesis: recorded as untested when a second OS/BLAS
  is unavailable.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml

# Default: frozen audit worktree. Override with --repo.
DEFAULT_REPO = Path(r"E:\dbact-proof-audit-source")


def dump(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def owned_sample_counts(env) -> list[int]:
    from dbact.boundary_density import BoundaryAwareDensity

    agents = env.agents
    neighbors = env.controller._neighbor_indices(agents)
    counts = []
    for i, agent in enumerate(agents):
        view = env.controller._views.get(agent.agent_id)
        if view is None or len(view) == 0:
            counts.append(0)
            continue
        crowd = np.vstack([agent.position] + [agents[j].position for j in neighbors[i]])
        goal = env.controller._goal_for(view) if env.controller.params.task_mode == "transport" else None
        shape = env.controller._enclosure_geometry(agent.agent_id, view)
        density = BoundaryAwareDensity.from_view(
            view, shape, robot_positions=crowd, goal_direction=goal
        )
        cell = env.controller.cvt.compute(i, agents, neighbors[i], density, env.domain)
        counts.append(int(cell.owned_samples))
    return counts


def map_fingerprint(env) -> dict:
    out = {}
    for agent in env.agents:
        view = env.controller._views.get(agent.agent_id)
        if view is None or len(view) == 0:
            out[agent.agent_id] = {"n": 0, "centroid": None, "hash": None}
            continue
        digest = hashlib.sha256(
            np.ascontiguousarray(view.points).tobytes()
            + np.ascontiguousarray(view.normals).tobytes()
            + np.ascontiguousarray(view.arc_length).tobytes()
        ).hexdigest()[:16]
        out[agent.agent_id] = {
            "n": int(len(view)),
            "centroid": view.points.mean(axis=0).tolist(),
            "hash": digest,
        }
    return out


def run_instrumented(repo: Path, out: Path, seed: int, frames: int, window: tuple[int, int]) -> dict:
    sys.path.insert(0, str(repo / "src"))
    from dbact_sim.environment import SimulationEnvironment

    cfg = yaml.safe_load((repo / "configs/sim/d/l_shape_closed_loop.yaml").read_text())
    env = SimulationEnvironment(cfg, seed=seed)
    env.controller.trace_enabled = True
    lo, hi = window
    rows = []
    positions = []
    velocities = []
    started = time.perf_counter()
    for frame in range(frames):
        before_maps = map_fingerprint(env) if lo <= frame <= hi else None
        before_owned = owned_sample_counts(env) if lo <= frame <= hi else None
        cmds = env.controller.step(env.agents, env.cargoes, env.t, env.dt)
        vel = np.vstack([c.velocity for c in cmds])
        env.controller.apply_commands(env.agents, cmds, env.dt)
        env.engine.step(env.cargoes, env.agents, env.dt)
        env.t += env.dt
        env._record()
        positions.append(np.vstack([a.position for a in env.agents]))
        velocities.append(vel)
        if lo <= frame <= hi:
            diags = env.controller.diagnostics
            rows.append(
                {
                    "frame": frame,
                    "time": float(frame * env.dt),
                    "phase": env.controller.phase.label,
                    "mode_counts": env.controller.mode_counts(),
                    "owned_samples": before_owned,
                    "map": before_maps,
                    "solver_status": [d.solver_status for d in diags],
                    "agent_rows": [d.agent_rows for d in diags],
                    "agent_rows_active": [d.agent_rows_active for d in diags],
                    "object_rows": [d.object_rows for d in diags],
                    "object_rows_active": [d.object_rows_active for d in diags],
                    "barrier_scalings_so_far": env.controller.safety.stats.barrier_scalings,
                    "min_barrier_scale_so_far": env.controller.safety.stats.min_barrier_scale,
                    "command_speed_max": float(np.max(np.linalg.norm(vel, axis=1))),
                }
            )
    summary = env.summary()
    stats = env.controller.safety.stats.as_dict()
    arr_p = np.stack(positions)
    arr_v = np.stack(velocities)
    cargo_reports = summary.get("cargoes", {})
    first_cargo = next(iter(cargo_reports.values()), {})
    g500 = first_cargo.get("g500", {})
    result = {
        "seed": seed,
        "frames": frames,
        "wall_seconds": time.perf_counter() - started,
        "git_sha": summary.get("provenance", {}).get("git_sha"),
        "position_sha256": hashlib.sha256(arr_p.tobytes()).hexdigest(),
        "velocity_sha256": hashlib.sha256(arr_v.tobytes()).hexdigest(),
        "barrier_scalings": stats["barrier_scalings"],
        "min_barrier_scale": stats["min_barrier_scale"],
        "statuses": stats["statuses"],
        "hold_frame": summary.get("phases", {}).get("hold_frame"),
        "g500": g500,
        "window": {"lo": lo, "hi": hi},
        "window_rows": rows,
    }
    np.savez_compressed(out / f"seed{seed}_trajectory.npz", positions=arr_p, velocities=arr_v)
    dump(out / f"seed{seed}_instrumented.json", result)
    return result


def compare_repeat(a: dict, b: dict) -> dict:
    return {
        "same_position_sha256": a["position_sha256"] == b["position_sha256"],
        "same_velocity_sha256": a["velocity_sha256"] == b["velocity_sha256"],
        "same_barrier_scalings": a["barrier_scalings"] == b["barrier_scalings"],
        "same_min_barrier_scale": a["min_barrier_scale"] == b["min_barrier_scale"],
        "same_hold_frame": a["hold_frame"] == b["hold_frame"],
        "same_g500_success": a.get("g500", {}).get("success") == b.get("g500", {}).get("success"),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=DEFAULT_REPO)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=8)
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--window-lo", type=int, default=380)
    parser.add_argument("--window-hi", type=int, default=450)
    parser.add_argument("--repeats", type=int, default=2)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    runs = []
    for k in range(args.repeats):
        tag_dir = args.out / f"repeat_{k}"
        tag_dir.mkdir(parents=True, exist_ok=True)
        runs.append(
            run_instrumented(
                args.repo,
                tag_dir,
                args.seed,
                args.frames,
                (args.window_lo, args.window_hi),
            )
        )

    repeatability = compare_repeat(runs[0], runs[1]) if len(runs) >= 2 else {}
    # Confirmed vs hypothesis split.
    report = {
        "frozen_repo": str(args.repo),
        "confirmed": {
            "same_machine_repeatability": repeatability,
            "instrumentation_window": [args.window_lo, args.window_hi],
            "barrier_scalings_this_host": runs[0]["barrier_scalings"] if runs else None,
            "min_barrier_scale_this_host": runs[0]["min_barrier_scale"] if runs else None,
            "hold_frame": runs[0]["hold_frame"] if runs else None,
            "note": (
                "Same-host repeats compare bit identity of the frozen controller. "
                "They do not by themselves prove or refute a cross-platform fork."
            ),
        },
        "hypothesis_cross_platform_float": {
            "status": "untested_on_second_platform",
            "reason": (
                "No second OS/BLAS environment is available in this session. "
                "Prior reproduction already showed seed-8 G500 flip vs the audit zip "
                "reference machine; that remains a cross-platform float amplification "
                "hypothesis, not a newly confirmed mechanism from this host alone."
            ),
            "supporting_prior_evidence": [
                "Identical seed, config blob, phase schedule through HOLD",
                "Position Frobenius divergence grows from ULP after HOLD",
                "This host records object barrier scalings; reference zip recorded zero",
            ],
        },
        "runs": [
            {
                "position_sha256": r["position_sha256"],
                "barrier_scalings": r["barrier_scalings"],
                "min_barrier_scale": r["min_barrier_scale"],
                "hold_frame": r["hold_frame"],
                "g500_success": r.get("g500", {}).get("success"),
            }
            for r in runs
        ],
    }
    dump(args.out / "seed8_diagnosis_report.json", report)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
