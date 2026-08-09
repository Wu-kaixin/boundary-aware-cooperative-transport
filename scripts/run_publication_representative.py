#!/usr/bin/env python3
"""Reproduce the audited representative episode and publication animations."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402
from dbact_sim.visualization import animate_simulation, plot_snapshot, plot_trajectories  # noqa: E402
from run_arbitrary_shape_monte_carlo import configure_case  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="configs/sim/research/adaptive_progress_closed_loop.yaml",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=1500)
    parser.add_argument("--output", default="artifacts/publication/representative")
    parser.add_argument("--animation-stride", type=int, default=4)
    parser.add_argument("--animation-fps", type=int, default=20)
    parser.add_argument("--skip-mp4", action="store_true")
    args = parser.parse_args()

    base = load_yaml(args.config)
    config, case = configure_case(base, "circle", args.seed, 0, 0.10)
    config["evaluation"]["online_truth_audit"] = True
    config["evaluation"]["require_measured_error_bounds"] = True
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    started = time.perf_counter()
    env = SimulationEnvironment(config, seed=args.seed)
    termination = env.run_until(args.max_steps)
    wall_seconds = time.perf_counter() - started
    summary = env.save_outputs(output)
    plot_snapshot(env, output / "closed_loop_final.png", title=termination.status)
    plot_trajectories(env, output / "closed_loop_trajectories.png")
    animate_simulation(
        env,
        output / "closed_loop.gif",
        title="DBACT SEARCH → MAP → ENCLOSE → TRANSPORT → BRAKE → HOLD",
        frame_stride=max(1, args.animation_stride),
        fps=args.animation_fps,
    )
    if not args.skip_mp4:
        animate_simulation(
            env,
            output / "closed_loop.mp4",
            title="DBACT SEARCH → MAP → ENCLOSE → TRANSPORT → BRAKE → HOLD",
            frame_stride=max(1, args.animation_stride),
            fps=args.animation_fps,
        )

    cargo = summary["cargoes"]["cargo_0"]
    manifest = {
        "schema_version": 1,
        "config": args.config,
        "case": case,
        "seed": args.seed,
        "truth_audit": True,
        "max_steps_timeout": args.max_steps,
        "termination": termination.status,
        "frame": termination.frame,
        "wall_seconds": wall_seconds,
        "task_success": bool(termination.success and cargo.get("success") is True),
        "J_m": cargo.get("J"),
        "efficiency": cargo.get("efficiency"),
        "max_cross_track_error_m": cargo.get("max_cross_track_error"),
        "max_abs_rotation_deg": cargo.get("max_abs_rotation_deg"),
        "phase_frames": cargo.get("phase_frames"),
        "measured_error_audit": summary.get("measured_error_audit"),
        "solver": summary.get("solver"),
        "artifacts": [
            "summary.json",
            "trajectories.csv",
            "safety_timeseries.csv",
            "cargo_timeseries.csv",
            "perception_errors.csv",
            "closed_loop_final.png",
            "closed_loop_trajectories.png",
            "closed_loop.gif",
        ] + ([] if args.skip_mp4 else ["closed_loop.mp4"]),
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0 if manifest["task_success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
