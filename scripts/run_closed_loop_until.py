#!/usr/bin/env python3
"""Run DBACT until HOLD, an explicit failure, or a safety timeout."""

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


DEFAULT_CONFIG = "configs/sim/research/adaptive_progress_closed_loop.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=0)
    parser.add_argument("--output", default="")
    parser.add_argument("--animation", action="store_true")
    parser.add_argument("--animation-stride", type=int, default=3)
    parser.add_argument("--animation-fps", type=int, default=20)
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    max_steps = int(args.max_steps or (cfg.get("episode", {}) or {}).get("max_steps", 1500))
    output = Path(args.output) if args.output else Path("runs") / f"adaptive_closed_loop_seed{args.seed}"
    output.mkdir(parents=True, exist_ok=True)

    env = SimulationEnvironment(cfg, seed=args.seed)
    started = time.perf_counter()
    termination = env.run_until(max_steps)
    wall_seconds = time.perf_counter() - started
    summary = env.save_outputs(output)
    plot_snapshot(env, output / "final_snapshot.png", title=f"DBACT {termination.status}")
    plot_trajectories(env, output / "trajectory.png", title="DBACT adaptive closed-loop trajectories")
    if args.animation:
        animate_simulation(
            env,
            output / "closed_loop.gif",
            title="DBACT SEARCH-MAP-ENCLOSE-TRANSPORT-BRAKE-HOLD",
            frame_stride=max(1, int(args.animation_stride)),
            fps=args.animation_fps,
        )

    cargo_success = all(entry.get("success") is True for entry in summary["cargoes"].values())
    manifest = {
        "config": args.config,
        "seed": args.seed,
        "max_steps_timeout": max_steps,
        "executed_steps": termination.frame,
        "termination": termination.status,
        "termination_detail": termination.detail,
        "success": bool(termination.success and cargo_success),
        "wall_seconds": wall_seconds,
        "control_frames_per_wall_second": termination.frame / max(wall_seconds, 1e-12),
        "cargoes": {
            cargo_id: {
                "J": entry.get("J"),
                "efficiency": entry.get("efficiency"),
                "rotation_deg": entry.get("rotation_deg"),
                "coverage": entry.get("final_strict_coverage"),
                "phase_frames": entry.get("phase_frames"),
                "failure_reasons": entry.get("failure_reasons", []),
            }
            for cargo_id, entry in summary["cargoes"].items()
        },
        "solver": summary["solver"],
    }
    (output / "episode_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0 if manifest["success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
