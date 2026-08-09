#!/usr/bin/env python3
"""One-command 500-frame DBACT closed-loop demonstration.

The default scenario samples a reproducible task direction from the configured
angular/workspace bounds, then executes SEARCH -> ENCLOSE -> TRANSPORT -> HOLD.
The cargo is moved only by the selected contact engine.  A non-zero exit status
means at least one validity gate failed; an animation is never presented as a
successful run merely because it looks plausible.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml
from dbact_sim.visualization import animate_simulation, plot_snapshot, plot_trajectories, write_paper_figures


DEFAULT_CONFIG = "configs/sim/v3/l_shape_search_closed_loop_500.yaml"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", default="")
    parser.add_argument("--no-animation", action="store_true")
    parser.add_argument("--animation-stride", type=int, default=1)
    parser.add_argument("--animation-fps", type=int, default=20)
    args = parser.parse_args()

    config_path = Path(args.config)
    output = Path(args.output) if args.output else Path("runs") / f"closed_loop_v3_500_seed{args.seed}"
    output.mkdir(parents=True, exist_ok=True)

    env = SimulationEnvironment(load_yaml(config_path), seed=args.seed)
    started = time.perf_counter()
    env.run(500)
    wall_seconds = time.perf_counter() - started
    summary = env.save_outputs(output)

    plot_snapshot(env, output / "final_snapshot.png", title="DBACT 500-frame final state")
    plot_trajectories(env, output / "trajectory.png", title="DBACT 500-frame trajectories")
    write_paper_figures(env, output, frame_indices=[0, 100, 200, 350, 500])
    if not args.no_animation:
        animate_simulation(
            env,
            output / "closed_loop_500.gif",
            title="DBACT constrained random-direction transport",
            frame_stride=args.animation_stride,
            fps=args.animation_fps,
        )

    entry = next(iter(summary["cargoes"].values()))
    stride = max(1, int(args.animation_stride))
    animation_states = 0 if args.no_animation else 500 // stride + 1 + int(500 % stride != 0)
    manifest = {
        "config": str(config_path),
        "seed": args.seed,
        "frames": 500,
        "rendered_states": animation_states,
        "wall_seconds": wall_seconds,
        "simulation_frames_per_wall_second": 500.0 / max(wall_seconds, 1e-9),
        "success": bool(entry.get("success")),
        "phase_frames": entry.get("phase_frames", {}),
        "initial_detection_count": entry.get("initial_detection_count"),
        "goal_angle_deg": entry.get("goal_angle_deg"),
        "goal_target": entry.get("goal_target"),
        "directional_progress_J": entry.get("J"),
        "progress_efficiency": entry.get("efficiency"),
        "final_strict_coverage": entry.get("final_strict_coverage"),
        "solver_fallbacks": summary["solver"]["fallbacks"],
        "multi_rate": summary.get("multi_rate", {}),
        "failure_reasons": entry.get("failure_reasons", []),
    }
    (output / "demo_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    verdict = "SUCCESS" if manifest["success"] else "FAIL"
    print(
        f"{verdict}: seed={args.seed}  wall={wall_seconds:.2f}s  "
        f"rate={manifest['simulation_frames_per_wall_second']:.2f} frame/s  "
        f"J={entry.get('J', float('nan')):.3f}m  coverage={entry.get('final_strict_coverage', 0.0):.3f}"
    )
    print(f"outputs: {output}")
    if not manifest["success"]:
        for reason in manifest["failure_reasons"]:
            print(f"  - {reason}")
        raise SystemExit(2)


if __name__ == "__main__":
    main()
