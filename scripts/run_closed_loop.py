#!/usr/bin/env python
"""D6 - one closed-loop episode: headless 500 frames, then render from the record.

    python scripts/run_closed_loop.py --seed 0 --out runs/d_seed0
    python scripts/run_closed_loop.py --seed 0 --out runs/d_seed0 --no-render

The simulation never draws. It writes ``replay.npz``, and the pictures are made
from that file afterwards -- so the reported frame rate is the frame rate of the
control loop rather than of matplotlib, and a figure can be revised without
re-running the physics. ``scripts/render_closed_loop.py`` does the same rendering
against an existing run.

Exit status is the verdict: non-zero when G500 fails, so the script can be used
as a gate.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.replay import Replay, render_animation, render_frames, render_summary_plot
from dbact_sim.scenarios import load_yaml


def render_outputs(out: Path, summary: dict, stride: int, fps: int) -> None:
    replay = Replay.load(out / "replay.npz")
    contracts = summary.get("contracts", {})
    d_min = float(contracts.get("d_min", 0.34))
    budget = float(contracts.get("delta_max", 0.05)) + float(contracts.get("discrete_overshoot", 0.0))
    render_animation(replay, out / "closed_loop.gif", stride=stride, fps=fps, d_min=d_min, penetration_budget=budget)
    render_frames(replay, out / "figures", d_min=d_min, penetration_budget=budget)
    render_summary_plot(replay, out / "timeseries.png", d_min=d_min)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/sim/d/l_shape_closed_loop.yaml")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--frames", type=int, default=500, help="Control frames; the G500 budget.")
    parser.add_argument("--until-settled", action="store_true",
                        help="Run to completion instead of to a budget.")
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--out", default="")
    parser.add_argument("--no-render", action="store_true", help="Simulate only; render later from replay.npz.")
    parser.add_argument("--animation-stride", type=int, default=4)
    parser.add_argument("--animation-fps", type=int, default=20)
    args = parser.parse_args()

    config = load_yaml(args.config)
    out = Path(args.out) if args.out else Path("runs") / f"{Path(args.config).stem}_seed{args.seed}"

    env = SimulationEnvironment(config, seed=args.seed)
    started = time.perf_counter()
    if args.until_settled:
        termination = env.run_until_settled(max_frames=args.max_frames)
    else:
        env.run(args.frames)
        termination = {"frames_run": args.frames, "terminated_by": "budget", "settled": None}
    wall = time.perf_counter() - started
    frames = termination["frames_run"]

    summary = env.save_outputs(out)
    summary["timing"] = {
        "frames": frames,
        "terminated_by": termination["terminated_by"],
        "simulation_seconds": wall,
        "frames_per_second": frames / wall if wall > 0 else float("inf"),
    }
    summary["feasibility_c5"] = env.controller.feasibility.as_dict()
    (out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    if not args.no_render:
        render_outputs(out, summary, args.animation_stride, args.animation_fps)

    print(f"seed {args.seed}: {wall:.1f} s of wall clock for {frames} frames "
          f"({summary['timing']['frames_per_second']:.1f} frame/s)")
    passed = True
    for cargo_id, entry in summary["cargoes"].items():
        g500 = entry.get("g500")
        if g500 is None:
            print(f"  {cargo_id}: no transport task; G500 not evaluated")
            continue
        metrics = g500["metrics"]
        task = entry["task"]
        print(f"  {cargo_id}: {'PASS' if g500['success'] else 'FAIL'}  "
              f"goal {task['angle_deg']:.1f} deg, L={task['target_distance']:.3f} m")
        print(f"    J={metrics['J']:.4f} m  efficiency={metrics['efficiency']:.3f}  "
              f"cross-track={metrics['max_cross_track']:.4f} m  yaw={metrics['rotation_deg']:+.2f} deg")
        print(f"    phases: detect={metrics['first_detection_frame']} "
              f"enclose={metrics['enclosure_frame']} contact={metrics['contact_ready_frame']} "
              f"transport={metrics['transport_frame']} brake={metrics['brake_frame']} "
              f"hold={metrics['hold_frame']}")
        print(f"    safety: min d_ij={metrics['min_inter_agent_distance']:.4f} m  "
              f"clearance={metrics['min_signed_clearance']:+.4f} m  "
              f"penetration={metrics['max_penetration']:.4f}/{metrics['penetration_budget']:.4f} m")
        solver = summary["solver"]
        print(f"    solver: {solver['statuses']}  fallbacks={solver['fallbacks']} "
              f"scaled={solver['barrier_scalings']} (min scale {solver['min_barrier_scale']:.3f})")
        for reason in g500["failure_reasons"]:
            print(f"      - {reason}")
        passed = passed and g500["success"]

    print(f"wrote {out}")
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
