#!/usr/bin/env python
"""Render an already-finished run. No physics, no seed, no re-simulation.

    python scripts/render_closed_loop.py runs/d_seed0
    python scripts/render_closed_loop.py runs/d_seed0 --stride 2 --fps 25

This exists so that changing how a figure looks costs seconds instead of a
re-run, and so that the frame rate a run reports is the frame rate of the control
loop rather than of the animation writer.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from dbact_sim.replay import Replay, render_animation, render_frames, render_summary_plot


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="Run directory containing replay.npz and summary.json.")
    parser.add_argument("--stride", type=int, default=4, help="Simulation frames per animation frame.")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--frames", default="", help="Comma-separated frame indices for stills.")
    args = parser.parse_args()

    run = Path(args.run)
    replay = Replay.load(run / "replay.npz")

    d_min, budget = 0.34, 0.07
    summary_path = run / "summary.json"
    if summary_path.exists():
        contracts = json.loads(summary_path.read_text(encoding="utf-8")).get("contracts", {})
        d_min = float(contracts.get("d_min", d_min))
        budget = float(contracts.get("delta_max", 0.05)) + float(contracts.get("discrete_overshoot", 0.0))

    stills = [int(v) for v in args.frames.split(",") if v.strip()] if args.frames else None
    render_animation(replay, run / "closed_loop.gif", stride=args.stride, fps=args.fps,
                     d_min=d_min, penetration_budget=budget)
    render_frames(replay, run / "figures", stills, d_min=d_min, penetration_budget=budget)
    render_summary_plot(replay, run / "timeseries.png", d_min=d_min)
    print(f"rendered {replay.frames} frames from {run / 'replay.npz'}")


if __name__ == "__main__":
    main()
