#!/usr/bin/env python3
"""Generate the audited v2 showcase, trace, video, GIF, and Figures A--G."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402
from dbact_sim.trace import SimulationTrace, VisualizationRecorder  # noqa: E402
from dbact_sim.visualization import (  # noqa: E402
    ResearchVisualizer,
    render_animation,
    write_research_paper_figures,
)
from run_arbitrary_shape_monte_carlo import configure_case  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sim/research/adaptive_progress_closed_loop.yaml")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=1500)
    parser.add_argument("--output", default="artifacts/v2_showcase")
    parser.add_argument("--animation-stride", type=int, default=4)
    parser.add_argument("--animation-fps", type=int, default=20)
    parser.add_argument("--animation-dpi", type=int, default=85)
    parser.add_argument("--view-mode", choices=("demo", "paper", "debug"), default="demo")
    args = parser.parse_args()

    base = load_yaml(args.config)
    config, case = configure_case(base, "circle", args.seed, 0, 0.10)
    config["evaluation"]["online_truth_audit"] = True
    config["evaluation"]["require_measured_error_bounds"] = True
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    started = time.perf_counter()
    env = SimulationEnvironment(config, seed=args.seed)
    recorder = VisualizationRecorder(stride=4, sensor_ray_stride=4)
    termination = env.run_until(args.max_steps, on_frame=recorder.capture)
    wall_seconds = time.perf_counter() - started
    simulation_fps = termination.frame / max(wall_seconds, 1e-12)
    summary = env.save_outputs(output)
    trace = SimulationTrace.from_environment(env, recorder, simulation_fps=simulation_fps)
    trace.save(output / "trace")

    visualizer = ResearchVisualizer(trace, view_mode=args.view_mode)
    visualizer.save_frame(trace.frame_count - 1, output / "closed_loop_final.png", dpi=160)
    visualizer.close()
    figures = write_research_paper_figures(trace, output / "paper_figures")
    gif = render_animation(
        trace,
        output / "closed_loop.gif",
        view_mode=args.view_mode,
        frame_stride=max(1, args.animation_stride),
        fps=args.animation_fps,
        dpi=args.animation_dpi,
    )
    mp4 = render_animation(
        trace,
        output / "closed_loop.mp4",
        view_mode=args.view_mode,
        frame_stride=max(1, args.animation_stride),
        fps=args.animation_fps,
        dpi=args.animation_dpi,
    )

    cargo = summary["cargoes"]["cargo_0"]
    manifest = {
        "schema_version": 2,
        "config": args.config,
        "case": case,
        "seed": args.seed,
        "truth_audit": True,
        "termination": termination.status,
        "frame": termination.frame,
        "wall_seconds": wall_seconds,
        "simulation_fps": simulation_fps,
        "render_reports": [gif.as_dict(), mp4.as_dict()],
        "task_success": bool(termination.success and cargo.get("success") is True),
        "J_m": cargo.get("J"),
        "efficiency": cargo.get("efficiency"),
        "max_cross_track_error_m": cargo.get("max_cross_track_error"),
        "max_abs_rotation_deg": cargo.get("max_abs_rotation_deg"),
        "phase_frames": cargo.get("phase_frames"),
        "solver": summary.get("solver"),
        "paper_figures": {
            key: [str(path.relative_to(output)) for path in paths]
            for key, paths in figures.items()
        },
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))
    return 0 if manifest["task_success"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
