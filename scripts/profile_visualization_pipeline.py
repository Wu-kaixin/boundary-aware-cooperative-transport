#!/usr/bin/env python3
"""Profile headless simulation, trace observation, and offline artist rendering."""

from __future__ import annotations

import argparse
import cProfile
import json
import statistics
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402
from dbact_sim.trace import SimulationTrace, VisualizationRecorder  # noqa: E402
from dbact_sim.visualization import ResearchVisualizer  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/sim/v3/arbitrary_shape_full_workspace_500.yaml")
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--trace-stride", type=int, default=5)
    parser.add_argument("--sensor-ray-stride", type=int, default=4)
    parser.add_argument("--render-frame-stride", type=int, default=10)
    parser.add_argument("--profile-steps", type=int, default=150)
    parser.add_argument("--output", default="runs/v2_performance/profile.json")
    args = parser.parse_args()
    if args.steps < 1 or args.repeats < 1:
        raise SystemExit("steps and repeats must be positive")

    config = load_yaml(args.config)
    headless_rates: list[float] = []
    observed_rates: list[float] = []
    numerical_identity = True
    observed_env = None
    observed_recorder = None
    for _ in range(args.repeats):
        headless = SimulationEnvironment(config, seed=args.seed)
        started = time.perf_counter()
        headless.run(args.steps)
        elapsed = time.perf_counter() - started
        headless_rates.append(args.steps / max(elapsed, 1e-12))

        observed = SimulationEnvironment(config, seed=args.seed)
        recorder = VisualizationRecorder(
            stride=args.trace_stride,
            sensor_ray_stride=args.sensor_ray_stride,
        )
        started = time.perf_counter()
        observed.run(args.steps, on_frame=recorder.capture)
        elapsed = time.perf_counter() - started
        observed_rates.append(args.steps / max(elapsed, 1e-12))
        numerical_identity &= _numerically_identical(headless, observed)
        observed_env, observed_recorder = observed, recorder

    assert observed_env is not None and observed_recorder is not None
    simulation_fps = statistics.median(headless_rates)
    trace = SimulationTrace.from_environment(
        observed_env,
        observed_recorder,
        simulation_fps=simulation_fps,
    )

    frame_stride = max(1, int(args.render_frame_stride))
    frames = list(range(0, trace.frame_count, frame_stride))
    if frames[-1] != trace.frame_count - 1:
        frames.append(trace.frame_count - 1)
    visualizer = ResearchVisualizer(trace, view_mode="demo")
    visualizer.fig.canvas.draw()
    started = time.perf_counter()
    for frame in frames:
        visualizer.update(frame)
        visualizer.fig.canvas.draw()
    render_seconds = time.perf_counter() - started
    visualizer.close()

    profiler = cProfile.Profile()
    profile_env = SimulationEnvironment(config, seed=args.seed)
    profile_steps = min(args.steps, max(1, int(args.profile_steps)))
    profiler.enable()
    profile_env.run(profile_steps)
    profiler.disable()

    headless_median = statistics.median(headless_rates)
    observer_median = statistics.median(observed_rates)
    report = {
        "config": args.config,
        "seed": args.seed,
        "steps": args.steps,
        "repeats": args.repeats,
        "safety_qp_frequency": "every physics step (unchanged)",
        "headless_simulation_fps_samples": headless_rates,
        "observer_simulation_fps_samples": observed_rates,
        "headless_simulation_fps_median": headless_median,
        "observer_simulation_fps_median": observer_median,
        "observer_overhead_percent": 100.0 * (headless_median - observer_median) / max(headless_median, 1e-12),
        "observer_numerically_identical": numerical_identity,
        "artist_rendering": {
            "frames": len(frames),
            "wall_seconds": render_seconds,
            "rendering_fps": len(frames) / max(render_seconds, 1e-12),
            "includes_canvas_draw": True,
            "video_encoding": False,
        },
        "profile_steps": profile_steps,
        "top_cumulative_functions": _top_functions(profiler, 15),
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


def _numerically_identical(a: SimulationEnvironment, b: SimulationEnvironment) -> bool:
    if a.log.times != b.log.times:
        return False
    for agent_id in a.log.agent_positions:
        if not np.array_equal(
            np.vstack(a.log.agent_positions[agent_id]),
            np.vstack(b.log.agent_positions[agent_id]),
        ):
            return False
    for cargo_id in a.log.cargo_vertices:
        if not np.array_equal(
            np.stack(a.log.cargo_vertices[cargo_id]),
            np.stack(b.log.cargo_vertices[cargo_id]),
        ):
            return False
    return a.controller.safety.stats.as_dict() == b.controller.safety.stats.as_dict()


def _top_functions(profiler: cProfile.Profile, count: int) -> list[dict]:
    rows = []
    for entry in profiler.getstats():
        code = entry.code
        if isinstance(code, str):
            file_name, line, function = "~", 0, code
        else:
            file_name = str(code.co_filename)
            line = int(code.co_firstlineno)
            function = str(code.co_name)
        rows.append(
            {
                "file": file_name,
                "line": line,
                "function": function,
                "calls": int(entry.callcount),
                "total_seconds": float(entry.inlinetime),
                "cumulative_seconds": float(entry.totaltime),
            }
        )
    rows.sort(key=lambda item: item["cumulative_seconds"], reverse=True)
    return rows[: int(count)]


if __name__ == "__main__":
    main()
