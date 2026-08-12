from __future__ import annotations

import argparse
import json
import time
from contextlib import contextmanager
from pathlib import Path

import matplotlib.pyplot as plt

from .environment import SimulationEnvironment
from .scenarios import load_yaml
from .trace import SimulationTrace, VisualizationRecorder
from .visualization import (
    LivePaperViewer,
    ResearchVisualizer,
    render_animation,
    write_research_paper_figures,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a DBACT simulation scenario.")
    parser.add_argument("--config", required=True, help="Path to scenario YAML file.")
    parser.add_argument("--steps", type=int, default=400, help="Number of simulation steps.")
    parser.add_argument("--seed", type=int, default=0, help="Run seed; recorded in summary.json.")
    parser.add_argument("--output", default="", help="Output directory. Defaults to runs/<config stem>_seed<seed>.")
    parser.add_argument(
        "--no-render",
        "--no-figures",
        dest="no_render",
        action="store_true",
        help="Run headless and skip all figure/video rendering; the compact core trace is still saved.",
    )
    parser.add_argument("--animate", action="store_true", help="Offline-render the saved trace after simulation.")
    parser.add_argument("--animation-format", choices=("mp4", "gif"), default="mp4")
    parser.add_argument("--animation-stride", type=int, default=6, help="Source-frame stride for offline video.")
    parser.add_argument("--animation-fps", type=int, default=20, help="Offline video playback FPS.")
    parser.add_argument("--view-mode", choices=("demo", "paper", "debug"), default="demo")
    parser.add_argument(
        "--trace-stride",
        type=int,
        default=5,
        help="Simulation-frame stride for sparse sensor/map visualization snapshots.",
    )
    parser.add_argument(
        "--sensor-ray-stride",
        type=int,
        default=3,
        help="Display-only downsampling applied to detected sensor rays.",
    )
    parser.add_argument("--live", action="store_true", help="Open a real-time paper-style simulation window.")
    parser.add_argument("--live-stride", type=int, default=5, help="Simulation steps between live window refreshes.")
    parser.add_argument("--live-pause", type=float, default=0.001, help="Matplotlib pause duration for live refresh.")
    parser.add_argument(
        "--live-close-at-end",
        action="store_true",
        help="Close/finalize the live run without blocking after outputs are saved.",
    )
    parser.add_argument(
        "--figure-frames",
        default="",
        help="Comma-separated iteration indices for paper-style FIG outputs. Defaults to 0/25/50/75/100 percent.",
    )
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    env = SimulationEnvironment(cfg, seed=args.seed)
    live_viewer = LivePaperViewer(env, update_stride=args.live_stride, pause_s=args.live_pause) if args.live else None
    recorder = (
        None
        if args.no_render
        else VisualizationRecorder(
            stride=args.trace_stride,
            sensor_ray_stride=args.sensor_ray_stride,
        )
    )

    def observe(step_index: int, simulation: SimulationEnvironment) -> None:
        if recorder is not None:
            recorder.capture(step_index, simulation)
        if live_viewer is not None:
            live_viewer.update(step_index, simulation)

    simulation_started = time.perf_counter()
    env.run(args.steps, on_frame=observe if recorder is not None or live_viewer is not None else None)
    simulation_seconds = time.perf_counter() - simulation_started
    if live_viewer is not None:
        live_viewer.update(args.steps, env, force=True)

    out = Path(args.output) if args.output else Path("runs") / f"{Path(args.config).stem}_seed{args.seed}"
    summary = env.save_outputs(out)
    simulation_fps = args.steps / max(simulation_seconds, 1e-12)
    trace = SimulationTrace.from_environment(env, recorder, simulation_fps=simulation_fps)
    trace.save(out / "trace")
    if not args.no_render:
        with _noninteractive_output_figures():
            visualizer = ResearchVisualizer(trace, view_mode=args.view_mode)
            visualizer.save_frame(trace.frame_count - 1, out / "final_snapshot.png")
            visualizer.close()
            write_research_paper_figures(trace, out / "paper_figures")
            if args.animate:
                report = render_animation(
                    trace,
                    out / f"animation.{args.animation_format}",
                    view_mode=args.view_mode,
                    frame_stride=args.animation_stride,
                    fps=args.animation_fps,
                )
                (out / "render_manifest.json").write_text(
                    json.dumps(report.as_dict(), indent=2, allow_nan=False),
                    encoding="utf-8",
                )
                print(
                    f"  rendering: {report.rendering_fps:.2f} FPS "
                    f"({report.rendered_frames} frames, {report.wall_seconds:.2f}s)"
                )

    print(f"Saved DBACT simulation outputs to {out}")
    print(_headline(summary))
    if live_viewer is not None:
        live_viewer.finish(block=not args.live_close_at_end)


def _headline(summary: dict) -> str:
    lines = [
        f"  engine={summary['engine']}  backend={summary['provenance']['backend']}  "
        f"git={summary['provenance']['git_sha']}  seed={summary['provenance']['seed']}",
        f"  solver: {json.dumps(summary['solver']['statuses'])}  fallbacks={summary['solver']['fallbacks']}",
    ]
    for cargo_id, entry in summary["cargoes"].items():
        verdict = "SUCCESS" if entry.get("success") else "FAIL"
        lines.append(
            f"  {cargo_id}: {verdict}  J={entry.get('J', float('nan')):.4f} m  "
            f"|dx|={entry['displacement']:.4f} m  rot={entry['rotation_deg']:+.2f} deg  "
            f"strict_cov={entry['final_strict_coverage']:.3f}  "
            f"min_clearance={entry['min_signed_clearance']:+.4f} m  inside={entry['max_agents_inside']}"
        )
        for reason in entry.get("failure_reasons", []):
            lines.append(f"      - {reason}")
    return "\n".join(lines)


@contextmanager
def _noninteractive_output_figures():
    """Save report figures without spawning extra GUI windows during --live."""
    was_interactive = plt.isinteractive()
    plt.ioff()
    try:
        yield
    finally:
        if was_interactive:
            plt.ion()


if __name__ == "__main__":
    main()
