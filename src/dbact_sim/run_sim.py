from __future__ import annotations

import argparse
from contextlib import contextmanager
from pathlib import Path

import matplotlib.pyplot as plt

from .environment import SimulationEnvironment
from .scenarios import load_yaml
from .visualization import (
    LivePaperViewer,
    animate_simulation,
    plot_snapshot,
    plot_trajectories,
    write_paper_figures,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a DBACT simulation scenario.")
    parser.add_argument("--config", required=True, help="Path to scenario YAML file.")
    parser.add_argument("--steps", type=int, default=400, help="Number of simulation steps.")
    parser.add_argument("--output", default="runs/demo", help="Output directory.")
    parser.add_argument("--animate", action="store_true", help="Write an animated GIF of the run.")
    parser.add_argument("--animation-stride", type=int, default=6, help="Simulation frames skipped between GIF frames.")
    parser.add_argument("--animation-fps", type=int, default=12, help="Animated GIF frames per second.")
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
    env = SimulationEnvironment(cfg)
    live_viewer = LivePaperViewer(env, update_stride=args.live_stride, pause_s=args.live_pause) if args.live else None
    env.run(
        args.steps,
        on_frame=live_viewer.update if live_viewer is not None else None,
    )
    if live_viewer is not None:
        live_viewer.update(args.steps, env, force=True)
    out = Path(args.output)
    with _noninteractive_output_figures():
        env.save_outputs(out)
        plot_snapshot(env, out / "final_snapshot.png")
        plot_trajectories(env, out / "trajectory.png")
        figure_frames = [int(item) for item in args.figure_frames.split(",") if item.strip()] if args.figure_frames else None
        write_paper_figures(env, out, figure_frames)
        if args.animate:
            animate_simulation(
                env,
                out / "animation.gif",
                frame_stride=args.animation_stride,
                fps=args.animation_fps,
            )
    print(f"Saved DBACT simulation outputs to {out}")
    if live_viewer is not None:
        live_viewer.finish(block=not args.live_close_at_end)


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
