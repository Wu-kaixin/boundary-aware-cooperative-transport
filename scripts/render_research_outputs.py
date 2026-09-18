#!/usr/bin/env python3
"""Offline-render MP4/GIF demos and paper Figures A--G from a saved trace."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact_sim.trace import SimulationTrace  # noqa: E402
from dbact_sim.visualization import render_animation, write_research_paper_figures  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument(
        "--trace",
        help="Hybrid trace directory containing metadata.json and arrays.npz.",
    )
    source.add_argument(
        "--replay",
        help="Claude v2 replay.npz; it is converted to an immutable hybrid trace.",
    )
    parser.add_argument("--output", required=True, help="Destination directory.")
    parser.add_argument("--view-mode", choices=("demo", "paper", "debug"), default="demo")
    parser.add_argument("--video", choices=("mp4", "gif", "both", "none"), default="mp4")
    parser.add_argument("--frame-stride", type=int, default=4)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--dpi", type=int, default=110)
    parser.add_argument("--show-ids", action="store_true")
    parser.add_argument("--skip-paper-figures", action="store_true")
    parser.add_argument("--paper-formats", default="png,pdf,svg")
    args = parser.parse_args()

    trace = (
        SimulationTrace.load(args.trace)
        if args.trace
        else SimulationTrace.from_replay(args.replay)
    )
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    if args.replay:
        trace.save(output / "trace")
    reports = []
    formats = ("mp4", "gif") if args.video == "both" else (() if args.video == "none" else (args.video,))
    for suffix in formats:
        report = render_animation(
            trace,
            output / f"closed_loop.{suffix}",
            view_mode=args.view_mode,
            frame_stride=args.frame_stride,
            fps=args.fps,
            dpi=args.dpi,
            show_ids=True if args.show_ids else None,
        )
        reports.append(report.as_dict())
        print(
            f"{suffix.upper()}: {report.rendered_frames} frames in {report.wall_seconds:.2f}s "
            f"({report.rendering_fps:.2f} rendering FPS)"
        )

    paper_outputs = {}
    if not args.skip_paper_figures:
        paper_formats = tuple(
            item.strip().lower() for item in args.paper_formats.split(",") if item.strip()
        )
        written = write_research_paper_figures(
            trace,
            output / "paper_figures",
            formats=paper_formats,
        )
        paper_outputs = {key: [str(path) for path in paths] for key, paths in written.items()}
        print(f"Paper: Figures A-G ({sum(map(len, written.values()))} files)")

    simulation_fps = float(trace.settings.get("simulation_fps", math.nan))
    manifest = {
        "source": str(Path(args.trace or args.replay)),
        "source_type": "hybrid_trace" if args.trace else "claude_v2_replay",
        "converted_trace": str(output / "trace") if args.replay else None,
        "view_mode": args.view_mode,
        "simulation_fps": simulation_fps if math.isfinite(simulation_fps) else None,
        "animation_reports": reports,
        "paper_figures": paper_outputs,
    }
    (output / "render_manifest.json").write_text(
        json.dumps(manifest, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    print(f"Outputs: {output.resolve()}")


if __name__ == "__main__":
    main()
