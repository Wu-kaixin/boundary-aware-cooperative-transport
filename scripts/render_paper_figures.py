#!/usr/bin/env python3
"""Render DBACT paper Figures A--G from a saved simulation trace."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact_sim.trace import SimulationTrace  # noqa: E402
from dbact_sim.visualization.paper_figures import write_research_paper_figures  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", required=True, help="Directory containing metadata.json and arrays.npz.")
    parser.add_argument("--output", required=True, help="Output directory for Figures A--G.")
    parser.add_argument("--formats", default="png,pdf,svg", help="Comma-separated subset of png,pdf,svg.")
    parser.add_argument("--dpi", type=int, default=220, help="PNG resolution.")
    args = parser.parse_args()

    formats = tuple(item.strip().lower() for item in args.formats.split(",") if item.strip())
    trace = SimulationTrace.load(args.trace)
    outputs = write_research_paper_figures(trace, args.output, formats=formats, dpi=args.dpi)
    count = sum(len(paths) for paths in outputs.values())
    print(f"Wrote Figures A-G ({count} files) to {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
