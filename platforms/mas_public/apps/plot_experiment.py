from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.controller.plotting.experiment_plotter import ExperimentPlotter


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot saved MAS S1 experiment data.")
    parser.add_argument("experiment_dir", help="Path to data/experiments/<timestamp>_<name>")
    args = parser.parse_args()
    plotter = ExperimentPlotter(args.experiment_dir)
    outputs = plotter.plot_all()
    if plotter.world_source is not None:
        print(f"plot world source: {plotter.world_source}")
    if plotter.command_source is not None:
        print(f"plot command source: {plotter.command_source}")
    for output in outputs:
        print(output)


if __name__ == "__main__":
    main()
