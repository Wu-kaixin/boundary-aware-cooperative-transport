from __future__ import annotations

import argparse
from pathlib import Path

from .environment import SimulationEnvironment
from .scenarios import load_yaml
from .visualization import plot_snapshot, plot_trajectories


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a DBACT simulation scenario.")
    parser.add_argument("--config", required=True, help="Path to scenario YAML file.")
    parser.add_argument("--steps", type=int, default=400, help="Number of simulation steps.")
    parser.add_argument("--output", default="runs/demo", help="Output directory.")
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    env = SimulationEnvironment(cfg)
    env.run(args.steps)
    out = Path(args.output)
    env.save_outputs(out)
    plot_snapshot(env, out / "final_snapshot.png")
    plot_trajectories(env, out / "trajectory.png")
    print(f"Saved DBACT simulation outputs to {out}")


if __name__ == "__main__":
    main()
