"""Compatibility entry point for dbact_sim.experiments.evidence."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from dbact_sim.experiments.evidence import *  # noqa: F401,F403

if __name__ == "__main__":
    main()
