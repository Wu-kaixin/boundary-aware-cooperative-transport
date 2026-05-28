from __future__ import annotations

import subprocess
from pathlib import Path

SCENARIOS = ["circle", "rectangle", "l_shape", "nonconvex", "multi_object"]


def main() -> None:
    for name in SCENARIOS:
        cfg = Path("configs/sim") / f"{name}.yaml"
        out = Path("runs") / name
        cmd = ["python", "-m", "dbact_sim.run_sim", "--config", str(cfg), "--steps", "400", "--output", str(out)]
        print("Running", " ".join(cmd))
        subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
