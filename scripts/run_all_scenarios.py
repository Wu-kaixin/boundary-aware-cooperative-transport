from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

SCENARIOS = ["circle", "rectangle", "l_shape", "nonconvex", "multi_object"]


def main() -> None:
    parser = argparse.ArgumentParser(description="Run standard DBACT scenarios.")
    parser.add_argument("--steps", type=int, default=400)
    parser.add_argument("--live", action="store_true", help="Open the real-time paper-style viewer for each scenario.")
    parser.add_argument("--live-stride", type=int, default=5)
    parser.add_argument("--live-close-at-end", action="store_true")
    parser.add_argument("--animate", action="store_true")
    args = parser.parse_args()

    for name in SCENARIOS:
        cfg = Path("configs/sim") / f"{name}.yaml"
        out = Path("runs") / name
        cmd = [
            "python",
            "-m",
            "dbact_sim.run_sim",
            "--config",
            str(cfg),
            "--steps",
            str(args.steps),
            "--output",
            str(out),
        ]
        if args.live:
            cmd.extend(["--live", "--live-stride", str(args.live_stride)])
        if args.live_close_at_end:
            cmd.append("--live-close-at-end")
        if args.animate:
            cmd.append("--animate")
        print("Running", " ".join(cmd))
        subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()
