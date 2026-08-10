from __future__ import annotations

import argparse
import os
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
import subprocess
import sys

SCENARIOS = ["circle", "rectangle", "l_shape", "nonconvex", "multi_object"]


def _build_cmd(
    name: str,
    steps: int,
    live: bool,
    live_stride: int,
    live_close_at_end: bool,
    animate: bool,
    workers: int | None,
) -> list[str]:
    cfg = Path("configs/sim") / f"{name}.yaml"
    out = Path("runs") / name
    cmd = [
        sys.executable,
        "-m",
        "dbact_sim.run_sim",
        "--config",
        str(cfg),
        "--steps",
        str(steps),
        "--output",
        str(out),
    ]
    if live:
        cmd.extend(["--live", "--live-stride", str(live_stride)])
    if live_close_at_end:
        cmd.append("--live-close-at-end")
    if animate:
        cmd.append("--animate")
    if workers is not None:
        cmd.extend(["--workers", str(workers)])
    return cmd


def _run_one(cmd: list[str]) -> tuple[str, int]:
    print("Running", " ".join(cmd), flush=True)
    completed = subprocess.run(cmd, check=False)
    return " ".join(cmd), int(completed.returncode)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run standard DBACT scenarios.")
    parser.add_argument("--steps", type=int, default=400)
    parser.add_argument("--live", action="store_true", help="Open the real-time paper-style viewer for each scenario.")
    parser.add_argument("--live-stride", type=int, default=5)
    parser.add_argument("--live-close-at-end", action="store_true")
    parser.add_argument("--animate", action="store_true")
    parser.add_argument(
        "--jobs",
        type=int,
        default=0,
        help="Parallel scenario processes (0=auto, 1=serial legacy behavior).",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=None,
        help="Forwarded to each run_sim as --workers.",
    )
    args = parser.parse_args()

    commands = [
        _build_cmd(
            name,
            args.steps,
            args.live,
            args.live_stride,
            args.live_close_at_end,
            args.animate,
            args.workers,
        )
        for name in SCENARIOS
    ]

    if args.live:
        # Live GUI windows should stay serial.
        jobs = 1
    elif args.jobs == 0:
        jobs = max(1, min(len(commands), os.cpu_count() or 1))
    else:
        jobs = max(1, int(args.jobs))

    if jobs <= 1:
        for cmd in commands:
            label, code = _run_one(cmd)
            if code != 0:
                raise SystemExit(f"Scenario failed ({code}): {label}")
        return

    failures: list[str] = []
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        futures = {pool.submit(_run_one, cmd): cmd for cmd in commands}
        for fut in as_completed(futures):
            label, code = fut.result()
            if code != 0:
                failures.append(f"{label} -> exit {code}")
    if failures:
        raise SystemExit("Scenario failures:\n" + "\n".join(failures))


if __name__ == "__main__":
    main()
