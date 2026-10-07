#!/usr/bin/env python3
"""One-shot check of the brake fix on archived C-shape seeds. Not a frozen test."""
from __future__ import annotations

import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("DBACT_CVT_WORKERS", "1")

ROOT = Path(__file__).resolve().parents[1]


def job(args):
    import sys

    sys.path.insert(0, str(ROOT / "scripts"))
    sys.path.insert(0, str(ROOT / "src"))
    from run_deployment_pairs import config_for, transport_arm

    shape, seed, variant = args
    out = ROOT / "runs/evidence_closure/phase_brake_v7" / f"{shape}_seed{seed}_{variant}"
    out.mkdir(parents=True, exist_ok=True)
    result = transport_arm(config_for(6, shape), seed, variant, 3000, out)
    phases = result["events"]
    return {
        "case": out.name,
        "ready": result["contact_ready_seconds"],
        "transport": phases["TRANSPORT"]["seconds"],
        "brake": phases["BRAKE"]["seconds"],
        "hold": phases["HOLD"]["seconds"],
        "abs": result["absolute_progress_error"],
        "endpoint": result["euclidean_endpoint_error"],
        "settled": result["settled"],
        "watchdog": result["watchdog"],
        "g500": result["g500_pass"],
        "task": result["task_completed"],
        "frames": result["termination"]["frames_run"],
        "reason": result["termination"]["terminated_by"],
    }


def main():
    cases = [
        ("c_shape", 8, "boundary_cvt"),
        ("c_shape", 8, "nearest_observed_target"),
        ("c_shape", 8, "local_spacing"),
        ("c_shape", 5, "boundary_cvt"),
    ]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(job, cases):
            print(row, flush=True)


if __name__ == "__main__":
    main()
