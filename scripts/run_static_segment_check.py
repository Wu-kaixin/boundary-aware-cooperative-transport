#!/usr/bin/env python3
"""Static local clearance versus motion, with and without fresh segments."""
from __future__ import annotations

import os
from pathlib import Path

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")
os.environ.setdefault("DBACT_CVT_WORKERS", "1")

ROOT = Path(__file__).resolve().parents[1]


def main():
    import sys
    sys.path.insert(0, str(ROOT / "scripts"))
    sys.path.insert(0, str(ROOT / "src"))
    from run_deployment_pairs import config_for, static_arm

    for segments in (False, True):
        cfg = config_for(5, "l_shape")
        cfg["controller"]["theorem_local_segments"] = segments
        name = "segments" if segments else "sample_rows"
        out = ROOT / "runs/evidence_closure/static_segment_v1" / f"l_shape_seed2_{name}"
        out.mkdir(parents=True, exist_ok=True)
        result = static_arm(cfg, 2, "local", 600, out)
        print({
            "case": name,
            "status": result.get("safety_evaluation_status"),
            "path_mm": None if result.get("clearance_path_min") is None else 1000 * result["clearance_path_min"],
            "displacement": result.get("mean_robot_displacement"),
            "frames": result.get("frames_completed"),
            "abort": result.get("abort"),
        }, flush=True)


if __name__ == "__main__":
    main()
