"""One-click a priori certificate-repair driver (full-CPU, resume-safe)."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "run_apriori_centroid_bound.py"


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv:
        argv = [
            "--out",
            str(ROOT / "artifacts" / "apriori_certificate_repair_2026-09-15"),
            "--shapes",
            "l_shape",
            "rectangle",
            "c_shape",
            "--seeds",
            "2",
            "5",
            "8",
            "--grids",
            "20",
            "--frames",
            "600",
            "--integration-method",
            "edge_green",
            "--edge-n-gon",
            "256",
            "--edge-h-max",
            "0.004",
            "--resume",
        ]
    cmd = [sys.executable, str(SCRIPT), *argv]
    print(" ".join(cmd), flush=True)
    return subprocess.call(cmd, cwd=str(ROOT))


if __name__ == "__main__":
    raise SystemExit(main())
