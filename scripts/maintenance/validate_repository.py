"""Record offline engineering checks in a new, dedicated maintenance attempt.

No hardware entry point or full experiment matrix is invoked. This does not
interpret simulation execution success as scientific task success.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def run(output: Path, extended: bool = False) -> int:
    output = output.resolve()
    if not output.is_relative_to(ROOT / "runs/maintenance"):
        raise ValueError("Output must be under runs/maintenance/<mission>/<attempt>")
    if Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        raise ValueError("Use the root .venv")
    output.mkdir(parents=True, exist_ok=False)
    temp = output / "process-temp"
    temp.mkdir()
    env = {**os.environ, "PYTHONUTF8": "1", "PYTHONIOENCODING": "utf-8", "MPLBACKEND": "Agg",
           "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1", "DBACT_CVT_WORKERS": "1",
           "TEMP": str(temp), "TMP": str(temp), "MPLCONFIGDIR": str(output / "mpl-config")}
    py = sys.executable
    report = {"started_at_utc": datetime.now(timezone.utc).isoformat(), "python": py,
              "version": sys.version, "base_prefix": sys.base_prefix, "commands": []}

    def check(name, args, cwd=ROOT, timeout=1200):
        started = time.monotonic()
        with (output / (name + ".log")).open("w", encoding="utf-8") as log:
            try:
                result = subprocess.run(args, cwd=cwd, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
                code, error = result.returncode, None
            except (OSError, subprocess.TimeoutExpired) as exc:
                code, error = None, str(exc)
        item = {"name": name, "command": args, "cwd": str(cwd), "exit_code": code,
                "seconds": time.monotonic() - started, "error": error,
                "status": "PASS" if code == 0 else "BLOCKED" if code is None else "FAIL"}
        report["commands"].append(item)
        (output / "commands.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(name, item["status"], code, f"{item['seconds']:.1f}s", flush=True)
        return code == 0

    check("git-diff", ["git", "diff", "--check"])
    check("git-fsck", ["git", "fsck", "--full"])
    check("pip-check", [py, "-m", "pip", "check"])
    check("environment", [py, "-c", "import dbact,dbact_sim,mas_adapter,sys; print(sys.executable); print(sys.path); print(dbact.__file__); print(dbact_sim.__file__); print(mas_adapter.__file__)"])
    cli = check("cli-help", [py, "-m", "dbact_sim", "--help"])
    check("health-help", [py, "scripts/check_repository_health.py", "--help"])
    check("health", [py, "scripts/check_repository_health.py", "--output", str(output / "health.json")])
    check("core-tests", [py, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                         "--basetemp", str(output / "core-temp"), "--junitxml", str(output / "core-tests.xml")])
    check("mas-tests", [py, "-m", "pytest", "-q", "-p", "no:cacheprovider",
                        "--basetemp", str(output / "mas-temp"), "--junitxml", str(output / "mas-tests.xml")],
          ROOT / "platforms/mas_public")
    if cli:
        study = [py, "-m", "dbact_sim", "study"]
        check("study-help", study + ["--help"])
        check("evidence-help", [py, "-m", "dbact_sim", "evidence", "--help"])
        protocol = ["--protocol", "experiments/ccta2027/smoke.yaml", "--split", "smoke"]
        if check("smoke-dry-run", study + protocol + ["--dry-run"]):
            smoke = output / "smoke"
            check("smoke", study + protocol + ["--workers", "2", "--out", str(smoke)])
            check("smoke-evidence", [py, "-m", "dbact_sim", "evidence", "--batch", f"smoke={smoke}",
                                     "--out", str(output / "smoke-evidence")])
            if extended:
                check("smoke-serial", study + protocol + ["--workers", "1", "--out", str(output / "smoke-serial")])
                check("smoke-resume", study + protocol + ["--workers", "2", "--out", str(smoke), "--resume"])
    return int(any(r["status"] != "PASS" for r in report["commands"]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--extended", action="store_true")
    args = parser.parse_args()
    raise SystemExit(run(args.output, args.extended))
