"""Run README and paper acceptance gates, then publish compact evidence.

Does not treat a zero exit code as scientific success. Closed-loop scripts
return non-zero on G500 failure by design. Every table is built from JSON/CSV
by ``scripts/build_consolidation_report.py``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
import time
from pathlib import Path

import numpy
import scipy

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "runs" / "consolidation"

L_SHAPE_PRIOR = {
    "B_E": 0.1702700924,
    "B_J_prior": 0.1811413578,
    "B_J_geom": 0.2295107776,
}


def git_sha() -> str:
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def config_hashes() -> dict[str, str]:
    return {
        p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted((ROOT / "configs").rglob("*.yaml"))
    }


def check_l_shape_priors() -> None:
    screen = json.loads((ROOT / "runs/paper/priors/prior_screen.json").read_text(encoding="utf-8"))
    row = screen["l_shape_seed2_n20"]
    for key, expected in L_SHAPE_PRIOR.items():
        got = float(row[key])
        if abs(got - expected) > 1e-9:
            raise SystemExit(f"L-shape {key}={got:.10g}, expected {expected:.10g}")
    if not (row["B_J_prior"] < row["B_J_geom"]):
        raise SystemExit("L-shape prior certificate does not beat the geometric bound")
    print("L-shape priors match the archived screen.", flush=True)


def missing(paths: list[str]) -> list[str]:
    return [p for p in paths if not (ROOT / p).exists()]


def output_hashes(artifacts: list[str]) -> dict[str, str]:
    return {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in artifacts}


def input_signature(args: list[str], seeds: list[int]) -> dict:
    paths = sorted([*ROOT.glob("src/**/*.py"), *ROOT.glob("scripts/*.py"),
                    *ROOT.glob("configs/**/*.yaml"), ROOT / "pyproject.toml",
                    ROOT / "requirements.txt"])
    sources = {p.relative_to(ROOT).as_posix(): hashlib.sha256(
        p.read_bytes().replace(b"\r\n", b"\n")).hexdigest() for p in paths}
    inputs = []
    if args[0].endswith("render_closed_loop.py"):
        inputs = [str(Path(args[1]) / f) for f in ("replay.npz", "summary.json")]
    elif args[0].endswith("analyse_enclosure_gate.py"):
        folder = Path(args[args.index("--run") + 1])
        payload = json.loads((ROOT / folder / "gate.json").read_text(encoding="utf-8"))
        inputs = [str(folder / "gate.json"),
                  *[str(folder / f"gate_seed{r['seed']}.npz") for r in payload["reports"]]]
    return {"schema": 1, "command": args, "seeds": seeds, "sources": sources,
            "inputs": output_hashes(inputs),
            "runtime": [sys.executable, platform.python_version(), numpy.__version__, scipy.__version__]}


def artifacts_current(artifacts: list[str], receipt: Path, signature: dict) -> bool:
    try:
        data = json.loads(receipt.read_text(encoding="utf-8"))
        return (data.get("signature") == signature
                and data.get("outputs") == output_hashes(artifacts))
    except (OSError, ValueError, AttributeError):
        return False


def scientific_failure(args, artifacts, returncode, started_ns, log_path) -> bool:
    # Exit 1 means a scored G500 failure only when this invocation actually
    # produced a complete run. Never accept old outputs left by a crash.
    if returncode != 1 or not args[0].endswith("run_closed_loop.py"):
        return False
    try:
        if any((ROOT / p).stat().st_mtime_ns < started_ns for p in artifacts):
            return False
        if "Traceback (most recent call last)" in log_path.read_text(encoding="utf-8"):
            return False
        summary = json.loads(next(ROOT / p for p in artifacts if p.endswith("summary.json")).read_text(encoding="utf-8"))
        gates = [c["g500"] for c in summary["cargoes"].values() if c.get("g500")]
        timing = summary["timing"]
        return (bool(gates) and any(g["success"] is False for g in gates)
                and timing["terminated_by"] in {"settled", "watchdog", "budget"}
                and timing["frames"] == summary["steps"] and summary["steps"] > 0)
    except (OSError, ValueError, KeyError, StopIteration, TypeError):
        return False


def run(name: str, args: list[str], seeds: list[int], artifacts: list[str],
        allow_nonzero: bool, manifest: dict) -> None:
    print(f"START {name}", flush=True)
    start = time.time()
    started_ns = time.time_ns()
    signature = input_signature(args, seeds)
    receipt = OUT / "receipts" / f"{name}.json"
    if artifacts_current(artifacts, receipt, signature):
        row = {
            "name": name,
            "command": [sys.executable, "-u", *args],
            "seeds": seeds,
            "exit_code": None,
            "seconds": 0.0,
            "skipped": True,
            "artifacts": artifacts,
        }
        manifest["runs"].append(row)
        (OUT / "readme_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        print(f"SKIP {name} artifacts present", flush=True)
        return
    receipt.unlink(missing_ok=True)
    log_path = OUT / f"{name}.log"
    with log_path.open("w", encoding="utf-8") as log:
        proc = subprocess.run([sys.executable, "-u", *args], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    row = {
        "name": name,
        "command": [sys.executable, "-u", *args],
        "seeds": seeds,
        "exit_code": proc.returncode,
        "seconds": time.time() - start,
        "log": log_path.relative_to(ROOT).as_posix(),
        "artifacts": artifacts,
        "allow_nonzero": allow_nonzero,
    }
    manifest["runs"].append(row)
    (OUT / "readme_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"END {name} {proc.returncode} {row['seconds']:.1f}s", flush=True)
    absent = missing(artifacts)
    if absent:
        raise SystemExit(f"{name} missing {absent}; see {log_path}")
    if proc.returncode != 0 and not (allow_nonzero and scientific_failure(
            args, artifacts, proc.returncode, started_ns, log_path)):
        raise SystemExit(f"{name} failed with exit code {proc.returncode}; see {log_path}")
    if input_signature(args, seeds) != signature:
        raise SystemExit(f"{name} inputs changed while running; refusing to cache the result")
    receipt.parent.mkdir(parents=True, exist_ok=True)
    temporary = receipt.with_suffix(".tmp")
    temporary.write_text(json.dumps({"signature": signature, "outputs": output_hashes(artifacts),
                                     "commit": git_sha(), "exit_code": proc.returncode}, indent=2), encoding="utf-8")
    os.replace(temporary, receipt)


def commands_for(from_gate: int) -> list[tuple[str, list[str], list[int], list[str], bool]]:
    jobs: list[tuple[str, list[str], list[int], list[str], bool]] = []
    if from_gate <= 1:
        for seed in (2, 4, 8):
            jobs.append((
                f"near{seed}",
                ["scripts/run_closed_loop.py", "--seed", str(seed), "--until-settled", "--no-render",
                 "--out", f"runs/readme/near_seed_{seed}"],
                [seed],
                [f"runs/readme/near_seed_{seed}/summary.json", f"runs/readme/near_seed_{seed}/replay.npz"],
                True,
            ))
        for seed in (2, 4, 8):
            jobs.append((
                f"render_near{seed}",
                ["scripts/render_closed_loop.py", f"runs/readme/near_seed_{seed}", "--stride", "2", "--fps", "25"],
                [seed],
                [f"runs/readme/near_seed_{seed}/closed_loop.gif"],
                False,
            ))
    if from_gate <= 2:
        jobs.append((
            "search7",
            ["scripts/run_closed_loop.py", "--config", "configs/sim/d/l_shape_search.yaml",
             "--seed", "7", "--until-settled", "--no-render", "--out", "runs/readme/search_seed7"],
            [7],
            ["runs/readme/search_seed7/summary.json", "runs/readme/search_seed7/replay.npz"],
            True,
        ))
        jobs.append((
            "render_search7",
            ["scripts/render_closed_loop.py", "runs/readme/search_seed7", "--stride", "2", "--fps", "25"],
            [7],
            ["runs/readme/search_seed7/closed_loop.gif"],
            False,
        ))
    if from_gate <= 3:
        jobs.extend([
            ("sweep", ["scripts/evaluate_closed_loop.py", "--seeds", "0..11", "--until-settled",
                       "--out", "runs/readme/d_sweep"], list(range(12)),
             ["runs/readme/d_sweep/g500_sweep.json"], False),
            ("d10_diag", ["scripts/diagnose_redeployment.py", "--seeds", "0..7",
                          "--out", "runs/readme/d10_diag"], list(range(8)),
             ["runs/readme/d10_diag/diagnosis.json", "runs/readme/d10_diag/figA_segments.png",
              "runs/readme/d10_diag/figB_coverage.png"], False),
            ("d10_ab", ["scripts/ab_explore.py", "--seeds", "0..7", "--gains", "0,6",
                        "--out", "runs/readme/d10_ab"], list(range(8)),
             ["runs/readme/d10_ab/ab.json"], False),
            ("d10_enc", ["scripts/diagnose_enclosure_gate.py", "--seeds", "0..7",
                         "--out", "runs/readme/d10_enc"], list(range(8)),
             ["runs/readme/d10_enc/gate.json"], False),
            ("d10_analyse", ["scripts/analyse_enclosure_gate.py", "--run", "runs/readme/d10_enc", "--figure"],
             list(range(8)), ["runs/readme/d10_enc/figF_gate_tradeoff.png"], False),
        ])
    if from_gate <= 4:
        jobs.extend([
            ("priors", ["scripts/run_apriori_centroid_bound.py", "--out", "runs/paper/priors",
                        "--shapes", "l_shape", "rectangle", "c_shape", "--seeds", "2", "--priors-only"],
             [2], ["runs/paper/priors/prior_screen.json"], False),
            ("jeh_matrix", ["scripts/run_apriori_centroid_bound.py", "--out", "runs/paper/jeh_matrix",
                            "--shapes", "l_shape", "rectangle", "c_shape",
                            "--seeds", "2", "5", "8", "11", "17", "23", "29", "31", "37",
                            "--grids", "20", "--frames", "600", "--resume"],
             [2, 5, 8, 11, 17, 23, 29, 31, 37],
             ["runs/paper/jeh_matrix/summary.json", "runs/paper/jeh_matrix/summary.csv",
              "runs/paper/jeh_matrix/prior_screen.json"], False),
            ("jeh_serial", ["scripts/run_apriori_centroid_bound.py", "--out", "runs/paper/jeh_serial_pair",
                            "--shapes", "l_shape", "rectangle", "--seeds", "2", "--grids", "20",
                            "--frames", "600", "--serial", "--resume"],
             [2],
             ["runs/paper/jeh_serial_pair/runs/l_shape/n20/seed_2/trajectory.npz",
              "runs/paper/jeh_serial_pair/runs/rectangle/n20/seed_2/trajectory.npz"], False),
        ])
    if from_gate <= 5:
        jobs.append((
            "gate5",
            ["scripts/run_deployment_pairs.py", "--gate", "5", "--out", "runs/paper/local_boundary_pairs"],
            [2, 5, 8],
            ["runs/paper/local_boundary_pairs/summary.json", "runs/paper/local_boundary_pairs/manifest.json"],
            False,
        ))
    if from_gate <= 6:
        jobs.append((
            "gate6",
            ["scripts/run_deployment_pairs.py", "--gate", "6", "--out", "runs/paper/deployment_transport_pairs"],
            [2, 5, 8],
            ["runs/paper/deployment_transport_pairs/summary.json",
             "runs/paper/deployment_transport_pairs/manifest.json"],
            False,
        ))
    # These raw products feed the published plots/safety tables, so their
    # integrity is part of the experiment receipt as well as the summaries.
    for name, _args, _seeds, artifacts, _allow in jobs:
        if name == "d10_enc":
            artifacts.extend(f"runs/readme/d10_enc/gate_seed{seed}.npz" for seed in range(8))
        if name == "jeh_matrix":
            for shape in ("l_shape", "rectangle", "c_shape"):
                for seed in (2, 5, 8, 11, 17, 23, 29, 31, 37):
                    base = f"runs/paper/jeh_matrix/runs/{shape}/n20/seed_{seed}"
                    artifacts.extend([f"{base}/trajectory.npz", f"{base}/step_records.json"])
    return jobs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--from-gate", type=int, default=1, choices=range(1, 7))
    parser.add_argument("--report-only", action="store_true")
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLBACKEND", "Agg")
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    manifest = {
        "commit": git_sha(),
        "python": platform.python_version(),
        "numpy": numpy.__version__,
        "scipy": scipy.__version__,
        "config_sha256": config_hashes(),
        "from_gate": args.from_gate,
        "note": "Closed-loop non-zero exit records G500 failure, not a crashed run.",
        "runs": [],
    }
    previous = OUT / "readme_manifest.json"
    if previous.exists():
        old = json.loads(previous.read_text(encoding="utf-8"))
        if old.get("runs"):
            manifest["prior_runs"] = old["runs"]
    if args.report_only:
        subprocess.check_call([sys.executable, "-u", "scripts/build_consolidation_report.py"], cwd=ROOT)
        return
    (OUT / "readme_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    for name, cmd, seeds, artifacts, allow_nonzero in commands_for(args.from_gate):
        run(name, cmd, seeds, artifacts, allow_nonzero, manifest)
        if name == "priors":
            check_l_shape_priors()
    subprocess.check_call([sys.executable, "-u", "scripts/build_consolidation_report.py"], cwd=ROOT)
    print(json.dumps({"commit": manifest["commit"], "runs": len(manifest["runs"])}, indent=2), flush=True)


if __name__ == "__main__":
    main()
