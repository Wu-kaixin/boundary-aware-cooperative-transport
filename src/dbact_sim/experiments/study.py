"""Run isolated cases with source snapshots and integrity-checked resume."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import platform
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

from dbact.cpu_budget import ResourceSampler, limit_blas_threads
from dbact.provenance import git_sha
from dbact_sim.experiments.deployment import dump, encode, static_arm, transport_arm
from dbact_sim.experiments.paths import repository_root
from dbact_sim.experiments.protocol import load_protocol

ROOT = repository_root()


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def source_hashes(root: Path | None = None) -> dict[str, str]:
    root = ROOT if root is None else root
    paths = [p for folder in ("src", "configs") for p in (root / folder).rglob("*")
             if p.is_file() and p.suffix in (".py", ".yaml", ".yml")]
    paths += [root / name for name in ("pyproject.toml", "requirements.txt", "environment.yml")
              if (root / name).is_file()]
    return {p.relative_to(root).as_posix(): digest(p) for p in sorted(paths)}


def environment_versions() -> dict:
    packages = {}
    for name in ("numpy", "scipy", "shapely", "PyYAML", "cvxpy", "clarabel", "osqp", "scs", "pymunk", "matplotlib", "threadpoolctl"):
        try:
            packages[name] = version(name)
        except PackageNotFoundError:
            packages[name] = None
    return {"python": platform.python_version(), "platform": platform.platform(),
            "executable": sys.executable, "packages": packages}


def limit_worker_threads() -> None:
    for name in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "OMP_NUM_THREADS", "DBACT_CVT_WORKERS"):
        os.environ[name] = "1"
    limit_blas_threads(1)


def worker(payload: tuple) -> dict:
    case, folder, identity = payload
    out = Path(folder)
    out.mkdir(parents=True, exist_ok=True)
    limit_worker_threads()
    started = time.perf_counter()
    dump(out / "resolved_case.json", case)
    try:
        runner = static_arm if case["gate"] == 5 else transport_arm
        result = runner(copy.deepcopy(case["config"]), case["seed"], case["variant"], case["frames"], out)
        result["execution_status"] = "COMPLETED"
        if case.get("expected_initial_state_sha256") and result.get("initial_state_sha256") != case["expected_initial_state_sha256"]:
            raise RuntimeError("Initial state differs from the frozen preflight")
        if case["gate"] == 6:
            tolerance = case["config"]["task"]["tolerance"]
            result["task_completed"] = bool(result["settled"] and result["euclidean_endpoint_error"] <= tolerance)
            result["task_completed_definition"] = "settled plus final Euclidean endpoint error <= existing task tolerance"
    except Exception as exc:
        result = {"execution_status": "ERROR", "error": repr(exc), "traceback": traceback.format_exc(),
                  "task_completed": False, "g500_pass": False, "settled": False}
    result.update(case_id=case["case_id"], shape=case["shape"], variant=case["variant"], seed=case["seed"],
                  stratum=case.get("stratum", "near"), condition_id=case.get("condition_id"),
                  arm=case.get("arm",case["variant"]), gate=case["gate"], split=case.get("split"), factors=case.get("factors",{}),
                  wall_seconds=time.perf_counter() - started)
    products = {p.relative_to(out).as_posix(): digest(p) for p in sorted(out.rglob("*"))
                if p.is_file() and p.name != "case_result.json"}
    dump(out / "case_result.json", {"identity": identity, "products": products, "result": result})
    return result


def cached_result(folder: Path, identity: dict) -> dict:
    cached = json.loads((folder / "case_result.json").read_text(encoding="utf-8"))
    if cached.get("identity") != identity:
        raise RuntimeError(f"Case identity mismatch: {folder.name}")
    products = cached.get("products")
    if not isinstance(products, dict) or not products:
        raise RuntimeError(f"Missing product receipt: {folder.name}")
    for name, expected in products.items():
        path = (folder / name).resolve()
        if not path.is_relative_to(folder.resolve()) or not path.is_file() or digest(path) != expected:
            raise RuntimeError(f"Cached product integrity failure: {folder.name}/{name}")
    return cached["result"]


def run_study(protocol_path: Path, split: str, out: Path | None, workers: int, resume: bool) -> dict:
    if workers <= 0:
        raise ValueError("workers must be positive")
    protocol_path = protocol_path.resolve()
    protocol, cases = load_protocol(protocol_path, split, ROOT)
    source = source_hashes()
    source_id = encode(source)
    if protocol.get("frozen_source_sha256") and protocol["frozen_source_sha256"] != source_id:
        raise RuntimeError("Source differs from frozen protocol; preserve the freeze and create a new study")
    out = (out or ROOT / protocol["run_dir"] / split).resolve()
    identity = {"schema": "ccta-study-v2", "protocol_sha256": digest(protocol_path), "split": split,
                "source_sha256": source_id, "environment": environment_versions()}
    manifest_path = out / "manifest.json"
    previous = None
    if manifest_path.exists():
        if not resume:
            raise RuntimeError("Existing study requires --resume or an independent directory")
        previous = json.loads(manifest_path.read_text(encoding="utf-8"))
        if previous.get("identity") != identity:
            raise RuntimeError("Existing study has different identity; use a new directory")
    elif out.exists() and any(out.iterdir()):
        raise RuntimeError("Nonempty output without a manifest; preserve it and use a new directory")
    jobs, results = [], []
    for case in cases:
        folder = out / case["case_id"]
        case_identity = {**identity, "case_sha256": encode(case)}
        if (folder / "case_result.json").is_file():
            results.append(cached_result(folder, case_identity))
        elif folder.exists() and any(folder.iterdir()):
            raise RuntimeError(f"Partial case preserved; use a new study directory: {case['case_id']}")
        else:
            jobs.append((case, str(folder), case_identity))
    # Check every receipt before writing anything into an existing study.
    manifest = {"identity": identity, "source_files": source, "git_sha": git_sha(ROOT),
                "workers": workers, "cases": len(cases),
                "started_utc": previous["started_utc"] if previous else datetime.now(timezone.utc).isoformat()}
    if previous:
        manifest["resumed_utc"] = datetime.now(timezone.utc).isoformat()
    dump(manifest_path, manifest)
    dump(out / "resolved_protocol.json", {**protocol, "cases": cases})
    for name in source:
        target = out / "source_snapshot" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((ROOT / name).read_bytes())
    started = time.perf_counter()

    def summarize() -> dict:
        return {"manifest": manifest, "results": sorted(results, key=lambda row: row["case_id"]),
                "planned_denominator": len(cases), "completed_denominator": len(results),
                "execution_errors": sum(row["execution_status"] != "COMPLETED" for row in results),
                "elapsed_seconds": time.perf_counter() - started}

    if jobs:
        limit_worker_threads()
        sampler = ResourceSampler(out / "resources.csv")
        sampler.start()
        try:
            with ProcessPoolExecutor(max_workers=workers) as pool:
                futures = {pool.submit(worker, payload): payload for payload in jobs}
                for future in as_completed(futures):
                    case, folder, case_identity = futures[future]
                    try:
                        result = future.result()
                    except Exception as exc:
                        result = {"case_id": case["case_id"], "shape": case["shape"], "variant": case["variant"],
                                  "seed": case["seed"], "stratum": case.get("stratum", "near"),
                                  "execution_status": "PROCESS_ERROR", "error": repr(exc),
                                  "task_completed": False, "g500_pass": False, "settled": False}
                        dump(Path(folder) / "process_error.json", result)
                        dump(Path(folder) / "case_result.json", {"identity": case_identity,
                             "products": {"process_error.json": digest(Path(folder) / "process_error.json")}, "result": result})
                    results.append(result)
                    dump(out / "summary.json", summarize())
                    print(json.dumps({"case": result["case_id"], "status": result["execution_status"]}), flush=True)
        finally:
            sampler.stop()
    summary = summarize()
    # Sources can otherwise be edited while a process pool is still importing them.
    # Preserve the measured rows but invalidate their attribution instead of
    # claiming that a mixed-source run belongs to the start-of-run snapshot.
    summary["source_integrity"] = "PASS" if source_hashes() == source else "SOURCE_CHANGED"
    dump(out / "summary.json", summary)
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, required=True)
    parser.add_argument("--split", required=True)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--dry-run", action="store_true", help="Resolve and validate without running or writing")
    args = parser.parse_args(argv)
    if args.workers <= 0:
        parser.error("--workers must be positive")
    if args.dry_run:
        protocol, cases = load_protocol(args.protocol, args.split, ROOT)
        frozen = protocol.get("frozen_source_sha256")
        print(json.dumps({"split": args.split, "cases": len(cases), "case_ids": [c["case_id"] for c in cases],
                          "source_matches_freeze": encode(source_hashes()) == frozen if frozen else None}, indent=2))
        return 0
    summary = run_study(args.protocol, args.split, args.out, args.workers, args.resume)
    return 1 if summary["execution_errors"] or summary["source_integrity"] != "PASS" else 0


if __name__ == "__main__":
    raise SystemExit(main())
