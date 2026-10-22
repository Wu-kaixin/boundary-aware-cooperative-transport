"""Paired development regression from the v2 archive and a source snapshot.

Run with the repository Python environment. Uses the same interpreter/packages
for both arms; these are development diagnostics, never new-seed validation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from dbact_sim.experiments.study import environment_versions, source_hashes, cached_result
from dbact_sim.experiments.deployment import encode
from dbact_sim.experiments.reliability import analyse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--baseline-run", type=Path, help="Reuse a hash-checked baseline in the same environment")
    args = parser.parse_args()
    out = args.out.resolve()
    if out.exists() and any(out.iterdir()):
        parser.error("Use a fresh output directory")
    if args.workers < 1:
        parser.error("workers must be positive")
    out.mkdir(parents=True, exist_ok=True)
    archive = ROOT / "docs/results/ccta2027_draft_v2/execution_snapshot.zip"
    with zipfile.ZipFile(archive) as bundle:
        for member in bundle.infolist():
            target = (out / "archive" / member.filename).resolve()
            if not target.is_relative_to(out / "archive"):
                raise ValueError("Archive path escapes output directory")
        bundle.extractall(out / "archive")
    old = out / "archive/execution_source"
    new = out / "revision_source"
    hashes = source_hashes(ROOT)
    for relative in hashes:
        target = new / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, target)
    frozen = yaml.safe_load((out / "archive/frozen_protocol.yaml").read_text(encoding="utf-8-sig"))
    # All nine CVT development cases plus both engines; the duplicate penalty
    # condition is retained to check deterministic execution within each arm.
    cases = [c for c in frozen["cases"] if
             (c["split"] == "development" and c["variant"] == "boundary_cvt") or c["split"] == "engine"]
    protocol = out / "protocol.yaml"
    protocol.write_text(yaml.safe_dump({"version": "reliability-development-2026-10-22", "cases": cases},
                                       sort_keys=False), encoding="utf-8")
    manifest = {"environment": environment_versions(), "revision_source": hashes,
                "baseline_archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
                "protocol_sha256": hashlib.sha256(protocol.read_bytes()).hexdigest(),
                "population": "reused development seeds; combined repair regression; no independent validation"}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    results = {}
    for arm, source in (("baseline", old), ("revision", new)):
        folder = out / arm
        reuse = arm == "baseline" and args.baseline_run is not None
        if reuse:
            folder = args.baseline_run.resolve()
            data = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
            identity = data["manifest"]["identity"]
            if (data.get("source_integrity") != "PASS"
                    or identity["protocol_sha256"] != manifest["protocol_sha256"]
                    or identity["source_sha256"] != encode(source_hashes(old))
                    or identity["environment"] != manifest["environment"]):
                raise RuntimeError("Reused baseline source/protocol/environment identity mismatch")
            expected = {c["case_id"]: c for c in cases}
            if len(data["results"]) != len(cases) or {r["case_id"] for r in data["results"]} != set(expected):
                raise RuntimeError("Reused baseline coverage mismatch")
            for result in data["results"]:
                case = expected[result["case_id"]]
                saved = cached_result(folder / result["case_id"], {**identity, "case_sha256": encode(case)})
                if saved != result:
                    raise RuntimeError("Reused baseline summary differs from receipt")
        env = dict(os.environ, PYTHONPATH=str(source / "src"), DBACT_ROOT=str(source),
                   OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1", DBACT_CVT_WORKERS="1")
        command = [sys.executable, "-m", "dbact_sim", "study", "--protocol", str(protocol),
                   "--split", "all", "--workers", str(args.workers), "--out", str(out / arm)]
        print(f"{'Reusing verified' if reuse else 'Starting'} {arm}: {len(cases)} paired cases", flush=True)
        if not reuse:
            with (out / f"{arm}.log").open("w", encoding="utf-8") as log:
                subprocess.run(command, cwd=source, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
        else:
            (out / "baseline_reuse.json").write_text(json.dumps({"directory": str(folder), "identity": identity}, indent=2), encoding="utf-8")
        receipts = folder / "summary.json"
        results[arm] = {r["case_id"]: r for r in json.loads(receipts.read_text())["results"]}
        report = analyse(receipts, folder)
        (out / f"{arm}_classification.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    pairs = []
    for case in cases:
        identifier = case["case_id"]
        before, after = (results[a][identifier] for a in ("baseline", "revision"))
        paired = (before.get("initial_state_sha256") == after.get("initial_state_sha256")
                  == case["expected_initial_state_sha256"] and before.get("task") == after.get("task")
                  and before.get("execution_status") == after.get("execution_status") == "COMPLETED")
        pairs.append({"case_id": identifier, "paired": paired,
                      "before": before, "after": after})
    comparison = {"manifest": manifest, "all_paired": all(p["paired"] for p in pairs), "pairs": pairs}
    (out / "comparison.json").write_text(json.dumps(comparison, indent=2), encoding="utf-8")
    print(json.dumps({"all_paired": comparison["all_paired"], "pairs": len(pairs)}), flush=True)
    if not comparison["all_paired"]:
        raise RuntimeError("Pair identity or execution gate failed; retain failed receipts")


if __name__ == "__main__":
    main()
