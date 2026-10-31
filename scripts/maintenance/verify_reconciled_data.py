"""Verify old identities through the new mapping, without rewriting old receipts.

Arrays receive header/shape/dtype checks only, with no pickle execution. This
is preservation and format validation, not a new analysis of every trajectory.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
from zipfile import ZipFile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def run(mission, output):
    mission = mission.resolve()
    output = output.resolve()
    if mission.parent != ROOT / "artifacts/migrations" or not output.is_relative_to(ROOT / "runs/maintenance"):
        raise ValueError("Use this repository's migration and maintenance directories")
    if output.exists():
        raise FileExistsError(output)
    mappings = list(csv.DictReader((mission / "migration_manifest.csv").open(encoding="utf-8")))
    pc2 = {r["relative_path"]: r["final_location"] for r in mappings if r["source"] == "PC2"}
    archives, cache, failures = {}, {}, []

    def sha(location):
        if location in cache:
            return cache[location]
        if "::" in location:
            name, member = location.split("::", 1)
            archive = archives.setdefault(name, None)
            if archive is None:
                archive = archives[name] = ZipFile(ROOT / name)
            with archive.open(member) as stream:
                value = hashlib.file_digest(stream, "sha256").hexdigest()
        else:
            with (ROOT / location).open("rb") as stream:
                value = hashlib.file_digest(stream, "sha256").hexdigest()
        cache[location] = value
        return value

    def verify(location, expected, group):
        try:
            if sha(location) != expected.lower():
                failures.append({"group": group, "location": location, "error": "SHA256_MISMATCH"})
        except (OSError, KeyError) as exc:
            failures.append({"group": group, "location": location, "error": str(exc)})

    receipts = ROOT / "docs/migrations/2026-10-23"
    transfer = [json.loads(line) for line in (receipts / "transfer_mapping.jsonl").read_text(encoding="utf-8").splitlines()]
    for row in transfer:
        verify(row["destination"], row["sha256"], "historical_transfer")
    print("Historical transfer checked", len(transfer), flush=True)
    local = [json.loads(line) for line in (receipts / "local_inventory_before.jsonl").read_text(encoding="utf-8").splitlines()]
    backup_name = "artifacts/migrations/2026-10-23/local-tracked-before.zip"
    with ZipFile(ROOT / backup_name) as archive:
        members = set(archive.namelist())
    excluded = []
    for row in local:
        name = row["path"]
        if name.startswith("scripts/maintenance/"):
            excluded.append(name)
            continue
        location = backup_name + "::" + name if name in members else pc2.get(name, name)
        verify(location, row["sha256"], "historical_local")
    print("Historical local inventory checked", len(local), flush=True)
    literature = json.loads((ROOT / "docs/literature_review_2026-10-22/source_manifest.json").read_text(encoding="utf-8"))
    for row in literature:
        candidates = list((ROOT / "data/literature/2026-10-22/pdfs").glob(row["reading_id"] + ".*"))
        if len(candidates) != 1:
            failures.append({"group": "literature", "id": row["reading_id"], "error": "MISSING_OR_AMBIGUOUS"})
        else:
            verify(candidates[0].relative_to(ROOT).as_posix(), row["sha256"], "literature")
    frozen = 0
    for group in ("ccta2027_draft_v2", "reliability_2026-10-22"):
        base = ROOT / "docs/results" / group
        for name, expected in json.loads((base / "artifact_integrity.json").read_text(encoding="utf-8"))["files"].items():
            verify((base / name).relative_to(ROOT).as_posix(), expected, "frozen_evidence")
            frozen += 1
    array_count, members_count, object_headers = 0, 0, []
    array_errors = []
    # These exact saved pytest outputs are intentionally invalid inputs, as
    # written by tests/test_consolidation_reproducibility.py. They are retained
    # and identified individually, never treated as failed research episodes.
    fixtures = {
        "runs/evidence_closure/pytest_full/test_resume_requires_config_an0/trajectory.npz": b"corrupt",
        "runs/evidence_closure/pytest_full/test_render_receipt_tracks_rep0/run/replay.npz": b"new",
    }
    fixture_checks = []
    for path in (ROOT / "runs").rglob("*.npz"):
        if path.is_relative_to(ROOT / "runs/maintenance"):
            continue
        name = path.relative_to(ROOT).as_posix()
        if name in fixtures and path.read_bytes() == fixtures[name]:
            fixture_checks.append({"path": name, "status": "NOT_APPLICABLE",
                                   "basis": "Exact intentional corruption fixture in test_consolidation_reproducibility.py"})
            continue
        array_count += 1
        try:
            with ZipFile(path) as archive:
                for info in archive.infolist():
                    if not info.filename.endswith(".npy"):
                        continue
                    with archive.open(info) as stream:
                        version = np.lib.format.read_magic(stream)
                        reader = {(1, 0): np.lib.format.read_array_header_1_0,
                                  (2, 0): np.lib.format.read_array_header_2_0}.get(version)
                        if reader is None:
                            raise ValueError(f"Unsupported NPY header version {version}; retain without deserializing")
                        shape, fortran, dtype = reader(stream, max_header_size=10000)
                    members_count += 1
                    if dtype.hasobject:
                        object_headers.append({"path": str(path.relative_to(ROOT)), "member": info.filename,
                                               "status": "OBJECT_ARRAY_NOT_DESERIALIZED"})
        except Exception as exc:
            array_errors.append({"path": str(path.relative_to(ROOT)), "error": str(exc)})
    failures.extend({"group": "array_header", **item} for item in array_errors)
    # Only fields actually present in receipts are reported; never infer seed,
    # method, commit or execution/analysis identity from directory spelling.
    batches = []
    for path in (ROOT / "runs").rglob("manifest.json"):
        if path.is_relative_to(ROOT / "runs/maintenance") or "remote-mirror.git" in path.parts:
            continue
        try:
            payload = json.loads(path.read_text(encoding="utf-8-sig"))
            batches.append({"receipt": str(path.relative_to(ROOT)), "git_sha": payload.get("git_sha", "unknown"),
                            "identity": payload.get("identity", "unknown"), "cases": payload.get("cases", "unknown"),
                            "analysis_version": payload.get("analysis_version", "unknown")})
        except (ValueError, OSError, AttributeError) as exc:
            failures.append({"group": "manifest", "path": str(path.relative_to(ROOT)), "error": str(exc)})
    output.parent.mkdir(parents=True, exist_ok=True)
    batch_path = output.with_name("historical_batch_index.json")
    with batch_path.open("x", encoding="utf-8") as stream:
        json.dump(batches, stream, ensure_ascii=False, indent=2)
    case_count = 0
    with output.with_name("historical_case_index.csv").open("x", encoding="utf-8", newline="") as stream:
        fields = ("receipt", "case_id", "seed", "method", "split", "execution_status",
                  "protocol_sha256", "source_sha256", "analysis_version", "products")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for path in (ROOT / "runs").rglob("case_result.json"):
            if path.is_relative_to(ROOT / "runs/maintenance"):
                continue
            try:
                payload = json.loads(path.read_text(encoding="utf-8-sig"))
                result = payload.get("result", payload)
                identity = payload.get("identity", {})
                writer.writerow({"receipt": path.relative_to(ROOT).as_posix(),
                    "case_id": result.get("case_id", "unknown"), "seed": result.get("seed", "unknown"),
                    "method": result.get("variant", "unknown"), "split": result.get("split", "unknown"),
                    "execution_status": result.get("execution_status", "unknown"),
                    "protocol_sha256": identity.get("protocol_sha256", "unknown"),
                    "source_sha256": identity.get("source_sha256", "unknown"),
                    "analysis_version": payload.get("analysis_version", "unknown"),
                    "products": json.dumps(payload.get("products", "unknown"), ensure_ascii=False)})
                case_count += 1
            except (ValueError, OSError, AttributeError) as exc:
                failures.append({"group": "case_receipt", "path": str(path.relative_to(ROOT)), "error": str(exc)})
    manuscript = ROOT / "manuscript/ccta2027/main.tex"
    tex = manuscript.read_text(encoding="utf-8")
    cited = {k.strip() for body in re.findall(r"\\cite(?:\[[^]]*\])?\{([^}]+)\}", tex) for k in body.split(",")}
    keys = set(re.findall(r"\\bibitem(?:\[[^]]*\])?\{([^}]+)\}", tex))
    for bibliography in manuscript.parent.glob("*.bib"):
        keys.update(re.findall(r"@\w+\s*\{\s*([^,]+),", bibliography.read_text(encoding="utf-8")))
    missing = sorted(cited - keys)
    if missing:
        failures.append({"group": "manuscript_citations", "missing": missing})
    figures = []
    for name in re.findall(r"\\includegraphics(?:\[[^]]*\])?\{([^}]+)\}", tex):
        path = manuscript.parent / name
        found = path.exists() or any(path.with_suffix(suffix).exists() for suffix in (".pdf", ".png", ".jpg"))
        figures.append({"path": name, "exists": found})
        if not found:
            failures.append({"group": "manuscript_figure", "missing": name})
    for archive in archives.values():
        archive.close()
    report = {"status": "PASS" if not failures else "FAIL", "historical_transfer_files": len(transfer),
              "historical_local_files": len(local), "historical_tool_exclusions": excluded,
              "literature_originals": len(literature), "frozen_evidence_files": frozen,
              "npz_files_header_checked": array_count, "npy_members_header_checked": members_count,
              "known_intentional_invalid_fixtures": fixture_checks,
              "array_scope": "Header, shape and dtype only; no full trajectory/frame review, no pickle",
              "object_array_headers": object_headers, "historical_manifests_indexed": len(batches),
              "historical_case_receipts_indexed": case_count,
              "manuscript_citation_keys": len(cited), "manuscript_figures": figures,
              "failures": failures}
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(json.dumps({k: v for k, v in report.items() if k not in {"object_array_headers", "failures"}}, ensure_ascii=False), flush=True)
    print("failures", len(failures), flush=True)
    return int(bool(failures))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    raise SystemExit(run(args.mission, args.output))
