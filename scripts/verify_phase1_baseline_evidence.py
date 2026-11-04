"""Verify preserved Phase 1 evidence in a fresh output directory, without physics."""

from __future__ import annotations

import argparse
import json
import shutil
import traceback
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import post_acquisition_support_study as study
import verify_post_support_onsets as onset_verifier
from audit_phase1_baseline import digest
from research_io import ROOT, guard, read


def save(path: Path, value: dict) -> None:
    with path.open("x", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--pusher-run", type=Path, required=True)
    args = parser.parse_args()
    output, pusher = args.out.resolve(), args.pusher_run.resolve()
    if not all(path.is_relative_to(ROOT / "runs") for path in (output, pusher)):
        raise ValueError("All analysis inputs/outputs must stay under repository runs/")
    guard()
    output.mkdir(parents=True, exist_ok=False)
    sources = [
        Path(__file__),
        ROOT / "scripts/audit_phase1_baseline.py",
        Path(onset_verifier.__file__),
    ]
    source_hashes = {
        path.relative_to(ROOT).as_posix(): digest(path) for path in sources
    }
    for path in sources:
        shutil.copy2(path, output / path.name)
    save(
        output / "started.json",
        {
            "started_at_utc": datetime.now(timezone.utc).isoformat(),
            "sources": source_hashes,
            "pusher_run": str(pusher),
            "new_physical_episodes": 0,
        },
    )
    checked = []

    def verify(path: Path, expected: str, group: str) -> None:
        actual = digest(path)
        if actual.lower() != expected.lower():
            raise ValueError(f"Hash mismatch: {path.relative_to(ROOT)}")
        checked.append(
            {
                "path": path.relative_to(ROOT).as_posix(),
                "sha256": actual,
                "bytes": path.stat().st_size,
                "group": group,
            }
        )

    try:
        plan = study.check()
        ready = ROOT / "docs/results/readiness_diagnostic_v1"
        ready_id = read(ready / "execution_identity.json")
        raw_manifest = (
            ROOT / "artifacts/research/2026-10-31-readiness-v1/raw_integrity.json"
        )
        verify(
            raw_manifest,
            ready_id["raw_integrity_manifest_sha256"],
            "readiness_manifest",
        )
        for name, expected in read(raw_manifest).items():
            verify(ROOT / name, expected, "readiness_raw")
        for name, expected in ready_id["derived_csv_hashes"].items():
            verify(ready / name, expected, "readiness_published")
        verify(
            ready / "paired_results_original.json",
            ready_id["results_sha256"],
            "readiness_published",
        )
        progress = ROOT / "docs/results/progress_local_v2"
        progress_id = read(progress / "identity.json")
        for name, expected in progress_id["derived"].items():
            verify(progress / name, expected, "progress_published")
        for phase in progress_id["raw"].values():
            for name, expected in phase.items():
                verify(
                    ROOT / "runs/research/2026-10-31-progress-local-v2" / name,
                    expected,
                    "progress_raw",
                )
        for manifest_name in (
            "docs/results/post_support_validation_checkpoint_v1/receipt_publication.json",
            "docs/results/post_support_contact_geometry_capability_v1/publication.json",
        ):
            manifest = ROOT / manifest_name
            for name, expected in read(manifest)["products"].items():
                verify(manifest.parent / name, expected, "published_support_evidence")
        inputs = read(onset_verifier.ONSET / "input_hashes.json")
        if set(inputs) != {case["case_id"] for case in plan["cases"]}:
            raise ValueError(
                "Frozen eight-case denominator differs from onset input manifest"
            )
        for case_id, identity in inputs.items():
            for name, expected in identity["products"].items():
                verify(study.OUT / case_id / name, expected, "frozen_eight_raw")
        save(
            output / "integrity.json",
            {
                "status": "PASS",
                "entries": checked,
                "counts": dict(Counter(item["group"] for item in checked)),
                "bytes_hashed": sum(item["bytes"] for item in checked),
            },
        )

        # Change only file routing. Every assertion and scientific computation in
        # the existing independent verifier remains intact. Never write its old OUT.
        onset_verifier.PUSH = pusher
        onset_verifier.OUT = output / "independent_onset_verification.json"
        onset_verifier.main()
        verified = read(onset_verifier.OUT)
        plan = study.check()
        for name, expected in source_hashes.items():
            if digest(ROOT / name) != expected:
                raise ValueError(f"Verification source changed: {name}")

        completed, prefixes = [], []
        for path in sorted((study.BASE / "analysis_attempt").glob("*.json")):
            item = read(path)
            if path.name.endswith("-prefix.json"):
                prefixes.append(
                    {
                        "path": path.relative_to(ROOT).as_posix(),
                        "frames": item.get("frames"),
                        "sha256": digest(path),
                    }
                )
            elif "case_id" in item:
                audit = item.get("post_support_audit") or {}
                completed.append(
                    {
                        "case_id": item["case_id"],
                        "sha256": digest(path),
                        "support_full_episode_verified": audit.get(
                            "full_episode_verified", False
                        ),
                        "support_frames": audit.get("frames"),
                        "capped_object_violations": audit.get(
                            "capped_object_violations"
                        ),
                        "uncapped_object_violations": audit.get(
                            "uncapped_object_violations"
                        ),
                        "hard_violations": audit.get("hard_violations"),
                    }
                )
        status = {
            "status": "VERIFIED_PHASE1_BASELINE_AUDIT",
            "completed_at_utc": datetime.now(timezone.utc).isoformat(),
            "runtime_source_sha256": plan["frozen_source_sha256"],
            "protocol_sha256": digest(study.PROTOCOL),
            "canonical_config_sha256": plan["canonical_sha256"],
            "sources": source_hashes,
            "integrity_entries": len(checked),
            "recorded_frames": verified["recorded_frames"],
            "systems": verified["systems"],
            "new_physical_episodes": 0,
            "original_full_audit": {
                "completed_case_rows": completed,
                "prefixes": prefixes,
                "final_analysis_exists": (study.BASE / "analysis.json").exists(),
                "final_outcome_exists": (study.BASE / "analysis_outcome.json").exists(),
                "scope": "Preserved partial attempt; this verifier does not complete the full constraint audit",
            },
        }
        save(output / "outcome.json", status)
        print(
            json.dumps(
                {
                    key: status[key]
                    for key in (
                        "status",
                        "integrity_entries",
                        "recorded_frames",
                        "systems",
                        "new_physical_episodes",
                    )
                }
            ),
            flush=True,
        )
    except Exception:
        save(
            output / "failure.json",
            {
                "status": "FAILED",
                "sources": source_hashes,
                "verified_entries": len(checked),
                "traceback": traceback.format_exc(),
            },
        )
        raise


if __name__ == "__main__":
    main()
