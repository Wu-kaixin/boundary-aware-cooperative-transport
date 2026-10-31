"""Read-only, bounded comparison of the retained PC3 containers with local backups.

This recognizes a verified reference prefix followed by a sparse periodic fill.
It never infers lost content from a filename, repairs an original, or deletes it.
An explained byte pattern is distinct from proof of pre-corruption identity.
Reports must be new and stay inside the original mission's local audit directory.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.maintenance.reconcile_sources import ROOT, confined, read_rows, source_path

PERIOD = 16384
CHUNK = 1024 * 1024


def compare_streams(source, reference, pattern: bytes, chunk_size=CHUNK):
    """Check every byte; report unexplained residuals rather than repairing them."""
    if not pattern or chunk_size <= 0:
        raise ValueError("Need a nonempty pattern and positive chunk size")
    offset = matched = zero_bytes = 0
    first_difference = None
    last_nonperiodic = -1
    source_hash, reference_hash = hashlib.sha256(), hashlib.sha256()
    while True:
        data, good = source.read(chunk_size), reference.read(chunk_size)
        if len(data) != len(good):
            raise ValueError("Reference and source lengths differ")
        if not data:
            break
        source_hash.update(data)
        reference_hash.update(good)
        start = offset % len(pattern)
        fill = (pattern * ((start + len(data) + len(pattern) - 1) // len(pattern)))[start:start + len(data)]
        values = np.frombuffer(data, dtype=np.uint8)
        differs = values != np.frombuffer(good, dtype=np.uint8)
        count = int(np.count_nonzero(differs))
        if first_difference is None and count:
            first_difference = offset + int(np.flatnonzero(differs)[0])
        matched += len(data) - count
        zero_bytes += int(np.count_nonzero(values == 0))
        nonperiodic = np.flatnonzero(values != np.frombuffer(fill, dtype=np.uint8))
        if len(nonperiodic):
            last_nonperiodic = offset + int(nonperiodic[-1])
        offset += len(data)
    prefix = offset if first_difference is None else first_difference
    fill_start = last_nonperiodic + 1
    return {
        "bytes": offset, "sha256": source_hash.hexdigest(),
        "reference_sha256": reference_hash.hexdigest(),
        "reference_matching_bytes": matched, "zero_bytes": zero_bytes,
        "reference_prefix_bytes": prefix, "periodic_fill_start": fill_start,
        "fully_explained": fill_start <= prefix,
        "classification": "EXACT_REFERENCE" if prefix == offset else
            "PERIODIC_FILL_ONLY" if fill_start == 0 else
            "REFERENCE_PREFIX_AND_FILL" if fill_start <= prefix else "UNEXPLAINED_BYTES",
        "pattern_sha256": hashlib.sha256(pattern).hexdigest(),
    }


def sparse_pattern(data: bytes):
    # The marker and low occupancy are an observed damage signature, not a
    # general rule that sparse files are disposable. Original bytes stay archived.
    if len(data) != PERIOD or not data.startswith(b"\x03\x02\x01\xff") or sum(b != 0 for b in data) > 64:
        raise ValueError("Not the reviewed sparse periodic damage signature")
    return data


def audit(mission: Path, output: Path):
    mission = confined(mission.resolve().relative_to(ROOT).as_posix())
    output = confined(output.resolve().relative_to(ROOT).as_posix())
    if not mission.is_relative_to(ROOT / "artifacts/migrations") or not output.is_relative_to(mission):
        raise ValueError("Reports must stay in the existing local audit mission")
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "byte_comparison.json"
    if report_path.exists():
        raise FileExistsError("Preserve the prior comparison; use a new attempt")
    rows = read_rows(mission / "retained_damaged_sources.csv")
    allowed_names = {"development-568b1fc.zip", "local-tracked-before.zip",
                     "repository-before-sync.bundle", "transfer-originals.zip"}

    def check(row):
        if row["source"] != "PC3":
            raise ValueError("Only retained PC3 sources are in scope")
        path = source_path(row)
        if path.name not in allowed_names:
            raise ValueError("Unreviewed container family")
        reference = confined("artifacts/migrations/2026-10-23/" + path.name)
        pattern_file = confined((path.parent / "local-tracked-before.zip").relative_to(ROOT).as_posix())
        before = (path.stat(), reference.stat(), pattern_file.stat())
        with pattern_file.open("rb") as stream:
            pattern = sparse_pattern(stream.read(PERIOD))
        with path.open("rb") as source, reference.open("rb") as good:
            result = compare_streams(source, good, pattern)
        after = (path.stat(), reference.stat(), pattern_file.stat())
        if any((a.st_size, a.st_mtime_ns) != (b.st_size, b.st_mtime_ns) for a, b in zip(before, after)):
            raise RuntimeError("Source, reference or pattern changed during verification")
        if result["sha256"] != row["sha256"] or result["bytes"] != int(row["size"]):
            raise RuntimeError("Source changed from the retained-original inventory")
        result.update(original_path=row["original_path"], reference_path=reference.relative_to(ROOT).as_posix(),
                      raw_recovery=row["final_location"],
                      identity_limit="Pattern coverage does not prove the exact pre-corruption contents; keep original bytes")
        print(path.relative_to(ROOT), result["classification"], "prefix", result["reference_prefix_bytes"], flush=True)
        return result

    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(check, rows))
    report = {"scope": "All bytes of every retained original; no archive execution or repair",
              "status": "PASS" if all(r["fully_explained"] for r in results) else "RETAIN_UNEXPLAINED",
              "files": results}
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    with (output / "byte_comparison.csv").open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    raise SystemExit(audit(args.mission, args.output))
