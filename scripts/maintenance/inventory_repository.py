"""Read-only, non-following inventory for the in-place DBACT reconciliation.

Run with the root .venv. Source payloads are never imported or executed. CSV is
the authoritative inventory; the summary only aggregates it. Existing output is
not replaced. Generated environment/audit/validation payloads are separate from
the original sources. Hashing checks metadata before and after each read.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import csv
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess

ROOT = Path(__file__).resolve().parents[2]
FIELDS = ("source", "original_path", "relative_path", "file_type", "size", "mtime_ns",
          "git_status", "sha256", "hash_status", "purpose", "review_scope",
          "decision", "basis", "final_location", "verification")


def io_path(path):
    """Use the Windows extended path spelling without changing stored identity."""
    return Path("\\\\?\\" + str(path.absolute())) if os.name == "nt" else path


def git(base, *args):
    return subprocess.run(["git", "--no-optional-locks", "-C", str(base), *args],
                          capture_output=True, env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"})


def purpose(name):
    parts = Path(name).parts
    if ".git" in parts:
        return "git_metadata"
    if ".venv" in parts or "venv" in parts:
        return "copied_environment_do_not_execute"
    if any(p in {"__pycache__", ".pytest_cache", ".ruff_cache"} for p in parts):
        return "cache_candidate"
    if name.startswith(("runs/", "platforms/mas_public/data/")):
        return "historical_run"
    if name.startswith("artifacts/migrations/"):
        return "prior_recovery_material"
    if name.startswith(("data/literature/", "manuscript/", "theory/", "docs/")):
        return "research_material"
    if name.startswith(("src/", "scripts/", "configs/", "experiments/", "platforms/")):
        return "code_configuration_platform"
    return "repository_material"


def scan(base, exclusions):
    stack = [base]
    while stack:
        directory = stack.pop()
        try:
            entries = sorted(os.scandir(io_path(directory)), key=lambda e: e.name)
        except OSError as exc:
            yield directory, "unreadable_directory", str(exc)
            continue
        for entry in entries:
            path = directory / entry.name
            name = path.relative_to(base).as_posix()
            if name in exclusions:
                continue
            try:
                metadata = entry.stat(follow_symlinks=False)
                attrs = getattr(metadata, "st_file_attributes", 0)
                if entry.is_symlink() or attrs & (stat.FILE_ATTRIBUTE_REPARSE_POINT | stat.FILE_ATTRIBUTE_OFFLINE):
                    yield path, "link_or_offline", "Not followed or hydrated"
                elif entry.is_dir(follow_symlinks=False):
                    yield path, "directory", ""
                    stack.append(path)
                else:
                    yield path, "file", ""
            except OSError as exc:
                yield path, "unreadable", str(exc)


def run(output):
    output = output.resolve()
    allowed = ROOT / "artifacts/migrations"
    if not output.is_relative_to(allowed) or output == allowed:
        raise ValueError("Inventory output must be inside artifacts/migrations/<mission>")
    output.mkdir(parents=True, exist_ok=True)
    destination = output / "inventory.csv"
    if destination.exists():
        raise FileExistsError("Keep the original inventory; use it when resuming")
    sources = {"MAIN": ROOT, "PC2": ROOT / "PC2" / ROOT.name,
               "PC3": ROOT / "PC3" / ROOT.name}
    generated = {".venv", "PC2", "PC3", "runs/maintenance", output.relative_to(ROOT).as_posix()}
    summary = {"sources": {}, "exclusions_main": sorted(generated), "scope":
               "All visible entries without following links; content hashes, not full semantic review"}
    with destination.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for source, base in sources.items():
            if not base.exists():
                summary["sources"][source] = {"status": "NOT_PRESENT"}
                continue
            tracked_result = git(base, "ls-files", "-z")
            if tracked_result.returncode:
                raise RuntimeError(tracked_result.stderr.decode(errors="replace"))
            tracked = set(tracked_result.stdout.decode("utf-8").strip("\0").split("\0"))
            ignored_result = git(base, "ls-files", "--others", "--ignored", "--exclude-standard", "-z")
            if ignored_result.returncode:
                raise RuntimeError(ignored_result.stderr.decode(errors="replace"))
            ignored = set(ignored_result.stdout.decode("utf-8").strip("\0").split("\0"))
            history = output / "backup" / source
            history.mkdir(parents=True, exist_ok=True)
            for name, args in {"status": ("status", "--porcelain=v2", "--branch"),
                               "refs": ("show-ref",), "head": ("rev-parse", "HEAD"),
                               "worktrees": ("worktree", "list", "--porcelain"),
                               "worktree_diff": ("diff", "--binary"),
                               "index_diff": ("diff", "--cached", "--binary")}.items():
                result = git(base, *args)
                (history / (name + ".txt")).write_bytes(result.stdout)
                if result.returncode:
                    (history / (name + ".error.txt")).write_bytes(result.stderr)
            counts, total, seen, issues = Counter(), 0, set(), []

            def record(item):
                path, kind, error = item
                name = path.relative_to(base).as_posix()
                row = dict.fromkeys(FIELDS, "")
                row.update(source=source, original_path=path.relative_to(ROOT).as_posix(),
                           relative_path=name, file_type=kind, purpose=purpose(name),
                           git_status="tracked" if name in tracked else "ignored" if name in ignored else "untracked",
                           review_scope="metadata and streaming SHA-256", decision="retain_pending_review",
                           final_location=path.relative_to(ROOT).as_posix(), verification="PENDING")
                if kind != "file":
                    row.update(hash_status=error or "DIRECTORY", review_scope="metadata only")
                    return row
                try:
                    before = io_path(path).stat()
                    row.update(size=before.st_size, mtime_ns=before.st_mtime_ns)
                    digest = hashlib.sha256()
                    with io_path(path).open("rb") as handle:
                        for block in iter(lambda: handle.read(1024 * 1024), b""):
                            digest.update(block)
                    after = io_path(path).stat()
                    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                        row["hash_status"] = "CHANGED_DURING_READ"
                    else:
                        row.update(sha256=digest.hexdigest(), hash_status="PASS", verification="HASHED")
                except OSError as exc:
                    row["hash_status"] = str(exc)
                return row

            with ThreadPoolExecutor(max_workers=4) as pool:
                for row in pool.map(record, scan(base, generated if source == "MAIN" else set())):
                    writer.writerow(row)
                    seen.add(row["relative_path"])
                    counts[row["file_type"]] += 1
                    total += int(row["size"] or 0)
                    if row["hash_status"] not in {"PASS", "DIRECTORY"}:
                        issues.append({"path": row["original_path"], "status": row["hash_status"]})
            missing = sorted(name for name in tracked if name and name not in seen)
            (history / "missing_tracked.json").write_text(json.dumps(missing, indent=2), encoding="utf-8")
            summary["sources"][source] = {"counts": dict(counts), "bytes": total,
                                           "issues": issues, "missing_tracked": len(missing)}
            stream.flush()
            print(source, dict(counts), total, "bytes; issues", len(issues), flush=True)
    (output / "inventory_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    run(args.output)
