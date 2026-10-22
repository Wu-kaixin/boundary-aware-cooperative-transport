"""Evidence-preserving, one-time reconciliation of the 2026-10-23 transfer.

Does not delete the transfer directory or change Git refs. Run prepare, integrate,
then verify; inspect the receipts before removing the explicitly named source.
"""
from __future__ import annotations

import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
from zipfile import ZipFile, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "其他电脑转移" / ROOT.name
DATE = "2026-10-23"
RECEIPTS = ROOT / "docs" / "migrations" / DATE
ARCHIVES = ROOT / "artifacts" / "migrations" / DATE
REVISION = "568b1fc1573af0989273a12477b9b1ad58cb7998"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def save(name, value):
    RECEIPTS.mkdir(parents=True, exist_ok=True)
    (RECEIPTS / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def inventory(base, exclude=(), access_errors=None):
    files, directories = [], []
    def fail(error):
        if access_errors is None:
            raise error
        access_errors.append({"path": str(error.filename), "error": str(error), "action": "left_untouched"})
    for directory, children, names in os.walk(base, onerror=fail):
        if Path(directory) == base:
            children[:] = [name for name in children if name not in exclude]
        for name in children:
            path = Path(directory) / name
            if path.is_symlink() or path.is_junction():
                raise RuntimeError(f"Reparse point requires manual review: {path}")
            directories.append(path.relative_to(base).as_posix())
        files.extend(Path(directory) / name for name in names)
    def record(path):
        return {"path": path.relative_to(base).as_posix(), "bytes": path.stat().st_size, "sha256": digest(path)}
    with ThreadPoolExecutor(max_workers=4) as pool:
        rows = list(pool.map(record, sorted(files)))
    return rows, sorted(directories)


def write_rows(name, rows):
    with (RECEIPTS / name).open("w", encoding="utf-8", newline="\n") as stream:
        for row in rows:
            stream.write(json.dumps(row, ensure_ascii=False) + "\n")


def read_rows(name):
    return [json.loads(line) for line in (RECEIPTS / name).read_text(encoding="utf-8").splitlines()]


def git(*args):
    return subprocess.check_output(["git", "-C", str(SOURCE), *args])


def prepare():
    if (RECEIPTS / "source_history.json").exists():
        raise RuntimeError("Preparation already complete; do not overwrite the original inventory")
    if not SOURCE.is_dir() or SOURCE.resolve().parent != ROOT / "其他电脑转移":
        raise RuntimeError("Unexpected transfer path")
    RECEIPTS.mkdir(parents=True, exist_ok=True)
    ARCHIVES.mkdir(parents=True, exist_ok=True)
    git("merge-base", "--is-ancestor", "e4b0a19", REVISION)
    if (RECEIPTS / "transfer_inventory.jsonl").exists():
        rows = read_rows("transfer_inventory.jsonl")
    else:
        rows, directories = inventory(SOURCE)
        write_rows("transfer_inventory.jsonl", rows)
        save("transfer_directories.json", directories)
    print(f"Inventoried transfer: {len(rows)} files, {sum(row['bytes'] for row in rows)} bytes", flush=True)
    access_errors = []
    local, _ = inventory(ROOT, {"其他电脑转移", ".git", ".venv", ".worktrees"}, access_errors)
    save("local_access_boundaries.json", access_errors)
    # Receipts and archives are produced by this reconciliation, not initial data.
    local = [row for row in local if not row["path"].startswith((f"docs/migrations/{DATE}/", f"artifacts/migrations/{DATE}/"))]
    write_rows("local_inventory_before.jsonl", local)
    tracked = subprocess.check_output(["git", "ls-files", "-z"], cwd=ROOT).decode("utf-8").split("\0")
    with ZipFile(ARCHIVES / "local-tracked-before.zip", "x", ZIP_DEFLATED, compresslevel=6) as archive:
        for name in tracked:
            if name and (ROOT / name).is_file():
                archive.write(ROOT / name, name)
    git("archive", "--format=zip", f"--output={ARCHIVES / 'development-568b1fc.zip'}", REVISION)
    save("source_history.json", {
        "local_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT).decode().strip(),
        "transfer_head": git("rev-parse", "HEAD").decode().strip(),
        "integrated_revision": REVISION,
        "transfer_refs": git("show-ref").decode().splitlines(),
        "transfer_files": len(rows), "transfer_bytes": sum(row["bytes"] for row in rows),
        "local_files_before": len(local), "local_bytes_before": sum(row["bytes"] for row in local),
        "local_inventory_exclusions": [".git (untouched metadata)", ".venv (new environment)", ".worktrees (only stale inaccessible pytest cache; left untouched)", "其他电脑转移 (separate complete inventory)"],
    })
    print(f"Prepared baseline and development archives; local inventory: {len(local)} files", flush=True)


def destination(name):
    parts = PurePosixPath(name).parts
    if any(part in {".git", ".venv", ".pytest_cache", "__pycache__"} or part.endswith(".egg-info") for part in parts):
        return None
    if name == "manuscript/ccta2027/main.log":
        return "artifacts/manuscript/ccta2027/2026-10-22/main.log"
    if name.startswith("tmp/literature/"):
        tail = name.removeprefix("tmp/literature/")
        if tail.endswith(".py"):
            return "scripts/literature/" + tail
        return "data/literature/2026-10-22/" + tail
    if name.startswith("tmp/pdfs/"):
        return "artifacts/manuscript/ccta2027/2026-10-22/" + name.removeprefix("tmp/pdfs/")
    if name.startswith("tmp/tectonic/"):
        return "tools/tectonic/" + name.removeprefix("tmp/tectonic/")
    if name.startswith("tmp/"):
        return "artifacts/validation/2026-10-22/" + name.removeprefix("tmp/")
    return name


def integrate():
    if (RECEIPTS / "transfer_mapping.jsonl").exists():
        raise RuntimeError("Integration already recorded; verify instead of repeating")
    rows = read_rows("transfer_inventory.jsonl")
    mapping = []
    # Preserve every conflicting original, interpreter file, Git object and cache.
    with ZipFile(ARCHIVES / "transfer-originals.zip", "x", ZIP_DEFLATED, compresslevel=6) as archive:
        for row in rows:
            name = row["path"]
            source = SOURCE / name
            if digest(source) != row["sha256"]:
                raise RuntimeError(f"Transfer changed after inventory: {name}")
            target = destination(name)
            if target is not None and not (ROOT / target).exists():
                (ROOT / target).parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(source, ROOT / target)
                action = "copied"
            elif target is not None and (ROOT / target).is_file() and digest(ROOT / target) == row["sha256"]:
                action = "identical"
            else:
                archive.write(source, name)
                target = f"artifacts/migrations/{DATE}/transfer-originals.zip::{name}"
                action = "archived_original"
            mapping.append({**row, "destination": target, "action": action})
    branch_rows = []
    with ZipFile(ARCHIVES / "development-568b1fc.zip") as archive:
        for item in archive.infolist():
            if item.is_dir():
                continue
            name = item.filename
            if PurePosixPath(name).is_absolute() or ".." in PurePosixPath(name).parts or name.startswith(".git/"):
                raise RuntimeError(f"Invalid Git archive member: {name}")
            data = archive.read(item)
            path = ROOT / name
            # Keep existing CRLF bytes when Git's text is otherwise identical.
            same = path.is_file() and path.read_bytes().replace(b"\r\n", b"\n") == data.replace(b"\r\n", b"\n")
            if not same:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
            branch_rows.append({"path": name, "archive_sha256": hashlib.sha256(data).hexdigest(), "installed_sha256": digest(path), "action": "retained_equivalent" if same else "installed"})
    # Branch import can replace a file shared with the transfer's older checkout.
    # In that case the exact older bytes also need an archive destination.
    with ZipFile(ARCHIVES / "transfer-originals.zip", "a", ZIP_DEFLATED, compresslevel=6) as archive:
        for row in mapping:
            if "::" not in row["destination"] and (
                digest(ROOT / row["destination"]) != row["sha256"]
                or not row["destination"].startswith(("runs/", "data/", "artifacts/", "tools/"))
            ):
                archive.write(SOURCE / row["path"], row["path"])
                row["destination"] = f"artifacts/migrations/{DATE}/transfer-originals.zip::{row['path']}"
                row["action"] = "archived_original"
    write_rows("transfer_mapping.jsonl", mapping)
    write_rows("development_mapping.jsonl", branch_rows)
    save("integration_summary.json", {"transfer_files": len(mapping), "actions": dict(Counter(row["action"] for row in mapping)), "development_files": len(branch_rows), "development_actions": dict(Counter(row["action"] for row in branch_rows))})
    print((RECEIPTS / "integration_summary.json").read_text(encoding="utf-8"), flush=True)


def verify_local_originals(rows, archive, root):
    """Use the original backup membership, independent of later Git commits."""
    failures = []
    tracked = set(archive.namelist())
    checked_local, checked_tracked = 0, 0
    for row in rows:
        if row["path"] in tracked:
            if hashlib.sha256(archive.read(row["path"])).hexdigest() != row["sha256"]:
                failures.append({"local_tracked_backup_changed": row["path"]})
            checked_tracked += 1
            continue
        if row["path"].startswith("scripts/maintenance/"):
            continue
        path = root / row["path"]
        if not path.is_file() or digest(path) != row["sha256"]:
            failures.append({"local_original_changed": row["path"]})
        checked_local += 1
    return failures, checked_local, checked_tracked


def verify():
    failures = []
    mappings = read_rows("transfer_mapping.jsonl")
    with ZipFile(ARCHIVES / "transfer-originals.zip") as archive:
        for row in mappings:
            target = row["destination"]
            if "::" in target:
                sha = hashlib.sha256(archive.read(target.split("::", 1)[1])).hexdigest()
            else:
                path = ROOT / target
                sha = digest(path) if path.is_file() else None
            if sha != row["sha256"]:
                failures.append({"source": row["path"], "destination": target})
    # Existing data must remain byte-identical. Active tracked files can change;
    # their original contents are preserved by local-tracked-before.zip.
    with ZipFile(ARCHIVES / "local-tracked-before.zip") as archive:
        original_failures, checked_local, checked_tracked = verify_local_originals(
            read_rows("local_inventory_before.jsonl"), archive, ROOT
        )
        failures.extend(original_failures)
    branch = read_rows("development_mapping.jsonl")
    with ZipFile(ARCHIVES / "development-568b1fc.zip") as archive:
        for row in branch:
            if hashlib.sha256(archive.read(row["path"])).hexdigest() != row["archive_sha256"]:
                failures.append({"development_archive_changed": row["path"]})
    receipt = {"status": "PASS" if not failures else "FAIL", "transfer_files_verified": len(mappings), "transfer_bytes_verified": sum(row["bytes"] for row in mappings), "preexisting_local_untracked_files_verified": checked_local, "preexisting_local_tracked_backups_verified": checked_tracked, "development_archive_files_verified": len(branch), "access_boundaries": "local_access_boundaries.json; inaccessible preexisting caches left untouched", "failures": failures, "archives": {p.name: {"bytes": p.stat().st_size, "sha256": digest(p)} for p in sorted(ARCHIVES.glob('*.zip'))}}
    save("preservation_verification.json", receipt)
    print(json.dumps(receipt, ensure_ascii=False, indent=2), flush=True)
    if failures:
        raise SystemExit(1)


def source_check():
    """Detect concurrent additions/edits before the separately authorized removal."""
    expected = {row["path"]: row for row in read_rows("transfer_inventory.jsonl")}
    current, _ = inventory(SOURCE)
    actual = {row["path"]: row for row in current}
    differences = [name for name in sorted(expected.keys() | actual.keys()) if expected.get(name) != actual.get(name)]
    receipt = {"status": "PASS" if not differences else "FAIL", "source": str(SOURCE),
               "files": len(current), "bytes": sum(row["bytes"] for row in current), "changed_paths": differences}
    save("source_before_removal_verification.json", receipt)
    print(json.dumps(receipt, ensure_ascii=False, indent=2))
    if differences:
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=["prepare", "integrate", "verify", "source-check"])
    {"prepare": prepare, "integrate": integrate, "verify": verify, "source-check": source_check}[parser.parse_args().phase]()
