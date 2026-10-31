"""Reconcile the inventoried PC2/PC3 sources inside this repository.

Default mode only plans. Apply publishes verified copies without overwriting.
Cleanup is a separate, gated operation: it rehashes every source and destination
before removing individual mapped files, then removes only empty directories.
An unreadable entry, new file, reparse point or changed source blocks its source.
No copied environment or source program is executed.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import csv
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import sys
from zipfile import ZipFile, ZIP_DEFLATED

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from scripts.maintenance.inventory_repository import ROOT, scan, io_path

FIELDS = ("source", "original_path", "relative_path", "size", "sha256", "purpose",
          "action", "basis", "final_location")


def digest(path):
    h = hashlib.sha256()
    with io_path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def confined(name):
    pure = PurePosixPath(name)
    if pure.is_absolute() or ".." in pure.parts or "\\" in name or ":" in name:
        raise ValueError(f"Unsafe relative path: {name}")
    path = ROOT.joinpath(*pure.parts)
    for parent in (path, *path.parents):
        if parent == ROOT:
            break
        if io_path(parent).exists() or io_path(parent).is_symlink():
            if getattr(io_path(parent).lstat(), "st_file_attributes", 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT or parent.is_symlink():
                raise ValueError(f"Reparse point: {parent}")
    if not path.resolve().is_relative_to(ROOT):
        raise ValueError(f"Path leaves repository: {name}")
    return path


def read_rows(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def log(mission, **entry):
    with (mission / "operations.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(json.dumps({"at_utc": datetime.now(timezone.utc).isoformat(), **entry}, ensure_ascii=False) + "\n")


def plan(mission):
    rows = read_rows(mission / "inventory.csv")
    existing = {r["relative_path"]: r["sha256"] for r in rows
                if r["source"] == "MAIN" and r["hash_status"] == "PASS"}
    # PC3 contains the same artifact subtree pasted under thirteen project
    # directories. Only this reviewed layout is eligible for cross-path reuse;
    # ordinary run samples, licenses, fixtures and configurations are not.
    pc3_artifacts = {r["relative_path"]: r for r in rows if r["source"] == "PC3"
                     and r["relative_path"].startswith("artifacts/") and r["hash_status"] == "PASS"}
    repeated_roots = {"artifacts", "configs", "data", "docs", "experiments", "manuscript",
                      "platforms", "runs", "scripts", "src", "tests", "theory", "tools"}
    damaged_archives = {"development-568b1fc.zip", "local-tracked-before.zip",
                        "repository-before-sync.bundle", "transfer-originals.zip"}
    items = []
    for row in rows:
        if row["source"] == "MAIN" or row["file_type"] == "directory":
            continue
        item = {k: row[k] for k in FIELDS if k in row}
        name = row["relative_path"]
        archive = (mission / "backup" / f"source-originals-{row['source']}.zip").relative_to(ROOT).as_posix()
        if (row["source"] == "PC3" and name.split("/")[0] in repeated_roots
                and Path(name).name in damaged_archives and row["hash_status"] == "PASS"):
            item.update(action="archive", basis="Reviewed misplaced archive family; preserve each distinct damaged container once by full SHA-256",
                        final_location=archive + "::damaged-archives/" + row["sha256"] + "/" + Path(name).name)
            items.append(item)
            continue
        artifact_name = "artifacts/" + name.split("/", 1)[1] if "/" in name else ""
        representative = pc3_artifacts.get(artifact_name)
        if (row["source"] == "PC3" and name.split("/")[0] in repeated_roots
                and representative and representative["sha256"] == row["sha256"]
                and row["hash_status"] == "PASS" and name != "artifacts/README.md"):
            if Path(name).name in damaged_archives:
                # These archive containers fail ZIP/bundle validation. Keep one
                # exact opaque copy of each, never replace the valid PC2 backup.
                target = (mission / "quarantine/PC3/damaged-archives" / Path(name).name).relative_to(ROOT).as_posix()
            else:
                target = artifact_name
            if target in existing and existing[target] != row["sha256"]:
                target = (mission / "quarantine/PC3" / artifact_name).relative_to(ROOT).as_posix()
            item.update(action="reuse" if target in existing else "copy",
                        basis="Reviewed replicated PC3 artifact subtree; exact representative SHA-256; damaged containers quarantined",
                        final_location=target)
            existing[target] = row["sha256"]
            items.append(item)
            continue
        if row["hash_status"] != "PASS":
            item.update(action="retain_blocked", basis=row["hash_status"], final_location=row["original_path"])
        elif row["git_status"] == "tracked" or row["purpose"] in {"git_metadata", "copied_environment_do_not_execute"}:
            item.update(action="archive", basis="Source Git/environment and tracked original must survive later edits to the active checkout",
                        final_location=archive + "::" + name)
        elif existing.get(name) == row["sha256"]:
            item.update(action="reuse", basis="Same relative role and exact SHA-256", final_location=name)
        elif name not in existing and name.startswith(("runs/", "data/literature/", "artifacts/", "tools/",
                                                       "platforms/mas_public/data/", "platforms/mas_public/logs/")):
            item.update(action="copy", basis="Missing local payload; preserve batch path and bytes", final_location=name)
            existing[name] = row["sha256"]
        else:
            item.update(action="archive", basis="Original environment/Git/cache or conflicting/historical content; preserve exact member",
                        final_location=archive + "::" + name)
        items.append(item)
    with (mission / "migration_manifest.csv").open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(items)
    summary = {"actions": dict(Counter(i["action"] for i in items)), "bytes_by_action":
               {a: sum(int(i["size"] or 0) for i in items if i["action"] == a) for a in {i["action"] for i in items}}}
    (mission / "migration_plan_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


def source_path(item):
    path = confined(item["original_path"])
    expected = ROOT / item["source"] / ROOT.name
    if item["source"] not in {"PC2", "PC3"} or not path.is_relative_to(expected):
        raise ValueError("Source outside exact PC2/PC3 whitelist")
    return path


def check_source(item):
    path = source_path(item)
    if io_path(path).stat().st_size != int(item["size"]) or digest(path) != item["sha256"]:
        raise RuntimeError(f"Source changed: {path}")
    return path


def publish(temporary, target):
    # Windows rename fails if the destination exists. POSIX rename would
    # overwrite, so use exclusive link creation there instead.
    if os.name == "nt":
        os.rename(temporary, target)
    else:
        os.link(temporary, target)
        temporary.unlink()


def apply(mission, items):
    for item in items:
        if item["action"] not in {"copy", "reuse"}:
            continue
        source = check_source(item)
        target = confined(item["final_location"])
        if target.exists():
            if digest(target) != item["sha256"]:
                raise RuntimeError(f"Target conflict: {target}")
        else:
            if item["action"] == "reuse":
                raise RuntimeError(f"Canonical copy missing: {target}")
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_name(target.name + ".dbact-migration-partial")
            if not temporary.exists():
                with source.open("rb") as src, temporary.open("xb") as dst:
                    shutil.copyfileobj(src, dst, 1024 * 1024)
            check_source(item)
            if digest(temporary) != item["sha256"]:
                raise RuntimeError(f"Copy mismatch: {temporary}")
            publish(temporary, target)
        log(mission, operation=item["action"], source=item["original_path"], target=item["final_location"], status="HASH_VERIFIED")
    for source in ("PC2", "PC3"):
        archived = [i for i in items if i["source"] == source and i["action"] == "archive"]
        if not archived:
            continue
        target = confined(archived[0]["final_location"].split("::", 1)[0])
        if target.exists():
            verify_archive(target, archived)
            continue
        temporary = target.with_suffix(".zip.partial")
        with ZipFile(temporary, "x", compression=ZIP_DEFLATED, compresslevel=3) as archive:
            written = set()
            for item in archived:
                member = item["final_location"].split("::", 1)[1]
                if member not in written:
                    archive.write(io_path(check_source(item)), member)
                    written.add(member)
                check_source(item)
        verify_archive(temporary, archived)
        publish(temporary, target)
        log(mission, operation="archive", source=source, target=str(target.relative_to(ROOT)), files=len(archived), sha256=digest(target), status="EVERY_MEMBER_VERIFIED")


def verify_archive(path, items):
    with ZipFile(path) as archive:
        if len(archive.namelist()) != len(set(archive.namelist())):
            raise ValueError(f"Duplicate archive members: {path}")
        checked = set()
        for item in items:
            member = item["final_location"].split("::", 1)[1]
            if (member, item["sha256"]) in checked:
                continue
            if archive.getinfo(member).file_size != int(item["size"]):
                raise RuntimeError(f"Archive size mismatch: {member}")
            with archive.open(member) as stream:
                if hashlib.file_digest(stream, "sha256").hexdigest() != item["sha256"]:
                    raise RuntimeError(f"Archive hash mismatch: {member}")
            checked.add((member, item["sha256"]))


def verify(mission, items):
    archives = {}
    for item in items:
        if item["action"] == "retain_blocked":
            continue
        if item["action"] == "archive":
            path = item["final_location"].split("::", 1)[0]
            archives.setdefault(path, []).append(item)
        elif digest(confined(item["final_location"])) != item["sha256"]:
            raise RuntimeError(f"Destination changed: {item['final_location']}")
    for name, members in archives.items():
        verify_archive(confined(name), members)
    blocked = [i["original_path"] for i in items if i["action"] == "retain_blocked"]
    receipt = {"status": "PASS" if not blocked else "BLOCKED", "mapped_files_verified": len(items) - len(blocked),
               "blocked": blocked, "manifest_sha256": digest(mission / "migration_manifest.csv")}
    (mission / "preservation_current.json").write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps(receipt), flush=True)
    return receipt


def cleanup(mission, items):
    gate = json.loads((mission / "cleanup_gate.json").read_text(encoding="utf-8"))
    if gate.get("status") != "PASS" or gate.get("manifest_sha256") != digest(mission / "migration_manifest.csv"):
        raise ValueError("Need the reviewed validation/recovery/reference cleanup gate for this manifest")
    verify(mission, items)
    retained = set(gate.get("retain_originals", []))
    if retained - {i["original_path"] for i in items}:
        raise ValueError("Retention decision contains an uninventoried source")
    intents = set()
    if (mission / "operations.jsonl").exists():
        for line in (mission / "operations.jsonl").read_text(encoding="utf-8-sig").splitlines():
            entry = json.loads(line)
            if entry.get("operation") in {"delete_intent", "delete_complete"}:
                intents.add(entry["source"])
    for label in ("PC2", "PC3"):
        selected = [i for i in items if i["source"] == label]
        if any(i["action"] == "retain_blocked" for i in selected):
            log(mission, operation="cleanup_blocked", source=label)
            continue
        base = ROOT / label / ROOT.name
        if not base.exists() and all(i["original_path"] in intents for i in selected):
            continue
        actual = list(scan(base, set()))
        actual_files = {p.relative_to(ROOT).as_posix() for p, kind, _ in actual if kind == "file"}
        expected = {i["original_path"] for i in selected}
        if (actual_files - expected or expected - actual_files - intents
                or any(kind not in {"file", "directory"} for _, kind, _ in actual)):
            raise RuntimeError(f"Source membership changed or unreadable: {label}")
        present = [i for i in selected if i["original_path"] in actual_files]
        for item in present:
            check_source(item)
        pending = [i for i in present if i["original_path"] not in retained]
        for item in pending:
            path = check_source(item)
            log(mission, operation="delete_intent", source=item["original_path"], recovery=item["final_location"], sha256=item["sha256"])
            io_path(path).unlink()
            log(mission, operation="delete_complete", source=item["original_path"], bytes=int(item["size"]))
        # No recursive deletion: any new/nonempty/locked directory remains.
        directories = [p for p, kind, _ in actual if kind == "directory"] + [base, base.parent]
        for directory in sorted(directories, key=lambda p: len(p.parts), reverse=True):
            target = io_path(confined(directory.relative_to(ROOT).as_posix()))
            if any(target.iterdir()):
                continue  # Retained or newly appeared content is never deleted.
            target.rmdir()
        kept = sorted(i["original_path"] for i in present if i["original_path"] in retained)
        log(mission, operation="source_retained" if kept else "source_removed", source=label,
            retained_originals=kept, status="PARTIALLY_COMPLETED" if kept else "PASS")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mission", type=Path, required=True)
    parser.add_argument("--mode", choices=("plan", "apply", "verify", "cleanup"), default="plan")
    args = parser.parse_args()
    mission = args.mission.resolve()
    if mission.parent != ROOT / "artifacts/migrations":
        raise ValueError("Mission must be directly under artifacts/migrations")
    if args.mode == "plan":
        plan(mission)
    else:
        items = read_rows(mission / "migration_manifest.csv")
        if args.mode == "apply":
            apply(mission, items)
        elif args.mode == "cleanup":
            cleanup(mission, items)
        else:
            verify(mission, items)
