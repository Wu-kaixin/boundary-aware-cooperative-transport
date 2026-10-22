"""Check the versioned project in both a full workspace and a fresh Git clone.

Large local runs, reference originals and .venv payloads are deliberately outside
the versioned repository. Their separate migration audit is not a CI prerequisite.
"""
from __future__ import annotations

import argparse
import ast
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
NAVIGATION = (
    "README.md", "docs/ARCHITECTURE.md", "docs/DATA_INDEX.md",
    "docs/REBUILD_2026-10-23.md", "docs/REPOSITORY_HEALTH.md", "docs/results/README.md",
    "docs/migrations/2026-10-23/README.md", "data/README.md", "artifacts/README.md",
    "scripts/literature/README.md", "platforms/mas_public/README.md",
    "docs/literature_review_2026-10-22/README.md",
)


def git(*args):
    return subprocess.check_output(["git", *args], cwd=ROOT).decode("utf-8").strip()


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def check():
    failures = []
    tracked = [name for name in git("ls-files", "-z").split("\0") if name]
    if Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        failures.append("Run this check with the root project .venv")
    if (Path(sys.base_prefix) / "conda-meta").exists():
        failures.append("The root .venv must use standalone Python")
    if (ROOT / "environment.yml").exists():
        failures.append("Obsolete active environment.yml remains")

    forbidden = []
    syntax_errors = []
    secret_candidates = []
    large_files = []
    python_files = 0
    secret_pattern = re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----|\bgh[pousr]_[A-Za-z0-9]{30,}")
    for name in tracked:
        path = ROOT / name
        parts = path.parts
        if name.startswith(("runs/", "tmp/", "data/literature/", "artifacts/migrations/")) or any(
            part in {".venv", "__pycache__", ".pytest_cache", ".env"} for part in parts
        ):
            forbidden.append(name)
        if not path.is_file():
            failures.append(f"Tracked file missing: {name}")
            continue
        if path.stat().st_size >= 50 * 1024 * 1024:
            large_files.append(name)
        if path.suffix == ".py":
            python_files += 1
            try:
                ast.parse(path.read_bytes(), filename=name)
            except (SyntaxError, UnicodeError) as error:
                syntax_errors.append({"file": name, "error": str(error)})
        if path.suffix in {".py", ".ps1", ".yaml", ".yml", ".toml", ".json", ".md", ".txt"}:
            if secret_pattern.search(path.read_bytes()):
                secret_candidates.append(name)
    if forbidden:
        failures.append({"local_payloads_tracked": forbidden})
    if large_files:
        failures.append({"files_over_50_mib": large_files})
    if syntax_errors:
        failures.append({"python_syntax": syntax_errors})
    if secret_candidates:
        failures.append({"possible_private_keys_or_tokens": secret_candidates})

    evidence = {}
    for batch in ("ccta2027_draft_v2", "reliability_2026-10-22"):
        base = ROOT / "docs/results" / batch
        files = json.loads((base / "artifact_integrity.json").read_text(encoding="utf-8"))["files"]
        for name, expected in files.items():
            path = base / name
            if not path.is_file() or sha256(path) != expected.lower():
                failures.append(f"Frozen evidence hash mismatch: {batch}/{name}")
        evidence[batch] = len(files)

    broken_links = []
    for name in NAVIGATION:
        path = ROOT / name
        if not path.is_file():
            broken_links.append({"file": name, "target": "document missing"})
            continue
        for target in re.findall(r"\]\(([^)]+)\)", path.read_text(encoding="utf-8")):
            if re.match(r"^(?:[a-zA-Z][\w+.-]*:|#)", target):
                continue
            target = unquote(target.split("#")[0])
            if target and not (path.parent / target).exists():
                broken_links.append({"file": name, "target": target})
    if broken_links:
        failures.append({"navigation_links": broken_links})

    pip = subprocess.run([sys.executable, "-m", "pip", "check"], capture_output=True, text=True)
    if pip.returncode:
        failures.append({"dependency_consistency": pip.stdout + pip.stderr})
    fsck = subprocess.run(["git", "fsck", "--full", "--no-dangling"], cwd=ROOT, capture_output=True, text=True)
    if fsck.returncode:
        failures.append({"git_object_integrity": fsck.stdout + fsck.stderr})
    return {
        "schema": "repository-health-v1", "checked_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "PASS" if not failures else "FAIL", "head": git("rev-parse", "HEAD"),
        "tree": git("rev-parse", "HEAD^{tree}"), "python": sys.version.split()[0],
        "executable": sys.executable, "tracked_files_checked": len(tracked),
        "python_files_parsed": python_files, "curated_evidence_verified": evidence,
        "navigation_documents_checked": len(NAVIGATION), "pip_check": pip.stdout.strip(),
        "git_fsck": "PASS" if not fsck.returncode else "FAIL",
        "local_data_scope": "Full raw runs, downloaded references, environment payloads and backup archives stay local; see docs/DATA_INDEX.md",
        "security_scope": "Filename and embedded private-key/token screening only; not a dependency vulnerability audit",
        "failures": failures,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = check()
    serialized = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized)
    raise SystemExit(0 if report["status"] == "PASS" else 1)
