"""Verify the rebuilt local workspace without rewriting historical evidence."""
from __future__ import annotations

import hashlib
from importlib.metadata import distributions, version
import json
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote
from xml.etree import ElementTree

ROOT = Path(__file__).resolve().parents[2]
RECEIPTS = ROOT / "docs/migrations/2026-10-23"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def junit(path):
    suites = ElementTree.parse(path).getroot().findall(".//testsuite")
    result = {key: sum(int(suite.get(key, 0)) for suite in suites)
              for key in ("tests", "errors", "failures", "skipped")}
    result["passed"] = result["tests"] - result["errors"] - result["failures"] - result["skipped"]
    result["seconds"] = sum(float(suite.get("time", 0)) for suite in suites)
    result["report"] = path.relative_to(ROOT).as_posix()
    return result


def verify():
    failures = []
    assert Path(sys.prefix).resolve() == (ROOT / ".venv").resolve()
    assert not (Path(sys.base_prefix) / "conda-meta").exists()
    dependencies = {}
    for name, expected in re.findall(r"^([A-Za-z0-9_.-]+)==([^\s;\\]+)", (ROOT / "requirements.lock.txt").read_text(), re.M):
        actual = version(name)
        dependencies[name] = actual
        if expected != actual:
            failures.append(f"Dependency mismatch: {name}: {actual} != {expected}")
    pip = subprocess.run([sys.executable, "-m", "pip", "check"], capture_output=True, text=True)
    if pip.returncode:
        failures.append(pip.stdout + pip.stderr)

    literature = load(ROOT / "docs/literature_review_2026-10-22/source_manifest.json")
    for row in literature:
        candidates = list((ROOT / "data/literature/2026-10-22/pdfs").glob(row["reading_id"] + ".*"))
        if len(candidates) != 1 or digest(candidates[0]) != row["sha256"].lower():
            failures.append(f"Reference mismatch: {row['reading_id']}")
    curated = {}
    for folder in ("ccta2027_draft_v2", "reliability_2026-10-22"):
        base = ROOT / "docs/results" / folder
        files = load(base / "artifact_integrity.json")["files"]
        for name, expected in files.items():
            if digest(base / name) != expected.lower():
                failures.append(f"Evidence mismatch: {folder}/{name}")
        curated[folder] = len(files)

    reports = {name: junit(ROOT / "artifacts/validation/2026-10-23" / filename)
               for name, filename in (("core", "pytest.xml"), ("mas", "mas-pytest.xml"))}
    for name, report in reports.items():
        if report["errors"] or report["failures"]:
            failures.append(f"Test failure: {name}")
    smoke = load(ROOT / "runs/rebuild_20261023/smoke/summary.json")
    if smoke["execution_errors"] or smoke["source_integrity"] != "PASS" or len(smoke["results"]) != 5:
        failures.append("Smoke pipeline failed")

    # Check newly authored navigation, not links quoted inside historical receipts.
    navigation = ["README.md", "docs/ARCHITECTURE.md", "docs/DATA_INDEX.md", "docs/REBUILD_2026-10-23.md",
                  "docs/results/README.md", "docs/migrations/2026-10-23/README.md", "data/README.md",
                  "artifacts/README.md", "scripts/literature/README.md", "platforms/mas_public/README.md",
                  "docs/literature_review_2026-10-22/README.md"]
    broken = []
    for name in navigation:
        path = ROOT / name
        for target in re.findall(r"\]\(([^)]+)\)", path.read_text(encoding="utf-8")):
            if re.match(r"^(?:[a-zA-Z][\w+.-]*:|#)", target):
                continue
            target = unquote(target.split("#")[0])
            generated_receipt = RECEIPTS / "workspace_verification.json"
            if target and (path.parent / target).resolve() != generated_receipt.resolve() and not (path.parent / target).exists():
                broken.append({"file": name, "target": target})
    failures.extend(broken)
    preservation = load(RECEIPTS / "preservation_verification.json")
    if preservation["status"] != "PASS":
        failures.append("Preservation verification failed")
    result = {
        "date": "2026-10-23", "status": "PASS" if not failures else "FAIL",
        "python": {"executable": sys.executable, "version": sys.version, "base_prefix": sys.base_prefix},
        "locked_dependencies": dependencies, "pip_check": pip.stdout.strip(), "tests": reports,
        "smoke": {"cases": len(smoke["results"]), "execution_errors": smoke["execution_errors"],
                  "source_integrity": smoke["source_integrity"], "output": "runs/rebuild_20261023/smoke"},
        "literature": {"originals_verified": len(literature), "pdfs": sum("pages" in row for row in literature),
                       "pdf_pages_in_original_manifest": sum(row.get("pages", 0) for row in literature)},
        "curated_evidence_files_verified": curated,
        "transfer_files_preserved": preservation["transfer_files_verified"],
        "transfer_directory_exists": (ROOT / "其他电脑转移").exists(),
        "navigation_documents_checked": len(navigation), "failures": failures,
        "installed_packages": dict(sorted((dist.metadata["Name"], dist.version) for dist in distributions())),
    }
    (RECEIPTS / "workspace_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in result.items() if key not in {"locked_dependencies", "installed_packages"}}, ensure_ascii=False, indent=2))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(verify())
