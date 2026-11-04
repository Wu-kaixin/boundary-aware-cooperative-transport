"""Read-only Phase 1 inventory; write a new receipt, never change research inputs."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import subprocess
import sys
from datetime import datetime, timezone
from importlib import metadata
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def digest(path: Path) -> str:
    with path.open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def snapshot() -> dict:
    if Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        raise RuntimeError("Use the repository root .venv")
    import numpy
    import scipy.linalg  # Load both native thread pools for the receipt.
    from threadpoolctl import threadpool_info

    import dbact
    import dbact_sim

    pins = re.findall(
        r"^([A-Za-z0-9_.-]+)==([^\s;\\]+)",
        (ROOT / "requirements.lock.txt").read_text(),
        re.MULTILINE,
    )
    dependencies = []
    for name, expected in pins:
        try:
            installed = metadata.version(name)
        except metadata.PackageNotFoundError:
            installed = None
        dependencies.append(
            {
                "name": name,
                "locked": expected,
                "installed": installed,
                "matches": installed == expected,
            }
        )
    suffixes = {
        ".py",
        ".ps1",
        ".json",
        ".yaml",
        ".yml",
        ".toml",
        ".txt",
        ".md",
        ".tex",
        ".bib",
        ".cls",
    }
    paths = git("ls-files", "--cached", "--others", "--exclude-standard").splitlines()
    files = {}
    for relative in sorted(set(paths)):
        path = ROOT / relative
        if path.is_file() and path.suffix.lower() in suffixes:
            files[relative] = {"sha256": digest(path), "bytes": path.stat().st_size}
    tree_digest = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
    return {
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "Current source/environment inventory; not a new scientific validation",
        "root": str(ROOT),
        "head": git("rev-parse", "HEAD"),
        "branch": git("branch", "--show-current"),
        "git_status": git("status", "--porcelain=v1"),
        "python": sys.version,
        "executable": sys.executable,
        "prefix": sys.prefix,
        "base_prefix": sys.base_prefix,
        "python_path": sys.path,
        "imports": {
            "dbact": dbact.__file__,
            "dbact_sim": dbact_sim.__file__,
            "numpy": numpy.__file__,
            "scipy": scipy.__file__,
        },
        "venv_config": (ROOT / ".venv/pyvenv.cfg").read_text(),
        "dependencies": dependencies,
        "dependency_mismatches": [item for item in dependencies if not item["matches"]],
        "installed_distributions": sorted(
            [
                {"name": item.metadata["Name"], "version": item.version}
                for item in metadata.distributions()
            ],
            key=lambda item: item["name"].lower(),
        ),
        "threadpools": threadpool_info(),
        "text_source_files": files,
        "text_source_tree_sha256": tree_digest,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / "runs"):
        raise ValueError("Receipt must be in a fresh location under runs/")
    if output.exists():
        raise FileExistsError(output)
    result = snapshot()
    with output.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    print(
        json.dumps(
            {
                "output": str(output),
                "source_files": len(result["text_source_files"]),
                "head": result["head"],
                "python": result["python"],
                "locked_packages": len(result["dependencies"]),
                "mismatches": result["dependency_mismatches"],
                "threadpools": result["threadpools"],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
