"""Locate repository-owned scenarios independently of the caller's directory."""
from __future__ import annotations

import os
from pathlib import Path


def repository_root() -> Path:
    override = os.environ.get("DBACT_ROOT")
    candidates = [Path(override).expanduser()] if override else Path(__file__).resolve().parents
    for candidate in candidates:
        if (candidate / "pyproject.toml").is_file() and (candidate / "configs").is_dir():
            return candidate.resolve()
    raise RuntimeError("Repository scenarios unavailable; set DBACT_ROOT to the repository checkout")
