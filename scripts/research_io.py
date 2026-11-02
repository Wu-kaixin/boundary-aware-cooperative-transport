"""Portable file helpers for research analyses; no process-monitor dependencies."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def guard():
    assert Path.cwd().resolve() == ROOT
    assert Path(sys.prefix).resolve() == ROOT / ".venv"
