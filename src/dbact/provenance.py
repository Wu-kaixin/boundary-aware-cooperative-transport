"""Run provenance: git SHA, config hash, seed, backend.

Every summary file carries these four fields. A run missing any of them is
rejected by ``scripts/validate_run.py`` rather than treated as a pass -- an
unattributable number cannot be defended in a review.

Frame-level randomness is derived through BLAKE2 rather than Python's built-in
``hash``, which is salted per process and therefore not reproducible across runs.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np


def git_sha(repo_root: str | Path | None = None, short: bool = False) -> str:
    """Current commit SHA, suffixed with ``-dirty`` when the tree has changes."""
    root = Path(repo_root) if repo_root is not None else Path(__file__).resolve().parents[2]
    args = ["git", "rev-parse", "--short" if short else "HEAD"]
    try:
        sha = subprocess.run(
            args, cwd=root, capture_output=True, text=True, timeout=10, check=True
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain"], cwd=root, capture_output=True, text=True, timeout=10, check=True
        ).stdout.strip()
    except (subprocess.SubprocessError, OSError):
        return "unknown"
    return f"{sha}-dirty" if dirty else sha


def config_hash(config: dict) -> str:
    """Stable digest of a resolved configuration dict."""
    payload = json.dumps(config, sort_keys=True, default=str).encode("utf-8")
    return hashlib.blake2b(payload, digest_size=8).hexdigest()


def stable_seed(*parts: object, base: int = 0) -> int:
    """Deterministic 32-bit seed from arbitrary parts.

    Used for per-agent, per-frame sensor noise. ``hash()`` is deliberately not
    used: it is randomised per interpreter process unless PYTHONHASHSEED is set,
    so a run seeded through it is not reproducible.
    """
    payload = "|".join(repr(p) for p in (base, *parts)).encode("utf-8")
    return int.from_bytes(hashlib.blake2b(payload, digest_size=4).digest(), "little")


def frame_rng(*parts: object, base: int = 0) -> np.random.Generator:
    return np.random.default_rng(stable_seed(*parts, base=base))


def run_provenance(config: dict, seed: int, backend: str, repo_root: str | Path | None = None) -> dict:
    return {
        "git_sha": git_sha(repo_root),
        "config_hash": config_hash(config),
        "seed": int(seed),
        "backend": str(backend),
    }


__all__ = ["git_sha", "config_hash", "stable_seed", "frame_rng", "run_provenance"]
