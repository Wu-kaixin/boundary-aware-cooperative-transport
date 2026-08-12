"""Shared fixtures.

``scripts/`` is not an importable package -- ``pyproject.toml`` puts only ``src``
on the path -- so a test that wants to check a script's behaviour has to load it
by file. Doing that here rather than in each test file keeps the module cached
once per session: ``run_arbitrary_shape_monte_carlo`` imports scipy and the whole
simulator, and loading it three times is three times the cost for no benefit.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]


def load_script_module(name: str):
    """Import ``scripts/<name>.py`` as a module."""
    path = REPO_ROOT / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"_script_{name}", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="session")
def repo_root() -> Path:
    return REPO_ROOT


@pytest.fixture(scope="session")
def matrix_runner():
    """The shape-matrix Monte Carlo runner used by the decisive experiment."""
    return load_script_module("run_arbitrary_shape_monte_carlo")
