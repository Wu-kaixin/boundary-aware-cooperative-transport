from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

from dbact.cargo import Cargo
from dbact.geometry import is_simple_polygon


def load_runner():
    path = Path(__file__).resolve().parents[1] / "scripts" / "run_arbitrary_shape_monte_carlo.py"
    spec = importlib.util.spec_from_file_location("arbitrary_shape_runner", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_seed_range_parser_is_inclusive_and_deduplicated():
    module = load_runner()
    assert module.parse_seeds("0..2,2,5") == [0, 1, 2, 5]


def test_every_catalog_shape_materialises_as_a_simple_polygon():
    module = load_runner()
    for index, name in enumerate(module.SHAPE_NAMES):
        rng = np.random.default_rng(index)
        cargo = Cargo.from_config(module.shape_config(name, rng))
        assert is_simple_polygon(cargo.vertices), name


def test_empirical_time_bound_is_unavailable_with_eligible_censoring():
    module = load_runner()
    result = module.empirical_completion_bound([300, 340], eligible_failures=1)
    assert result["available"] is False
    assert "right-censored" in result["reason"]


def test_wilson_interval_contains_observed_proportion():
    module = load_runner()
    lower, upper = module.wilson_interval(2, 4)
    assert lower < 0.5 < upper
    assert lower == pytest.approx(0.15003898915214947)
