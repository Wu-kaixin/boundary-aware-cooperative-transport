"""Deterministic audit sampling must not hide unavailable or later witnesses."""
import importlib.util
from pathlib import Path

import pytest


@pytest.fixture
def choose(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'scripts'))
    spec = importlib.util.spec_from_file_location('support_validation_audit',
        Path(__file__).resolve().parents[1]/'scripts/audit_finite_support_review.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.WitnessSelector()


def test_unavailable_geometry_cannot_consume_first_admitted_witness(choose):
    assert choose.select('c', changed=True, ready=False, violates=False, residual=False) == []
    assert choose.select('c', changed=True, ready=True, violates=False, residual=False) == ['first_changed_admitted']
    assert choose.select('c', changed=True, ready=True, violates=False, residual=False) == []


def test_first_violation_is_independent_of_first_geometry_change(choose):
    assert choose.select('c', changed=True, ready=True, violates=False, residual=False) == ['first_changed_admitted']
    assert choose.select('c', changed=True, ready=True, violates=True, residual=False) == ['first_old_command_violation']
    assert choose.select('c', changed=True, ready=True, violates=True, residual=False) == []


def test_conditions_are_separate_and_original_residuals_are_never_skipped(choose):
    both = ['first_changed_admitted', 'first_old_command_violation']
    assert choose.select('c', changed=True, ready=True, violates=True, residual=False) == both
    assert choose.select('skew', changed=True, ready=True, violates=True, residual=False) == both
    assert choose.select('c', changed=False, ready=True, violates=False, residual=True) == ['original_residual']
    assert choose.select('c', changed=True, ready=False, violates=False, residual=True) == ['original_residual']
