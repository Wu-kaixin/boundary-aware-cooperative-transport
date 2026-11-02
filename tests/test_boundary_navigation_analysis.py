"""Horizon accounting must not turn passing near a target into completion."""
import sys
from pathlib import Path
import numpy as np
import pytest


def analyzer(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/"scripts"))
    from analyze_boundary_navigation_study import at_horizon
    return at_horizon


def test_timeout_at_target_is_not_stopped_completion(monkeypatch):
    at_horizon = analyzer(monkeypatch)
    r = at_horizon([0.,150.,600.], np.array([[0.,0.],[1.2,0.],[1.3,0.]]), [1.2,0.],150.,600.,False)
    assert r['endpoint_error_m'] == 0.
    assert not r['completed'] and not r['terminated']
    assert r['index'] == 1


def test_early_off_target_hold_is_preserved_at_later_horizon(monkeypatch):
    at_horizon = analyzer(monkeypatch)
    r = at_horizon([0.,13.5], np.array([[0.,0.],[1.2,-1.]]), [1.2,0.],600.,13.5,False)
    assert r['terminated'] and not r['completed']
    assert r['endpoint_error_m'] == 1.
    assert r['time'] == 13.5


def test_later_completion_not_backdated_to_short_horizon(monkeypatch):
    at_horizon = analyzer(monkeypatch)
    times = [0.,150.,200.]
    points = np.array([[0.,0.],[.6,0.],[1.15,0.]])
    assert not at_horizon(times,points,[1.2,0.],150.,200.,True)['completed']
    assert at_horizon(times,points,[1.2,0.],600.,200.,True)['completed']


def test_reanalysis_cannot_overwrite_existing_evidence_or_leave_mission(monkeypatch, tmp_path):
    analyzer(monkeypatch)
    import analyze_boundary_navigation_study as m
    monkeypatch.setattr(m, 'ROOT', tmp_path)
    allowed = tmp_path/'artifacts/research'/m.MISSION
    allowed.mkdir(parents=True)
    (allowed/'analysis.json').write_text('{}')
    with pytest.raises(RuntimeError, match='preserve its identity'): m.analysis_output()
    with pytest.raises(ValueError, match='inside this mission'): m.analysis_output(tmp_path/'runs')
    assert m.analysis_output(allowed/'rebuild-001') == allowed/'rebuild-001'
