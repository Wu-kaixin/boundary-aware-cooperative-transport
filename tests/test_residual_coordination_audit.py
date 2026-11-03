"""Keep speed infeasibility and sampled physical timing distinct in diagnosis."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest


@pytest.fixture
def audit(monkeypatch):
    root = Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'scripts'))
    spec = importlib.util.spec_from_file_location('residual_coordination_audit',
        root/'scripts/analyze_residual_coordination.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_individually_feasible_rows_can_conflict_only_with_speed_cap(audit):
    result = audit.decompose(np.eye(2), [.3, .3], ['object', 'agent'], .35)
    assert result['groups']['object']['bounded']['feasible']
    assert result['groups']['agent']['bounded']['feasible']
    assert not result['groups']['joint']['bounded']['feasible']
    assert result['groups']['joint']['unbounded']['feasible']
    assert result['groups']['joint']['unbounded']['minimum_speed'] == pytest.approx(np.sqrt(.18), abs=1e-7)
    assert result['minimal_conflict']['indices'] == [0, 1]
    assert not result['minimal_conflict']['cap_independent']


def test_cap_independent_conflict_removes_redundant_row(audit):
    result = audit.decompose([[1., 0.], [-1., 0.], [0., 1.]],
        [.1, .1, -1.], ['object', 'agent', 'agent'], .35)
    conflict = result['minimal_conflict']
    assert conflict['indices'] == [0, 1]
    assert conflict['cap_independent']
    assert all(row['feasible'] for row in conflict['one_row_removals'])


def physics_fixture():
    rows = [{'frame': 1, 'phase': 'CONTACT_READY'}, {'frame': 2, 'phase': 'BRAKE'}]
    samples = [dict(frame=f, substep=s, offset_seconds=.05*s, object_id='cargo_0',
        force=[force, 0.], torque=torque)
        for f, s, force, torque in [(1, 0, 10., 2.), (1, 1, 0., 0.),
                                     (2, 0, 0., -2.), (2, 1, 0., 4.)]]
    return rows, samples


def test_impulses_use_all_substeps_and_command_phase(audit):
    rows, samples = physics_fixture()
    result = audit.substep_impulses(samples, rows, dt=.1, substeps=2)
    assert result['CONTACT_READY']['force_impulse'] == pytest.approx([.5, 0.])
    assert result['CONTACT_READY']['torque_impulse'] == pytest.approx(.1)
    assert result['BRAKE']['torque_impulse'] == pytest.approx(.1)
    assert result['BRAKE']['absolute_torque_impulse'] == pytest.approx(.3)
    assert result['CONTACT_READY']['samples'] == result['BRAKE']['samples'] == 2


def test_incomplete_or_misordered_substeps_are_rejected(audit):
    rows, samples = physics_fixture()
    with pytest.raises(AssertionError):
        audit.substep_impulses(samples[:-1], rows, dt=.1, substeps=2)
    samples[1]['frame'] = 2
    with pytest.raises(AssertionError):
        audit.substep_impulses(samples, rows, dt=.1, substeps=2)


def test_control_contact_report_is_the_first_substep_sample(audit):
    rows, samples = physics_fixture()
    row = dict(rows[0], actual_net_force=[10., 0.], actual_net_torque=2.)
    audit.check_first_contact_sample(row, samples[:2])
    with pytest.raises(AssertionError):
        audit.check_first_contact_sample(dict(row, actual_net_force=[0., 0.],
            actual_net_torque=0.), samples[:2])
