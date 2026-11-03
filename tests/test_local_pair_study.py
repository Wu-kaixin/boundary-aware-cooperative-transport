"""Physical evaluation uses executed evidence, full schedules and original gates."""
import copy
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from test_local_pair_safety import pair, module as protocol_module


def module():
    assert importlib.util.find_spec('local_pair_audit') is not None, 'Independent pair execution audit is missing'
    import local_pair_audit
    return local_pair_audit


def record():
    a, b, _, _ = pair()
    outputs, _ = protocol_module().exchange({'a': a, 'b': b})
    r = outputs['a'].record()
    positions = {'a': np.array(a.position), 'b': np.array(b.position)}
    velocities = {aid: np.array(v) for aid, v in zip(r['members'], r['applied_commands'])}
    return r, positions, velocities


def test_independent_audit_accepts_real_commands_and_rejects_tampering():
    m = module()
    r, p, u = record()
    result = m.verify_commit(r, p, u, frame=5, timestamp=.25, dt=.05, comm_range=1.6)
    assert result['minimum_residual'] >= -1e-7
    bad = copy.deepcopy(u); bad['b'] *= -1.
    with pytest.raises(AssertionError):
        m.verify_commit(r, p, bad, frame=5, timestamp=.25, dt=.05, comm_range=1.6)


@pytest.mark.parametrize('change', ['missing_ack', 'stale', 'changed_peer_position'])
def test_audit_rejects_missing_ack_stale_frame_and_falsified_neighbor_position(change):
    m = module()
    r, p, u = record()
    if change == 'missing_ack': r['acknowledgements'].pop()
    if change == 'stale': r['offers'][0]['frame'] -= 1
    if change == 'changed_peer_position': p['b'] += [0., .01]
    with pytest.raises(AssertionError):
        m.verify_commit(r, p, u, frame=5, timestamp=.25, dt=.05, comm_range=1.6)


def examples():
    rows = []
    conditions = ['circle', 'l', 'c', 'skew']
    for c in conditions:
        for arm in ('finite_acquisition_support', 'local_pair_candidate'):
            rows.append(dict(condition=c, arm=arm, execution='COMPLETED', safe_completed=True,
                false_hold=False, hard_safety=[], zero_epoch_valid=True, g500=True,
                phase_response=dict(transport_frame=10), scaling=dict(acquisition=0,post_acquisition=0),
                pair_audit=dict(verified=True)))
    return rows, {c:dict(exact=True) for c in conditions}, conditions


def test_full_gates_reject_missing_cases_scaling_and_nonactivation_without_mutating_rows():
    m = module()
    rows, repro, conditions = examples()
    before = copy.deepcopy(rows)
    mechanism, complete = m.evaluate_gates(rows, repro, conditions)
    assert all(mechanism.values()) and all(complete.values()) and rows == before
    assert not all(m.evaluate_gates(rows[:-1], repro, conditions)[0].values())
    rows[1]['scaling']['acquisition'] = 1
    assert not all(m.evaluate_gates(rows, repro, conditions)[0].values())
    rows[1]['scaling']['acquisition'] = 0
    rows[1]['phase_response']['transport_frame'] = None
    assert not all(m.evaluate_gates(rows, repro, conditions)[1].values())


def test_safe_completion_cannot_replace_baseline_g500_or_execution_audit():
    m = module()
    rows, repro, conditions = examples()
    rows[1]['g500'] = False
    assert not all(m.evaluate_gates(rows, repro, conditions)[0].values())
    rows[1]['g500'] = True
    rows[1]['pair_audit']['verified'] = False
    assert not all(m.evaluate_gates(rows, repro, conditions)[0].values())


def test_execution_error_with_no_audit_is_a_failed_gate_not_an_analysis_exception():
    m = module()
    rows, repro, conditions = examples()
    rows[1].update(execution='ERROR', safe_completed=False, pair_audit=None, scaling=None)
    mechanism, complete = m.evaluate_gates(rows, repro, conditions)
    assert not mechanism['all_execution_audits'] and not all(complete.values())
