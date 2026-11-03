"""Local responsibility exchange must preserve every other participant's rows."""
from dataclasses import replace
import importlib.util

import numpy as np
import pytest

from dbact.safety.filter import PreparedVelocity, SafetyFilter, SafetyFilterParams


def module():
    assert importlib.util.find_spec('dbact.safety.local_pair') is not None, 'Local pair protocol is not implemented'
    from dbact.safety import local_pair
    return local_pair


def prepared(position, peers, object_rows, wall=False):
    par = SafetyFilterParams(d_min=.28, gamma_agent=6., gamma_obj=8., rho=.02,
        r_safe=.195, max_speed=.35, dt=.05, enable_wall_rows=wall,
        domain=(-1., .305, -1., 1.) if wall else None)
    safety = SafetyFilter(par)
    aa, ba = safety._agent_rows(np.array(position), list(peers.values()))
    aw, bw = safety._wall_rows(np.array(position))
    ao = np.array([r[:2] for r in object_rows], dtype=float).reshape(-1, 2)
    bo = np.array([r[2] for r in object_rows], dtype=float)
    A, b = np.vstack([aa, aw, ao]), np.r_[ba, bw, bo]
    return PreparedVelocity(np.zeros(2), A, b, b.copy(), np.r_[ba, bw, bo-.02],
        (.02-bo)/8., len(aa), len(aw), len(ao),
        ['agent']*len(aa)+['wall']*len(aw)+['object']*len(ao), False, par)


def offer(aid, position, peers, rows, *, frame=5, stamp=.25, wall=False,
          domain=(-2., 2., -2., 2.), changes=None, acquiring=True):
    m = module()
    problem = prepared(position, peers, rows, wall)
    support = dict(applied=True, cap_active=0, motion_bound=0.,
        stage='stationary_acquisition', source_agents=[aid], source_timestamps=[stamp])
    if changes:
        support.update(changes)
    result, reason = m.build_offer(aid, position, peers, problem, frame, stamp, .05,
        domain, support, acquiring=acquiring)
    return result, problem, reason


def pair(peer_rows=((-1., 0., -.30),), **options):
    a, pa, _ = offer('a', [0., 0.], {'b': [.3, 0.]}, [(1., 0., .15)], **options)
    b, pb, _ = offer('b', [.3, 0.], {'a': [0., 0.]}, peer_rows, **options)
    return a, b, pa, pb


def test_pair_repairs_real_independent_infeasibility_with_full_peer_constraints():
    m = module()
    a, b, pa, pb = pair()
    outputs, traces = m.exchange({'a': a, 'b': b})
    assert set(outputs) == {'a', 'b'}
    assert traces['a']['independent_feasible'] is False
    assert traces['b']['independent_feasible'] is True
    ua, rhs_a = outputs['a'].for_problem(pa)
    ub, rhs_b = outputs['b'].for_problem(pb)
    assert ua[0] >= .15-1e-7 and ub[0] <= .30+1e-7
    assert np.linalg.norm(ua) <= .35+1e-7 and np.linalg.norm(ub) <= .35+1e-7
    assert (pa.A@ua-rhs_a).min() >= -1e-7
    assert (pb.A@ub-rhs_b).min() >= -1e-7
    relative = 2*(np.array(a.position)-b.position)
    h = .3**2-.28**2
    assert relative@(ua-ub) >= -6*h-1e-7
    assert outputs['a'].record()['certificate']['minimum_hold_separation'] >= .28-1e-7
    np.testing.assert_array_equal(rhs_a[1:], pa.b[1:])
    np.testing.assert_array_equal(rhs_b[1:], pb.b[1:])


def test_peer_object_constraint_can_make_the_pair_impossible():
    m = module()
    a, b, _, _ = pair(peer_rows=((-1., 0., 0.),))
    outputs, traces = m.exchange({'a': a, 'b': b})
    assert not outputs
    assert traces['a']['attempts'] and not traces['a']['accepted']


def test_failed_proposals_retain_both_original_row_sets_for_independent_audit():
    m = module()
    a, b, pa, pb = pair(peer_rows=((-1., 0., 0.),))
    commands, traces = m.exchange({'a': a, 'b': b})
    assert not commands
    trace = traces['a']
    assert 'original_offer' in trace, 'Unexecuted infeasibility needs its original rows'
    np.testing.assert_array_equal(trace['original_offer']['A'], pa.A)
    np.testing.assert_array_equal(trace['original_offer']['b'], pa.b)
    assert 'peer_offer' in trace['attempts'][0], 'Failed peer trials must retain peer constraints too'
    np.testing.assert_array_equal(trace['attempts'][0]['peer_offer']['A'], pb.A)
    np.testing.assert_array_equal(trace['attempts'][0]['peer_offer']['b'], pb.b)


def test_non_mutual_neighbor_row_cannot_be_deleted_to_make_pair_feasible():
    m = module()
    a, _, _, _ = pair()
    b, _, _ = offer('b', [.3, 0.], {'a': [0., 0.], 'c': [.58, 0.]}, [(-1., 0., -.3)])
    outputs, _ = m.exchange({'a': a, 'b': b})
    assert not outputs


def test_peer_wall_rows_are_retained():
    m = module()
    a, b, _, _ = pair(wall=True, domain=(-1., .305, -1., 1.))
    # b may move at most0.1m/s, while a requires0.25 and mutual safety needs b>=0.134.
    a, _, _ = offer('a', [0., 0.], {'b': [.3, 0.]}, [(1., 0., .25)],
        wall=True, domain=(-1., .305, -1., 1.))
    outputs, _ = m.exchange({'a': a, 'b': b})
    assert not outputs


@pytest.mark.parametrize('changes', [dict(applied=False), dict(cap_active=1),
    dict(motion_bound=.1), dict(stage='fresh_reference'), dict(source_timestamps=[.2])])
def test_invalid_support_is_not_an_offer(changes):
    item, _, reason = offer('a', [0., 0.], {'b': [.3, 0.]}, [(1., 0., .15)], changes=changes)
    assert item is None and reason


def test_no_offer_outside_acquisition():
    item, _, _ = offer('a', [0., 0.], {'b': [.3, 0.]}, [(1., 0., .15)], acquiring=False)
    assert item is None


@pytest.mark.parametrize('change', ['stale', 'position', 'contract', 'asymmetric'])
def test_stale_mismatched_and_one_way_offers_do_not_commit(change):
    m = module()
    a, b, _, _ = pair()
    if change == 'stale':
        b = replace(b, frame=4, timestamp=.2)
    elif change == 'position':
        b = replace(b, position=(.31, 0.))
    elif change == 'contract':
        b = replace(b, contract=tuple((k, .04 if k == 'rho' else v) for k, v in b.contract))
    else:
        b = replace(b, neighbors=())
    outputs, _ = m.exchange({'a': a, 'b': b})
    assert not outputs


def test_missing_acknowledgement_cannot_commit():
    m = module()
    a, b, _, _ = pair()
    proposal, reason = m.propose(a, b)
    assert proposal is not None, reason
    aa = m.choose(a, {'b': b}, [proposal])
    bb = m.choose(b, {'a': a}, [proposal])
    assert m.commit(proposal, a, b, aa, bb)
    assert not m.commit(proposal, a, b, aa, None)
    assert not m.commit(proposal, a, b, aa, replace(bb, proposal_id='different'))


def test_competing_local_proposals_commit_disjointly_and_ignore_iteration_order():
    m = module()
    a, _, _, _ = pair()
    # b has neighbors a/c on opposite sides; each requires b's help. Both
    # proposals can be feasible separately, but b may accept only one.
    b, _, _ = offer('b', [.3, 0.], {'a': [0., 0.], 'c': [.6, 0.]}, [(-1., 0., -.3), (1., 0., -.3)])
    c, _, _ = offer('c', [.6, 0.], {'b': [.3, 0.]}, [(-1., 0., .15)])
    first, _ = m.exchange({'a': a, 'b': b, 'c': c})
    second, _ = m.exchange({'c': c, 'b': b, 'a': a})
    assert set(first) == set(second) == {'a', 'b'}
    assert first['a'].proposal.proposal_id == second['a'].proposal.proposal_id


def test_clipping_is_checked_before_ack_and_tampered_application_is_rejected():
    m = module()
    a, b, pa, _ = pair()
    outputs, _ = m.exchange({'a': a, 'b': b})
    command = outputs['a']
    u, _ = command.for_problem(pa)
    command.audit_applied(a.position, u, .05)
    with pytest.raises(ValueError):
        command.audit_applied(a.position, -u, .05)
    with pytest.raises(ValueError):
        command.audit_applied([.001, 0.], u, .05)
    # Existing domain clipping would erase b's retreat and violate the mutual row.
    a, b, _, _ = pair(domain=(-1., .3, -1., 1.))
    outputs, _ = m.exchange({'a': a, 'b': b})
    assert not outputs


def test_unsafe_initial_pair_is_not_rebranded_as_safe_coordination():
    m = module()
    a, _, _ = offer('a', [0., 0.], {'b': [.27, 0.]}, [(1., 0., .15)])
    b, _, _ = offer('b', [.27, 0.], {'a': [0., 0.]}, [(-1., 0., -.3)])
    outputs, _ = m.exchange({'a': a, 'b': b})
    assert not outputs
