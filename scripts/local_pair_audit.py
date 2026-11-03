"""Independent arithmetic checks of recorded pair executions and study gates."""
from collections import Counter
import hashlib
import json

import numpy as np

from research_io import read

ARMS = ('finite_acquisition_support', 'local_pair_candidate')


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        allow_nan=False).encode()).hexdigest()


def verify_commit(record, positions, commands, *, frame, timestamp, dt, comm_range):
    """Check received rows against measured commands, without importing the protocol."""
    members, offers = record['members'], record['offers']
    assert len(members) == len(set(members)) == len(offers) == 2
    assert members == sorted(members) == [o['agent_id'] for o in offers]
    assert record['requester'] in members
    proposal = dict(requester=record['requester'], offers=offers,
        velocities=record['applied_commands'], raw_velocities=record['raw_commands'],
        solver_status=record['solver_status'])
    assert digest(proposal) == record['proposal_id']
    acknowledgements = record['acknowledgements']
    assert len(acknowledgements) == 2 and {a['agent_id'] for a in acknowledgements} == set(members)
    residuals, speeds = [], []
    matrices, bounds, mutual = [], [], []
    for index, offer in enumerate(offers):
        aid, peer = members[index], members[1-index]
        assert offer['frame'] == frame and abs(offer['timestamp']-timestamp) <= 1e-9
        assert offer['dt'] == dt
        params = dict(offer['contract'])
        assert params == dict(offers[1-index]['contract'])
        assert params['backend'] == 'qp' and not params['clamp_margin_to_keep_zero']
        assert params['dt'] == dt and params['gamma_agent']*dt <= 1.+1e-9
        assert offer['domain'] == offers[1-index]['domain']
        np.testing.assert_allclose(offer['position'], positions[aid], rtol=0., atol=1e-12)
        neighbor_ids = [x[0] for x in offer['neighbors']]
        assert len(neighbor_ids) == len(set(neighbor_ids)) == offer['agent_count']
        assert peer in neighbor_ids and aid not in neighbor_ids
        A, b = np.asarray(offer['A']), np.asarray(offer['b'])
        assert A.shape == (len(b), 2)
        assert offer['object_count'] > 0
        assert len(b) == offer['agent_count']+offer['wall_count']+offer['object_count']
        assert np.isfinite(A).all() and np.isfinite(b).all()
        for k, (neighbor, claimed_position) in enumerate(offer['neighbors']):
            np.testing.assert_allclose(claimed_position, positions[neighbor], rtol=0., atol=1e-12)
            delta = positions[aid]-positions[neighbor]
            assert np.linalg.norm(delta) <= comm_range+1e-12
            np.testing.assert_allclose(A[k], 2*delta, rtol=0., atol=1e-12)
            np.testing.assert_allclose(b[k], -.5*params['gamma_agent']*(delta@delta-params['d_min']**2),
                rtol=0., atol=1e-12)
        source = json.loads(offer['source_json'])
        assert source['applied'] and source['stage'] == 'stationary_acquisition'
        assert source['motion_bound'] == 0. and source.get('cap_active', 0) == 0
        assert aid in source['source_agents'] and set(source['source_agents']) <= {aid, *neighbor_ids}
        assert len(source['source_agents']) == len(source['source_timestamps'])
        assert all(abs(t-timestamp) <= 1e-9 for t in source['source_timestamps'])
        ack = next(a for a in acknowledgements if a['agent_id'] == aid)
        assert ack['peer_id'] == peer and ack['frame'] == frame and ack['timestamp'] == offer['timestamp']
        assert ack['offer_id'] == digest(offer) and ack['proposal_id'] == record['proposal_id']
        velocity = np.asarray(commands[aid])
        np.testing.assert_allclose(velocity, record['applied_commands'][index], rtol=0., atol=1e-10)
        assert np.isfinite(velocity).all()
        speed = float(np.linalg.norm(velocity))
        assert speed <= params['max_speed']+1e-7
        xmin, xmax, ymin, ymax = offer['domain']
        end = positions[aid]+dt*velocity
        assert xmin-1e-12 <= end[0] <= xmax+1e-12 and ymin-1e-12 <= end[1] <= ymax+1e-12
        omit = neighbor_ids.index(peer)
        kept = np.delete(A@velocity-b, omit)
        assert len(kept) and kept.min() >= -1e-7
        residuals.extend(kept.tolist()); speeds.append(speed)
        matrices.append(A); bounds.append(b); mutual.append(omit)
    first, second = members
    relative = positions[first]-positions[second]
    relative_velocity = commands[first]-commands[second]
    params = dict(offers[0]['contract'])
    assert relative@relative >= params['d_min']**2
    np.testing.assert_allclose(matrices[0][mutual[0]], -matrices[1][mutual[1]], rtol=0., atol=1e-12)
    demand = -params['gamma_agent']*(relative@relative-params['d_min']**2)
    np.testing.assert_allclose(bounds[0][mutual[0]]+bounds[1][mutual[1]], demand, rtol=0., atol=1e-12)
    mutual_residual = float(2*relative@relative_velocity-demand)
    assert mutual_residual >= -1e-7
    squared_speed = relative_velocity@relative_velocity
    tau = float(np.clip(-(relative@relative_velocity)/squared_speed, 0., dt)) if squared_speed else 0.
    minimum_distance = float(np.linalg.norm(relative+tau*relative_velocity))
    assert minimum_distance >= params['d_min']-1e-7
    residuals.append(mutual_residual)
    result = dict(frame=frame, timestamp=timestamp, members=members, speeds=speeds,
        minimum_residual=min(residuals), mutual_residual=mutual_residual,
        minimum_hold_separation=minimum_distance, proposal_id=record['proposal_id'])
    np.testing.assert_allclose(record['certificate']['row_residuals'], residuals, rtol=0., atol=1e-9)
    assert record['certificate']['valid']
    return result


def audit_episode(folder, case):
    steps = read(folder/'paired_metrics.json')['steps']
    counts, reasons, events = Counter(), Counter(), []
    previous_paired = 0
    with np.load(folder/'replay.npz', allow_pickle=False) as replay:
        xy, times = replay['agent_positions'], replay['times']
        assert len(xy) == len(times) == len(steps)+1
        for k, step in enumerate(steps):
            ids = [d['agent_id'] for d in step['diagnostics']]
            positions = dict(zip(ids, xy[k]))
            actual = dict(zip(ids, (xy[k+1]-xy[k])/case['config']['dt']))
            declared = dict(zip(ids, np.asarray(step['commands'])))
            grouped = {}
            for diag in step['diagnostics']:
                aid = diag['agent_id']
                counts['robot_steps'] += 1
                if case['arm'] == ARMS[0]:
                    assert 'pair_safety' not in diag and diag['solver_status'] != 'paired'
                    continue
                trace = diag['pair_safety']
                reasons[trace['reason']] += 1
                counts['eligible_robot_steps'] += bool(trace['eligible'])
                if trace.get('independent_feasible') is False:
                    counts['independent_infeasible_proposals'] += 1
                    assert trace['independent_proposal_unexecuted'] and trace.get('original_offer')
                counts['peer_trials'] += len(trace.get('attempts', []))
                for trial in trace.get('attempts', []):
                    assert trial.get('peer_offer')
                    reasons['trial:'+trial['reason']] += 1
                accepted = bool(trace.get('accepted', False))
                assert accepted == (diag['solver_status'] == 'paired')
                if not accepted:
                    continue
                assert step['initial_reference']['status'] == 'acquiring'
                assert trace['eligible'] and trace['executed_status'] == 'paired'
                assert trace['executed_barrier_scale'] == 1.
                record = trace['execution']
                own = next(o for o in record['offers'] if o['agent_id'] == aid)
                assert own['object_count'] == diag['acquisition_support']['object_rows']
                assert own['source_json'] == json.dumps(diag['acquisition_support'], sort_keys=True, allow_nan=False)
                grouped.setdefault(record['proposal_id'], []).append((aid, record))
                counts['paired_robot_steps'] += 1
            occupied = set()
            for same in grouped.values():
                assert len(same) == 2 and same[0][1] == same[1][1]
                record = same[0][1]
                assert {a for a, _ in same} == set(record['members'])
                assert not occupied.intersection(record['members'])
                occupied.update(record['members'])
                for aid in record['members']:
                    np.testing.assert_allclose(actual[aid], declared[aid], rtol=0., atol=1e-10)
                event = verify_commit(record, positions, actual, frame=k, timestamp=float(times[k]),
                    dt=case['config']['dt'], comm_range=case['config']['controller']['comm_range'])
                events.append(dict(**event, record=record))
            current_paired = step['solver_statistics']['statuses'].get('paired', 0)
            assert current_paired-previous_paired == 2*len(grouped)
            previous_paired = current_paired
            counts['frames'] += 1
    assert counts['paired_robot_steps'] == 2*len(events) == previous_paired
    return dict(verified=True, counts=dict(counts), reasons=dict(reasons), events=events,
        scope='All recorded current pair rows and measured execution, not unobserved geometry or asynchronous safety')


def evaluate_gates(rows, reproductions, conditions):
    expected = {(c, a) for c in conditions for a in ARMS}
    keys = [(r['condition'], r['arm']) for r in rows]
    schedule = len(rows) == len(set(keys)) == 8 and len(conditions) == 4 and set(keys) == expected
    baseline = {r['condition']: r for r in rows if r['arm'] == ARMS[0]}
    candidate = [r for r in rows if r['arm'] == ARMS[1]]
    mechanism = dict(resolved_schedule_complete=schedule,
        all_eight_execution_complete=schedule and all(r['execution']=='COMPLETED' for r in rows),
        four_original_controls_exact=set(reproductions)==set(conditions) and all(r['exact'] for r in reproductions.values()),
        four_candidates_safe_complete=schedule and all(r['safe_completed'] and not r['false_hold']
            and not r['hard_safety'] and r['zero_epoch_valid'] for r in candidate),
        candidate_acquisition_scaling_zero=schedule and all(r.get('scaling') is not None
            and r['scaling']['acquisition']==0 for r in candidate),
        no_post_acquisition_scaling_increase=schedule and all(r.get('scaling') is not None
            and baseline[r['condition']].get('scaling') is not None
            and r['scaling']['post_acquisition']<=baseline[r['condition']]['scaling']['post_acquisition'] for r in candidate),
        preserve_each_baseline_g500_pass=schedule and all(not baseline[r['condition']]['g500'] or r['g500'] for r in candidate),
        all_execution_audits=schedule and all((r.get('pair_audit') or {}).get('verified', False) for r in rows))
    full = dict(**mechanism, candidate_all_original_g500=schedule and all(r['g500'] for r in candidate),
        candidate_all_formal_transport=schedule and all(r.get('phase_response', {}).get('transport_frame') is not None for r in candidate))
    return mechanism, full
