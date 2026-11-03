"""Independent scalar checks of packet geometry and directional certificates."""
from __future__ import annotations

from datetime import datetime, timezone
import json
import math
from pathlib import Path

import numpy as np

from research_io import ROOT, guard, read, sha


GEOMETRY = ROOT / 'runs/research/2026-11-03-post-support-contact-packet-geometry-v1'
DIRECTION = ROOT / 'runs/research/2026-11-03-post-support-contact-direction-capability-v1'
CONTACT = ROOT / 'runs/research/2026-11-03-post-support-contact-compatibility-v2'
OUT = ROOT / 'runs/maintenance/2026-11-03-contact-packet-geometry-v1/independent-check.json'


def nearest(point, vertices):
    """Scalar segment projection and ray crossings, independent of runtime geometry."""
    inside = False
    candidates = []
    for start, end in zip(vertices, vertices[1:]+vertices[:1]):
        edge = [end[j]-start[j] for j in range(2)]
        squared = math.fsum(v*v for v in edge)
        along = math.fsum((point[j]-start[j])*edge[j] for j in range(2))/squared if squared else 0.
        along = min(1., max(0., along))
        foot = [start[j]+along*edge[j] for j in range(2)]
        candidates.append((math.hypot(*(point[j]-foot[j] for j in range(2))), foot))
        if (start[1] > point[1]) != (end[1] > point[1]):
            crossing = start[0]+(point[1]-start[1])*(end[0]-start[0])/(end[1]-start[1])
            inside ^= point[0] < crossing
    distance, foot = min(candidates, key=lambda pair: pair[0])
    return -distance if inside else distance, foot


def main():
    import post_acquisition_support_study as study
    guard()
    study.check()
    contact, geometry, direction = [read(path / 'analysis.json') for path in (CONTACT, GEOMETRY, DIRECTION)]
    for folder, analysis in ((GEOMETRY, geometry), (DIRECTION, direction)):
        assert sha(folder / 'analysis.json') == read(folder / 'outcome.json')['analysis_sha256']
        for name, digest in analysis['sources'].items():
            assert sha(ROOT / name) == sha(folder / Path(name).name) == digest
    assert geometry['contact_analysis_sha256'] == direction['contact_analysis_sha256'] == sha(CONTACT / 'analysis.json')
    assert direction['packet_geometry_sha256'] == sha(GEOMETRY / 'analysis.json')
    captures = {}
    for item in contact['schedule']:
        case = item['case']
        cid = case['condition_id']
        rows = read(GEOMETRY / (cid+'-packets.json'))['captures']
        path = study.OUT / case['case_id'] / 'replay.npz'
        assert sha(path) == direction['goals'][cid]['replay_sha256']
        with np.load(path, allow_pickle=False) as saved:
            assert saved['task/cargo_0/direction'].reshape(2).tolist() == direction['goals'][cid]['direction']
            for row in rows:
                k = row['frame']-1
                index = int(row['agent_id'].split('_')[-1])
                np.testing.assert_array_equal(row['position'], saved['agent_positions'][k, index])
                np.testing.assert_array_equal(row['evaluation_precommand_center'], saved['cargo/cargo_0/centers'][k])
                assert row['evaluation_precommand_angle'] == saved['cargo/cargo_0/angles'][k]
                assert row['timestamp'] == saved['times'][k]
                assert set(row['source_agents']) <= {row['agent_id'], *row['neighbors']}
                captures[cid, row['frame'], row['agent_id']] = row
    checked_rows = 0
    for state in geometry['states']:
        saved = captures[state['condition'], state['frame'], state['agent_id']]
        distance, foot = nearest(saved['position'], saved['evaluation_precommand_vertices'])
        assert abs(distance-state['evaluation_signed_distance']) < 1e-12
        assert abs(distance-saved['parameters']['r_safe']-state['evaluation_h']) < 1e-12
        assert math.hypot(*(a-b for a, b in zip(foot, state['evaluation_foot']))) < 1e-12
        if not state['admitted']:
            continue
        packets = dict(zip(saved['source_agents'], saved['packets']))
        residuals, without_structural, without_noise, without_both = [], [], [], []
        for row in state['rows']:
            source = packets[row['source_agent']]
            assert row['own_source'] == (row['source_agent'] == state['agent_id'])
            assert row['noise'] == source['noise_allowance']
            assert source['timestamp'] == saved['timestamp']
            assert row['start'] == source['points'][row['indices'][0]]
            assert row['end'] == source['points'][row['indices'][-1]]
            if row['kind'] == 'chord':
                assert row['indices'] in source['chords']
                structural = .5*math.hypot(*(a-b for a, b in zip(row['end'], row['start'])))
            else:
                object_id = source['object_ids'][row['indices'][0]]
                gaps = [math.hypot(*(a-b for a, b in zip(point, row['start'])))
                    for p in packets.values() for point, oid in zip(p['points'], p['object_ids']) if oid == object_id]
                structural = .5*min(gap for gap in gaps if gap > 1e-8)
            assert abs(structural-row['structural']) < 1e-12
            edge = [b-a for a, b in zip(row['start'], row['end'])]
            norm2 = math.fsum(v*v for v in edge)
            t = min(1., max(0., math.fsum((p-a)*e for p, a, e in zip(saved['position'], row['start'], edge))/norm2)) if norm2 else 0.
            feature_foot = [a+t*e for a, e in zip(row['start'], edge)]
            actual_distance = math.hypot(*(p-f for p, f in zip(saved['position'], feature_foot)))
            assert abs(actual_distance-row['distance']) < 1e-12
            assert actual_distance > 1e-12  # No degenerate selected feature in this fixed cohort.
            normal = [(p-f)/actual_distance for p, f in zip(saved['position'], feature_foot)]
            assert max(abs(a-b) for a, b in zip(normal, row['normal'])) < 1e-12
            lower = actual_distance-structural-source['noise_allowance']
            assert abs(lower-row['lower_distance']) < 1e-12
            h = lower-state['r_safe']
            assert abs(h-row['h']) < 1e-12
            residual = math.fsum(n*u for n, u in zip(normal, state['original_command']))+state['gamma_obj']*h-state['rho']-state['demand_speed']
            assert abs(residual-row['uncapped_residual']) < 1e-12
            residuals.append(residual)
            without_structural.append(residual+state['gamma_obj']*structural)
            without_noise.append(residual+state['gamma_obj']*row['noise'])
            without_both.append(residual+state['gamma_obj']*(structural+row['noise']))
            checked_rows += 1
        for name, values in (('minimum_uncapped_residual', residuals), ('minimum_without_structural', without_structural),
                             ('minimum_without_noise', without_noise), ('minimum_without_both', without_both)):
            assert abs(min(values)-state[name]) < 1e-12
    assert len(captures) == 128 and checked_rows == 3813
    original = {(r['condition'], e['saved_frame'], s['agent_id']): s
        for r in contact['results'] for e in r['events'] for s in e['states']}
    gaps, reverse, separators, chosen_negative = [], [], [], []
    systems = 0
    for state in direction['states']:
        key = state['condition'], state['frame'], state['agent_id']
        old = original[key]
        params = captures[key]['parameters']
        for model, result in state['systems'].items():
            matrix = old['original_rows'] if model == 'original_aggregate' else old['counterfactual_rows']
            assert result['A'] == matrix['A'] and result['kinds'] == matrix['kinds']
            expected_b = list(matrix['b'])
            if model == 'finite_uncapped':
                object_index = 0
                for i, kind in enumerate(matrix['kinds']):
                    if kind == 'object':
                        expected_b[i] = old['support']['demand_speed']-params['gamma_obj']*matrix['h'][object_index]+params['rho']
                        object_index += 1
            assert result['b'] == expected_b and result['speed_limit'] == params['max_speed']
            A, b, g, V = result['A'], result['b'], result['goal'], result['speed_limit']
            assert g == direction['goals'][state['condition']]['direction']
            if result['status'] == 'VERIFIED_FEASIBLE':
                lam = result['dual_multipliers']
                assert min(lam) >= 0.
                c = [g[j]+math.fsum(row[j]*w for row, w in zip(A, lam)) for j in range(2)]
                upper = V*math.hypot(*c)-math.fsum(bb*w for bb, w in zip(b, lam))
                assert abs(upper-result['dual_upper_bound']) < 1e-12
                lower = math.fsum(gg*uu for gg, uu in zip(g, result['u']))
                residual = min(math.fsum(aa*uu for aa, uu in zip(row, result['u']))-bb for row, bb in zip(A, b))
                assert residual >= -1e-7 and math.hypot(*result['u']) <= V+1e-7
                gaps.append(upper-lower)
                assert -1e-6 <= upper-lower <= 1e-6
                if result['classification'] == 'MUST_REVERSE':
                    relaxed = upper+1e-7*(math.hypot(*c)+math.fsum(lam))
                    assert relaxed < -1e-7
                    reverse.append(relaxed)
                elif result['classification'] == 'POSITIVE_CAPABILITY':
                    assert lower > 1e-7
                    if model == 'finite_capped' and state['positive_force'] and state['counterfactual_goal_velocity'] < 0.:
                        chosen_negative.append(dict(condition=key[0], frame=key[1], agent_id=key[2],
                            declared_pusher=state['declared_pusher'], maximum=lower, chosen=state['counterfactual_goal_velocity']))
                else:
                    raise AssertionError(result['classification'])
            elif result['status'] == 'VERIFIED_INFEASIBLE':
                lam = result['separator_multipliers']
                assert min(lam) >= 0. and abs(math.fsum(lam)-1.) < 1e-12
                c = [math.fsum(row[j]*w for row, w in zip(A, lam)) for j in range(2)]
                bound = V*math.hypot(*c)-math.fsum(bb*w for bb, w in zip(b, lam))
                assert abs(bound-result['separator_upper_bound']) < 1e-12
                assert bound+1e-7*(math.hypot(*c)+math.fsum(lam)) < -1e-7
                separators.append(bound)
            else:
                raise AssertionError(result['status'])
            systems += 1
    assert systems == 336
    receipt = dict(status='VERIFIED', checked_at_utc=datetime.now(timezone.utc).isoformat(),
        verifier_sha256=sha(__file__), geometry_analysis_sha256=sha(GEOMETRY / 'analysis.json'),
        direction_analysis_sha256=sha(DIRECTION / 'analysis.json'),
        states=128, object_rows=checked_rows, systems=systems, evaluation_geometry='SCALAR_RECOMPUTED',
        maximum_primal_dual_gap=max(gaps), reversing_certificates=len(reverse),
        least_negative_tolerance_adjusted_upper_bound=max(reverse),
        infeasible_certificates=len(separators), least_negative_separator=max(separators),
        negative_selected_despite_positive_capability=chosen_negative,
        frozen_identities='UNCHANGED', new_physical_episodes=0)
    study.check()
    with OUT.open('x', encoding='utf-8') as stream:
        json.dump(receipt, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(receipt), flush=True)


if __name__ == '__main__':
    main()
