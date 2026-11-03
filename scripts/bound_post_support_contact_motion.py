"""Bound every signed-motion explanation of a fixed original contact command."""
from __future__ import annotations

from collections import Counter
import json
from pathlib import Path
import shutil
import traceback

import numpy as np

from research_io import ROOT, guard, read, sha
from dbact_sim.experiments.deployment import dump


DECOMPOSITION = ROOT / 'runs/research/2026-11-03-post-support-contact-row-decomposition-v1'
CONTACT = ROOT / 'runs/research/2026-11-03-post-support-contact-compatibility-v2'
OUT = ROOT / 'runs/research/2026-11-03-post-support-contact-motion-envelope-v1'
DESIGN = ROOT / 'experiments/ccta2027/post_support_contact_motion_envelope_v1_design.md'


def main():
    guard()
    prior = read(DECOMPOSITION / 'analysis.json')
    contact = read(CONTACT / 'analysis.json')
    assert read(DECOMPOSITION / 'outcome.json')['analysis_sha256'] == sha(DECOMPOSITION / 'analysis.json')
    assert prior['input_sha256'] == sha(CONTACT / 'analysis.json')
    originals = {(r['condition'], e['saved_frame'], s['agent_id']): s
                 for r in contact['results'] for e in r['events'] for s in e['states']}
    identity = dict(decomposition_sha256=sha(DECOMPOSITION / 'analysis.json'),
        contact_analysis_sha256=sha(CONTACT / 'analysis.json'), helper_sha256=sha(__file__),
        design_sha256=sha(DESIGN), runtime_source_sha256=prior['runtime_source_sha256'],
        protocol_sha256=prior['protocol_sha256'], new_physical_episodes=0)
    OUT.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, OUT / Path(__file__).name)
    shutil.copy2(DESIGN, OUT / DESIGN.name)
    dump(OUT / 'started.json', identity)
    rows, counts = [], Counter()
    try:
        for source in prior['rows']:
            row = {key: source[key] for key in ('condition', 'frame', 'agent_id', 'admitted', 'positive_force', 'declared_pusher')}
            counts['all_states'] += 1
            if row['admitted']:
                saved = originals[row['condition'], row['frame'], row['agent_id']]
                matrix = saved['counterfactual_rows']
                mask = np.asarray(matrix['kinds']) == 'object'
                n = np.asarray(matrix['A'])[mask][source['limiting_row']]
                h = float(np.asarray(matrix['h'])[source['limiting_row']])
                u = np.asarray(saved['original_command'])
                p = prior['parameters'][row['condition']]
                B = p['object_velocity_bound']
                norm = float(np.linalg.norm(n))
                assert norm > 0.
                np.testing.assert_array_equal(n, source['limiting_normal'])
                assert h == source['limiting_h']
                without_motion = float(n @ u + p['gamma_obj'] * h - p['rho'])
                np.testing.assert_allclose(without_motion, source['minimum_residual_without_motion'], rtol=0., atol=1e-12)
                upper = without_motion + B * norm
                attaining = -B * n / norm
                np.testing.assert_allclose(np.linalg.norm(attaining), B, rtol=0., atol=1e-12)
                np.testing.assert_allclose(n @ u + p['gamma_obj'] * h - p['rho'] - n @ attaining, upper, rtol=0., atol=1e-12)
                row.update(row_normal=n.tolist(), h=h, original_command=u.tolist(), gamma_obj=p['gamma_obj'],
                    rho=p['rho'], configured_motion_bound=B, attaining_motion=attaining.tolist(),
                    maximum_row_residual=float(upper), original_command_rejected_for_all_bounded_motion=bool(upper < -1e-7))
                for group in ('admitted', *(['positive_force'] if row['positive_force'] else []),
                              *(['positive_force_pusher'] if row['positive_force'] and row['declared_pusher'] else [])):
                    counts[group] += 1
                    counts[group + '_original_command_rejected_for_all_bounded_motion'] += int(upper < -1e-7)
            rows.append(row)
        assert len(rows) == 128 and counts['admitted'] == 112 and counts['positive_force'] == 17 and counts['positive_force_pusher'] == 11
        dump(OUT / 'analysis.json', dict(**identity, counts=dict(counts), rows=rows,
            scope='One-row upper-bound witnesses for the fixed original commands over the configured motion ball; nonnegative bounds are inconclusive about joint feasibility or other commands.'))
        dump(OUT / 'outcome.json', dict(**identity, status='VERIFIED_MOTION_ENVELOPE', analysis_sha256=sha(OUT / 'analysis.json')))
        print(json.dumps(dict(counts)), flush=True)
    except Exception:
        dump(OUT / 'failure.json', dict(**identity, traceback=traceback.format_exc()))
        raise


if __name__ == '__main__':
    main()
