"""Attribute existing finite-row residuals without changing any control input."""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import traceback

import numpy as np

import post_acquisition_support_study as study
from research_io import guard, read, sha
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump


SOURCE = study.ROOT / 'runs/research/2026-11-03-post-support-contact-compatibility-v2'
OUT = study.ROOT / 'runs/research/2026-11-03-post-support-contact-row-decomposition-v1'
DESIGN = study.ROOT / 'experiments/ccta2027/post_support_contact_row_decomposition_v1_design.md'


def main():
    guard()
    plan = study.check()
    source = read(SOURCE / 'analysis.json')
    outcome = read(SOURCE / 'outcome.json')
    assert outcome['status'] == 'VERIFIED_CONTACT_COMPATIBILITY'
    assert sha(SOURCE / 'analysis.json') == outcome['analysis_sha256']
    assert source['runtime_source_sha256'] == plan['frozen_source_sha256']
    assert source['protocol_sha256'] == sha(study.PROTOCOL)
    params = {}
    for item in source['schedule']:
        case = item['case']
        params[case['condition_id']] = asdict(SimulationEnvironment(case['config'], seed=case['seed']).controller.safety.params)
    identity = dict(input_sha256=outcome['analysis_sha256'], protocol_sha256=sha(study.PROTOCOL),
        runtime_source_sha256=plan['frozen_source_sha256'], helper_sha256=sha(__file__),
        design_sha256=sha(DESIGN), parameters=params, new_physical_episodes=0)
    OUT.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, OUT / Path(__file__).name)
    shutil.copy2(DESIGN, OUT / DESIGN.name)
    dump(OUT / 'started.json', identity)
    rows, counts = [], Counter()
    try:
        for condition in source['results']:
            p = params[condition['condition']]
            for event in condition['events']:
                for state in event['states']:
                    row = dict(condition=condition['condition'], frame=event['saved_frame'],
                        agent_id=state['agent_id'], positive_force=state['positive_physical_force'] > 0.,
                        declared_pusher=state['declared_pusher'], admitted=state['support']['applied'],
                        guard=state['support']['reason'])
                    counts['all_states'] += 1
                    if row['admitted']:
                        matrix = state['counterfactual_rows']
                        mask = np.asarray(matrix['kinds']) == 'object'
                        A = np.asarray(matrix['A'])[mask]
                        h = np.asarray(matrix['h'])
                        b = np.asarray(matrix['b'])[mask]
                        u = np.asarray(state['original_command'])
                        motion = state['support']['demand_speed']
                        command = A @ u
                        geometric = p['gamma_obj'] * h
                        residual = command + geometric - p['rho'] - motion
                        assert len(residual) == len(h) > 0
                        np.testing.assert_allclose(residual.min(), state['original_command_min_uncapped_object_residual'], rtol=0., atol=1e-12)
                        np.testing.assert_allclose((command-b).min(), state['original_command_min_capped_object_residual'], rtol=0., atol=1e-12)
                        k = int(np.argmin(residual))
                        without_motion = residual + motion
                        row.update(minimum_uncapped_residual=float(residual[k]),
                            minimum_residual_without_motion=float(without_motion.min()),
                            still_conflicting_without_motion=bool(without_motion.min() < -1e-7),
                            original_command=u.tolist(), limiting_row=k, limiting_normal=A[k].tolist(),
                            limiting_h=float(h[k]), command_term=float(command[k]),
                            geometric_term=float(geometric[k]), margin_term=-p['rho'], motion_term=-motion,
                            object_rows=len(h), r_safe=p['r_safe'], gamma_obj=p['gamma_obj'], rho=p['rho'])
                        for group in ('admitted', *(['positive_force'] if row['positive_force'] else []),
                                      *(['positive_force_pusher'] if row['positive_force'] and row['declared_pusher'] else [])):
                            counts[group] += 1
                            counts[group + '_still_conflicting_without_motion'] += int(row['still_conflicting_without_motion'])
                            counts[group + '_negative_h'] += int(h[k] < 0.)
                    rows.append(row)
        assert len(rows) == counts['all_states'] == 128
        assert counts['admitted'] == 112 and counts['positive_force'] == 17 and counts['positive_force_pusher'] == 11
        report = dict(**identity, rows=rows, counts=dict(counts),
            scope='Post-hoc algebra on all preselected control contact events. Removing the motion term here is a diagnostic, never an executed or proposed parameter change.')
        dump(OUT / 'analysis.json', report)
        dump(OUT / 'outcome.json', dict(status='VERIFIED_ROW_DECOMPOSITION', analysis_sha256=sha(OUT / 'analysis.json'), **identity))
        print(json.dumps(dict(counts)), flush=True)
    except Exception:
        dump(OUT / 'failure.json', dict(**identity, traceback=traceback.format_exc()))
        raise


if __name__ == '__main__':
    main()
