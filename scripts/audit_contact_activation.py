"""Reconstruct recorded contact-activation gates; never rerun or alter physics."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import gc
import json
from pathlib import Path
import shutil
import sys

import numpy as np

from research_io import ROOT, guard, read, sha


def audit(destination, case_id=None):
    guard()
    if any(k == 'dbact' or k.startswith(('dbact.', 'dbact_sim')) for k in sys.modules):
        raise RuntimeError('Frozen imports require a fresh process')
    protocol = ROOT / 'experiments/ccta2027/acquisition_circulation_v1.json'
    plan = read(protocol)
    batch = ROOT / plan['run_dir']
    for name, digest in plan['source_files'].items():
        assert sha(batch / 'source_snapshot' / name) == digest
    sys.path.insert(0, str(batch / 'source_snapshot/src'))
    from dbact_sim.experiments.study import cached_result
    from dbact_sim.experiments.deployment import dump, encode
    from dbact.control.controller import DBACTParams
    manifest = read(batch / 'manifest.json')
    assert manifest['identity']['protocol_sha256'] == sha(protocol)
    cases = [c for c in plan['cases'] if case_id is None or c['case_id'] == case_id]
    assert cases
    target = Path(destination).resolve()
    if not target.is_relative_to(ROOT / 'runs'):
        raise ValueError('Use a fresh runs directory')
    target.mkdir(parents=True, exist_ok=False)
    for name in ('audit_contact_activation.py', 'research_io.py'):
        shutil.copy2(ROOT / 'scripts' / name, target / name)
    design = ROOT / 'experiments/ccta2027/contact_activation_v1_design.md'
    shutil.copy2(design, target / 'design.md')
    identity = dict(protocol_sha256=sha(protocol), source_sha256=plan['frozen_source_sha256'],
                    design_sha256=sha(design), script_sha256=sha(__file__),
                    scope='Retrospective saved-predicate reconstruction, not controller replay or new physics',
                    new_physical_episodes=0, cases=[c['case_id'] for c in cases])
    dump(target / 'identity.json', identity)
    summaries = []
    try:
        for case in cases:
            name = case['case_id']
            folder = batch / name
            result = cached_result(folder, {**manifest['identity'], 'case_sha256': encode(case)})
            p = DBACTParams(**case['config']['controller'])
            assert p.communication_dropout_prob == 0 and p.boundary_navigation == 'vector_goal'
            assert p.task_mode == 'transport' and p.transport_law == 'progress'
            assert not (p.dynamic_contact_roles or p.confirmed_contact_handoff or p.contact_reinforcement)
            steps = read(folder / 'paired_metrics.json')['steps']
            with np.load(folder / 'replay.npz', allow_pickle=False) as source:
                positions = source['agent_positions'].copy()
            phase_counts = defaultdict(Counter)
            per_robot = defaultdict(Counter)
            previous_push, checked = 0, 0
            selected = {}
            with (target / (name + '.csv')).open('x', encoding='utf-8', newline='') as stream:
                fields = ['frame', 'phase', 'ready', 'called', 'push', 'ready_feedback_missing',
                          'ready_quorum_missing', 'ready_wrong_alignment', 'map_unavailable',
                          'positive_contacts', 'forward_force', 'net_force_x', 'net_force_y']
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for k, row in enumerate(steps):
                    diags = row['diagnostics']
                    assert row['phase_guards']['transport_active'] == previous_push, (name, k, 'phase lag')
                    previous_push = sum(d['push_side'] for d in diags)
                    np.testing.assert_array_equal(positions[k+1], row['robot_positions'])
                    distances = np.linalg.norm(positions[k, :, None, :] - positions[k, None, :, :], axis=2)
                    np.fill_diagonal(distances, np.inf)
                    links = distances <= p.comm_range
                    ready = np.array([d['contact_ready'] for d in diags], bool)
                    counts = Counter(frames=1, robot_steps=len(diags))
                    for i, d in enumerate(diags):
                        called = d['mode'] in ('push', 'cage')
                        nav = d['navigation_remaining']
                        normal = d['contact_selected_normal']
                        command = d['contact_desired_direction']
                        geometric = all(v is not None for v in (*normal, *command))
                        alignment = float(np.dot(normal, command)) if geometric else None
                        quorum = 1 + int(ready[links[i]].sum())
                        predicates = dict(ready=bool(ready[i]), called=called,
                            feedback=nav is not None, remaining=nav is not None and nav > 0,
                            quorum=quorum >= p.min_push_agents,
                            alignment=alignment is not None and alignment <= -p.push_side_threshold,
                            gap=d['contact_used_gap'] is not None and d['contact_used_gap'] <= p.standoff_range,
                            enabled=row['phase'] in ('CONTACT_READY', 'TRANSPORT', 'BRAKE') and not row['phase_response']['push_suspended'])
                        if called:
                            expected = all(predicates.values())
                            assert expected == d['push_side'], (name, k+1, d['agent_id'], predicates, d['push_side'])
                            checked += 1
                        events = {'mode:' + d['mode']: 1, 'ready': int(ready[i]),
                                  'called': int(called), 'push': int(d['push_side']),
                                  'ready_feedback_missing': int(ready[i] and called and not predicates['feedback']),
                                  'ready_quorum_missing': int(ready[i] and not predicates['quorum']),
                                  'ready_wrong_alignment': int(ready[i] and geometric and not predicates['alignment']),
                                  'map_unavailable': int(d.get('map_reference_status') == 'unavailable'),
                                  'ready_gap_rejected': int(ready[i] and called and not predicates['gap']),
                                  'ready_no_redeploy': int(ready[i] and d['redeploy_reason'] == 'contact_ready'),
                                  'push_nonzero_effort': int(d['push_side'] and d['effort'] > 0),
                                  'push_speed_nominal_sum': d['speed_nominal'] if d['push_side'] else 0.,
                                  'push_speed_command_sum': d['speed_command'] if d['push_side'] else 0.}
                        counts.update(events)
                        per_robot[(row['phase'], d['agent_id'])].update(events)
                    phase_counts[row['phase']].update(counts)
                    writer.writerow(dict(frame=k+1, phase=row['phase'], **{key: counts[key] for key in fields[2:9]},
                        positive_contacts=row['positive_contacts'], forward_force=row['forward_force'],
                        net_force_x=row['actual_net_force'][0], net_force_y=row['actual_net_force'][1]))
                    if row['phase'] not in selected or k == len(steps)-1:
                        selected[row['phase'] if row['phase'] not in selected else 'final'] = dict(
                            frame=k+1, phase=row['phase'], positions=positions[k].tolist(),
                            diagnostics=diags, contact_ids=row['contact_ids'],
                            contact_normals=row['contact_normals'], contact_forces=row['contact_forces'])
            report = dict(case_id=name, condition=case['condition_id'], arm=case['arm'], frames=len(steps),
                receipt_sha256=sha(folder / 'case_result.json'), checked_transport_calls=checked,
                all_recorded_push_predicates_exact=True, all_supervisor_lags_exact=True,
                phase_counts=dict(phase_counts), per_robot={phase+'|'+aid: value for (phase, aid), value in per_robot.items()},
                selected=selected, task_completed=result['task_completed'])
            dump(target / (name + '.json'), report)
            summaries.append({key:value for key,value in report.items() if key not in ('selected', 'per_robot')})
            print(json.dumps(dict(case=name, checked=checked, contact_ready=dict(phase_counts['CONTACT_READY']))), flush=True)
            del steps, positions, result, report, selected
            gc.collect()
        dump(target / 'summary.json', dict(**identity, status='EXACT_SAVED_PREDICATES', summaries=summaries,
            products={p.name:sha(p) for p in target.iterdir() if p.suffix in ('.csv', '.json') and p.name != 'summary.json'}))
    except Exception as exc:
        dump(target / 'failure.json', dict(**identity, completed_cases=len(summaries), error=repr(exc)))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--case')
    args = parser.parse_args()
    audit(args.out, args.case)
