"""Stream the complete frozen episodes and identify their first command change."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

from collections import Counter
import argparse
import csv
from datetime import datetime, timezone
from itertools import islice
import json
from pathlib import Path
import shutil
import traceback

import numpy as np

from research_io import ROOT, guard, read, sha
from stream_step_receipts import iter_step_receipts
from trace_boundary_map import plain
import post_acquisition_support_study as study
from dbact_sim.experiments.deployment import dump

OUT = ROOT / 'runs/research/2026-11-03-post-support-candidate-onset-v1'
DESIGN = ROOT / 'experiments/ccta2027/post_support_candidate_onset_v1_design.md'
EPS = 1e-7
EVENTS = ('first_frozen_reference', 'first_finite_support',
          'first_positive_nominal_pusher_nonpositive_command', 'first_frozen_positive_force')


def saved_arrays(folder):
    keys = ('times', 'agent_ids', 'agent_positions', 'domain', 'push_flags',
            'cargo/cargo_0/centers', 'cargo/cargo_0/angles',
            'cargo/cargo_0/progress_estimate', 'task/cargo_0/direction', 'task/cargo_0/start')
    with np.load(folder / 'replay.npz', allow_pickle=False) as archive:
        return {key: archive[key].copy() for key in keys}


def selected_row(folder, frame):
    with (folder / 'paired_metrics.json').open(encoding='utf-8') as stream:
        row = next(islice(iter_step_receipts(stream), frame - 1, frame))
    assert row['frame'] == frame
    return row


def scan(case):
    folder = study.OUT / case['case_id']
    saved = saved_arrays(folder)
    ids = saved['agent_ids'].tolist()
    goal = saved['task/cargo_0/direction'].reshape(2)
    assert abs(np.linalg.norm(goal) - 1.) < 1e-12
    total = len(saved['times']) - 1
    commands = np.empty((total, len(ids), 2))
    nominal = np.empty_like(commands)
    admission = np.zeros((total, len(ids)), dtype=bool)
    events, totals, phases, statuses, modes = {}, Counter(), Counter(), Counter(), Counter()
    curves = []
    audit_path = OUT / 'audit_inputs' / (case['case_id'] + '.json')
    audited = (read(audit_path).get('post_support_audit') or {}) if audit_path.exists() else {}
    violations = {(v['frame'], v['agent_id']): v for v in audited.get('violations', [])}
    violation_matches = []
    last = None
    with (folder / 'paired_metrics.json').open(encoding='utf-8') as stream:
        iterator = iter_step_receipts(stream)
        while True:
            try:
                row = next(iterator)
            except StopIteration as end:
                parsed_summary = end.value
                break
            frame = len(curves) + 1
            assert row['frame'] == frame and frame <= total
            assert [d['agent_id'] for d in row['diagnostics']] == ids
            k = frame - 1
            u = np.asarray(row['commands'], dtype=float)
            n = np.asarray([d['nominal_command'] for d in row['diagnostics']], dtype=float)
            assert u.shape == n.shape == (len(ids), 2)
            assert np.isfinite(u).all() and np.isfinite(n).all()
            commands[k], nominal[k] = u, n
            np.testing.assert_array_equal(row['robot_positions'], saved['agent_positions'][frame])
            domain = saved['domain']
            applied_position = saved['agent_positions'][k] + u * case['config']['dt']
            applied_position[:, 0] = np.clip(applied_position[:, 0], domain[0], domain[1])
            applied_position[:, 1] = np.clip(applied_position[:, 1], domain[2], domain[3])
            np.testing.assert_array_equal(applied_position, saved['agent_positions'][frame])
            displacement = saved['cargo/cargo_0/centers'][frame] - saved['task/cargo_0/start']
            np.testing.assert_array_equal(row['true_center_displacement'], displacement)
            assert abs(row['true_progress'] - float(displacement @ goal)) < 1e-12
            assert row['rotation'] == saved['cargo/cargo_0/angles'][frame]
            assert row['estimated_progress'] == saved['cargo/cargo_0/progress_estimate'][frame]
            push = np.asarray([d['push_side'] for d in row['diagnostics']], dtype=bool)
            np.testing.assert_array_equal(push, saved['push_flags'][frame])
            admitted = np.asarray([bool((d.get('post_acquisition_support') or {}).get('applied'))
                                   for d in row['diagnostics']])
            admission[k] = admitted
            frozen = row['initial_reference']['status'] == 'frozen'
            positive = n @ goal > EPS
            nonpositive = u @ goal <= EPS
            blocked = frozen & push & positive & nonpositive
            state_counts = dict(agents=len(ids), support_admitted=int(admitted.sum()),
                pushers=int(push.sum()), positive_nominal=int(positive.sum()),
                positive_nominal_nonpositive_command=int((positive & nonpositive).sum()),
                frozen_pusher_positive_nominal=int((frozen & push & positive).sum()),
                frozen_pusher_positive_nominal_nonpositive_command=int(blocked.sum()),
                admitted_frozen_pusher_positive_nominal_nonpositive_command=int((blocked & admitted).sum()))
            flags = (frozen, bool(admitted.any()), bool(blocked.any()), frozen and row['positive_force_sum'] > 0.)
            for name, flag in zip(EVENTS, flags):
                if flag and name not in events:
                    events[name] = dict(frame=frame, row=row)
            status = Counter(d['solver_status'] for d in row['diagnostics'])
            statuses.update(status)
            modes.update(d['mode'] for d in row['diagnostics'])
            phases[row['phase']] += 1
            totals.update(state_counts)
            for d in row['diagnostics']:
                key = (frame, d['agent_id'])
                if key in violations:
                    violation_matches.append(dict(**violations[key], solver_status=d['solver_status'],
                        post_support_applied=bool((d.get('post_acquisition_support') or {}).get('applied'))))
            curves.append(dict(frame=frame, time_s=float(saved['times'][frame]), phase=row['phase'],
                reference_status=row['initial_reference']['status'], true_progress=row['true_progress'],
                estimated_progress=row['estimated_progress'],
                true_goal_velocity=float(np.asarray(row['true_velocity']) @ goal),
                positive_force_sum=row['positive_force_sum'], reverse_force_sum=row['reverse_force_sum'],
                **state_counts, solver_status_counts=json.dumps(dict(status), sort_keys=True)))
            last = row
            if frame % 2000 == 0:
                print(json.dumps(dict(case=case['case_id'], streamed_frames=frame)), flush=True)
    assert len(curves) == total and parsed_summary is not None and last is not None
    result = read(folder / 'case_result.json')['result']
    assert all(value == result[key] for key, value in parsed_summary.items())
    summary = read(folder / 'summary.json')
    assert summary['steps'] == total and summary['solver'] == last['solver_statistics']
    assert dict(statuses) == summary['solver']['statuses']
    assert len(violation_matches) == len(violations)
    with (OUT / (case['case_id'] + '-frames.csv')).open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(curves[0]))
        writer.writeheader()
        writer.writerows(curves)
    np.savez_compressed(OUT / (case['case_id'] + '-commands.npz'), commands=commands,
                        nominal=nominal, support_admission=admission)
    dump(OUT / (case['case_id'] + '-events.json'), events)
    if violation_matches:
        dump(OUT / (case['case_id'] + '-audit-status-matches.json'), violation_matches)
    report = dict(case_id=case['case_id'], condition=case['condition_id'], arm=case['arm'],
        frames=total, agent_steps=totals['agents'], events={name: events.get(name, {}).get('frame') for name in EVENTS},
        totals=dict(totals), phases=dict(phases), solver_statuses=dict(statuses), modes=dict(modes),
        maximum_true_progress=max(r['true_progress'] for r in curves),
        maximum_positive_force=max(r['positive_force_sum'] for r in curves),
        final=curves[-1], parsed_summary_matches=True, npz_frames_verified=total,
        audited_violations_matched=len(violation_matches),
        capped_violation_statuses=dict(Counter(v['solver_status'] for v in violation_matches
            if v['minimum_capped_object'] < -EPS)),
        result=dict(task_completed=result['task_completed'], g500_pass=result['g500_pass'],
                    euclidean_endpoint_error=result['euclidean_endpoint_error']))
    dump(OUT / (case['case_id'] + '-summary.json'), report)
    print(json.dumps(dict(case=case['case_id'], completed_frames=total, events=report['events'])), flush=True)
    return report


def compare_pair(control, candidate):
    cases = [control, candidate]
    with np.load(OUT / (control['case_id'] + '-commands.npz')) as a, np.load(OUT / (candidate['case_id'] + '-commands.npz')) as b:
        end = min(len(a['commands']), len(b['commands']))
        mismatch = np.flatnonzero(np.any(a['commands'][:end] != b['commands'][:end], axis=(1, 2)))
        if not len(mismatch):
            return dict(condition=control['condition_id'], common_frames=end, first_divergence=None)
        k = int(mismatch[0])
        different = np.flatnonzero(np.any(a['commands'][k] != b['commands'][k], axis=1)).tolist()
        nominals_equal = np.array_equal(a['nominal'][k], b['nominal'][k])
    saved = [saved_arrays(study.OUT / case['case_id']) for case in cases]
    physical = {key: np.array_equal(saved[0][key][:k+1], saved[1][key][:k+1])
                for key in ('agent_positions', 'cargo/cargo_0/centers', 'cargo/cargo_0/angles')}
    assert all(physical.values()), (control['condition_id'], physical)
    rows = [selected_row(study.OUT / c['case_id'], k+1) for c in cases]
    compared = ('phase', 'phase_guards', 'phase_response', 'initial_reference',
                'navigation_supervisor', 'navigation_voter_trace', 'estimated_velocities', 'integrals')
    fields_equal = {key: rows[0][key] == rows[1][key] for key in compared}
    data = dict(condition=control['condition_id'], common_frames=end, first_divergence=k+1,
        prior_commands_equal=True, precommand_physical_prefix_equal=physical,
        all_nominals_equal=nominals_equal, control_fields_equal=fields_equal,
        changed_agent_ids=[saved[0]['agent_ids'][i] for i in different],
        rows={case['arm']: row for case, row in zip(cases, rows)})
    dump(OUT / (control['condition_id'] + '-first-divergence.json'), plain(data))
    return {key: plain(value) for key, value in data.items() if key != 'rows'}


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    OUT = args.out.resolve()
    if not OUT.is_relative_to(ROOT / 'runs'):
        raise ValueError('New analysis output must be under this repository runs/')
    guard()
    plan = study.check()
    sources = [Path(__file__), DESIGN, ROOT / 'scripts/stream_step_receipts.py',
               ROOT / 'tests/test_stream_step_receipts.py', ROOT / 'scripts/research_io.py',
               ROOT / 'scripts/trace_boundary_map.py']
    identities = {p.relative_to(ROOT).as_posix(): sha(p) for p in sources}
    OUT.mkdir(parents=True, exist_ok=False)
    for p in sources:
        shutil.copy2(p, OUT / p.name)
    identity = dict(started_at_utc=datetime.now(timezone.utc).isoformat(),
        runtime_source_sha256=plan['frozen_source_sha256'], protocol_sha256=sha(study.PROTOCOL),
        sources=identities, new_physical_episodes=0)
    dump(OUT / 'started.json', identity)
    completed = []
    try:
        inputs = {}
        (OUT / 'audit_inputs').mkdir()
        for case in plan['cases']:
            folder = study.OUT / case['case_id']
            receipt = read(folder / 'case_result.json')
            inputs[case['case_id']] = dict(case_result_sha256=sha(folder / 'case_result.json'), products={})
            for name in ('paired_metrics.json', 'replay.npz', 'summary.json', 'config.json', 'resolved_case.json'):
                digest = sha(folder / name)
                assert digest == receipt['products'][name]
                inputs[case['case_id']]['products'][name] = digest
            audit_path = study.BASE / 'analysis_attempt' / (case['case_id'] + '.json')
            if audit_path.exists():
                original = audit_path.read_bytes()
                json.loads(original)
                target = OUT / 'audit_inputs' / audit_path.name
                target.write_bytes(original)
                inputs[case['case_id']]['optional_completed_audit_sha256'] = sha(target)
        dump(OUT / 'input_hashes.json', inputs)
        for case in plan['cases']:
            completed.append(scan(case))
        pairs = []
        for condition in study.CONDITIONS:
            matched = {c['arm']: c for c in plan['cases'] if c['condition_id'] == condition}
            pairs.append(compare_pair(matched['map_bridge_control'], matched['post_support_candidate']))
        study.check()
        assert all(sha(ROOT / name) == value for name, value in identities.items())
        analysis = dict(**identity, episodes=completed, pairs=pairs,
            scope='All recorded frames verified against original receipts/NPZ; same-state attribution limited to initial command divergence; no new physics or candidate adoption.')
        dump(OUT / 'analysis.json', analysis)
        dump(OUT / 'outcome.json', dict(**identity, status='VERIFIED_ACTUAL_TRAJECTORY_ONSETS',
            analysis_sha256=sha(OUT / 'analysis.json'), episodes=len(completed),
            recorded_frames=sum(r['frames'] for r in completed)))
        print(json.dumps(dict(status='VERIFIED_ACTUAL_TRAJECTORY_ONSETS', pairs=pairs)), flush=True)
    except Exception:
        dump(OUT / 'failure.json', dict(**identity, completed_cases=[r['case_id'] for r in completed],
                                      traceback=traceback.format_exc()))
        raise


if __name__ == '__main__':
    main()
