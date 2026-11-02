"""Post-hoc localization of recorded barrier scaling; no new physics."""
from __future__ import annotations

from collections import Counter
import gc
import json

import numpy as np

import task_translation_physical_study_v2 as study
from research_io import ROOT, guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, verify_source_snapshot


def main():
    guard()
    plan = study.check_freeze()
    analysis = read(study.PUBLIC/'analysis.json')
    assert analysis['protocol_sha256'] == sha(study.PROTOCOL)
    manifest = read(study.OUT/'manifest.json')
    target = study.BASE/'barrier_diagnostic_v1'
    public = study.PUBLIC/'barrier_diagnostic.json'
    if target.exists() or public.exists():
        raise RuntimeError('Preserve existing diagnostic')
    old = read(study.OLD)
    assert sha(study.OLD) == plan['original_protocol_sha256']
    verify_source_snapshot(ROOT/old['run_dir'], plan['original_source_files'])
    reports, traces = [], {}
    for case in plan['cases']:
        folder = study.OUT/case['case_id']
        result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
        assert result['execution_status'] == 'COMPLETED'
        rows = read(folder/'paired_metrics.json')['steps']
        by_phase, by_state, by_agent = Counter(), Counter(), Counter()
        previous, events = 0, []
        for row in rows:
            total = row['solver_statistics']['barrier_scalings']
            delta, previous = total-previous, total
            agents = [d['agent_id'] for d in row['diagnostics'] if d['solver_status']=='scaled_barrier']
            assert delta == len(agents), (case['case_id'], row['frame'])
            if delta:
                phase, state = row['phase'], row['initial_reference']['status']
                by_phase[phase] += delta
                by_state[state] += delta
                by_agent.update(agents)
                events.append(dict(frame=row['frame'], phase=phase, capture_state=state, agents=agents))
        expected = next(r for r in analysis['episodes'] if r['case_id']==case['case_id'])
        assert previous == expected['barrier_scalings'] == sum(by_agent.values())
        report = dict(case_id=case['case_id'], condition=case['condition_id'], arm=case['arm'],
            scaled_robot_steps=previous, control_frames_with_scaling=len(events), by_phase=dict(by_phase),
            by_capture_state=dict(by_state), by_agent=dict(by_agent),
            min_scale=rows[-1]['solver_statistics']['min_barrier_scale'],
            first_event=events[0] if events else None, last_event=events[-1] if events else None,
            receipt_sha256=sha(folder/'case_result.json'))
        # Verify the previously executed SE(2) initial-reference acquisition
        # prefix, separately from the protected-common reference control.
        if case['arm']=='translation_all_captured' and case['condition_id'].startswith('far_'):
            old_case = next(c for c in old['cases'] if c['condition_id']==case['condition_id'] and c['arm']=='task_reference')
            old_folder = ROOT/old['run_dir']/old_case['case_id']
            old_result = cached_result(old_folder, {**plan['original_identity'], 'case_sha256':encode(old_case)})
            epoch = result['initial_reference']['epoch']
            assert epoch == old_result['initial_reference']['epoch']
            before_frames = round(epoch/case['config']['dt'])-1
            old_rows = read(old_folder/'paired_metrics.json')['steps']
            fields = ('times','phase','min_distance','map_coverage','push_flags','contact_flags','efforts',
                'agent_positions','sensed_counts','cargo/cargo_0/centers','cargo/cargo_0/angles',
                'cargo/cargo_0/strict_coverage','cargo/cargo_0/contacts','cargo/cargo_0/penetration',
                'cargo/cargo_0/speed','cargo/cargo_0/progress_estimate')
            with np.load(old_folder/'replay.npz',allow_pickle=False) as a, np.load(folder/'replay.npz',allow_pickle=False) as b:
                checks = {k:bool(np.array_equal(a[k][:before_frames+1],b[k][:before_frames+1])) for k in fields}
                count_a = int(a['sensed_counts'][:before_frames+1].sum())
                count_b = int(b['sensed_counts'][:before_frames+1].sum())
                checks['packed_sensed_points'] = bool(count_a==count_b and np.array_equal(a['sensed_points'][:count_a], b['sensed_points'][:count_b]))
            report['old_se2_acquisition_prefix'] = dict(old_case_id=old_case['case_id'],
                old_receipt_sha256=sha(old_folder/'case_result.json'), epoch_seconds=epoch,
                control_frames_before_freeze=before_frames, replay_fields_exact=checks,
                paired_steps_exact=old_rows[:before_frames]==rows[:before_frames],
                old_scaled_robot_steps=old_rows[before_frames-1]['solver_statistics']['barrier_scalings'],
                new_scaled_robot_steps=rows[before_frames-1]['solver_statistics']['barrier_scalings'],
                scope='Strict pre-freeze prefix of an earlier SE(2) initial-reference episode, not another new control or evidence for whole-episode equality')
            del old_rows
        reports.append(report)
        traces[case['case_id']] = events
        del rows
        gc.collect()
    target.mkdir(parents=True, exist_ok=False)
    dump(target/'events.json', traces)
    result = dict(scope='Post-hoc barrier localization and pre-freeze compatibility audit; no new physical episodes or safety-certificate claim',
        cases=reports, new_physical_episodes=0, original_advance_gate_unchanged=analysis['advance_gate'],
        protocol_sha256=sha(study.PROTOCOL), analysis_sha256=sha(study.PUBLIC/'analysis.json'),
        script_sha256=sha(__file__), event_trace=(target/'events.json').relative_to(ROOT).as_posix(),
        event_trace_sha256=sha(target/'events.json'))
    dump(target/'summary.json', result)
    dump(public, result)
    print(json.dumps([dict(case=r['case_id'], scaled_robot_steps=r['scaled_robot_steps'],
        by_capture_state=r['by_capture_state'], prefix=r.get('old_se2_acquisition_prefix')) for r in reports], indent=2), flush=True)


if __name__ == '__main__':
    main()
