"""Offline median-membership accounting at the first HOLD in each CVT case.

No fabricated missing reading: common-voter medians use only agents accepted at
both endpoints. The algebra explains a recorded transition, not counterfactual
closed-loop performance or absolute estimation accuracy.
"""
import json
import numpy as np
import validate_transport_prior as study
from dbact_sim.experiments.deployment import dump
from dbact_sim.experiments.study import cached_result


def main():
    study.guard()
    dest = study.ROOT / 'docs/results/transport_validation_v1/stop_entry_audit.json'
    if dest.exists():
        raise RuntimeError('Preserve existing audit')
    plan = study.read(study.ROOT / study.read(study.AUDIT / 'freeze_receipt.json')['new_starts']['path'])
    cases = []
    for c in plan['cases']:
        if c['variant'] != 'boundary_cvt':
            continue
        folder = study.ROOT / plan['run_dir'] / c['case_id']
        receipt = study.read(folder / 'case_result.json')
        r = cached_result(folder, receipt['identity'])
        steps = study.read(folder / 'paired_metrics.json')['steps']
        frame = r['phase_response'].get('hold_frame')
        if frame is None or frame == 0:
            cases.append(dict(case_id=c['case_id'], status='NO_PRE_HOLD_PAIR'))
            continue
        assert steps[frame]['phase'] == 'HOLD' and steps[frame-1]['phase'] != 'HOLD'
        previous, current = steps[frame-1], steps[frame]
        a, b = previous['navigation_supervisor'], current['navigation_supervisor']
        if a is None or b is None:
            cases.append(dict(case_id=c['case_id'], status='MISSING_NAVIGATION_AT_ENDPOINT'))
            continue
        da = {d['agent_id']: d for d in previous['diagnostics']}
        db = {d['agent_id']: d for d in current['diagnostics']}
        common = sorted(set(a['voters']) & set(b['voters']))
        assert common
        ma = np.median([da[i]['reference_displacement'] for i in a['voters']], axis=0)
        mb = np.median([db[i]['reference_displacement'] for i in b['voters']], axis=0)
        ca = np.median([da[i]['reference_displacement'] for i in common], axis=0)
        cb = np.median([db[i]['reference_displacement'] for i in common], axis=0)
        assert np.max(np.abs(ma-a['displacement'])) < 1e-12
        assert np.max(np.abs(mb-b['displacement'])) < 1e-12
        pieces = [ca-ma, cb-ca, mb-cb]
        assert np.max(np.abs(sum(pieces)-(mb-ma))) < 1e-12
        with np.load(folder / 'replay.npz', allow_pickle=False) as z:
            centers = z['cargo/cargo_0/centers'][frame-1:frame+1]
            goal = z['task/cargo_0/goal_point'].copy()
        cases.append(dict(case_id=c['case_id'], condition=c['condition_id'], arm=c['arm'],
            status='EXACT_ENDPOINT_ACCOUNTING', hold_control_frame=frame, seconds=frame*c['config']['dt'],
            previous_estimated_goal_error_m=float(np.linalg.norm(a['error'])),
            hold_estimated_goal_error_m=float(np.linalg.norm(b['error'])),
            hold_true_goal_error_m=float(np.linalg.norm(goal-centers[-1])),
            controller_arrival_band_m=c['config']['controller']['cross_track_deadband'],
            previous_guards=previous['phase_guards'], hold_guards=current['phase_guards'],
            physical_translation_m=(centers[-1]-centers[0]).tolist(),
            team_displacement_change_m=(mb-ma).tolist(),
            remove_noncommon_previous_voters_m=pieces[0].tolist(),
            common_readout_change_m=pieces[1].tolist(), add_noncommon_current_voters_m=pieces[2].tolist(),
            removed=sorted(set(a['voters'])-set(b['voters'])),
            added=sorted(set(b['voters'])-set(a['voters'])), common=common,
            endpoint_error_m=r['euclidean_endpoint_error'], task_completed=r['task_completed'],
            input_receipt_sha256=study.sha(folder / 'case_result.json')))
    assert len(cases) == 24
    doc = dict(cases=cases, scope='Post-hoc explanation of all 24 CVT first-HOLD transitions, not another experiment or independent statistical samples.',
        formula='M(B,t1)-M(A,t0)=[M(C,t0)-M(A,t0)]+[M(C,t1)-M(C,t0)]+[M(B,t1)-M(C,t1)], C=A intersect B',
        timing='Controller frame k is row index k with post-physics receipt.frame=k+1; use replay centers[k] for the control input.',
        limitation='The common-voter population is selected using two recorded endpoints offline. This is accounting, not proof of bias-free readout or a deployable replacement; earlier accumulated errors remain.',
        script_sha256=study.sha(__file__))
    dump(dest, doc)
    study.event('audit_first_hold', cases=24, output=dest.relative_to(study.ROOT).as_posix(), sha256=study.sha(dest))
    selected = next(r for r in cases if r['case_id'] == 'far_skew_box_s0_r0_pos0_boundary_cvt_candidate')
    print(json.dumps(selected), flush=True)


if __name__ == '__main__':
    main()
