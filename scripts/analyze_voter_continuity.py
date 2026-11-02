"""All scheduled episodes, same-input readout error, and first-HOLD audit."""
import collections
import copy
import gc
import json
import numpy as np

import voter_continuity_study as study
from analyze_cargo_discovery import one_case
from analyze_dynamic_contact_study import replay_equal, old_fields_equal
from analyze_feedback_components import csv_rows
from analyze_transport_validation import aggregate, paired
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, source_hashes, verify_source_snapshot
from dbact.provenance import config_hash


def navigation_audit(case, folder):
    steps=study.read(folder/'paired_metrics.json')['steps']
    result=study.read(folder/'case_result.json')['result']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        xy=z['cargo/cargo_0/centers'].copy()
        start=z['task/cargo_0/start'].copy()
        goal=z['task/cargo_0/goal_point'].copy()
    series=[]; statuses=collections.Counter(); max_identity=0.; first_hold=None
    previous=None; confirmed=case['config']['controller']['navigation_hold_dwell']
    common_counts=[];local_reanchors=0;max_local_offset=0.
    guard_streak=0; max_guard=0
    for k,s in enumerate(steps):
        nav=s['navigation_supervisor']
        trace=s.get('navigation_voter_trace')
        t=None if trace is None else trace['supervisor']
        if t is not None:
            if t['status']=='continuous': common_counts.append(len(t['common_voters']))
            for local in trace['local'].values():
                local_reanchors+=int(local['status']=='reanchored')
                if local['offset'] is not None:
                    max_local_offset=max(max_local_offset,float(np.linalg.norm(local['offset'])))
        truth=xy[k]-start
        est=np.full(2,np.nan) if nav is None else np.asarray(nav['displacement'])
        raw=est if t is None else np.full(2,np.nan) if t['raw_displacement'] is None else np.asarray(t['raw_displacement'])
        offset=np.zeros(2) if t is None or t['offset'] is None else np.asarray(t['offset'])
        status='disabled' if t is None else t['status']; statuses[status]+=1
        common=np.full(2,np.nan) if t is None or t['common_delta'] is None else np.asarray(t['common_delta'])
        roster=np.full(2,np.nan) if t is None or t['roster_delta'] is None else np.asarray(t['roster_delta'])
        if previous is not None and t is not None and status=='continuous' and previous['stream']==trace['stream']:
            # Both endpoints must have admitted estimates; reanchors are not bridges.
            if np.isfinite(est).all() and np.isfinite(previous['est']).all():
                max_identity=max(max_identity,float(np.max(np.abs(est-previous['est']-common))))
        previous=dict(est=est,stream=None if trace is None else trace['stream'])
        g=s['phase_guards']
        ready=(g['progress']>=g['target_distance'] and abs(g['estimated_speed'])<=.02 and g['estimate_quality']!='unavailable')
        guard_streak=guard_streak+1 if ready else 0; max_guard=max(max_guard,guard_streak)
        if first_hold is None and s['phase']=='HOLD':
            first_hold=dict(control_frame=k,seconds=k*case['config']['dt'],true_goal_error_m=float(np.linalg.norm(goal-xy[k])),
                estimated_goal_error_m=None if nav is None else float(np.linalg.norm(nav['error'])),
                estimate_error_vector_m=(est-truth).tolist(),raw_error_vector_m=(raw-truth).tolist(),
                offset_m=offset.tolist(),voters=[] if nav is None else nav['voters'],
                confirmation_streak=s['hold_confirmation_streak'],guards=g)
            assert s['hold_confirmation_streak']>=confirmed
            first_hold['false_hold']=first_hold['true_goal_error_m']>case['config']['task']['tolerance']
        series.append(dict(frame=k,time_s=k*case['config']['dt'],phase=s['phase'],
            true_x=truth[0],true_y=truth[1],estimate_x=est[0],estimate_y=est[1],
            raw_x=raw[0],raw_y=raw[1],offset_x=offset[0],offset_y=offset[1],
            signed_error_x=est[0]-truth[0],signed_error_y=est[1]-truth[1],
            raw_error_norm=float(np.linalg.norm(raw-truth)),estimate_error_norm=float(np.linalg.norm(est-truth)),
            true_goal_error_m=float(np.linalg.norm(goal-xy[k])),
            estimated_goal_error_m=float('nan') if nav is None else float(np.linalg.norm(nav['error'])),
            common_dx=common[0],common_dy=common[1],roster_dx=roster[0],roster_dy=roster[1],
            voter_count=0 if nav is None else len(nav['voters']),status=status,contacts=s['contacts'],
            confirmation_streak=s['hold_confirmation_streak']))
    # Estimator accuracy on identical admitted inputs within each arm's trajectory.
    selected=[r for r in series if np.isfinite(r['estimate_error_norm']) and np.isfinite(r['raw_error_norm'])]
    audit=dict(first_hold=first_hold,status_counts=dict(statuses),max_bridge_identity_residual_m=max_identity,
        same_input_samples=len(selected),
        same_input_raw_MAE_m=float(np.mean([r['raw_error_norm'] for r in selected])) if selected else None,
        same_input_used_MAE_m=float(np.mean([r['estimate_error_norm'] for r in selected])) if selected else None,
        max_guard_streak=max_guard,minimum_common_voters=min(common_counts) if common_counts else None,
        one_common_voter_frames=common_counts.count(1),local_reanchors=local_reanchors,
        max_local_offset_m=max_local_offset,
        max_offset_m=max(float(np.hypot(r['offset_x'],r['offset_y'])) for r in series))
    assert max_identity<1e-12
    return audit,series


def main():
    study.guard()
    if (study.PUBLIC/'analysis.json').exists(): raise RuntimeError('Preserve completed analysis')
    freeze=study.read(study.AUDIT/'freeze_receipt.json')
    assert study.sha(study.PROTOCOL)==freeze['protocol_sha256']
    plan=study.read(study.PROTOCOL);batch=study.ROOT/plan['run_dir']
    manifest=study.read(batch/'manifest.json');summary=study.read(batch/'summary.json')
    if summary.get('source_integrity') != 'PASS':
        raise RuntimeError(f"Batch not finalized: {summary.get('completed_denominator')}/32; preserve and wait for its existing process")
    assert summary['completed_denominator']==len(plan['cases'])==32
    verify_source_snapshot(batch,manifest['source_files'])
    assert source_hashes()==manifest['source_files']==freeze['source_files']
    study.PUBLIC.mkdir(parents=True,exist_ok=True)
    cache=study.AUDIT/'analysis_cache_v2';cache.mkdir(exist_ok=True)
    ts=study.OUT/'timeseries';ts.mkdir(exist_ok=True)
    rows=[];audits={};compat={};inputs={}
    for case in plan['cases']:
        folder=batch/case['case_id'];receipt=study.read(folder/'case_result.json')
        digest=study.sha(folder/'case_result.json')
        cached_result(folder,receipt['identity'])
        record=cache/(case['case_id']+'.json')
        if record.exists():
            r=study.read(record);assert r['input_sha256']==digest and r['analysis_sha256']==study.sha(__file__)
            row,audit=r['row'],r['audit']
        else:
            row,_,_=one_case(case,folder,manifest)
            if row['execution_status']=='COMPLETED':
                audit,series=navigation_audit(case,folder)
                csv_rows(ts/(case['case_id']+'.csv'),series)
                del series
                row.update(false_hold=bool(audit['first_hold'] and audit['first_hold']['false_hold']),
                    hold_true_goal_error_m=None if audit['first_hold'] is None else audit['first_hold']['true_goal_error_m'],
                    reanchors=audit['status_counts'].get('reanchored',0),
                    same_input_raw_MAE_m=audit['same_input_raw_MAE_m'],
                    same_input_used_MAE_m=audit['same_input_used_MAE_m'])
            else:
                audit=dict(status='EXECUTION_ERROR'); row['false_hold']=None
            dump(record,dict(input_sha256=digest,analysis_sha256=study.sha(__file__),row=row,audit=audit))
        rows.append(row);audits[case['case_id']]=audit
        inputs[str((folder/'case_result.json').relative_to(study.ROOT))]=digest
        if case['arm']=='reference' and row['execution_status']=='COMPLETED':
            original=study.ROOT/freeze['origins'][case['condition_id']]['path']
            assert study.sha(original/'case_result.json')==freeze['origins'][case['condition_id']]['receipt_sha256']
            ok,keys=replay_equal(original/'replay.npz',folder/'replay.npz')
            old_config=study.read(original/'config.json')
            new_config=study.read(folder/'config.json')
            expected=copy.deepcopy(old_config)
            expected['controller'].update(navigation_voter_continuity=False,navigation_hold_dwell=1)
            assert new_config==expected
            scientific=True;identity_changes={}
            for filename in ('paired_metrics.json','physics_substeps.json','summary.json'):
                old_data,new_data=study.read(original/filename),study.read(folder/filename)
                if filename=='paired_metrics.json':
                    h0,h1=old_data['summary'].pop('config_sha256'),new_data['summary'].pop('config_sha256')
                    assert (h0,h1)==(encode(old_config),encode(new_config))
                    identity_changes['paired_metrics.summary.config_sha256']=[h0,h1]
                elif filename=='summary.json':
                    h0,h1=old_data['provenance'].pop('config_hash'),new_data['provenance'].pop('config_hash')
                    assert (h0,h1)==(config_hash(old_config),config_hash(new_config))
                    identity_changes['summary.provenance.config_hash']=[h0,h1]
                scientific=scientific and old_fields_equal(old_data,new_data)
            compat[case['condition_id']]=dict(replay_exact=ok,arrays=keys,old_scientific_fields_exact=scientific,
                verified_identity_changes=identity_changes,
                configuration_equivalence='Only new flags explicit False/1. Both config hashes independently recomputed, excluded only from numerical equality. All other historical fields exact.')
        print(case['case_id']+': '+row.get('outcome','EXECUTION_ERROR'),flush=True)
        gc.collect()
    prefix={}
    for condition in study.CONDITIONS:
        original=batch/(condition+'_reference')
        confirmed=batch/(condition+'_confirmation')
        hold=audits[condition+'_reference'].get('first_hold')
        if hold is None:
            prefix[condition]=dict(status='NOT_APPLICABLE_NO_REFERENCE_HOLD')
            continue
        k=hold['control_frame']
        with np.load(original/'replay.npz',allow_pickle=False) as x, np.load(confirmed/'replay.npz',allow_pickle=False) as y:
            arrays={name:bool(np.array_equal(x[name][:k+1],y[name][:k+1],equal_nan=True))
                    for name in ('times','agent_positions','cargo/cargo_0/centers','cargo/cargo_0/angles')}
        before=study.read(original/'paired_metrics.json')['steps'][:k]
        after=study.read(confirmed/'paired_metrics.json')['steps'][:k]
        exact=old_fields_equal(before,after)
        prefix[condition]=dict(control_frames_before_divergence=k,arrays=arrays,scientific_fields_exact=exact,
                               status='PASS' if all(arrays.values()) and exact else 'FAIL')
        assert prefix[condition]['status']=='PASS'
        del before,after
        gc.collect()
    aggregates={a:aggregate([r for r in rows if r['arm']==a]) for a in study.ARMS}
    comparisons={a:paired(rows,'reference',a) for a in study.ARMS if a!='reference'}
    for arm,v in aggregates.items():
        v['false_hold_cases']=sum(r.get('false_hold') is True for r in rows if r['arm']==arm)
        v['reanchors']=sum(r.get('reanchors',0) for r in rows if r['arm']==arm)
    diagnostic=next(r for r in rows if r['condition']=='far_skew_box_s0_r0_pos0' and r['arm']=='combined')
    adoption=(diagnostic['safe_completed'] and not comparisons['combined']['right_losses']
              and not comparisons['combined']['new_hard']
              and aggregates['combined']['g500']>=aggregates['reference']['g500'])
    doc=dict(mission=study.MISSION,episodes=rows,aggregates=aggregates,comparisons=comparisons,
        original_reproduction=compat,stop_only_prefix_equivalence=prefix,diagnostics=audits,adoption_gate='PASS' if adoption else 'FAIL',
        input_receipts=inputs,source_sha256=encode(manifest['source_files']),environment=manifest['identity']['environment'],
        script_sha256=study.sha(__file__),shared_analysis_sha256=study.sha(study.ROOT/'scripts/analyze_cargo_discovery.py'),
        statistical_scope='8 deliberately selected, already consumed paired development conditions; no population success-rate or independent-frame inference.',
        timing='Row k describes control input at replay time k, followed by physical step k+1. Evaluator truth never enters control.',
        limitations='Continuous/common-roster increments can still be biased; reanchors lose continuity. Dwell tests persistence, not absolute accuracy. Same-input errors within an arm do not compare estimators on a common cross-arm trajectory.')
    dump(study.PUBLIC/'analysis.json',doc)
    csv_rows(study.PUBLIC/'episodes.csv',rows)
    dump(study.PUBLIC/'stop_audit.json',audits)
    assert len(compat)==8 and all(c['replay_exact'] and c['old_scientific_fields_exact'] for c in compat.values())
    study.event('analysis',sha256=study.sha(study.PUBLIC/'analysis.json'),gate=doc['adoption_gate'])
    print(json.dumps(dict(aggregates=aggregates,adoption_gate=doc['adoption_gate'])))


if __name__=='__main__': main()
