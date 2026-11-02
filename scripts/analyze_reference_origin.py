"""Analyze all fixed reference-origin arms, including failures and missing data."""
import copy
import gc
import json
import numpy as np
import reference_origin_study as study
from audit_reference_origin import components
from analyze_cargo_discovery import one_case
from analyze_transport_validation import aggregate, paired
from analyze_dynamic_contact_study import replay_equal, old_fields_equal
from analyze_feedback_components import csv_rows
from dbact.provenance import config_hash
from dbact_sim.experiments.study import source_hashes, verify_source_snapshot, cached_result
from dbact_sim.experiments.deployment import dump, encode


def compatibility(original,folder):
    ok,keys=replay_equal(original/'replay.npz',folder/'replay.npz')
    old_config=study.read(original/'config.json');new_config=study.read(folder/'config.json')
    expected=copy.deepcopy(old_config)
    expected['controller'].update(reference_early_anchor=False,reference_bootstrap_gain=None)
    assert expected==new_config
    checks={};identities={}
    for name in ('paired_metrics.json','physics_substeps.json','summary.json'):
        a,b=study.read(original/name),study.read(folder/name)
        if name=='paired_metrics.json':
            h0,h1=a['summary'].pop('config_sha256'),b['summary'].pop('config_sha256')
            assert (h0,h1)==(encode(old_config),encode(new_config));identities['config_sha256']=[h0,h1]
        elif name=='summary.json':
            h0,h1=a['provenance'].pop('config_hash'),b['provenance'].pop('config_hash')
            assert (h0,h1)==(config_hash(old_config),config_hash(new_config));identities['config_hash']=[h0,h1]
        checks[name]=old_fields_equal(a,b)
    assert ok and all(checks.values())
    return dict(replay_exact=ok,arrays=keys,scientific_fields_exact=checks,verified_identity_changes=identities)


def endpoint_audit(case,folder):
    rows=study.read(folder/'paired_metrics.json')['steps']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        xy=z['cargo/cargo_0/centers'].copy();start=z['task/cargo_0/start'].copy();goal=z['task/cargo_0/goal_point'].copy()
    errors=np.linalg.norm(xy-goal,axis=1);best=int(errors.argmin())
    first=None;series=[]
    for k,r in enumerate(rows):
        nav=r['navigation_supervisor']
        est=np.full(2,np.nan) if nav is None else np.array(nav['displacement'])
        est_goal=float(np.linalg.norm(goal-start-est))
        if first is None and r['phase']=='HOLD':
            first=dict(frame=k,time_s=k*case['config']['dt'],true_goal_error_m=float(errors[k]),
                estimated_goal_error_m=est_goal,false_hold=bool(errors[k]>case['config']['task']['tolerance']))
        series.append(dict(frame=k,time_s=k*case['config']['dt'],phase=r['phase'],true_goal_error_m=float(errors[k]),
            estimated_goal_error_m=est_goal,true_x=xy[k,0]-start[0],true_y=xy[k,1]-start[1],
            estimate_x=est[0],estimate_y=est[1],contacts=r['contacts']))
    return dict(first_hold=first,closest_approach=dict(frame=best,time_s=best*case['config']['dt'],
        error_m=float(errors[best]),scope='Closest physical approach is not the unchanged completion/settling verdict.')),series


def main():
    if (study.PUBLIC/'analysis.json').exists():raise RuntimeError('Preserve final analysis')
    freeze=study.read(study.AUDIT/'freeze_receipt.json');plan=study.read(study.PROTOCOL)
    assert study.sha(study.PROTOCOL)==freeze['protocol_sha256']
    batch=study.ROOT/plan['run_dir'];summary=study.read(batch/'summary.json');manifest=study.read(batch/'manifest.json')
    assert summary.get('source_integrity')=='PASS' and summary['completed_denominator']==24
    assert source_hashes()==freeze['source_files']==manifest['source_files']
    verify_source_snapshot(batch,manifest['source_files'])
    study.PUBLIC.mkdir(parents=True,exist_ok=True)
    cache=study.AUDIT/'case_analysis';cache.mkdir(exist_ok=True)
    series_dir=study.OUT/'timeseries';series_dir.mkdir(exist_ok=True)
    rows=[];origins={};stops={};compat={};inputs={}
    version=encode({str(p):study.sha(p) for p in [study.ROOT/'scripts/analyze_reference_origin.py',
        study.ROOT/'scripts/audit_reference_origin.py',study.ROOT/'scripts/analyze_cargo_discovery.py']})
    for case in plan['cases']:
        folder=batch/case['case_id'];receipt=study.read(folder/'case_result.json')
        cached_result(folder,receipt['identity']);digest=study.sha(folder/'case_result.json')
        saved=cache/(case['case_id']+'.json')
        if saved.exists():
            cached=study.read(saved);assert cached['version']==version and cached['input_sha256']==digest
            row,origin,stop=cached['row'],cached['origin'],cached['stop']
        else:
            row,_,_=one_case(case,folder,manifest)
            if row['execution_status']=='COMPLETED':
                origin=components(case,folder)
                origin_series=origin.pop('series')
                dump(series_dir/(case['case_id']+'-components.json'),origin_series)
                stop,timeseries=endpoint_audit(case,folder)
                csv_rows(series_dir/(case['case_id']+'.csv'),timeseries)
                row.update(false_hold=bool(stop['first_hold'] and stop['first_hold']['false_hold']),
                    bootstrap_mean_norm_m=origin['bootstrap_mean_norm_m'],
                    first_bootstrap_time_s=origin['min_bootstrap_time_s'],
                    closest_goal_error_m=stop['closest_approach']['error_m'])
                del origin_series,timeseries
            else:origin=stop=dict(status='EXECUTION_ERROR')
            dump(saved,dict(version=version,input_sha256=digest,row=row,origin=origin,stop=stop))
        rows.append(row);origins[case['case_id']]=origin;stops[case['case_id']]=stop
        inputs[str((folder/'case_result.json').relative_to(study.ROOT))]=digest
        if case['arm']=='reference':
            original=study.ROOT/freeze['origins'][case['condition_id']]['path']
            compat[case['condition_id']]=compatibility(original,folder)
        print(case['case_id']+': '+row.get('outcome','EXECUTION_ERROR'),flush=True);gc.collect()
    prefix={}
    for condition in study.CONDITIONS:
        ref=batch/(condition+'_reference');result=study.read(ref/'case_result.json')['result']
        stop=result['contact_ready_frame']+1
        for arm in ('early_reference','unscaled_bootstrap'):
            other=batch/(condition+'_'+arm)
            assert study.read(other/'case_result.json')['result']['contact_ready_frame']==result['contact_ready_frame']
            with np.load(ref/'replay.npz',allow_pickle=False) as x,np.load(other/'replay.npz',allow_pickle=False) as y:
                exact=all(np.array_equal(x[k][:stop+1],y[k][:stop+1],equal_nan=True) for k in
                    ('times','agent_positions','cargo/cargo_0/centers','cargo/cargo_0/angles'))
            a=study.read(ref/'paired_metrics.json')['steps'][:stop]
            b=study.read(other/'paired_metrics.json')['steps'][:stop]
            control=all(old_fields_equal({k:r[k] for k in ('commands','phase','phase_guards','navigation_supervisor')},
                                        {k:s[k] for k in ('commands','phase','phase_guards','navigation_supervisor')}) for r,s in zip(a,b))
            assert exact and control
            prefix[condition+'_'+arm]=dict(physical_arrays_exact=exact,control_records_exact=control,frames=stop,
                scope='Through the CONTACT_READY control interval; observer diagnostics intentionally differ.')
            del a,b;gc.collect()
    aggregates={a:aggregate([r for r in rows if r['arm']==a]) for a in study.ARMS}
    comparisons={a:paired(rows,'reference',a) for a in study.ARMS if a!='reference'}
    gates={}
    for arm,group in aggregates.items():
        group['false_hold_cases']=sum(r.get('false_hold',False) for r in rows if r['arm']==arm)
        group['cutoffs']=sum(r.get('outcome','').startswith('CENSORED') for r in rows if r['arm']==arm)
        if arm!='reference':
            pair=comparisons[arm]
            rescued=next(r for r in rows if r['arm']==arm and r['condition']=='far_skew_box_s0_r0_pos0')['safe_completed']
            gates[arm]='PASS' if (rescued and not pair['right_losses'] and not pair['new_hard'] and
                                 group['g500']>=aggregates['reference']['g500']) else 'FAIL'
    result=dict(mission=study.MISSION,episodes=rows,aggregates=aggregates,comparisons=comparisons,adoption_gates=gates,
        original_reproduction=compat,pre_reference_control_equivalence=prefix,origin_audits=origins,stop_audits=stops,
        source_sha256=encode(manifest['source_files']),environment=manifest['identity']['environment'],
        input_receipts=inputs,analysis_version=version,
        limitations='Eight reused development condition blocks. No population reliability or unseen-shape claim. Truth only in evaluation. Reference-point rotation and matching error remain distinct; per-frame samples are not independent episodes.')
    assert len(compat)==8
    dump(study.PUBLIC/'analysis.json',result);csv_rows(study.PUBLIC/'episodes.csv',rows)
    study.event('analyze',sha256=study.sha(study.PUBLIC/'analysis.json'),gates=gates)
    print(json.dumps(dict(aggregates=aggregates,gates=gates)),flush=True)


if __name__=='__main__':main()
