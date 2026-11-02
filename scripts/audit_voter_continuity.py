"""Read-only adjacent-roster accounting on the selected, already consumed logs."""
import json
import numpy as np
import voter_continuity_study as study
from dbact_sim.experiments.deployment import dump
from dbact_sim.experiments.study import cached_result


def main():
    output=study.AUDIT/'prior_log_audit.json'
    if output.exists(): raise RuntimeError('Preserve existing audit')
    plan=study.read(study.OLD_PROTOCOL)
    entries=study.read(study.ROOT/'docs/results/transport_validation_v1/stop_entry_audit.json')['cases']
    result=[]
    for condition in study.CONDITIONS:
        case=next(c for c in plan['cases'] if c['condition_id']==condition and c['variant']=='boundary_cvt' and c['arm']=='candidate')
        folder=study.ROOT/plan['run_dir']/case['case_id']
        receipt=study.read(folder/'case_result.json')
        r=cached_result(folder,receipt['identity'])
        steps=study.read(folder/'paired_metrics.json')['steps']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            truth=z['cargo/cargo_0/centers'].copy(); start=z['task/cargo_0/start_position'].copy() if 'task/cargo_0/start_position' in z else truth[0].copy()
        # Each segment has a known initial bias and exactly accounted changes.
        previous=None; segments=[]; segment=None; max_residual=0.
        end=r['phase_response'].get('hold_frame')
        for k,s in enumerate(steps[:None if end is None else end+1]):
            nav=s['navigation_supervisor']
            if nav is None or s['phase'] in ('SEARCH','DISCOVER','ENCLOSE'):
                previous=None; continue
            readings={d['agent_id']:np.array(d['reference_displacement'], dtype=float) for d in s['diagnostics']}
            ids=nav['voters']; raw=np.array(nav['displacement'])
            if not ids or not all(np.isfinite(readings[i]).all() for i in ids) or not np.allclose(np.median([readings[i] for i in ids],axis=0),raw,rtol=0,atol=1e-12):
                previous=None; continue  # Map-to-reference transition is a distinct source.
            common=[] if previous is None else sorted(set(ids)&set(previous[1]))
            if not common:
                segment=dict(start_frame=k,end_frame=k,initial_error_m=(raw-(truth[k]-start)).tolist(),
                    roster_sum_m=[0.,0.],common_error_sum_m=[0.,0.],roster_changes=0)
                segments.append(segment)
            else:
                pk,pids,pr,praw=previous
                ca=np.median([pr[i] for i in common],axis=0)
                cb=np.median([readings[i] for i in common],axis=0)
                roster=ca-praw+raw-cb
                common_error=cb-ca-(truth[k]-truth[pk])
                segment['roster_sum_m']=(np.array(segment['roster_sum_m'])+roster).tolist()
                segment['common_error_sum_m']=(np.array(segment['common_error_sum_m'])+common_error).tolist()
                segment['roster_changes']+=int(set(ids)!=set(pids))
                reconstructed=sum(np.array(segment[n]) for n in ('initial_error_m','roster_sum_m','common_error_sum_m'))
                max_residual=max(max_residual,float(np.max(np.abs(reconstructed-(raw-(truth[k]-start))))))
            segment.update(end_frame=k,final_error_m=(raw-(truth[k]-start)).tolist())
            previous=k,ids,readings,raw
        entry=next(e for e in entries if e['case_id']==case['case_id'])
        result.append(dict(condition=condition,receipt_sha256=study.sha(folder/'case_result.json'),
            first_hold=entry,segments=segments,max_accounting_residual_m=max_residual))
    dump(output,dict(cases=result,scope='Selected development evidence. Exact signed accounting on uninterrupted common-voter segments, not independent samples or counterfactual simulation.',
        distinction='Roster sum is an aggregation contribution. Initial and common-readout errors include accumulated estimator bias, matching/rotation error and selection effects; they are not established causal mechanisms.',script_sha256=study.sha(__file__)))
    study.event('prior_logs_audited',output=str(output.relative_to(study.ROOT)),sha256=study.sha(output))
    for r in result:
        print(json.dumps(dict(condition=r['condition'],true_error=r['first_hold'].get('hold_true_goal_error_m'),
            segments=len(r['segments']),last=r['segments'][-1] if r['segments'] else None,residual=r['max_accounting_residual_m'])))


if __name__=='__main__': main()
