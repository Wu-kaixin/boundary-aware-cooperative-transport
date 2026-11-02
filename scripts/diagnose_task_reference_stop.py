"""Read-only stop-guard accounting for the frozen initial-reference pilot."""
import numpy as np
from research_io import ROOT, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result


def main():
    plan=read(ROOT/'experiments/ccta2027/task_reference_v1.json');out=ROOT/plan['run_dir']
    manifest=read(out/'manifest.json');records=[]
    for case in plan['cases']:
        if case['arm']!='task_reference':continue
        folder=out/case['case_id'];r=cached_result(folder,{**manifest['identity'],'case_sha256':encode(case)})
        rows=read(folder/'paired_metrics.json')['steps']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            xy=z['cargo/cargo_0/centers'].copy();goal=z['task/cargo_0/goal_point'].copy()
        eligible=[]; insufficient=large_bound=0
        for s in rows:
            nav=s['navigation_supervisor']
            if s['phase']=='BRAKE' and nav and nav['remaining']<=0 and np.linalg.norm(nav['velocity'])<=.02:
                eligible.append(s)
                insufficient+=len(nav['voters'])<case['config']['controller']['min_push_agents']
                large_bound+=s['initial_reference']['center_error_bound']>case['config']['task']['tolerance']
        def describe(s):
            nav=s['navigation_supervisor']
            return dict(frame=s['frame'],phase=s['phase'],true_goal_error_m=float(np.linalg.norm(xy[s['frame']-1]-goal)),
                estimated_nominal_error_m=None if nav is None else float(np.linalg.norm(nav['error'])),
                voters=[] if nav is None else nav['voters'],conditional_workspace_bound_m=s['initial_reference']['center_error_bound'],
                estimated_velocity=None if nav is None else nav['velocity'],true_velocity=s['true_velocity'])
        records.append(dict(condition=case['condition_id'],basic_stop_eligible_frames=len(eligible),
            insufficient_quorum_frames=int(insufficient),workspace_bound_reject_frames=int(large_bound),
            first_basic_stop_eligible=None if not eligible else describe(eligible[0]),final=describe(rows[-1]),
            all_frozen_trackers_initialized=all(t['initialized'] for t in r['reference_initializations'].values()),
            zero_epoch=r['initial_reference'],raw_receipt_sha256=sha(folder/'case_result.json')))
        print(case['condition_id'],records[-1],flush=True)
    path=ROOT/'docs/results/task_reference_v1/stop_diagnosis.json'
    if path.exists():raise RuntimeError('Preserve the stop diagnosis')
    dump(path,dict(scope='Same recorded controls and trajectories; a gate counterfactual is not a new success',
        protocol_sha256=sha(ROOT/'experiments/ccta2027/task_reference_v1.json'),script_sha256=sha(__file__),cases=records))


if __name__=='__main__':main()
