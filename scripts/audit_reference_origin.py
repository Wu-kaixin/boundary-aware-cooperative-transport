"""Separate bootstrap, material-point rotation and registration on saved paths.

Simulation truth is used ONLY for offline evaluation. Historical bootstrap scans
are reconstructed and checked by count/pivot diagnostics, not claimed original
scan recordings. New receipts can carry the actual controller anchor metadata.
"""
import argparse
import gc
import numpy as np
import reference_origin_study as study
from dbact.control.controller import _merge_scans, DBACTParams
from dbact.control.observed_reference import ObservedReferenceTracker, rotation
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.study import cached_result
from dbact_sim.experiments.deployment import dump


def components(case,folder):
    receipt=study.read(folder/'case_result.json')
    r=cached_result(folder,receipt['identity'])
    content=study.read(folder/'paired_metrics.json');steps=content['steps']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        times,robots,xy,angles,start,goal=[z[k].copy() for k in
            ('times','agent_positions','cargo/cargo_0/centers','cargo/cargo_0/angles','task/cargo_0/start','task/cargo_0/goal_point')]
    cfg=case['config'];gain=cfg['controller'].get('reference_bootstrap_gain')
    if gain is None:gain=cfg['controller'].get('progress_estimate_gain',DBACTParams().progress_estimate_gain)
    saved=content['summary'].get('reference_initializations')
    boot={};checks=[]
    if saved is not None:
        for aid,b in saved.items():
            if not b['initialized']:continue
            k=int(round(b['time']/cfg['dt']))
            assert abs(times[k]-b['time'])<1e-10
            boot[aid]=dict(frame=k,time_s=b['time'],offset=np.array(b['offset']),
                center=xy[k],angle=angles[k],lever=np.array(b['anchor'])+np.array(b['pivot_shift'])-xy[k],
                pivot_status=b['pivot_status'],origin='RECORDED_CONTROLLER_METADATA')
    else:
        env=SimulationEnvironment(cfg,seed=case['seed']);p=env.controller.params
        assert p.perception_every==1 and p.communication_dropout_prob==0
        for k,s in enumerate(steps):
            new=[i for i,d in enumerate(s['diagnostics']) if d['reference_status']=='bootstrap_only' and d['agent_id'] not in boot]
            if not new:continue
            env.cargoes[0].set_pose(xy[k],angles[k])
            for i,agent in enumerate(env.agents):agent.position=robots[k,i].copy()
            scans=[env.controller.sensor.sense_view(a,env.cargoes,times[k]) for a in env.agents]
            neighbors=env.controller._neighbor_indices(env.agents)
            for i in new:
                d=s['diagnostics'][i]
                for j in [i]+neighbors[i]:assert len(scans[j])==s['diagnostics'][j]['own_scan_points']
                batch,_=_merge_scans(scans,[i]+neighbors[i])
                tracker=ObservedReferenceTracker(gate=p.registration_gate,normal_cosine=p.registration_normal_cosine,
                    max_speed=p.max_object_speed,max_yaw_rate=p.max_object_yaw_rate,
                    velocity_filter=p.object_velocity_filter,radial_readout=p.reference_radial_pivot,
                    range_noise_std=p.range_noise_std,innovation_trim=p.reference_innovation_trim)
                tracker.initialize(batch,d['reference_bootstrap_offset'],times[k])
                assert tracker.initialized and tracker.pivot_status==d['reference_pivot_status']
                if d['reference_pivot_rms'] is not None:assert abs(tracker.pivot_rms-d['reference_pivot_rms'])<1e-12
                aid=d['agent_id']
                boot[aid]=dict(frame=k,time_s=float(times[k]),offset=tracker.offset,center=xy[k],angle=angles[k],
                    lever=tracker.anchor+tracker.pivot_shift-xy[k],pivot_status=tracker.pivot_status,
                    origin='RECONSTRUCTED_BOOTSTRAP_COUNTS_AND_PIVOT_CHECKED')
                checks.append(dict(agent=aid,frame=k,own_and_neighbor_scan_counts=True,pivot_status=True,
                                   pivot_rms_checked=d['reference_pivot_rms'] is not None))
        del env
    series=[];max_identity=0.;max_median=0.
    ready=r.get('contact_ready_frame')
    for k,s in enumerate(steps):
        nav=s['navigation_supervisor']
        if ready is None or k<=ready or nav is None:continue
        by_id={d['agent_id']:d for d in s['diagnostics']}
        initial=[];rot=[];observed=[];no_gain=[]
        for aid in nav['voters']:
            d=by_id[aid];assert d['reference_status']=='accepted'
            b=boot[aid];truth=xy[k]-start
            offset_error=b['offset']-(b['center']-start)
            point_rotation=(rotation(angles[k]-b['angle'])-np.eye(2))@b['lever']
            value=np.array(d['reference_displacement'])
            initial.append(offset_error);rot.append(point_rotation);observed.append(value)
            no_gain.append(value-b['offset']*(1-1/gain))
        if not observed:continue
        actual=np.median(observed,axis=0)
        max_median=max(max_median,float(np.max(np.abs(actual-np.array(nav['displacement'])))))
        # Nested medians, not an invalid sum of medians of the separate components.
        initial_part=np.median(initial,axis=0)
        geometry=np.median(np.array(initial)+np.array(rot),axis=0)
        rotation_part=geometry-initial_part
        total=actual-truth;registration_part=total-geometry
        max_identity=max(max_identity,float(np.max(np.abs(initial_part+rotation_part+registration_part-total))))
        nongain=np.median(no_gain,axis=0)-truth
        series.append(dict(frame=k,time_s=float(times[k]),phase=s['phase'],voters=len(observed),
            initial_xy_m=initial_part.tolist(),rotation_xy_m=rotation_part.tolist(),
            registration_xy_m=registration_part.tolist(),total_xy_m=total.tolist(),
            raw_bootstrap_gain_one_error_xy_m=nongain.tolist(),true_goal_error_m=float(np.linalg.norm(goal-xy[k]))))
    assert max_identity<1e-12 and max_median<1e-12
    stats={}
    for label in ('initial','rotation','registration','total'):
        x=np.array([r[label+'_xy_m'] for r in series])
        stats[label]=dict(mean_signed_xy_m=x.mean(axis=0).tolist(),mean_norm_m=float(np.linalg.norm(x,axis=1).mean())) if series else None
    hold=next((x for x in series if x['phase']=='HOLD'),None)
    metadata={aid:{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in b.items()} for aid,b in boot.items()}
    boot_errors=np.array([b['offset']-(b['center']-start) for b in boot.values()])
    raw_errors=np.array([b['offset']/gain-(b['center']-start) for b in boot.values()])
    result=dict(case=case['case_id'],condition=case['condition_id'],arm=case['arm'],bootstrap=metadata,gain=gain,
        bootstrap_checks=checks,bootstrap_mean_norm_m=float(np.linalg.norm(boot_errors,axis=1).mean()) if boot else None,
        unscaled_bootstrap_mean_norm_m=float(np.linalg.norm(raw_errors,axis=1).mean()) if boot else None,
        min_bootstrap_time_s=min((b['time_s'] for b in boot.values()),default=None),max_bootstrap_time_s=max((b['time_s'] for b in boot.values()),default=None),
        hold=hold,final=series[-1] if series else None,statistics=stats,samples=len(series),
        no_gain_same_input_MAE_m=float(np.mean([np.linalg.norm(r['raw_bootstrap_gain_one_error_xy_m']) for r in series])) if series else None,
        maximum_identity_residual_m=max_identity,maximum_median_residual_m=max_median,series=series,
        receipt_sha256=study.sha(folder/'case_result.json'),scope='Conditional recorded voters and trajectory; counterfactual signals are not new closed-loop experiments. Norms of components are not additive. Material reference point is not known COM.')
    del content,steps;gc.collect()
    return result


def main():
    dest=study.AUDIT/'prior_origin_audit.json'
    if dest.exists():raise RuntimeError('Preserve original audit')
    plan=study.read(study.OLD_PROTOCOL);rows=[]
    for condition in study.CONDITIONS:
        case=next(c for c in plan['cases'] if c['condition_id']==condition and c['arm']=='reference')
        r=components(case,study.ROOT/plan['run_dir']/case['case_id']);rows.append(r)
        print(study.json.dumps(dict(condition=condition,bootstrap=r['bootstrap_mean_norm_m'],
            raw_bootstrap=r['unscaled_bootstrap_mean_norm_m'],stats=r['statistics'],hold=r['hold'])),flush=True)
    dump(dest,dict(cases=rows,script_sha256=study.sha(__file__),scope='Offline historical analysis only; sensor reconstruction is not a new episode or new raw recording.'))
    study.event('prior_origin_audit',sha256=study.sha(dest))


if __name__=='__main__':main()
