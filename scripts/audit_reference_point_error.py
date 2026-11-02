"""Offline decomposition: local material-point motion versus cargo-centroid motion.

Reconstruct only bootstrap scans, never integrate physics or feed truth to control.
Accepted readouts are taken from frozen logs. Component norms are not additive.
"""
import gc
import json
from pathlib import Path
import numpy as np
from dbact.cpu_budget import limit_blas_threads
from dbact.control.controller import _merge_scans
from dbact.control.observed_reference import ObservedReferenceTracker, rotation
from dbact_sim.environment import SimulationEnvironment
from prepare_transport_tuning import ROOT, MISSION, sha

CASES=['near/circle_boundary_cvt','near/ellipse_parallel_boundary_cvt',
       'near/c_open_away_boundary_cvt','near/u_tray_s0_r90_boundary_cvt',
       'far/skew_box_s0_r0_pos0_unknown_location','far/c_open_away_pos1_initial_location_hint']

def read(path):return json.loads(path.read_text(encoding='utf-8'))

def audit(name):
    folder=ROOT/'runs/research/2026-11-02-cargo-discovery-v1'/name
    case=read(folder/'resolved_case.json');steps=read(folder/'paired_metrics.json')['steps']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        times,robots,xy,angles,start=[z[k].copy() for k in ('times','agent_positions',
            'cargo/cargo_0/centers','cargo/cargo_0/angles','task/cargo_0/start')]
    env=SimulationEnvironment(case['config'],seed=case['seed']);p=env.controller.params
    assert p.perception_every==1 and p.communication_dropout_prob==0
    boot={};samples=[];team=[];max_identity=0.;max_aggregation=0.;counts=0
    for k,s in enumerate(steps):
        new=[i for i,d in enumerate(s['diagnostics']) if d['reference_status']=='bootstrap_only' and i not in boot]
        if new:
            env.cargoes[0].set_pose(xy[k],angles[k])
            for i,a in enumerate(env.agents):a.position=robots[k,i].copy()
            scans=[env.controller.sensor.sense_view(a,env.cargoes,times[k]) for a in env.agents]
            neighbors=env.controller._neighbor_indices(env.agents)
            for i in new:
                d=s['diagnostics'][i]
                for j in [i]+neighbors[i]:assert len(scans[j])==s['diagnostics'][j]['own_scan_points']
                batch,_=_merge_scans(scans,[i]+neighbors[i])
                tracker=ObservedReferenceTracker(gate=p.registration_gate,
                    normal_cosine=p.registration_normal_cosine,max_speed=p.max_object_speed,
                    max_yaw_rate=p.max_object_yaw_rate,velocity_filter=p.object_velocity_filter,
                    radial_readout=p.reference_radial_pivot,range_noise_std=p.range_noise_std,
                    innovation_trim=p.reference_innovation_trim)
                tracker.initialize(batch,d['reference_bootstrap_offset'],times[k])
                assert tracker.initialized and tracker.pivot_status==d['reference_pivot_status']
                boot[i]=dict(frame=k,time_s=float(times[k]),offset=tracker.offset,
                    center=xy[k],angle=angles[k],lever=tracker.anchor+tracker.pivot_shift-xy[k],
                    pivot_status=tracker.pivot_status)
        actual=[];ideal=[];voter_ids=[]
        for i,d in enumerate(s['diagnostics']):
            if d['reference_status']!='accepted':continue
            b=boot[i];true=xy[k]-start
            bootstrap=b['offset']-(b['center']-start)
            lever=(rotation(angles[k]-b['angle'])-np.eye(2))@b['lever']
            perfect=true+bootstrap+lever
            observed=np.asarray(d['reference_displacement'])
            registration=observed-perfect
            identity=observed-true-bootstrap-lever-registration
            max_identity=max(max_identity,float(np.max(np.abs(identity))))
            actual.append(observed);ideal.append(perfect);voter_ids.append(d['agent_id'])
            samples.append(np.r_[bootstrap,lever,registration,observed-true]);counts+=1
        nav=s['navigation_supervisor']
        if actual and nav is not None:
            assert set(voter_ids)==set(nav['voters'])
            a=np.median(actual,axis=0);i=np.median(ideal,axis=0);true=xy[k]-start
            max_aggregation=max(max_aggregation,float(np.max(np.abs(a-np.asarray(nav['displacement'])))))
            team.append(dict(time_s=float(times[k]),actual_error_m=float(np.linalg.norm(a-true)),
                perfect_registration_error_m=float(np.linalg.norm(i-true)),
                aggregation_registration_residual_m=float(np.linalg.norm(a-i)),
                actual_error=(a-true).tolist(),perfect_reference_bias=(i-true).tolist(),
                registration_residual=(a-i).tolist()))
    assert max_aggregation < 1e-12
    a=np.asarray(samples);stats={}
    for j,key in enumerate(('bootstrap','material_point_rotation','registration','total')):
        value=a[:,2*j:2*j+2]
        stats[key]=dict(mean_signed_xy_m=value.mean(axis=0).tolist(),
                       mean_norm_m=float(np.linalg.norm(value,axis=1).mean()))
    serial_boot={str(i):{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in b.items()} for i,b in boot.items()}
    result=dict(case=name,accepted_readouts=counts,bootstrap=serial_boot,local_components=stats,
        maximum_vector_identity_residual_m=max_identity,maximum_team_aggregation_residual_m=max_aggregation,
        final=team[-1],team_mean_actual_error_m=float(np.mean([r['actual_error_m'] for r in team])),
        team_mean_perfect_registration_error_m=float(np.mean([r['perfect_registration_error_m'] for r in team])),
        inputs={p:sha(folder/p) for p in ('resolved_case.json','paired_metrics.json','replay.npz')},series=team)
    del steps;gc.collect()
    return result

def main():
    limit_blas_threads(1)
    dest=ROOT/'artifacts/research'/MISSION/'reference_point_audit.json'
    if dest.exists():raise RuntimeError('Preserve existing analysis')
    result={}
    for name in CASES:
        r=audit(name);result[name]=r
        print(json.dumps(dict(case=name,actual_MAE=r['team_mean_actual_error_m'],
            perfect_registration_MAE=r['team_mean_perfect_registration_error_m'],final=r['final'])),flush=True)
    dest.write_text(json.dumps(dict(cases=result,script_sha256=sha(Path(__file__)),
        scope='Offline exact rigid-geometry counterfactual on the observed reference points. Same historical accepted voters; no new controller or physical run. Ground truth is only an analysis input. Norms of signed components cannot be added. Frames/readouts are not independent experiments.'),indent=2),encoding='utf-8')

if __name__=='__main__':main()
