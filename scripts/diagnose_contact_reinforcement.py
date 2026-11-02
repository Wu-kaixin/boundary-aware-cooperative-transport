"""Audit three frozen failure logs before developing a recovery/recruitment policy."""
from pathlib import Path
import gc,json
import numpy as np
from dbact_sim.experiments.study import cached_result,verify_source_snapshot
from dbact_sim.experiments.deployment import dump
from analyze_dynamic_contact_study import read,sha
from diagnose_feedback_contact_loss import extract
ROOT=Path(__file__).resolve().parents[1]
MISSION='2026-11-01-contact-reinforcement-v1'
OLD=ROOT/'runs/research/2026-11-01-feedback-components-v1/run'
out=ROOT/'artifacts/research'/MISSION/'diagnosis'
if out.exists():raise RuntimeError('Keep previous diagnosis')
out.mkdir()
plan=read(OLD/'resolved_protocol.json');manifest=read(OLD/'manifest.json')
verify_source_snapshot(OLD,manifest['source_files'])
reports=[];inputs={};series={}
for c in plan['cases']:
    if c['condition_id'] not in ('circle','ellipse_parallel','c_open_away') or c['arm']!='reference_cvt':continue
    print('Read '+c['case_id'],flush=True)
    folder=OLD/c['case_id'];receipt=read(folder/'case_result.json');cached_result(folder,receipt['identity'])
    inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
    inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
    steps=read(folder/'paired_metrics.json')['steps'];summary,rows=extract(c,folder,receipt,steps=steps)
    cfg=c['config']['controller'];tail=steps[-200:]
    mass=next(r['mass_kg'] for r in plan['conditions'] if r['condition']==c['condition_id'])
    threshold=mass*c['config']['transport']['ground_friction']*c['config']['transport']['gravity']
    forces=np.linalg.norm([s['actual_net_force'] for s in tail],axis=1)
    with np.load(folder/'replay.npz',allow_pickle=False) as z:positions=z['agent_positions'].copy()
    potential_counts=[];reasons={};mismatches=0
    for k,s in enumerate(steps):
        ds=s['diagnostics'];ready=np.array([d['contact_ready'] for d in ds])
        dist=np.linalg.norm(positions[k,:,None,:]-positions[k,None,:,:],axis=2);np.fill_diagonal(dist,np.inf)
        potential=0
        for i,d in enumerate(ds):
            n=np.array(d['contact_selected_normal'],float);direction=np.array(d['contact_desired_direction'],float)
            align=float(n@direction) if np.isfinite(n).all() and np.isfinite(direction).all() else None
            quorum=1+int(np.count_nonzero(ready&(dist[i]<=cfg['comm_range'])))
            remaining=d['navigation_remaining']
            reason=('not_ready' if not d['contact_ready'] else 'no_usable_patch' if align is None else
                'wrong_normal' if align > -cfg['push_side_threshold'] else 'local_quorum' if quorum<cfg['min_push_agents'] else
                'arrival_or_no_feedback' if remaining is None or remaining<=0 else 'outside_phase' if s['phase'] not in ('CONTACT_READY','TRANSPORT','BRAKE') else 'eligible')
            eligible=reason=='eligible';potential+=eligible
            predicted=eligible and not s['phase_response']['push_suspended']
            mismatches+=bool(predicted)!=d['push_side']
            if k>=len(steps)-200:reasons[reason]=reasons.get(reason,0)+1
        potential_counts.append(potential)
    assert mismatches==0
    paused=[k for k,s in enumerate(steps) if s['phase_response']['push_suspended']]
    final=steps[-1]['navigation_supervisor']
    summary.update(condition=c['condition_id'],recorded_pusher_predicates_exact=True,
        paused_frames=len(paused),pause_max_geometric_eligibility=max((potential_counts[k] for k in paused),default=None),
        final10s_predicate_counts=reasons,final10s_frames=len(tail),ground_breakaway_N=threshold,
        final10s_force_below_breakaway=int(np.count_nonzero(forces<threshold)),
        final10s_max_net_force_N=float(forces.max()),final10s_mean_net_force_N=float(forces.mean()),
        final10s_max_speed_m_s=max(float(np.linalg.norm(s['true_velocity'])) for s in tail),
        final_shared_remaining_m=final['remaining'] if final else None)
    reports.append(summary);series[c['case_id']]=rows
    print(json.dumps({k:summary[k] for k in ['condition','endpoint_error_m','paused_frames','pause_max_geometric_eligibility','final10s_predicate_counts','final10s_max_net_force_N','final10s_max_speed_m_s']}),flush=True)
    del steps;gc.collect()
dump(out/'diagnosis.json',dict(kind='OFFLINE_NO_NEW_PHYSICS',cases=reports,inputs=inputs,script_sha256=sha(Path(__file__)),
    helper_sha256=sha(ROOT/'scripts/diagnose_feedback_contact_loss.py'),
    limits='Full-frame predicate/event checks; final-ten-second force description is post hoc. Geometric eligibility is not actual contact. No truth enters control.'))
dump(out/'timeseries.json',series)
