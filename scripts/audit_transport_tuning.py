"""Read frozen logs: separate G500 gates, task stopping and estimated geometry."""
import collections
import gc
import json
from pathlib import Path
import numpy as np
from prepare_transport_tuning import ROOT, MISSION, sha

def read(path):
    return json.loads(path.read_text(encoding='utf-8'))

def main():
    old=ROOT/'runs/research/2026-11-02-cargo-discovery-v1'
    output=ROOT/'artifacts/research'/MISSION/'baseline_audit.json'
    if output.exists():
        raise RuntimeError('Audit exists; preserve it')
    records=[]
    for stage in ('near','far'):
        for path in sorted((old/stage).glob('*/summary.json')):
            if stage=='near' and not path.parent.name.endswith('_boundary_cvt'):
                continue
            contract=read(path)['cargoes']['cargo_0']['g500']
            result=read(path.parent/'case_result.json')['result']
            records.append(dict(stage=stage,case=path.parent.name,
                complete=result['task_completed'],g500=contract['success'],
                endpoint_m=result['euclidean_endpoint_error'],
                failure_reasons=contract['failure_reasons'],
                barrier_scalings=contract['metrics']['barrier_scalings'],
                summary_sha256=sha(path)))
    selected=['near/rectangle_parallel_boundary_cvt','near/c_open_away_boundary_cvt',
              'near/u_tray_s0_r90_boundary_cvt','far/skew_box_s0_r0_pos0_unknown_location',
              'far/c_open_away_pos1_initial_location_hint']
    details={}
    for name in selected:
        folder=old/name
        steps=read(folder/'paired_metrics.json')['steps']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            xy=z['cargo/cargo_0/centers']; start=z['task/cargo_0/start']
            speed=z['cargo/cargo_0/speed']; goal=z['task/cargo_0/goal_point']
            estimation=[float(np.linalg.norm(np.asarray(s['navigation_supervisor']['displacement'])-(xy[k]-start)))
                        for k,s in enumerate(steps) if s['navigation_supervisor'] is not None]
            first_scaling=next((k for k,s in enumerate(steps) if s['solver_statistics']['barrier_scalings']>0),None)
            snap_frames={max(0,len(steps)-41),len(steps)-1}
            if first_scaling is not None:snap_frames.add(first_scaling)
            if name.endswith('rectangle_parallel_boundary_cvt'):
                snap_frames.update(k for k,s in enumerate(steps) if k and s['phase']!=steps[k-1]['phase'])
            if name.endswith('initial_location_hint'):
                snap_frames.update([4017,4018,4019])
            snapshots=[]
            for k in sorted(snap_frames):
                s=steps[k]
                snapshots.append(dict(frame_zero_based=k,time_control_s=k*.05,
                    phase=s['phase'],guards=s['phase_guards'],response=s['phase_response'],
                    true_displacement=(xy[k]-start).tolist(),navigation=s['navigation_supervisor'],
                    solver_statistics=s['solver_statistics'],
                    robot_diagnostics=[{key:d[key] for key in ('agent_id','contact_ready','push_side',
                        'object_rows','object_rows_active','effort','modification','reference_status',
                        'contact_map_gap','contact_used_gap','contact_distance_source','reference_matches',
                        'reference_residual_rms','reference_displacement','navigation_remaining')}
                        for d in s['diagnostics']]))
            details[name]=dict(frames=len(steps),maximum_true_speed_m_s=float(max(speed)),
                frames_speed_gt_estimator_bound=int(np.sum(speed>.25)),
                endpoint_m=float(np.linalg.norm(xy[-1]-goal)),
                navigation_error_max_m=max(estimation),navigation_error_final_m=estimation[-1],
                phase_counts=dict(collections.Counter(s['phase'] for s in steps)),
                maximum_transport_active=max(s['phase_guards']['transport_active'] for s in steps),
                reference_status_counts=dict(collections.Counter(d['reference_status'] for s in steps for d in s['diagnostics'])),
                first_barrier_scaling_control_time_s=None if first_scaling is None else first_scaling*.05,
                snapshots=snapshots,
                inputs={p:sha(folder/p) for p in ('paired_metrics.json','replay.npz','summary.json')})
        print(json.dumps({name:{k:v for k,v in details[name].items() if k not in ('snapshots','inputs','reference_status_counts')}}),flush=True)
        del steps;gc.collect()
    result=dict(records=records,details=details,
        scope='All selected saved frames aggregated; listed time snapshots are diagnostic examples, not causal proof.',
        g500='Original phase/event, coverage, projected arrival, holding, solver and hard-safety contract; not 2-D task completion.',
        script_sha256=sha(Path(__file__)))
    output.write_text(json.dumps(result,indent=2),encoding='utf-8')

if __name__=='__main__':
    main()
