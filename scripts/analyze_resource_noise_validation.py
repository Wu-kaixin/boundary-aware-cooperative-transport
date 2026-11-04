"""Analyze every frozen resource/noise case with bounded step-log memory."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path

import resource_noise_validation as study
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from dbact.geometry import rotate
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import verify_source_snapshot
from analyze_feedback_components import safety_summary
from analyze_cargo_discovery import deployment_snapshot
from stream_step_receipts import iter_step_receipts


def episode(case, folder, manifest):
    receipt = study.read(folder / 'case_result.json')
    assert receipt['identity'] == {**manifest['identity'], 'case_sha256': encode(case)}
    assert study.read(folder / 'resolved_case.json') == case
    for name, expected in receipt['products'].items():
        assert study.sha(folder / name) == expected, (case['case_id'], name)
    r = receipt['result']
    row = dict(case_id=case['case_id'], shape=case['factors']['shape_condition'],
               regime=case['arm'], method=case['variant'], robots=case['factors']['robots'],
               seed=case['seed'], execution_status=r['execution_status'],
               completed=bool(r.get('task_completed')), safe_completed=False,
               g500=bool(r.get('g500_pass')), error=r.get('error',''),
               receipt_sha256=study.sha(folder / 'case_result.json'))
    if r['execution_status'] != 'COMPLETED':
        row['outcome'] = 'EXECUTION_ERROR'
        return row, {}
    metrics, hard, verdict = safety_summary(folder)
    with np.load(folder / 'replay.npz', allow_pickle=False) as z:
        times, robots, xy, start, goal, local, angles = [z[k].copy() for k in
            ('times','agent_positions','cargo/cargo_0/centers','task/cargo_0/start',
             'task/cargo_0/goal_point','cargo/cargo_0/local_vertices','cargo/cargo_0/angles')]
    assert encode(robots[0].tolist()) == case['expected_initial_state_sha256']
    endpoint = float(np.linalg.norm(xy[-1] - goal))
    assert abs(endpoint - r['euclidean_endpoint_error']) < 1e-12
    assert row['completed'] == bool(r['settled'] and endpoint <= .12)
    frames = r['termination']['frames_run']
    assert frames <= 12000 and len(times) == frames + 1
    nav, contacts, force, phase = [], [], [], []
    first_detection = first_contact = first_forward = None
    with (folder / 'paired_metrics.json').open(encoding='utf-8') as f:
        for k, s in enumerate(iter_step_receipts(f)):
            assert s['frame'] == k+1
            if first_detection is None and any(d['own_scan_points'] > 0 for d in s['diagnostics']):
                first_detection = k
            if first_contact is None and s['contacts'] > 0:
                first_contact = k
            if first_forward is None and s['true_progress'] > 1e-9:
                first_forward = k
            sup = s['navigation_supervisor']
            nav.append([np.nan,np.nan] if sup is None else sup['displacement'])
            contacts.append(s['contacts'])
            force.append(s['forward_force'])
            phase.append(s['phase'])
    assert len(nav) == frames  # Exhaustion also checks JSON suffix/truncation.
    nav = np.asarray(nav)
    nav_error = np.linalg.norm(nav - (xy[:-1] - start), axis=1)
    usable = np.isfinite(nav_error)
    ready = r['contact_ready_frame']
    snapshot_frame = None if ready is None else min(ready+1, len(times)-1)
    snapshot = None if snapshot_frame is None else deployment_snapshot(robots[snapshot_frame],
                rotate(local, angles[snapshot_frame]) + xy[snapshot_frame])
    final = deployment_snapshot(robots[-1], rotate(local, angles[-1]) + xy[-1])
    if row['completed']:
        outcome = 'COMPLETED'
    elif first_detection is None:
        outcome = 'NO_DETECTION'
    elif ready is None:
        outcome = 'DEPLOYMENT_NOT_READY'
    elif first_contact is None:
        outcome = 'NO_ACTUAL_CONTACT'
    elif first_forward is None:
        outcome = 'NO_FORWARD_MOTION'
    elif r['settled']:
        outcome = 'SETTLED_OFF_TARGET'
    else:
        outcome = 'WATCHDOG_' + phase[-1]
    row.update(outcome=outcome, safe_completed=bool(row['completed'] and not hard),
        hard_safety='|'.join(hard), endpoint_error_m=endpoint, seconds=frames*.05,
        completion_seconds=frames*.05 if row['completed'] else None,
        settled=r['settled'], watchdog=r['watchdog'], final_phase=phase[-1],
        first_detection_s=None if first_detection is None else first_detection*.05,
        first_contact_s=None if first_contact is None else first_contact*.05,
        first_forward_motion_s=None if first_forward is None else first_forward*.05,
        ready_s=None if ready is None else ready*.05,
        physical_contact_quorum_s=r['physical_contact_ready']['seconds'],
        ready_coverage=None if snapshot is None else snapshot['coverage'],
        ready_gap_upper_m=None if snapshot is None else snapshot['max_uncovered_arc_upper_m'],
        final_coverage=final['coverage'], final_gap_upper_m=final['max_uncovered_arc_upper_m'],
        mean_robot_path_m=float(np.linalg.norm(np.diff(robots,axis=0),axis=2).sum(axis=0).mean()),
        mean_contacts=r['mean_contacts'], peak_actual_net_force_N=r['peak_actual_net_force'],
        direction_error_deg=r['direction_error_deg'], max_cross_track_m=r['max_cross_track'],
        rotation_deg=r['rotation_deg'], final_speed=r['final_speed'],
        max_penetration_m=metrics['max_penetration'], min_separation_m=metrics['min_inter_agent_distance'],
        barrier_scalings=metrics['barrier_scalings'], g500_reasons=' | '.join(verdict.reasons),
        navigation_MAE_m=float(nav_error[usable].mean()) if usable.any() else None,
        source_sha256=receipt['identity']['source_sha256'])
    for label, key in [('transport','transport_frame'),('brake','brake_frame'),('hold','hold_frame')]:
        frame = r['phase_response'].get(key)
        row[label+'_s'] = None if frame is None else frame*.05
    d = np.asarray(r['goal_direction'])
    series = dict(time=times.tolist(), xy=xy.tolist(), start=start.tolist(), goal=goal.tolist(),
        control_time=times[:-1].tolist(), true_progress=((xy[:-1]-start) @ d).tolist(),
        estimated_navigation_progress=(nav @ d).tolist(), navigation_error=nav_error.tolist(),
        contacts=contacts, forward_force=force, phase=phase)
    return row, series


def aggregate(rows):
    return dict(planned=len(rows), execution_completed=sum(r['execution_status']=='COMPLETED' for r in rows),
        completed=sum(r['completed'] for r in rows), safe_completed=sum(r['safe_completed'] for r in rows),
        hard_safety_cases=sum(bool(r.get('hard_safety')) for r in rows),
        g500=sum(r['g500'] for r in rows), watchdog=sum(bool(r.get('watchdog')) for r in rows),
        barrier_scalings=sum(r.get('barrier_scalings',0) for r in rows))


def main(out):
    study.guard()
    study.verify()
    out = out.resolve()
    if not out.is_relative_to(study.BASE) or out.exists():
        raise ValueError('Use a fresh directory under the study')
    plan = study.read(study.PROTOCOL)
    run = study.BASE / 'run'
    manifest, summary = [study.read(run / p) for p in ('manifest.json','summary.json')]
    assert summary['completed_denominator'] == summary['planned_denominator'] == 60
    assert summary['source_integrity'] == 'PASS'
    verify_source_snapshot(run, manifest['source_files'])
    out.mkdir(parents=True)
    rows, series = [], {}
    for c in plan['cases']:
        print('Verify '+c['case_id'], flush=True)
        row, trace = episode(c, run / c['case_id'], manifest)
        rows.append(row)
        if c['arm']=='reference' and c['variant']=='boundary_cvt':
            series[c['case_id']] = trace
        dump(out / (c['case_id']+'.json'), row)
    grouped = {g: {m: aggregate([r for r in rows if r['regime']==g and r['method']==m])
                  for m in study.METHODS} for g in study.REGIMES}
    fields = list(dict.fromkeys(k for r in rows for k in r))
    with (out / 'all_60_episodes.csv').open('w',newline='',encoding='utf-8') as f:
        writer = csv.DictWriter(f,fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    dump(out / 'analysis.json', dict(episodes=rows, by_regime=grouped, series=series,
         planned=60, source_sha256=manifest['identity']['source_sha256'], protocol_sha256=study.sha(study.PROTOCOL),
         analysis_sha256=study.sha(Path(__file__)), statistical_unit='One shared new seed; 20 designed shape/regime blocks',
         failure_policy='All 60 retained; execution errors and watchdogs are not omitted'))
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':7.5, 'pdf.fonttype':42,
                         'ps.fonttype':42,'svg.fonttype':'none','axes.spines.top':False,'axes.spines.right':False})
    fig, axes = plt.subplots(1,4,figsize=(7.1,2.55),layout='constrained',sharey=True)
    markers, colors = ['o','s','^'], ['#0072B2','#D55E00','#009E73']
    names = ['Reference (16)','8 robots','24 robots','Range sigma 0.02 m']
    for ax, regime, title in zip(axes,study.REGIMES,names):
        for j, m in enumerate(study.METHODS):
            for k, shape in enumerate(study.SHAPES):
                row = next(r for r in rows if r['regime']==regime and r['method']==m and r['shape']==shape)
                if 'endpoint_error_m' in row:
                    ax.plot(k+(j-1)*.15,row['endpoint_error_m'],marker=markers[j],ms=4,
                            color=colors[j],mfc=colors[j] if row['safe_completed'] else 'none',ls='none')
                else:
                    ax.text(k, .96, 'E',transform=ax.get_xaxis_transform(),ha='center')
        ax.axhline(.12,c='.4',ls=':',lw=.8)
        ax.set(title=title,xticks=range(5),xticklabels=['Cir','Ell','Rect','L','C'])
    axes[0].set_ylabel('Final spatial error (m)')
    for ax in axes: ax.set_xlabel('Geometry condition')
    handles = [Line2D([],[],marker=m,color=c,ls='none',label=l) for m,c,l in zip(markers,colors,['CVT','Spacing','Nearest'])]
    fig.legend(handles=handles,loc='outside upper center',ncol=3,fontsize=7)
    for ext in ('pdf','png','svg'):fig.savefig(out/('resource_noise_outcomes.'+ext),dpi=230)
    plt.close(fig)
    tex = ['The current-source protocol records all 60 planned episodes on one new shared seed, with no outcome-driven retries. '
           'The five conditions are circle, ellipse-perpendicular, rectangle-parallel, L-reference and C-away. '
           'Each regime has five tasks per deployment module. Eight- and 16-robot layouts are prefixes of the same 24-robot layout; '
           'the noisier regime shares the reference initial positions. Fixed gains, four-pusher quorum and 600 s limits are retained.\n',
           '\\begin{table}[t]\n\\centering\\caption{Current-source sensitivity. Each cell: spatial/hard-safety completion count out of five, followed by G500 count. One shared seed; descriptive counts.}\\label{tab:sensitivity}\n'
           '\\begin{tabular}{lccc}\\toprule\nRegime & CVT & Spacing & Nearest\\\\\\midrule\n']
    for g,label in zip(study.REGIMES,['Reference','8 robots','24 robots','Noise $\\sigma=0.02$ m']):
        tex.append(label+' & '+' & '.join(f"{grouped[g][m]['safe_completed']}/5; {grouped[g][m]['g500']}/5" for m in study.METHODS)+'\\\\\n')
    tex += ['\\bottomrule\\end{tabular}\\end{table}\n',
            'The full case table retains execution errors, watchdogs, endpoint errors, contact and phase events, trajectory quality and safety measurements. '
            'Robot-resource changes test the fixed controller, including its unchanged quorum; they are not separately optimized team policies. '
            'These designed conditions support sensitivity statements, not a population robustness probability.\n']
    (out/'resource_noise_results.tex').write_text(''.join(tex),encoding='utf-8')
    print(json.dumps(grouped),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    main(ap.parse_args().out)
