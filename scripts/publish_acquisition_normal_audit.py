"""Publish the complete acquisition-normal and phase audit without new physics."""
from __future__ import annotations

import csv
import shutil
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon
import numpy as np

from research_io import ROOT, guard, read, sha

BASE = ROOT/'runs/research/2026-11-03-acquisition-normal-audit-v1'
PUBLIC = ROOT/'docs/results/acquisition_normal_audit_v1'


def main():
    guard()
    protocol = ROOT/'experiments/ccta2027/reference_reacquisition_v1.json'
    plan = read(protocol)
    batch = ROOT/plan['run_dir']
    for name, digest in plan['source_files'].items():
        assert sha(batch/'source_snapshot'/name) == digest
    sys.path.insert(0, str(batch/'source_snapshot/src'))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump
    normal_dir, phase_dir = BASE/'normal-analysis-v1', BASE/'phases'
    outcome, phase_outcome = read(normal_dir/'outcome.json'), read(phase_dir/'outcome.json')
    assert outcome['status'] == 'COMPLETED' and sha(normal_dir/'analysis.json') == outcome['analysis_sha256']
    assert phase_outcome['status'] == 'EXACT_PHASE_RECONSTRUCTION'
    assert sha(phase_dir/'analysis.json') == phase_outcome['analysis_sha256']
    analysis, phases = read(normal_dir/'analysis.json'), read(phase_dir/'analysis.json')
    assert analysis['source_sha256'] == phases['source_sha256'] == plan['frozen_source_sha256']
    assert analysis['protocol_sha256'] == phases['protocol_sha256'] == sha(protocol)
    assert sum(r['conflict_events'] for r in analysis['cases'].values()) == 26
    assert phases['phase_reconstruction_frames'] == 2589
    PUBLIC.mkdir(parents=True, exist_ok=False)
    shutil.copy2(normal_dir/'analysis.json', PUBLIC/'analysis.json')
    shutil.copy2(phase_dir/'analysis.json', PUBLIC/'phases.json')
    records = [dict(case=alias, **event) for alias,case in analysis['cases'].items() for event in case['events']]
    with (PUBLIC/'all_26_conflicts.csv').open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    phase_records = []
    for episode in phases['episodes']:
        events = episode['events']
        first = events.get('first_quorum') or {}
        phase_records.append(dict(case_id=episode['case_id'], verified_frames=episode['verified_frames'],
            transport_quorum=episode['gates']['transport_quorum'],
            formal_transport_frame=episode['final_response']['transport_frame'],
            brake_frame=episode['final_response']['brake_frame'],
            contact_ready_maximum_active=episode['by_phase'].get('CONTACT_READY',{}).get('maximum_active'),
            first_quorum_frame=first.get('frame'), first_quorum_seconds=first.get('time'),
            first_quorum_phase=first.get('phase'), brake_reason=episode['final_response']['brake_reason']))
    with (PUBLIC/'phase_summary.csv').open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(phase_records[0]))
        writer.writeheader()
        writer.writerows(phase_records)
    fig, axes = plt.subplots(2,2, figsize=(11.5,8.5))
    fig.subplots_adjust(left=.085, right=.98, top=.91, bottom=.10, wspace=.28, hspace=.38)
    for ax, alias, title, color in zip(axes[0], ('candidate-c','candidate-skew'),
            ('Far C, frame326', 'Far skew, frame384'), ('#0072b2','#d55e00')):
        case_report = analysis['cases'][alias]
        trace_dir = BASE/alias
        trace_outcome = read(trace_dir/'outcome.json')
        assert sha(trace_dir/'events.json') == trace_outcome['products']['events.json']
        event = read(trace_dir/'events.json')['events'][0]
        case = next(c for c in plan['cases'] if c['case_id'] == case_report['case_id'])
        env = SimulationEnvironment(case['config'], seed=case['seed'])
        p = np.asarray(event['position'])
        n = np.asarray(event['object_trace']['n_bar'])
        offset = event['object_trace']['offset_from_n_k']
        foot = p-n*(n@p-offset)
        tangent = np.array([-n[1],n[0]])
        line = foot+np.array([[-1.],[1.]])*tangent
        ax.add_patch(Polygon(env.cargoes[0].vertices, closed=True, facecolor='#cccccc', edgecolor='#444444', alpha=.6))
        ax.add_patch(Circle(p, event['params']['r_safe'], facecolor='#009e73', alpha=.12, edgecolor='#009e73'))
        ax.plot(line[:,0],line[:,1],'--',color=color,linewidth=1.5,label='Extrapolated aggregate plane')
        points=np.asarray(event['object_trace']['face_points'])
        normals=np.asarray(event['object_trace']['face_normals'])
        ax.scatter(points[:,0],points[:,1],color=color,s=24,zorder=4,label='Selected map cells')
        ax.quiver(points[:,0],points[:,1],normals[:,0],normals[:,1],angles='xy',scale_units='xy',scale=12,width=.006,color=color)
        ax.scatter(*p,color='#111111',s=35,zorder=5,label='Robot center')
        corner=np.asarray(event['true_foot'])
        ax.plot([p[0],corner[0]],[p[1],corner[1]],color='#009e73',linewidth=1.5,label='True nearest boundary')
        ax.set_xlim(p[0]-.34,p[0]+.34)
        ax.set_ylim(p[1]-.25,p[1]+.43)
        ax.set_aspect('equal')
        ax.set_title(title)
        ax.set_xlabel('World x (m)')
        ax.set_ylabel('World y (m)')
        ax.grid(alpha=.15)
    axes[0,1].legend(fontsize=7,loc='lower right',framealpha=.9)
    ax=axes[1,0]
    for alias,label,color in (('candidate-c','C','#0072b2'),('candidate-skew','Skew','#d55e00')):
        rows=analysis['cases'][alias]['events']
        x=np.array([r['true_margin_m'] for r in rows])*1000
        ax.scatter(x,[r['aggregate_h_m']*1000 for r in rows],s=27,color=color,label=label+' original')
        ax.scatter(x,[r['corrected_true_supporting_planes_h_m']*1000 for r in rows],s=38,color=color,marker='x',label=label+' true incident planes')
    ax.axhline(0,color='#555555',linewidth=1)
    ax.set_xlabel('True clearance minus original safety margin (mm)')
    ax.set_ylabel('Local plane barrier h (mm)')
    ax.set_title('All26 saved acquisition conflicts')
    ax.legend(fontsize=7,ncols=2,loc='lower left')
    ax.grid(alpha=.2)
    ax=axes[1,1]
    c=next(r for r in phases['episodes'] if r['case_id'].startswith('far_c_'))
    folder=batch/c['case_id']
    assert sha(folder/'case_result.json') == c['receipt_sha256']
    steps=read(folder/'paired_metrics.json')['steps']
    times=np.arange(len(steps))*.05
    ax.step(times,[r['phase_guards']['transport_active'] for r in steps],where='post',color='#0072b2',linewidth=1,label='Prior-step push-side count')
    brake=c['events']['BRAKE']['time']
    hold=c['events']['HOLD']['time']
    ax.axvspan(brake,hold,color='#e69f00',alpha=.18,label='BRAKE phase')
    ax.axhline(c['gates']['transport_quorum'],color='#b2182b',linestyle='--',linewidth=1,label='Original quorum4')
    ax.axvline(brake,color='#e69f00',linewidth=1)
    ax.set_xlim(c['events']['CONTACT_READY']['time']-1,times[-1]+.05)
    ax.set_ylim(-.2,5.7)
    ax.set_yticks(range(6))
    ax.set_xlabel('Control time (s)')
    ax.set_ylabel('Active count used by phase monitor')
    ax.set_title('C: first quorum occurs4.8s after braking')
    ax.legend(fontsize=7,loc='upper left')
    ax.grid(alpha=.2)
    fig.suptitle('Acquisition geometry and formal activation: retrospective audit',fontsize=13)
    fig.text(.5,.035,'Truth is observer-only. Two reused far conditions; zero new physical episodes. No controller repair is claimed.',ha='center',fontsize=9)
    for suffix in ('png','pdf'):
        fig.savefig(PUBLIC/f'geometry_and_activation.{suffix}',dpi=180)
    plt.close(fig)
    outputs=('analysis.json','phases.json','all_26_conflicts.csv','phase_summary.csv',
             'geometry_and_activation.png','geometry_and_activation.pdf')
    dump(PUBLIC/'publication.json',dict(script_sha256=sha(__file__),
        outputs={name:sha(PUBLIC/name) for name in outputs},new_physical_episodes=0,
        new_exact_controller_frames=784, separate_preflight_controller_frames=20,
        phase_only_reconstruction_frames=2589, acquisition_conflicts=26,
        unchanged_canonical_sha256=sha(ROOT/'configs/research/best_known_transport_v1.yaml')))
    print(dict(physical_episodes=0,controller_frames=784,phase_only_frames=2589,conflicts=26),flush=True)


if __name__ == '__main__':
    main()
