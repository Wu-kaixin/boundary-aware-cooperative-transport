"""Summarize every fixed acquisition-trace frame and independently check witnesses."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_key]='1'

from collections import Counter
import csv
import gzip
import json
from pathlib import Path

import numpy as np

from research_io import ROOT, guard, read, sha
from qp_conflict_study import independent

BASE=ROOT/'runs/research/2026-11-03-acquisition-trace-v1'
PUBLIC=ROOT/'docs/results/acquisition_trace_v1'


def main():
    guard()
    from dbact_sim.experiments.deployment import dump
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    if PUBLIC.exists() or (BASE/'analysis.json').exists():
        raise RuntimeError('Preserve existing analysis/publication')
    plan=read(ROOT/'experiments/ccta2027/reference_map_v1.json')
    data, summaries, witnesses={}, {}, {}
    capture_rows,timeline=[],[]
    for arm,end in (('baseline',791),('candidate',1201)):
        folder=BASE/arm
        outcome=read(folder/'outcome.json')
        assert outcome['status']=='EXACT_RECONSTRUCTION' and outcome['verified_frames']==end
        assert outcome['source_sha256']==plan['frozen_source_sha256']
        assert outcome['protocol_sha256']==sha(ROOT/'experiments/ccta2027/reference_map_v1.json')
        for name,digest in outcome['products'].items(): assert sha(folder/name)==digest,name
        for name,digest in outcome['scripts'].items(): assert sha(folder/name)==digest,name
        with gzip.open(folder/'frames.jsonl.gz','rt',encoding='utf-8') as stream:
            frames=[json.loads(line) for line in stream]
        assert [f['frame'] for f in frames]==list(range(1,end+1))
        data[arm]=frames
        events=read(folder/'events.json')
        assert len(events['events'])==outcome['scaled_robot_steps']
        assert all(e['frame']<=end for e in events['events'])
        original=next(c for c in plan['cases'] if c['case_id']==outcome['case_id'])
        assert sha(ROOT/plan['run_dir']/original['case_id']/'case_result.json')==outcome['original_receipt_sha256']
        first_support={}
        capacity_rejected=0
        for f in frames:
            for aid,s in f['captures'].items():
                if s['supported']: first_support.setdefault(aid,f['frame'])
            capacity_rejected+=sum(x['capacity_rejected_cells'] for x in f['incoming'].values())
            timeline.append(dict(arm=arm,frame=f['frame'],time=f['time'],status=f['status'],
                supported=sum(s['supported'] for s in f['captures'].values()),
                agent03_gap_deg=np.degrees(f['captures']['agent_03']['max_normal_gap_rad']),
                agent03_speed=next(d['speed_command'] for d in f['diagnostics'] if d['agent_id']=='agent_03'),
                scaled=sum(d['solver_status']=='scaled_barrier' for d in f['diagnostics'])))
        last=frames[-1]
        for aid,s in last['captures'].items():
            capture_rows.append(dict(arm=arm,agent_id=aid,first_support_frame=first_support.get(aid),**s))
        counts={key:sum(e['feasible'][key] for e in events['events']) for key in events['events'][0]['feasible']}
        summaries[arm]=dict(case_id=outcome['case_id'],verified_frames=end,
            status=last['status'],epoch=last['epoch'],first_support_frames=first_support,
            max_supported=max(sum(s['supported'] for s in f['captures'].values()) for f in frames),
            max_capture_cells=max(s['points'] for f in frames for s in f['captures'].values()),
            capacity=4096,capacity_rejected_cells=capacity_rejected,
            final_failures={aid:[k for k in ('enough_points','conditioned','spatial','normal_wrap') if not s[k]]
                for aid,s in last['captures'].items() if not s['supported']},
            scaled_robot_steps=len(events['events']),scaled_by_status=dict(Counter(e['status'] for e in events['events'])),
            scaled_by_agent=dict(Counter(e['agent_id'] for e in events['events'])),feasible_counts=counts,
            max_true_translation_m=max(float(np.linalg.norm(np.array(f['true_center'])-frames[0]['true_center'])) for f in frames),
            max_true_rotation_rad=max(abs(f['true_angle']-frames[0]['true_angle']) for f in frames),
            outcome_sha256=sha(folder/'outcome.json'))
        for label,w in events['witnesses'].items():
            A,full,free,zero=[np.asarray(w[k]) for k in ('A','b_original','b_no_margin','b_zero_motion')]
            obj=np.asarray(w['row_kinds'])=='object'
            cap=w['filter_params']['max_speed']
            checks={key:independent(a,b,cap) for key,a,b in (
                ('full',A,full),('no_rho',A,free),('hard_only',A[~obj],free[~obj]),
                ('object_only',A[obj],free[obj]),('zero_estimated_motion',A,zero))}
            assert all(v['numerically_valid'] and v['feasible']==w['feasible'][k] for k,v in checks.items())
            witnesses[arm+'/'+label]=dict(**w,independent=checks,without_speed_cap=independent(A,free,None))
    result=dict(scope='Retrospective fixed-prefix diagnosis; no controller repair or new physical validation',
        new_physical_episodes=0,exact_frames=sum(v['verified_frames'] for v in summaries.values()),
        cases=summaries,witnesses=witnesses,all_four_witness_checks_agree=True,
        analyzer_sha256=sha(__file__),independent_helper_sha256=sha(ROOT/'scripts/qp_conflict_study.py'))
    dump(BASE/'analysis.json',result)
    PUBLIC.mkdir(parents=True,exist_ok=False)
    dump(PUBLIC/'analysis.json',result)
    for name,rows in (('capture_support.csv',capture_rows),('support_timeline.csv',timeline)):
        with (PUBLIC/name).open('x',newline='',encoding='utf-8') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    fig,axes=plt.subplots(2,2,figsize=(12,9),layout='constrained')
    colors={'baseline':'#285b9a','candidate':'#c75e27'}
    labels={'baseline':'Translation baseline','candidate':'Reference-map candidate'}
    for arm,frames in data.items():
        rows=[r for r in timeline if r['arm']==arm]
        axes[0,0].plot([r['time'] for r in rows],[r['supported'] for r in rows],label=labels[arm],color=colors[arm])
        axes[0,1].plot([r['time'] for r in rows],[r['agent03_gap_deg'] for r in rows],label=labels[arm],color=colors[arm])
    axes[0,0].axhline(4,color='#555',ls='--',label='Original quorum: 4')
    axes[0,0].set(xlabel='Time (s)',ylabel='Supported captures',title='Capture admission stalls at three',ylim=(-.1,4.6))
    axes[0,0].legend(fontsize=8,loc='upper left')
    axes[0,1].axhline(180,color='#555',ls='--')
    axes[0,1].set(xlabel='Time (s)',ylabel='Largest normal gap (degrees)',title='Agent 03 never obtains normal wrap')
    for ax,arm in zip(axes[1],('baseline','candidate')):
        frames=data[arm]
        frame=frames[-1] if arm=='baseline' else frames[-2]
        case=next(c for c in plan['cases'] if c['case_id']==summaries[arm]['case_id'])
        vertices=np.asarray(case['config']['cargoes'][0]['vertices'])
        ax.add_patch(Polygon(vertices,closed=True,facecolor='#e4e7eb',edgecolor='#555',label='Cargo truth (observer only)'))
        paths=np.array([f['positions'] for f in frames[max(0,len(frames)-201):-1]])
        points=np.asarray(frame['positions'])
        for i,point in enumerate(points):
            aid=f'agent_{i:02d}'
            supported=frame['captures'][aid]['supported']
            ax.plot(paths[:,i,0],paths[:,i,1],color='#b5b5b5',lw=.7)
            ax.scatter(*point,color='#218463' if supported else '#8c3b3b',s=24)
            if i in (3,4,5,6,7,12): ax.annotate(str(i),point,xytext=(3,4),textcoords='offset points',fontsize=8)
        lo,hi=vertices.min(axis=0)-.9,vertices.max(axis=0)+.9
        outside=sum(np.any((p<lo)|(p>hi)) for p in points)
        ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='x (m)',ylabel='y (m)',
            title=f"{labels[arm]} at {frame['time']:.2f}s\nGreen: supported; red: unsupported; {outside} outside local view")
        ax.set_aspect('equal',adjustable='box')
    for ax in axes.flat: ax.grid(alpha=.17)
    fig.suptitle('Skew acquisition: exact replay of 1,992 existing frames; no new physics',fontsize=13)
    fig.savefig(PUBLIC/'acquisition_support.png',dpi=170)
    fig.savefig(PUBLIC/'acquisition_support.pdf')
    plt.close(fig)
    names=('analysis.json','capture_support.csv','support_timeline.csv','acquisition_support.png','acquisition_support.pdf')
    dump(PUBLIC/'publication.json',dict(outputs={n:sha(PUBLIC/n) for n in names},
        analyzer_sha256=sha(__file__),new_physical_episodes=0,exact_frames=result['exact_frames']))
    print(json.dumps(dict(exact_frames=result['exact_frames'],summaries=summaries,all_four_witness_checks_agree=True)),flush=True)


if __name__=='__main__': main()
