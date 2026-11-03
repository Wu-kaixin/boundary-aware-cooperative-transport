"""Publish complete paired outcomes and the independently traced map point."""
from __future__ import annotations

import csv
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from research_io import ROOT,guard,read,sha
from map_bridge_study import BASE,PROTOCOL,CHECKS,ARMS,check
from dbact_sim.experiments.deployment import dump

PUBLIC=ROOT/'docs/results/map_reference_bridge_v1'
ANCESTRY=ROOT/'runs/research/2026-11-03-map-ancestry-v1'


def main():
    guard()
    plan=check()
    outcome=read(BASE/'analysis_outcome.json')
    assert outcome['status']=='VERIFIED' and outcome['episodes']==8
    assert sha(BASE/'analysis.json')==outcome['analysis_sha256']
    report=read(BASE/'analysis.json')
    assert report['protocol_sha256']==sha(PROTOCOL)
    PUBLIC.mkdir(parents=True,exist_ok=False)
    lineage_outcome=read(ANCESTRY/'analysis/outcome.json')
    assert lineage_outcome['status']=='VERIFIED_AFFINE_ANCESTRY'
    for name,digest in lineage_outcome['products'].items():
        assert sha(ANCESTRY/'analysis'/name)==digest
        if name!='large_alignment_events.json':
            shutil.copy2(ANCESTRY/'analysis'/name,PUBLIC/name)
    shutil.copy2(BASE/'analysis.json',PUBLIC/'analysis.json')
    rows=[]
    order={c:i for i,c in enumerate(plan['conditions'])}
    for r in sorted(report['episodes'],key=lambda r:(order[r['condition']],ARMS.index(r['arm']))):
        assert r['execution']=='COMPLETED'
        bridge=r['bridge_audit']
        rows.append(dict(condition=r['condition'],arm=r['arm'],safe_completed=r['safe_completed'],
            false_hold=r['false_hold'],hard_safety_count=len(r['hard_safety']),g500=r['g500'],
            g500_reasons=' | '.join(r['g500_failure_reasons']),budget_terminated=r['budget_terminated'],
            formal_transport=r['phase_response'].get('transport_frame') is not None,
            seconds=r['seconds'],endpoint_mm=1000*r['endpoint_m'],
            max_true_rotation_deg=r['maximum_absolute_rotation_deg'],
            acquisition_scaling=r['scaling']['acquisition'],post_acquisition_scaling=r['scaling']['post_acquisition'],
            minimum_scale=r['scaling']['min_scale'],
            bridge_agent_steps=None if bridge is None else bridge['agent_steps'],
            bridge_resets=None if bridge is None else bridge['counts'].get('reanchored',0),
            bridge_audit_verified=None if bridge is None else bridge['verified']))
    with (PUBLIC/'all_eight.csv').open('x',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
        writer.writeheader();writer.writerows(rows)
    dump(PUBLIC/'gates.json',dict(mechanism=report['mechanism_gates'],full=report['full_gates'],
        mechanism_verdict=report['mechanism_gate'],full_verdict=report['full_gate'],candidate_adopted=False))
    dump(PUBLIC/'original_reproduction.json',report['original_reproduction'])
    dump(PUBLIC/'engineering_checks.json',dict(tests=plan['tests'],canonical_sha256=plan['canonical_sha256'],
        runtime_source_sha256=plan['frozen_source_sha256'],physical_episodes=8,
        diagnosis_replay_frames=989,separate_preflight_frames=20,
        trace_outcome=read(ANCESTRY/'complete/outcome.json'),lineage_outcome=lineage_outcome,
        environment=plan['environment'],installed_locked_packages=plan['installed_locked_packages']))

    far=['far_c_open_away_pos0','far_skew_box_s0_r0_pos0']
    lookup={(r['condition'],r['arm']):r for r in rows}
    fig,axes=plt.subplots(2,2,figsize=(11,8))
    specs=[('post_acquisition_scaling','Post-acquisition scaled robot steps','Count'),
        ('max_true_rotation_deg','Maximum actual rotation','Degrees'),
        ('endpoint_mm','Final task error','mm'),('seconds','Elapsed simulation time','Seconds')]
    colors=['#777777','#0072B2']
    for ax,(key,title,unit) in zip(axes.flat,specs):
        top=0.
        for j,arm in enumerate(ARMS):
            values=[lookup[c,arm][key] for c in far]
            locations=np.arange(2)+(j-.5)*.34
            ax.bar(locations,values,.34,color=colors[j],label=['Local-pair control','Map-bridge candidate'][j])
            top=max(top,*values)
            for x,value in zip(locations,values):
                ax.annotate(f'{value:.0f}' if 'scaling' in key else f'{value:.3f}',(x,value),
                    xytext=(0,5),textcoords='offset points',ha='center',fontsize=9)
        ax.set(xticks=[0,1],xticklabels=['Far C','Far skew box'],title=title,ylabel=unit,ylim=(0,max(1,top*1.25)))
        ax.spines[['top','right']].set_visible(False)
        ax.set_axisbelow(True);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Same-voter map increments: fixed-parameter paired development study',fontsize=14)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.955),ncol=2,frameon=False)
    fig.text(.5,.025,f"8 episodes: 4 reused conditions x 2 arms; all outcomes retained.\n"
        f"Mechanism: {report['mechanism_gate']}; full chain: {report['full_gate']}. Candidate remains default-off.",
        ha='center',fontsize=10)
    fig.subplots_adjust(top=.85,bottom=.13,hspace=.36,wspace=.3)
    fig.savefig(PUBLIC/'comparison.png',dpi=170)
    fig.savefig(PUBLIC/'comparison.pdf')
    plt.close(fig)
    products={p.name:sha(p) for p in PUBLIC.iterdir() if p.is_file()}
    dump(PUBLIC/'publication.json',dict(script_sha256=sha(__file__),protocol_sha256=sha(PROTOCOL),
        raw_analysis_sha256=sha(BASE/'analysis.json'),products=products))
    print('Published complete eight-episode results and map ancestry')


if __name__=='__main__':
    main()
