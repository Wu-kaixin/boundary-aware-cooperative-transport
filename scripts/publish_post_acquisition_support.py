"""Publish every physical outcome and constraint-audit denominator of the pilot."""
from __future__ import annotations

import copy
import csv
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from research_io import ROOT,guard,read,sha
from post_acquisition_support_study import BASE,PROTOCOL,CHECKS,ARMS,check
from dbact_sim.experiments.deployment import dump


PUBLIC=ROOT/'docs/results/post_acquisition_finite_support_v1'


def main():
    guard()
    plan=check()
    outcome=read(BASE/'analysis_outcome.json')
    assert outcome['status']=='VERIFIED' and outcome['episodes']==8
    assert sha(BASE/'analysis.json')==outcome['analysis_sha256']
    report=read(BASE/'analysis.json')
    assert report['protocol_sha256']==sha(PROTOCOL)
    adapter=read(BASE/'analysis_adapter_v1/outcome.json')
    assert adapter['status']=='VERIFIED' and adapter['analysis_sha256']==outcome['analysis_sha256']
    PUBLIC.mkdir(parents=True,exist_ok=False)
    public_report=copy.deepcopy(report)
    for episode in public_report['episodes']:
        audit=episode.get('post_support_audit')
        if audit is not None:
            violations=audit.pop('violations')
            audit['raw_violation_record_count']=len(violations)
            audit['raw_violation_location']=str((BASE/'analysis.json').relative_to(ROOT)).replace('\\','/')
    public_report['raw_analysis_sha256']=sha(BASE/'analysis.json')
    dump(PUBLIC/'analysis.json',public_report)
    rows=[]
    order={condition:i for i,condition in enumerate(plan['conditions'])}
    for row in sorted(report['episodes'],key=lambda r:(order[r['condition']],ARMS.index(r['arm']))):
        audit=row.get('post_support_audit') or {}
        scaling=row.get('scaling') or {}
        if row['execution']=='COMPLETED':
            for key in ('budget_terminated','g500_failure_reasons','seconds','endpoint_m','maximum_absolute_rotation_deg'):
                assert key in row and row[key] is not None, ('Missing completed-episode metric',row['case_id'],key)
            assert scaling.get('acquisition') is not None and scaling.get('post_acquisition') is not None
        rows.append(dict(condition=row['condition'],arm=row['arm'],execution=row['execution'],
            execution_error=row.get('error'),
            safe_completed=row['safe_completed'],zero_epoch_valid=row['zero_epoch_valid'],
            false_hold=row['false_hold'],hard_safety_count=len(row['hard_safety']),
            budget_terminated=row.get('budget_terminated'),g500=row['g500'],
            g500_reasons=' | '.join(row.get('g500_failure_reasons',[])),
            formal_transport=None if row['execution']!='COMPLETED' else (row.get('phase_response') or {}).get('transport_frame') is not None,
            seconds=row.get('seconds'),endpoint_mm=None if row.get('endpoint_m') is None else 1000*row['endpoint_m'],
            max_true_rotation_deg=row.get('maximum_absolute_rotation_deg'),
            acquisition_scaling=scaling.get('acquisition'),post_acquisition_scaling=scaling.get('post_acquisition'),
            support_audit_verified=audit.get('verified'),support_full_episode_verified=audit.get('full_episode_verified'),
            audited_frames=audit.get('frames'),audited_agent_steps=audit.get('agent_steps'),
            support_applied=audit.get('applied'),admitted_hard_violations=audit.get('hard_violations'),
            admitted_capped_object_violations=audit.get('capped_object_violations'),
            admitted_uncapped_object_violations=audit.get('uncapped_object_violations'),
            min_uncapped_object_residual=audit.get('minimum_uncapped_object_residual')))
    assert len(rows)==8
    with (PUBLIC/'all_eight.csv').open('x',encoding='utf-8',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    dump(PUBLIC/'gates.json',dict(mechanism=report['mechanism_gates'],full=report['full_gates'],
        mechanism_verdict=report['mechanism_gate'],full_verdict=report['full_gate'],candidate_adopted=False))
    dump(PUBLIC/'original_reproduction.json',report['original_reproduction'])
    dump(PUBLIC/'engineering_checks.json',dict(tests=plan['tests'],canonical_sha256=plan['canonical_sha256'],
        runtime_source_sha256=plan['frozen_source_sha256'],physical_episodes=8,
        environment=plan['environment'],installed_locked_packages=plan['installed_locked_packages'],
        source_files=plan['source_files'],scripts=plan['scripts'],preflight=plan['preflight']))
    dump(PUBLIC/'audit_summary.json',dict(
        candidate_support_audits={r['condition']:r.get('post_support_audit') for r in public_report['episodes'] if r['arm']==ARMS[1]},
        pair_audits={r['case_id']:r.get('pair_audit') for r in report['episodes']},
        bridge_audits={r['case_id']:r.get('bridge_audit') for r in report['episodes']},
        raw_analysis_sha256=sha(BASE/'analysis.json'),new_physical_episodes=8,
        scope='All outcomes retained; detailed offending frame/agent residuals remain in the hashed raw analysis.'))
    shutil.copy2(BASE/'execution_outcome.json',PUBLIC/'execution_outcome.json')
    shutil.copy2(BASE/'analysis_adapter_v1/outcome.json',PUBLIC/'analysis_adapter_receipt.json')

    fig,axes=plt.subplots(2,2,figsize=(12,8))
    lookup={(r['condition'],r['arm']):r for r in rows}
    specs=[('post_acquisition_scaling','Post-acquisition scaled agent steps','Count'),
        ('max_true_rotation_deg','Maximum actual rotation','Degrees'),
        ('endpoint_mm','Final task error','mm'),('seconds','Simulation duration','Seconds')]
    for ax,(key,title,unit) in zip(axes.flat,specs):
        values_by_arm=[[lookup[c,arm][key] for c in plan['conditions']] for arm in ARMS]
        largest=max((v for group in values_by_arm for v in group if v is not None),default=0.)
        for j,values in enumerate(values_by_arm):
            locations=np.arange(4)+(j-.5)*.36
            ax.bar(locations,[np.nan if v is None else v for v in values],.36,
                color=['#777777','#0072B2'][j],label=['Map-bridge control','Finite-support candidate'][j])
            for x,value in zip(locations,values):
                label='NA' if value is None else (f'{value:.0f}' if 'scaling' in key else f'{value:.2f}')
                ax.annotate(label,(x,0 if value is None else value),xytext=(0,5),textcoords='offset points',ha='center',fontsize=8,rotation=30)
        ax.set(xticks=np.arange(4),xticklabels=['Near circle','Near L','Far C','Far skew'],title=title,ylabel=unit,
            xlim=(-.6,3.6),ylim=(0,max(1,largest*1.35)))
        ax.spines[['top','right']].set_visible(False)
        ax.set_axisbelow(True)
        ax.grid(axis='y',alpha=.2)
    fig.suptitle('Current finite support: fixed-parameter paired development study',fontsize=14)
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.955),ncol=2,frameon=False)
    completed=sum(r['execution']=='COMPLETED' for r in rows)
    fig.text(.5,.025,f"8 scheduled episodes; {completed} completed execution. All outcomes retained; unavailable metrics: NA.\n"
        f"Mechanism: {report['mechanism_gate']}; full chain: {report['full_gate']}. Candidate remains default-off.",
        ha='center',fontsize=10)
    fig.subplots_adjust(top=.85,bottom=.13,hspace=.4,wspace=.3)
    fig.savefig(PUBLIC/'comparison.png',dpi=180)
    fig.savefig(PUBLIC/'comparison.pdf')
    plt.close(fig)
    dump(PUBLIC/'publication.json',dict(raw_analysis_sha256=sha(BASE/'analysis.json'),
        protocol_sha256=sha(PROTOCOL),publisher_sha256=sha(__file__),new_physical_episodes=8,
        products={path.name:sha(path) for path in sorted(PUBLIC.iterdir()) if path.is_file()},
        scope='All eight matched development outcomes. No generalization or independent deployment-benefit claim.'))
    print(f'Published all8 episodes; mechanism={report["mechanism_gate"]}, full={report["full_gate"]}',flush=True)


if __name__=='__main__':
    main()
