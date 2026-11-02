"""Publish the complete paired physical pilot without promoting its controller."""
from __future__ import annotations

import csv
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

import task_translation_physical_study_v2 as study
from research_io import guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result


def main():
    guard()
    plan = study.check_freeze()
    public = study.PUBLIC
    analysis = read(public/'analysis.json')
    assert sha(public/'analysis.json') == sha(study.BASE/'analysis.json')
    assert analysis['protocol_sha256'] == sha(study.PROTOCOL)
    assert len(analysis['episodes']) == 8 and analysis['candidate_adopted'] is False
    manifest = read(study.OUT/'manifest.json')
    diagnostic = read(public/'barrier_diagnostic.json')
    assert diagnostic['analysis_sha256'] == sha(public/'analysis.json')
    outputs = ('all_eight_episodes.csv','paired_outcomes.png','paired_outcomes.pdf','publication.json',
               'execution_outcome.json','superseded_v1_outcome.json')
    if any((public/n).exists() for n in outputs):
        raise RuntimeError('Preserve existing publication')
    data = {}
    for case in plan['cases']:
        folder = study.OUT/case['case_id']
        cached_result(folder, {**manifest['identity'],'case_sha256':encode(case)})
        row = next(r for r in analysis['episodes'] if r['case_id']==case['case_id'])
        assert row['receipt_sha256'] == sha(folder/'case_result.json')
        data[(case['condition_id'],case['arm'])] = row
    records = []
    for condition in study.CONDITIONS:
        for arm in study.ARMS:
            row = data[(condition,arm)]
            first = row.get('first_hold') or {}
            est = row.get('estimation') or {}
            records.append(dict(condition=condition, arm=arm, execution=row['execution'],
                task_completed=row['completed'], safe_completed=row['safe_completed'], g500=row['g500'],
                false_hold=row['false_hold'], hard_safety_failures=';'.join(row['hard_safety']),
                endpoint_error_m=row.get('endpoint_m'), termination_seconds=row.get('seconds'),
                budget_terminated=row.get('budget_terminated'), barrier_scalings=row.get('barrier_scalings'),
                first_hold_true_error_m=first.get('physical_error_before_m'),
                first_hold_nominal_error_m=first.get('nominal_error_m'),
                valid_zero_epoch=row['zero_epoch_valid'] if arm!='reference' else 'not_applicable',
                maximum_absolute_rotation_deg=row.get('maximum_absolute_rotation_deg'),
                **{'estimation_'+k:est.get(k) for k in ('post_ready_frames','available_frames','quorum_frames','mae_m','p95_m','max_m')},
                g500_failure_reasons='; '.join(row.get('g500_failure_reasons',[]))))
    with (public/'all_eight_episodes.csv').open('x',newline='',encoding='utf-8') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(records[0]))
        writer.writeheader(); writer.writerows(records)
    fig, axes = plt.subplots(1,3,figsize=(12.5,4.3))
    fig.subplots_adjust(left=.06,right=.99,bottom=.25,top=.86,wspace=.30)
    x,width = np.arange(4),.36
    for a,(arm,label,color) in enumerate(zip(study.ARMS,('Protected reference','Translation + all captured'),('#777777','#0072b2'))):
        values = [data[(condition,arm)] for condition in study.CONDITIONS]
        for ax,key,multiplier in zip(axes,('endpoint_m','seconds','barrier_scalings'),(1000,1,1)):
            bars = ax.bar(x+(a-.5)*width,[v[key]*multiplier for v in values],width,color=color,label=label)
            for bar,row in zip(bars,values):
                if not row['completed']: bar.set_hatch('///'); bar.set_edgecolor('#8b0000')
            ax.bar_label(bars,fmt='%.1f' if key!='barrier_scalings' else '%g',fontsize=7,padding=3)
    axes[0].axhline(120,color='#b2182b',linestyle='--',linewidth=1,label='Task tolerance: 120 mm')
    axes[0].set_ylabel('Final true goal error (mm)'); axes[0].set_ylim(0,160)
    axes[1].set_ylabel('Termination time (s)'); axes[1].set_ylim(0,105)
    axes[2].set_ylabel('Scaled-barrier robot-steps'); axes[2].set_ylim(0,280)
    for ax in axes:
        ax.set_xticks(x,['Near circle','Near L','Far C','Far skew'])
        ax.grid(axis='y',alpha=.2); ax.set_axisbelow(True)
        ax.spines[['top','right']].set_visible(False)
    handles,labels = axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='lower center',ncols=3,fontsize=9,frameon=False,bbox_to_anchor=(.5,.07))
    fig.text(.5,.025,'Eight fresh episodes on four reused conditions. Hatched bar: failed task; time is termination, not successful completion.',ha='center',fontsize=8)
    fig.suptitle('Stopping improves in this pilot; far-field G500 still fails',fontsize=12)
    for suffix in ('png','pdf'): fig.savefig(public/f'paired_outcomes.{suffix}',dpi=180)
    plt.close(fig)
    shutil.copy2(study.BASE/'execution_outcome.json',public/'execution_outcome.json')
    old_outcome = study.ROOT/'runs/research/2026-11-03-task-translation-physical-v1/preexecution_outcome.json'
    shutil.copy2(old_outcome,public/'superseded_v1_outcome.json')
    dump(public/'publication.json',dict(script_sha256=sha(__file__),analysis_sha256=sha(public/'analysis.json'),
        protocol_sha256=sha(study.PROTOCOL),barrier_diagnostic_sha256=sha(public/'barrier_diagnostic.json'),
        outputs={n:sha(public/n) for n in outputs if n!='publication.json'},new_physical_episodes=8,
        candidate_adopted=False, scope='Complete paired pilot, unchanged gates; not a general reliability estimate or deployment superiority claim'))
    print(dict(rows=len(records),advance_gate=analysis['advance_gate'],candidate_adopted=False),flush=True)


if __name__ == '__main__':
    main()
