"""Publish the complete selected-state contact-compatibility diagnostic."""
from __future__ import annotations

import csv
from collections import Counter

from research_io import ROOT,guard,read,sha
from dbact_sim.experiments.deployment import dump


BASE=ROOT/'runs/research/2026-11-03-post-support-contact-compatibility-v2'
PUBLIC=ROOT/'docs/results/post_support_contact_compatibility_v1'


def main():
    guard()
    outcome=read(BASE/'outcome.json')
    assert outcome['status']=='VERIFIED_CONTACT_COMPATIBILITY' and outcome['conditions']==4
    assert sha(BASE/'analysis.json')==outcome['analysis_sha256']
    report=read(BASE/'analysis.json')
    assert report['new_physical_episodes']==0 and len(report['results'])==4
    rows=[]
    counts=Counter()
    events=[]
    for condition in report['results']:
        assert not condition['missing']
        for event in condition['events']:
            assert len(event['states'])==16
            counts['events']+=1
            events.append(dict(condition=condition['condition'],frame=event['saved_frame'],
                rules=event['selection_rules'],phase=event['phase'],positive_force_sum=event['positive_force_sum']))
            for state in event['states']:
                row=dict(condition=condition['condition'],frame=event['saved_frame'],
                    selection_rules=' | '.join(event['selection_rules']),phase=event['phase'],
                    agent_id=state['agent_id'],positive_physical_force=state['positive_physical_force'],
                    declared_pusher=state['declared_pusher'],admitted=state['support']['applied'],
                    admission_reason=state['support']['reason'],
                    original_goal_velocity=state['original_goal_velocity'],
                    counterfactual_goal_velocity=state['counterfactual_goal_velocity'],
                    counterfactual_status=state['counterfactual_status'],
                    counterfactual_barrier_scale=state['counterfactual_barrier_scale'],
                    capped_object_residual=state['original_command_min_capped_object_residual'],
                    uncapped_object_residual=state['original_command_min_uncapped_object_residual'])
                rows.append(row)
                counts['robot_states']+=1
                counts['admitted']+=int(row['admitted'])
                if row['positive_physical_force']>0:
                    counts['positive_force_states']+=1
                    counts['positive_force_admitted']+=int(row['admitted'])
                    counts['positive_force_capped_conflicts']+=int(row['capped_object_residual'] is not None and row['capped_object_residual'] < -1e-7)
                    counts['positive_force_uncapped_conflicts']+=int(row['uncapped_object_residual'] is not None and row['uncapped_object_residual'] < -1e-7)
                    counts['positive_force_goal_command_reversed']+=int(row['counterfactual_goal_velocity']<0)
                    if row['declared_pusher']:
                        counts['positive_force_declared_pushers']+=1
                        counts['positive_force_declared_pushers_reversed']+=int(row['counterfactual_goal_velocity']<0)
    assert counts['events']==8 and len(rows)==128 and counts['positive_force_states']==17
    assert counts['positive_force_declared_pushers']==counts['positive_force_declared_pushers_reversed']==11
    PUBLIC.mkdir(parents=True,exist_ok=False)
    for name,subset in (('all_128_states.csv',rows),('all_17_positive_force_states.csv',
            [row for row in rows if row['positive_physical_force']>0])):
        with (PUBLIC/name).open('x',encoding='utf-8',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(subset)
    dump(PUBLIC/'summary.json',dict(new_physical_episodes=0,verified_replay_frames=outcome['verified_frames'],
        conditions=4,counts=dict(counts),events=events,
        inference_limit='Counterfactual robot commands at selected old-control force events do not establish candidate closed-loop forces or outcomes.'))
    dump(PUBLIC/'provenance.json',dict(raw_directory=BASE.relative_to(ROOT).as_posix(),
        raw_analysis_sha256=outcome['analysis_sha256'],runtime_source_sha256=report['runtime_source_sha256'],
        protocol_sha256=report['protocol_sha256'],script_sha256=report['script_sha256'],
        design_sha256=report['design_sha256'],serialization_helper_sha256=report['serialization_helper_sha256'],
        control_receipts={item['case']['condition_id']:item['receipt_sha256'] for item in report['schedule']},
        failed_first_attempt='runs/research/2026-11-03-post-support-contact-compatibility-v1',
        failed_first_attempt_reason='Exact first-control replay completed, then ndarray output serialization failed; original attempt preserved; v2 only changes output conversion.',
        new_physical_episodes=0))
    dump(PUBLIC/'publication.json',dict(publisher_sha256=sha(__file__),new_physical_episodes=0,
        products={path.name:sha(path) for path in sorted(PUBLIC.iterdir()) if path.is_file()}))
    print(dict(counts),flush=True)


if __name__=='__main__':
    main()
