"""Freeze the authorized nine-episode contact recovery/reinforcement comparison."""
from __future__ import annotations
import argparse,copy,json,sys
from pathlib import Path
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import source_hashes,environment_versions,cached_result,verify_source_snapshot
from analyze_dynamic_contact_study import read,sha

ROOT=Path(__file__).resolve().parents[1]
MISSION='2026-11-01-contact-reinforcement-v1'
HISTORY=ROOT/'runs/research/2026-11-01-feedback-components-v1/run'
ARMS=('reference','recovery','reinforcement')
CONDITIONS=('circle','ellipse_parallel','c_open_away')


def freeze(dry_run=False):
    if Path.cwd().resolve()!=ROOT or Path(sys.prefix).resolve()!=ROOT/'.venv':raise RuntimeError('Wrong root/environment')
    target=ROOT/'experiments/ccta2027/contact_reinforcement_v1.json'
    if target.exists():raise RuntimeError('Already frozen; preserve it')
    audit=ROOT/'artifacts/migrations'/MISSION;check=ROOT/'runs/maintenance'/MISSION
    for name in ('baseline','diagnosis','targeted-03','after'):
        if (check/f'{name}.exit.txt').read_text(encoding='utf-8-sig').strip()!='0':raise RuntimeError('Check failed: '+name)
    old=read(HISTORY/'resolved_protocol.json');manifest=read(HISTORY/'manifest.json')
    verify_source_snapshot(HISTORY,manifest['source_files'])
    current=source_hashes()
    changed={n for n in set(current)|set(manifest['source_files']) if current.get(n)!=manifest['source_files'].get(n)}
    expected={'src/dbact/control/controller.py','src/dbact/control/phase.py',
              'src/dbact/control/contact_reinforcement.py','src/dbact_sim/experiments/deployment.py'}
    if changed!=expected:raise RuntimeError('Unplanned runtime changes: '+str(changed))
    inputs={};controls=[];cases=[]
    for condition in CONDITIONS:
        original=next(c for c in old['cases'] if c['condition_id']==condition and c['arm']=='reference_cvt')
        folder=HISTORY/original['case_id'];receipt=read(folder/'case_result.json');cached_result(folder,receipt['identity'])
        inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
        inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
        controls.append(dict(condition=condition,path=folder.relative_to(ROOT).as_posix(),identity=receipt['identity']))
        for arm in ARMS:
            c=copy.deepcopy(original);c.update(case_id=f'{condition}_{arm}',arm=arm,split='development',frames=12000)
            if arm!='reference':c['config']['controller']['contact_recovery_fix']=True
            if arm=='reinforcement':c['config']['controller']['contact_reinforcement']=True
            cases.append(c)
        group=cases[-3:];configs=[]
        for c in group:
            cfg=copy.deepcopy(c['config']);cfg['controller'].pop('contact_recovery_fix',None);cfg['controller'].pop('contact_reinforcement',None)
            configs.append(encode(cfg))
            assert c['config']['dt']==.05 and c['config']['controller']['navigation_estimator']=='observed_reference'
        assert len(set(configs))==1 and len({c['expected_initial_state_sha256'] for c in group})==1
    diagnosis=ROOT/'artifacts/research'/MISSION/'diagnosis/diagnosis.json'
    design=ROOT/'docs/CONTACT_REINFORCEMENT_RESEARCH_2026-11-01.md'
    plan=dict(version='contact-reinforcement-v1',status='FROZEN_BEFORE_EXECUTION',cases=cases,
        conditions=[c for c in old['conditions'] if c['condition'] in CONDITIONS],arms=list(ARMS),
        run_dir=f'runs/research/{MISSION}/run',historical_controls=controls,historical_inputs=inputs,
        budget=read(audit/'design_before_diagnosis.json'),information=copy.deepcopy(old['information']),
        evaluation=copy.deepcopy(old['evaluation']),source_changes=sorted(changed),
        frozen_source_sha256=encode(current),environment=environment_versions(),
        diagnosis=dict(path=diagnosis.relative_to(ROOT).as_posix(),sha256=sha(diagnosis)),
        design=dict(path=design.relative_to(ROOT).as_posix(),sha256=sha(design)),freeze_script_sha256=sha(Path(__file__)),
        mechanism={
            'reference':'Unmodified both-reference vector CVT. Not the historical scalar CVT or spacing.',
            'recovery':'While suspended, original quorum/dwell use locally prospective geometric push eligibility instead of disabled action flags. No new contact-force assumption.',
            'reinforcement':'Recovery plus retain useful incumbent patches (suppress tangential CVT at these patches), original pressure law, spare-first observed local-route recruitment. No donor release, no new pressure allocation.'},
        fixed_policy=dict(attempts_per_robot=3,route_radius_voxels=3,adjacent_normal_min_dot=.5,
            path_limit='2 * local_radius',travel_timeout='2 * local_radius / max_speed',
            join_timeout='2 * travel_timeout',dwell='original contact_dwell * dt',
            message_max_age='2 * dt',new_target_alignment='original push_side_threshold',
            incumbent_release_alignment='original push_share_floor',
            speed_response='original transport_deadband * transport_reference_speed'),
        preserved='Root environment, observed-reference fitting/admission, physical model, no attachment, all safety limits, task tolerance, settle/stop criteria and manuscript/title remain unchanged. Existing team supervisor remains explicit; not fully decentralized.',
        comparison_scope='One reused development seed; three preselected failures, nine new episodes, no significance or reliability claim. Reference must reproduce all 25 historical replay arrays and old step fields. No tuning or additional scientific episodes.')
    plan['evaluation']['gate']='Mechanism may continue only if at least one original failure becomes completed without any hard safety violation in the candidate arm, reference checks pass, and logs show the mechanism was used. Report every endpoint and all regressions. No default promotion; no post-result tuning.'
    plan['evaluation']['statistics']='Paired condition-level deltas, all 9 launched episodes in denominator. Completion, hard safety, G500, off-target settling and 600 s censoring separate; no frame-level independence or population confidence interval.'
    if dry_run:print(json.dumps(dict(cases=len(cases),source_changes=sorted(changed))));return
    dump(target,plan);load_protocol(target,'development',ROOT)
    dump(audit/'freeze_receipt.json',dict(protocol=target.relative_to(ROOT).as_posix(),sha256=sha(target)))
    print(json.dumps(dict(cases=len(cases),protocol=str(target),sha256=sha(target))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true')
    freeze(p.parse_args().dry_run)
