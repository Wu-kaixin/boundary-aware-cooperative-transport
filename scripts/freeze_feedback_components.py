"""Freeze a finite common-admission 2x2 readout experiment; never refreeze."""
from __future__ import annotations
import argparse
import copy
import json
import sys
from pathlib import Path
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import source_hashes, environment_versions, cached_result, verify_source_snapshot
from analyze_dynamic_contact_study import sha,read

ROOT=Path(__file__).resolve().parents[1]
MISSION='2026-11-01-feedback-components-v1'
HISTORY=ROOT/'runs/research/2026-11-01-displacement-stop-v1/run'
ARMS={'gate_cvt':'observed_reference_gate','position_cvt':'observed_reference_position',
      'velocity_cvt':'observed_reference_velocity','reference_cvt':'observed_reference'}


def freeze(dry_run=False):
    if Path.cwd().resolve()!=ROOT or Path(sys.prefix).resolve()!=ROOT/'.venv':raise RuntimeError('Wrong root/environment')
    target=ROOT/'experiments/ccta2027/feedback_components_v1.json'
    if target.exists():raise RuntimeError('Already frozen; no overwrite or refreeze')
    audit=ROOT/'artifacts/migrations'/MISSION;check=ROOT/'runs/maintenance'/MISSION
    for name in ('baseline','components','after','log-diagnosis-02'):
        if (check/f'{name}.exit.txt').read_text(encoding='utf-8-sig').strip()!='0':raise RuntimeError('Required check failed: '+name)
    old=read(HISTORY/'resolved_protocol.json');manifest=read(HISTORY/'manifest.json')
    verify_source_snapshot(HISTORY,manifest['source_files'])
    current=source_hashes()
    changed={name for name in set(current)|set(manifest['source_files']) if current.get(name)!=manifest['source_files'].get(name)}
    if changed!={'src/dbact/control/controller.py','src/dbact_sim/experiments/deployment.py'}:
        raise RuntimeError('Unplanned runtime changes: '+str(changed))
    if sha(ROOT/'src/dbact/control/observed_reference.py')!=manifest['source_files']['src/dbact/control/observed_reference.py']:
        raise RuntimeError('Reference estimator changed')
    inputs={};controls=[]
    for c in old['cases']:
        folder=HISTORY/c['case_id'];receipt=read(folder/'case_result.json');cached_result(folder,receipt['identity'])
        inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
        inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
        controls.append(dict(case=c,path=folder.relative_to(ROOT).as_posix(),identity=receipt['identity'],
            role='repeat-verification' if c['arm']=='reference_cvt' else 'historical-task-reference'))
    cases=[]
    for original in [c for c in old['cases'] if c['arm']=='reference_cvt']:
        for arm,mode in ARMS.items():
            c=copy.deepcopy(original);c.update(case_id=f"{c['condition_id']}_{arm}",arm=arm,split='development',frames=12000)
            c['config']['controller']['navigation_estimator']=mode
            cases.append(c)
    for condition in {c['condition_id'] for c in cases}:
        group=[c for c in cases if c['condition_id']==condition];configs=[]
        for c in group:
            cfg=copy.deepcopy(c['config']);cfg['controller'].pop('navigation_estimator');configs.append(encode(cfg))
        assert len(set(configs))==1 and len({c['expected_initial_state_sha256'] for c in group})==1
    diagnosis=ROOT/'artifacts/research'/MISSION/'log-diagnosis-02/diagnosis.json'
    plan=dict(version='feedback-components-v1',status='FROZEN_BEFORE_EXECUTION',cases=cases,conditions=old['conditions'],
        arms=list(ARMS),run_dir=f'runs/research/{MISSION}/run',historical_controls=controls,historical_inputs=inputs,
        budget=dict(new_episodes=36,historical_task_reference_episodes=27,repeated_reference_checks=9,
                    seed=2,physical_seconds=600,report_horizons=[150,600],workers=8,internal_threads=1,wall_limit_seconds=3600,versions=1),
        information=copy.deepcopy(old['information']),evaluation=copy.deepcopy(old['evaluation']),
        source_changes=sorted(changed),frozen_source_sha256=encode(current),environment=environment_versions(),
        diagnosis=dict(path=diagnosis.relative_to(ROOT).as_posix(),sha256=sha(diagnosis)),freeze_script_sha256=sha(Path(__file__)),
        channel_design={'gate_cvt':['old map position','old map velocity'],
            'position_cvt':['reference position','old map velocity'],'velocity_cvt':['old map position','reference velocity'],
            'reference_cvt':['reference position','reference velocity']},
        common_admission='Every arm sends only fresh accepted reference agents after CONTACT_READY. Old values on this gate are explicitly diagnostic and do not retain original map admission. Before CONTACT_READY every arm retains original map feedback. No stale fallback.',
        comparison_scope='New common-admission 2x2 arms isolate readout channels within this intervention. Original CVT/spacing/vector-CVT use their separately frozen historical source and admission; do not relabel them as newly executed. Repeated both-reference cases must exactly reproduce prior trajectories and historical fields.',
        preserved='No estimator fitting/gain, no thresholds, no contact-role/physics/safety/stopping change. Position affects direction/arrival; velocity affects convoy/effort and shared slow guard. This study does not separate velocity consumers.')
    plan['evaluation']['gate']='Primary: C-away original completion plus no hard safety limit failure. Report all nine conditions; describe endpoint changes and G500 separately. No default promotion on reused seed. No tuning, extra seed, extended horizon, or follow-on candidate inside this mission.'
    plan['evaluation']['factorial']='Per condition report Y10-Y00, Y11-Y01 for position, Y01-Y00, Y11-Y10 for velocity, and Y11-Y10-Y01+Y00 interaction. Completion, endpoint error, off-target settling and hard safety separate. No frame-based p-values or population confidence intervals.'
    if dry_run:print(json.dumps(dict(cases=len(cases),changed=sorted(changed))));return
    dump(target,plan);load_protocol(target,'development',ROOT)
    dump(audit/'freeze_receipt.json',dict(protocol=target.relative_to(ROOT).as_posix(),sha256=sha(target)))
    print(json.dumps(dict(cases=len(cases),protocol=str(target),sha256=sha(target))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true')
    freeze(p.parse_args().dry_run)
