"""A bounded follow-up on reference initialization; no historical output edits."""
from __future__ import annotations
import argparse
import copy
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import benchmark_workstation as monitor
from validate_transport_prior import ROOT, guard, read, sha, initial_metadata
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import source_hashes, environment_versions, cached_result
from dbact_sim.experiments.protocol import load_protocol

MISSION='2026-11-02-reference-origin-v1'
AUDIT=ROOT/'artifacts/migrations'/MISSION
OUT=ROOT/'runs/maintenance'/MISSION
PUBLIC=ROOT/'docs/results/reference_origin_v1'
PROTOCOL=ROOT/'experiments/ccta2027/reference_origin_v1.json'
OLD_PROTOCOL=ROOT/'experiments/ccta2027/voter_continuity_v1.json'
CONDITIONS=('near_circle','near_ellipse_parallel','near_ellipse_perpendicular','near_rectangle_parallel',
            'near_l_reference','near_c_open_away','far_c_open_away_pos0','far_skew_box_s0_r0_pos0')
ARMS={'reference':(False,None),'early_reference':(True,None),'unscaled_bootstrap':(False,1.)}


def event(action,**values):
    with (AUDIT/'operations.jsonl').open('a',encoding='utf-8') as f:
        f.write(json.dumps(dict(action=action,**values))+'\n')


def command(label,args,seconds=120):
    monitor.OUT,monitor.AUDIT=OUT,AUDIT
    result=monitor.run_command(label,args,seconds,os.environ.copy())
    dump(OUT/(label+'-result.json'),result);event('command',label=label,result=result)
    if result['exit_code']!=0 or result['status']!='COMPLETED':raise SystemExit(result['exit_code'] or 1)


def prepare():
    guard()
    if AUDIT.exists():raise RuntimeError('Existing mission; inspect and continue')
    assert encode(source_hashes())==read(ROOT/'artifacts/migrations/2026-11-02-voter-continuity-v1/final_integrity.json')['source_sha256']
    assert shutil.disk_usage(ROOT).free>60*1024**3
    for p in (AUDIT,OUT):p.mkdir(parents=True,exist_ok=False)
    originals=['src/dbact/control/controller.py','src/dbact/control/observed_reference.py',
               'src/dbact_sim/experiments/deployment.py','README.md','docs/DATA_INDEX.md',
               'docs/results/README.md','experiments/ccta2027/README.md']
    protected={}
    for name in originals:
        dst=AUDIT/'backup'/name;dst.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,dst);protected[name]=sha(ROOT/name);assert sha(dst)==protected[name]
    for name,args in [('worktree.patch',['diff','--binary','--no-ext-diff','--no-textconv']),
                      ('index.patch',['diff','--cached','--binary','--no-ext-diff','--no-textconv']),
                      ('refs.txt',['show-ref']),('status.txt',['status','--porcelain=v1','-uall'])]:
        (AUDIT/'backup'/name).write_bytes(subprocess.check_output(['git','--no-optional-locks',*args]))
    state=dict(mission=MISSION,status='IN_PROGRESS',stage='AUDIT_PRIOR_LOGS',
        source_before=source_hashes(),environment=environment_versions(),protected_originals=protected,
        head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        index_sha256=sha(ROOT/'.git/index'),
        manuscript={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'manuscript/ccta2027').rglob('*') if p.is_file()})
    dump(AUDIT/'mission_state.json',state)
    (AUDIT/'CONTINUE.md').write_text('Prepared and protected. Audit original eight reference trajectories before choosing/fixing the intervention. No new simulation yet.\n',encoding='utf-8')
    event('prepare',source_sha256=encode(state['source_before']))
    print(json.dumps(dict(mission=MISSION,status=state['stage'])))


def checks(label):
    args=[sys.executable,'-m','pytest','-q','-p','no:cacheprovider',
          '--basetemp',str(OUT/(label+'-temp')),'--junitxml',str(OUT/(label+'.xml'))]
    if label=='baseline':args+=['tests/test_observed_reference.py','tests/test_boundary_navigation.py',
                                'tests/test_voter_continuity.py','tests/test_contact_scan_permissions.py']
    command(label,args,1200)


def freeze():
    guard()
    if PROTOCOL.exists() or (AUDIT/'freeze_receipt.json').exists():raise RuntimeError('Cannot refreeze')
    assert read(OUT/'checks-result.json')['exit_code']==0
    design=read(AUDIT/'design.json')
    old=read(OLD_PROTOCOL);cases=[];origins={}
    for condition in CONDITIONS:
        origin=next(c for c in old['cases'] if c['condition_id']==condition and c['arm']=='reference')
        folder=ROOT/old['run_dir']/origin['case_id'];r=read(folder/'case_result.json')
        cached_result(folder,r['identity'])
        assert read(folder.parent/'manifest.json')['identity']['environment']==environment_versions()
        initial=initial_metadata(origin['config'],origin['seed'])
        assert initial['agent_initial_sha256']==origin['expected_initial_state_sha256']
        origins[condition]=dict(path=str(folder.relative_to(ROOT)),receipt_sha256=sha(folder/'case_result.json'),initial=initial)
        for arm,(early,gain) in ARMS.items():
            case=copy.deepcopy(origin)
            case.update(case_id=condition+'_'+arm,arm=arm,split='development',frames=12000)
            case['config']['controller']['reference_early_anchor']=early
            case['config']['controller']['reference_bootstrap_gain']=gain
            assert case['config']['controller']['navigation_voter_continuity'] is False
            assert case['config']['controller']['navigation_hold_dwell']==1
            assert case['config']['controller']['max_object_speed']==.45
            case['factors'].update(cohort='reference_origin_development',historical_source_case=origin['case_id'])
            assert initial_metadata(case['config'],case['seed'])==initial
            cases.append(case)
    random.Random(2026110203).shuffle(cases)
    plan=dict(version='reference-origin-v1',status='FROZEN_BEFORE_EXECUTION',cases=cases,
        run_dir=f'runs/research/{MISSION}/comparison',design_sha256=sha(AUDIT/'design.json'),
        frozen_source_sha256=encode(source_hashes()),environment=environment_versions())
    dump(PROTOCOL,plan);load_protocol(PROTOCOL,'development',ROOT)
    dump(AUDIT/'freeze_receipt.json',dict(protocol_sha256=sha(PROTOCOL),source_files=source_hashes(),
        environment=environment_versions(),origins=origins,script_sha256=sha(__file__)))
    state=read(AUDIT/'mission_state.json');state.update(stage='FROZEN',budget=design['budget'])
    dump(AUDIT/'mission_state.json',state);event('freeze',episodes=len(cases),source=plan['frozen_source_sha256'])
    print(json.dumps(dict(episodes=len(cases),source=plan['frozen_source_sha256'])))


def run(dry_run=False,resume=False):
    f=read(AUDIT/'freeze_receipt.json');plan=read(PROTOCOL)
    assert sha(PROTOCOL)==f['protocol_sha256'] and source_hashes()==f['source_files']
    assert environment_versions()==f['environment']
    dest=ROOT/plan['run_dir']
    if dest.exists() and not(dry_run or resume):raise RuntimeError('Output exists; inspect for identity-safe resume')
    args=[sys.executable,'-m','dbact_sim','study','--protocol',str(PROTOCOL),
          '--split','development','--workers','4','--out',str(dest)]
    if dry_run:args+=['--dry-run']
    if resume:args+=['--resume']
    label='dry-run' if dry_run else 'resume' if resume else 'comparison'
    state=read(AUDIT/'mission_state.json');state.update(stage='RUNNING_'+label.upper());dump(AUDIT/'mission_state.json',state)
    command(label,args,120 if dry_run else 7200)
    state.update(stage=label.upper()+'_FINISHED');dump(AUDIT/'mission_state.json',state)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['prepare','baseline','checks','freeze','run'])
    parser.add_argument('--dry-run',action='store_true');parser.add_argument('--resume',action='store_true')
    a=parser.parse_args()
    if a.action=='prepare':prepare()
    elif a.action in ('baseline','checks'):checks(a.action)
    elif a.action=='freeze':freeze()
    else:run(a.dry_run,a.resume)
