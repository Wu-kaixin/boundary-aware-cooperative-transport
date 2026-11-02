"""Freeze one 12-episode development batch; never rewrite an existing protocol."""
import argparse, copy, json, sys
from pathlib import Path
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import source_hashes, environment_versions, cached_result, verify_source_snapshot
from dbact_sim.experiments.protocol import load_protocol
from analyze_dynamic_contact_study import read, sha

ROOT=Path(__file__).resolve().parents[1]
MISSION='2026-11-02-contact-geometry-v1'
HISTORY=ROOT/'runs/research/2026-11-01-contact-reinforcement-v1/run'
ARMS=('reference','radial','patch','combined')
CONDITIONS=('circle','ellipse_parallel','c_open_away')


def freeze(dry_run=False):
    assert Path.cwd().resolve()==ROOT and Path(sys.prefix).resolve()==ROOT/'.venv'
    target=ROOT/'experiments/ccta2027/contact_geometry_v1.json'
    if target.exists():raise RuntimeError('Already frozen; preserve protocol')
    check=ROOT/'runs/maintenance'/MISSION
    for name in ('baseline','diagnosis-02','readout-screen','targeted-02','after'):
        if (check/f'{name}.exit.txt').read_text(encoding='utf-8-sig').strip()!='0':
            raise RuntimeError('Unsuccessful prerequisite: '+name)
    old=read(HISTORY/'resolved_protocol.json');manifest=read(HISTORY/'manifest.json')
    verify_source_snapshot(HISTORY,manifest['source_files']);current=source_hashes()
    changed={n for n in set(current)|set(manifest['source_files']) if current.get(n)!=manifest['source_files'].get(n)}
    expected={'src/dbact/control/controller.py','src/dbact/control/observed_reference.py',
              'src/dbact/control/observed_geometry.py','src/dbact_sim/experiments/deployment.py'}
    if changed!=expected:raise RuntimeError('Unexpected runtime delta: '+str(changed))
    inputs={};cases=[];controls=[]
    for condition in CONDITIONS:
        original=next(c for c in old['cases'] if c['condition_id']==condition and c['arm']=='reference')
        folder=HISTORY/original['case_id'];receipt=read(folder/'case_result.json');cached_result(folder,receipt['identity'])
        inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
        inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
        controls.append(dict(condition=condition,path=folder.relative_to(ROOT).as_posix(),identity=receipt['identity']))
        for arm in ARMS:
            case=copy.deepcopy(original);case.update(case_id=f'{condition}_{arm}',arm=arm,split='development',frames=12000)
            cfg=case['config']['controller'];cfg['reference_radial_pivot']=arm in ('radial','combined')
            cfg['contact_patch_distance']=arm in ('patch','combined')
            assert not cfg.get('contact_recovery_fix',False) and not cfg.get('contact_reinforcement',False)
            cases.append(case)
        configs=[]
        for case in cases[-4:]:
            cfg=copy.deepcopy(case['config']);cfg['controller'].pop('reference_radial_pivot');cfg['controller'].pop('contact_patch_distance')
            configs.append(encode(cfg));assert case['seed']==2 and cfg['dt']==.05
        assert len(set(configs))==1 and len({c['expected_initial_state_sha256'] for c in cases[-4:]})==1
    design=ROOT/'docs/CONTACT_GEOMETRY_RESEARCH_2026-11-02.md'
    screens=[ROOT/'artifacts/research'/MISSION/p/'summary.json' for p in ('diagnosis-02','readout-screen')]
    plan=dict(version='contact-geometry-v1',status='FROZEN_BEFORE_EXECUTION',cases=cases,
        conditions=[c for c in old['conditions'] if c['condition'] in CONDITIONS],arms=list(ARMS),
        run_dir=f'runs/research/{MISSION}/run',historical_controls=controls,historical_inputs=inputs,
        information=copy.deepcopy(old['information']),evaluation=copy.deepcopy(old['evaluation']),
        budget=dict(episodes=12,conditions=3,arms=4,seed=2,frames=12000,physical_seconds=600,
                    workers=8,internal_threads=1,wall_seconds=3600,additional_episodes=0),
        source_changes=sorted(changed),frozen_source_sha256=encode(current),environment=environment_versions(),
        design=dict(path=design.relative_to(ROOT).as_posix(),sha256=sha(design)),
        offline_screens={p.relative_to(ROOT).as_posix():sha(p) for p in screens},freeze_script_sha256=sha(Path(__file__)),
        mechanism=dict(reference='Both-channel observed-reference vector CVT, no recovery/reinforcement',
            radial='Observed radial bootstrap pivot; readout point and consistent velocity only',
            patch='Own-scan nominal patch distance only; original normal and QP geometry',
            combined='Both independent factors'),
        comparison_scope='Three reused development failures, not independent validation or broad superiority',
        preserved='Physics, sensing budget, pressure gains, safety, arrival, settling and G500 unchanged. Simulation only, no attachment, manuscript/title unchanged.')
    plan['evaluation']['gate']='At least one original failure becomes completed without candidate hard safety violations; exact reference reproduction and mechanism use required. No default promotion, retuning or extra episodes.'
    plan['evaluation']['statistics']='Paired condition-level differences; all started episodes in denominators; no population inference from frames. Report complete, cutoff, settling and G500 separately.'
    if dry_run:print(json.dumps(dict(cases=len(cases),changed=sorted(changed))));return
    dump(target,plan);load_protocol(target,'development',ROOT)
    dump(ROOT/'artifacts/migrations'/MISSION/'freeze_receipt.json',dict(protocol=target.relative_to(ROOT).as_posix(),sha256=sha(target)))
    print(json.dumps(dict(protocol=str(target),sha256=sha(target),cases=len(cases))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');freeze(p.parse_args().dry_run)
