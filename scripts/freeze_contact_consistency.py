"""Freeze one 18-case coordinate-consistency comparison before execution."""
import argparse,copy,json,sys
from pathlib import Path
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.study import source_hashes,environment_versions,cached_result,verify_source_snapshot
from dbact_sim.experiments.protocol import load_protocol
from audit_contact_consistency import ROOT,MISSION,OLD,read,sha

ARMS=('legacy_aggregate','consistent_aggregate')

def freeze(dry_run=False):
    assert Path.cwd().resolve()==ROOT and Path(sys.prefix).resolve()==ROOT/'.venv'
    dest=ROOT/'experiments/ccta2027/contact_consistency_v1.json'
    if dest.exists():raise RuntimeError('Protocol already frozen')
    check=ROOT/'runs/maintenance'/MISSION
    for stage in ('baseline','targeted2','after','shadow-c','shadow-circle','screen'):
        assert (check/f'{stage}.exit.txt').read_text(encoding='utf-8-sig').strip()=='0',stage
    manifest=read(OLD/'manifest.json');verify_source_snapshot(OLD,manifest['source_files'])
    current=source_hashes()
    changed={p for p in set(current)|set(manifest['source_files']) if current.get(p)!=manifest['source_files'].get(p)}
    assert changed=={'src/dbact/control/controller.py','src/dbact/safety/filter.py','src/dbact_sim/experiments/deployment.py'},changed
    previous=read(ROOT/'experiments/ccta2027/feedback_components_v1.json')
    geometric=read(OLD/'resolved_protocol.json');cases=[];controls=[];inputs={}
    for meta in previous['conditions']:
        condition=meta['condition']
        base=next(c for c in previous['cases'] if c['condition_id']==condition and c['arm']=='reference_cvt')
        for arm in ARMS:
            c=copy.deepcopy(base);c.update(case_id=condition+'_'+arm,arm=arm,split='development',frames=12000)
            cfg=c['config']['controller'];cfg.update(reference_radial_pivot=True,contact_patch_distance=True,
                aggregate_frame_consistent=arm=='consistent_aggregate')
            assert not cfg.get('contact_recovery_fix',False) and not cfg.get('contact_reinforcement',False)
            cases.append(c)
        left,right=[copy.deepcopy(c['config']) for c in cases[-2:]]
        left['controller'].pop('aggregate_frame_consistent');right['controller'].pop('aggregate_frame_consistent')
        assert encode(left)==encode(right)
        old=next((c for c in geometric['cases'] if c['condition_id']==condition and c['arm']=='combined'),None)
        if old:
            assert encode(left)==encode(old['config']),condition
            folder=OLD/old['case_id'];receipt=read(folder/'case_result.json');cached_result(folder,receipt['identity'])
            controls.append(dict(condition=condition,path=folder.relative_to(ROOT).as_posix(),identity=receipt['identity']))
            inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
            inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
    assert len(cases)==18 and len(controls)==3
    design=ROOT/'docs/CONTACT_CONSISTENCY_RESEARCH_2026-11-02.md'
    plan=dict(version='contact-consistency-v1',status='FROZEN_BEFORE_EXECUTION',cases=cases,
        conditions=previous['conditions'],arms=list(ARMS),run_dir=f'runs/research/{MISSION}/run',
        historical_controls=controls,historical_inputs=inputs,information=previous['information'],
        budget=dict(episodes=18,conditions=9,arms=2,seed=2,frames=12000,physical_seconds=600,
                    workers=8,internal_threads=1,wall_seconds=5400,additional_episodes=0,versions=1),
        evaluation=dict(completion='Original HOLD + physical speed<=.005 for40frames and endpoint error<=.12m',
            safety='Original separation/penetration/solver checks, G500 and scaling separately retained',
            censoring='600s unfinished is observation cutoff, never successful completion time',
            gate='At least one failure rescued, net completion count increase and zero candidate hard safety failures. Report all lost successes and all G500 changes. No automatic default promotion.',
            statistics='Paired condition differences on one reused development seed; no general probability estimate',
            error_policy='Every planned/started task remains in denominator; no retuning or added cases'),
        frozen_source_sha256=encode(current),environment=environment_versions(),source_changes=sorted(changed),
        design=dict(path=design.relative_to(ROOT).as_posix(),sha256=sha(design)),
        offline_screen_sha256=sha(ROOT/'artifacts/research'/MISSION/'screen.json'),
        freeze_script_sha256=sha(Path(__file__)),
        comparison_scope='Both arms use common geometry readouts; legacy arm is not original scalar CVT or spacing',
        preserved='No truth inputs, new sensing, attachment, force/stop changes, safety threshold changes, physics/evaluator changes, manuscript changes, push or tag changes')
    if dry_run:print(json.dumps(dict(cases=len(cases),source_changes=sorted(changed),historical_repeats=len(controls))));return
    dump(dest,plan);load_protocol(dest,'development',ROOT)
    dump(ROOT/'artifacts/migrations'/MISSION/'freeze_receipt.json',dict(protocol=dest.relative_to(ROOT).as_posix(),sha256=sha(dest)))
    state=read(ROOT/'artifacts/migrations'/MISSION/'mission_state.json');state.update(stage='FROZEN',protocol_sha256=sha(dest))
    dump(ROOT/'artifacts/migrations'/MISSION/'mission_state.json',state)
    print(json.dumps(dict(cases=len(cases),protocol=str(dest),sha256=sha(dest))))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');freeze(p.parse_args().dry_run)
