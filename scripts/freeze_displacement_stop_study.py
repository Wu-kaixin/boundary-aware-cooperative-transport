"""Freeze one displacement readout comparison only after its offline gate passes."""
from __future__ import annotations
import argparse
import copy
import json
import sys
from pathlib import Path
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import cached_result, source_hashes, environment_versions
from audit_displacement_replay import ROOT, MISSION, HISTORY, read, sha

ARMS={'cvt':('boundary_cvt','legacy','accumulated'),
      'spacing':('local_spacing','legacy','accumulated'),
      'vector_cvt':('boundary_cvt','vector_goal','accumulated'),
      'reference_cvt':('boundary_cvt','vector_goal','observed_reference')}


def freeze(dry_run=False):
    if Path(sys.prefix).resolve()!=(ROOT/'.venv').resolve():raise RuntimeError('Use root .venv')
    target=ROOT/'experiments/ccta2027/displacement_stop_v1.json'
    if target.exists():raise RuntimeError('Protocol exists; never refreeze')
    gate_path=ROOT/'artifacts/research'/MISSION/'offline-reference/offline_gate.json'
    gate=read(gate_path)
    if gate['status']!='PASS':raise RuntimeError('Failed same-input gate; stop this candidate')
    module='src/dbact/control/observed_reference.py'
    gate_sources={name.replace('\\','/'):digest for name,digest in gate['source_sha256'].items()}
    if sha(ROOT/module)!=gate_sources[module]:raise RuntimeError('Estimator changed after offline gate')
    old=read(HISTORY/'resolved_protocol.json')
    originals=[c for c in old['cases'] if c['arm']=='cvt']
    cases=[];inputs={}
    for original in originals:
        for arm in ('cvt','spacing','vector_cvt'):
            folder=HISTORY/f"{original['condition_id']}_{arm}"
            receipt=read(folder/'case_result.json');cached_result(folder,receipt['identity'])
            inputs[(folder/'case_result.json').relative_to(ROOT).as_posix()]=sha(folder/'case_result.json')
            inputs.update({(folder/p).relative_to(ROOT).as_posix():h for p,h in receipt['products'].items()})
        for arm,(variant,mode,estimator) in ARMS.items():
            c=copy.deepcopy(original)
            c.update(case_id=f"{c['condition_id']}_{arm}",arm=arm,variant=variant,frames=12000,split='development')
            c['config']['controller'].update(boundary_navigation=mode,navigation_estimator=estimator)
            cases.append(c)
    assert len(cases)==36
    for name in {c['condition_id'] for c in cases}:
        group=[c for c in cases if c['condition_id']==name]
        configs=[]
        for c in group:
            cfg=copy.deepcopy(c['config']);cfg['controller'].pop('boundary_navigation');cfg['controller'].pop('navigation_estimator')
            configs.append(encode(cfg))
        assert len(set(configs))==1
        assert len({c['expected_initial_state_sha256'] for c in group})==1
    plan=dict(version='displacement-stop-v1',status='FROZEN_BEFORE_EXECUTION',
        cases=cases,conditions=old['conditions'],arms=list(ARMS),run_dir=f'runs/research/{MISSION}/run',
        budget=dict(episodes=36,seed=2,physical_seconds=600,report_horizons=[150,600],workers=8,
                    internal_threads=1,wall_limit_seconds=3600,versions=1),
        evaluation=old['evaluation'],information=dict(
            local='Fixed reference from own/one-hop boundary returns, bootstrapped at CONTACT_READY from the existing displacement estimate. Metric SE(2) readout thereafter; no physical pose/contact truth.',
            shared='Existing all-team vector median drives the existing common phase supervisor; not fully decentralized.',
            safety='Original map, object velocity, QP, contact physics, thresholds and independent stopping/completion unchanged.',
            abstention='Initialization has no measured velocity vote; failed/missing reference matches do not send a fresh navigation vote.',
            limitations='Persistent observed centroid is not known center of mass; bootstrap bias remains; no absolute-error certificate.'),
        candidate='Replace vector-CVT displacement/transport-velocity messages only. Same deployment, compressive control, common phase gates, 0.02m estimated arrival tube; no truth-based HOLD.',
        historical_inputs=inputs,offline_gate=dict(path=gate_path.relative_to(ROOT).as_posix(),sha256=sha(gate_path)),
        frozen_source_sha256=encode(source_hashes()),environment=environment_versions(),freeze_script_sha256=sha(Path(__file__)))
    plan['evaluation']=copy.deepcopy(plan['evaluation'])
    plan['evaluation']['comparisons']='Four methods paired within nine reused development conditions; previous vector-CVT isolates readout change. Every failure/error retained.'
    plan['evaluation']['gate']='Primary mechanistic outcome: C-away reaches and settles under the original completion criterion with no hard safety violation. Broader development support requires a net completion gain vs both original controls, at most one lost completion and no new hard safety failure. No default promotion or reliability claim on this reused seed.'
    if dry_run:print(json.dumps({'episodes':len(cases),'source':plan['frozen_source_sha256']}));return
    dump(target,plan);load_protocol(target,'development',ROOT)
    dump(ROOT/'artifacts/migrations'/MISSION/'freeze_receipt.json',{'protocol':target.relative_to(ROOT).as_posix(),'sha256':sha(target)})
    print(f'Frozen {len(cases)} episodes: {target}')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true')
    freeze(p.parse_args().dry_run)
