"""Fixed-budget voter-continuity ablation; original evidence is read-only."""
from __future__ import annotations

import argparse
import copy
import gc
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys

import benchmark_workstation as monitor  # Freeze native thread limits before NumPy.
import numpy as np
from validate_transport_prior import guard, read, sha, initial_metadata, ROOT
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import source_hashes, environment_versions, cached_result, verify_source_snapshot
from dbact_sim.experiments.protocol import load_protocol

MISSION = '2026-11-02-voter-continuity-v1'
AUDIT = ROOT / 'artifacts/migrations' / MISSION
OUT = ROOT / 'runs/maintenance' / MISSION
PUBLIC = ROOT / 'docs/results/voter_continuity_v1'
PROTOCOL = ROOT / 'experiments/ccta2027/voter_continuity_v1.json'
OLD_PROTOCOL = ROOT / 'experiments/ccta2027/transport_validation_v1_new_starts.json'
CONDITIONS = ('near_circle', 'near_ellipse_parallel', 'near_ellipse_perpendicular',
              'near_rectangle_parallel', 'near_l_reference', 'near_c_open_away',
              'far_c_open_away_pos0', 'far_skew_box_s0_r0_pos0')
ARMS = {'reference': (False, 1), 'continuity': (True, 1),
        'confirmation': (False, 20), 'combined': (True, 20)}


def event(action, **values):
    with (AUDIT / 'operations.jsonl').open('a', encoding='utf-8') as f:
        f.write(json.dumps(dict(action=action, **values)) + '\n')


def command(label, args, seconds=120):
    monitor.OUT, monitor.AUDIT = OUT, AUDIT
    result = monitor.run_command(label, args, seconds, os.environ.copy())
    dump(OUT / (label + '-result.json'), result)
    event('command', label=label, result=result)
    if result['exit_code'] != 0 or result['status'] != 'COMPLETED':
        raise SystemExit(result['exit_code'] or 1)


def prepare():
    guard()
    if (AUDIT/'mission_state.json').exists() or (AUDIT/'design.json').exists():
        raise RuntimeError('Existing mission: continue, do not reinitialize')
    assert shutil.disk_usage(ROOT).free > 60 * 1024**3
    for path in (AUDIT, OUT):
        path.mkdir(parents=True, exist_ok=True)
    originals = ['src/dbact/control/controller.py', 'src/dbact/control/boundary_navigation.py',
                 'src/dbact_sim/experiments/deployment.py', 'README.md', 'docs/DATA_INDEX.md',
                 'docs/results/README.md', 'experiments/ccta2027/README.md']
    protected = {}
    for name in originals:
        dst = AUDIT / 'backup' / name
        dst.parent.mkdir(parents=True, exist_ok=True)
        if not dst.exists():
            shutil.copy2(ROOT / name, dst)
        protected[name] = sha(ROOT / name)
        assert sha(dst) == protected[name]
    for name, args in [('worktree.patch', ['diff', '--binary', '--no-ext-diff', '--no-textconv']),
                       ('index.patch', ['diff', '--cached', '--binary', '--no-ext-diff', '--no-textconv']),
                       ('refs.txt', ['show-ref']), ('status.txt', ['status', '--porcelain=v1', '-uall'])]:
        (AUDIT / 'backup' / name).write_bytes(subprocess.check_output(['git', '--no-optional-locks', *args]))
    state = dict(mission=MISSION, status='IN_PROGRESS', stage='PREPARED',
        head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        index_sha256=sha(ROOT / '.git/index'), protected_originals=protected,
        source_before=source_hashes(), environment=environment_versions(),
        manuscript={str(p.relative_to(ROOT)): sha(p) for p in (ROOT/'manuscript/ccta2027').rglob('*') if p.is_file()})
    dump(AUDIT / 'mission_state.json', state)
    design = dict(mission=MISSION, conditions=CONDITIONS, arms=ARMS,
        hypothesis='Voter-set jumps can cause single-frame false HOLD; removing roster offsets and/or confirming arrival may reduce it, without correcting common or prior bias.',
        reference='Previous development candidate max_object_speed=0.45, observed_reference; all other settings unchanged. This is not the global default 0.25.',
        design='2x2 factorial: membership continuity on/off, arrival dwell 1/20 frames. 20 uses the existing contact_dwell duration (1s), chosen before runs, not fitted to outcomes.',
        continuity='Per-receiver common fresh voters connect adjacent medians; accumulated offset removes only roster change. No overlap: abstain and reanchor a new segment; log discontinuity. No missing message is reused as fresh.',
        confirmation='Only irreversible shared HOLD needs consecutive original arrival+speed+quality guards; braking and reversible local unloading remain available. Original independent evaluator unchanged.',
        pairing='8 already-consumed development conditions, seed20261102, exactly identical initial state within four arms. No independent generalization inference.',
        budget=dict(episodes=32, physical_seconds=600, workers=8, internal_threads=1, wall_seconds=7200),
        primary='Safe completion of all scheduled episodes; false HOLD (true error > existing task tolerance at first HOLD).',
        secondary=['G500 and reasons', 'Endpoint error', '600s censoring', 'Hard safety and barrier scaling',
                   'Roster vs common-readout vs true-motion decomposition', 'Raw vs bridged same-input bias', 'Reanchoring and availability'],
        adoption_gate='Combined must rescue diagnostic far skew, lose no previously safe completion, add no hard safety case, and not reduce total G500. Otherwise remain experimental; report every arm regardless.',
        stopping='One run per scheduled arm. No retuning, replacement seed or scientific retry. Resource/identity faults stop affected work, preserving partial output.',
        restrictions=['Simulation only', 'No attachment or truth in control', 'No safety/success/physics changes', 'Paper title unchanged'])
    dump(AUDIT / 'design.json', design)
    (AUDIT / 'CONTINUE.md').write_text('Design fixed; run baseline checks, audit old logs, implement and test, then freeze source and execute 32 cases.\n', encoding='utf-8')
    event('prepare', design_sha256=sha(AUDIT/'design.json'))
    print(json.dumps(design['budget']))


def checks(label):
    args = [sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
            '--basetemp', str(OUT/(label+'-temp')), '--junitxml', str(OUT/(label+'.xml'))]
    if label == 'baseline':
        args += ['tests/test_boundary_navigation.py', 'tests/test_observed_reference.py',
                 'tests/test_contact_scan_permissions.py', 'tests/test_aggregate_frame_consistency.py']
    command(label, args, 1200)


def freeze():
    guard()
    if PROTOCOL.exists() or (AUDIT/'freeze_receipt.json').exists():
        raise RuntimeError('Never refreeze an existing protocol')
    assert read(OUT/'checks-result.json')['exit_code'] == 0
    old = read(OLD_PROTOCOL)
    cases, origins = [], {}
    for condition in CONDITIONS:
        origin = next(c for c in old['cases'] if c['condition_id']==condition and c['variant']=='boundary_cvt' and c['arm']=='candidate')
        folder = ROOT / old['run_dir'] / origin['case_id']
        receipt = read(folder/'case_result.json')
        cached_result(folder, receipt['identity'])
        assert read(folder.parent/'manifest.json')['identity']['environment'] == environment_versions()
        meta = initial_metadata(origin['config'], origin['seed'])
        assert meta['agent_initial_sha256'] == origin['expected_initial_state_sha256']
        origins[condition] = dict(path=str(folder.relative_to(ROOT)), receipt_sha256=sha(folder/'case_result.json'), initial=meta)
        assert origin['config']['controller']['max_object_speed'] == .45
        assert origin['config']['controller']['contact_dwell'] == 20
        for arm, (continuity, dwell) in ARMS.items():
            case = copy.deepcopy(origin)
            case.update(case_id=condition+'_'+arm, arm=arm, split='development', frames=12000)
            case['config']['controller'].update(navigation_voter_continuity=continuity, navigation_hold_dwell=dwell)
            case['factors'].update(cohort='voter_continuity_development', historical_source_case=origin['case_id'])
            assert initial_metadata(case['config'], case['seed']) == meta
            cases.append(case)
    random.Random(2026110202).shuffle(cases)
    plan = dict(version='voter-continuity-v1', status='FROZEN_BEFORE_EXECUTION', cases=cases,
        run_dir=f'runs/research/{MISSION}/ablation', frozen_source_sha256=encode(source_hashes()),
        environment=environment_versions(), design_sha256=sha(AUDIT/'design.json'))
    dump(PROTOCOL, plan)
    load_protocol(PROTOCOL, 'development', ROOT)
    dump(AUDIT/'freeze_receipt.json', dict(protocol_sha256=sha(PROTOCOL), source_files=source_hashes(),
         environment=environment_versions(), origins=origins, harness_sha256=sha(__file__)))
    state=read(AUDIT/'mission_state.json'); state.update(stage='FROZEN')
    dump(AUDIT/'mission_state.json',state)
    event('freeze', protocol_sha256=sha(PROTOCOL), episodes=len(cases))
    print(json.dumps(dict(episodes=len(cases), source=plan['frozen_source_sha256'])))


def run(dry_run=False, resume=False):
    freeze_record=read(AUDIT/'freeze_receipt.json')
    assert sha(PROTOCOL)==freeze_record['protocol_sha256']
    assert source_hashes()==freeze_record['source_files']
    assert environment_versions()==freeze_record['environment']
    path=ROOT/read(PROTOCOL)['run_dir']
    if path.exists() and not resume and not dry_run:
        raise RuntimeError('Output exists: inspect before same-identity resume')
    args=[sys.executable,'-m','dbact_sim','study','--protocol',str(PROTOCOL),
          '--split','development','--workers','8','--out',str(path)]
    if dry_run: args+=['--dry-run']
    if resume: args+=['--resume']
    label='dry-run' if dry_run else 'resume' if resume else 'ablation'
    command(label,args,120 if dry_run else 7200)
    event('execution_finished', stage=label)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['prepare','baseline','checks','freeze','run'])
    parser.add_argument('--dry-run',action='store_true')
    parser.add_argument('--resume',action='store_true')
    a=parser.parse_args()
    if a.action=='prepare': prepare()
    elif a.action in ('baseline','checks'): checks(a.action)
    elif a.action=='freeze': freeze()
    else: run(a.dry_run,a.resume)
