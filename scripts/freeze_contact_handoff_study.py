"""Freeze the bounded handoff investigation; never overwrite an execution identity."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, source_hashes, environment_versions
from dbact_sim.experiments.protocol import load_protocol

ROOT = Path(__file__).resolve().parents[1]
MISSION = '2026-11-01-contact-handoff-v1'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def freeze(stage, dry_run=False):
    if Path(sys.prefix).resolve() != (ROOT / '.venv').resolve():
        raise RuntimeError('Use the root .venv')
    path = ROOT / f'experiments/ccta2027/contact_handoff_v1_{stage}.json'
    if path.exists():
        raise RuntimeError('Already frozen; resume its identity, do not refreeze')
    historical = ROOT / 'runs/research/2026-10-31-multishape-census-v1/run'
    previous = read(historical / 'resolved_protocol.json')
    originals = [c for c in previous['cases'] if c['variant'] == 'boundary_cvt']
    if stage == 'ablation':
        originals = [c for c in originals if c['condition_id'] in
                     ('circle', 'rectangle_parallel', 'c_open_away')]
        arms = ('feedback_fix', 'allocation_only', 'relocation_only', 'joint_v1')
    else:
        arms = ('legacy', 'spacing', 'handoff') if stage == 'main' else ('legacy', 'handoff')
    frames = 12000 if stage == 'extended' else 3000
    cases, inputs = [], {}
    for original in originals:
        folder = historical / original['case_id']
        receipt = read(folder / 'case_result.json')
        cached_result(folder, receipt['identity'])
        inputs[(folder / 'case_result.json').relative_to(ROOT).as_posix()] = sha(folder / 'case_result.json')
        for name, digest in receipt['products'].items():
            inputs[(folder / name).relative_to(ROOT).as_posix()] = digest
        for arm in arms:
            case = copy.deepcopy(original)
            case.update(case_id=f"{original['condition_id']}_{arm}", arm=arm,
                        split='development', frames=frames)
            controller = case['config']['controller']
            if stage == 'ablation':
                controller.update(transport_safety_feedback=True,
                                  dynamic_contact_roles=arm != 'feedback_fix',
                                  dynamic_contact_allocation=arm != 'relocation_only',
                                  dynamic_contact_relocation=arm != 'allocation_only')
            else:
                controller.update(transport_safety_feedback=False, dynamic_contact_roles=False,
                                  confirmed_contact_handoff=arm == 'handoff')
                case['variant'] = 'local_spacing' if arm == 'spacing' else 'boundary_cvt'
            case['factors']['source_case'] = original['case_id']
            cases.append(case)
    expected = {'ablation':12, 'main':27, 'extended':18}[stage]
    assert len(cases) == expected
    for condition in {c['condition_id'] for c in cases}:
        group = [c for c in cases if c['condition_id'] == condition]
        normalized = []
        for case in group:
            cfg = copy.deepcopy(case['config'])
            for key in ('transport_safety_feedback', 'dynamic_contact_roles', 'dynamic_contact_allocation',
                        'dynamic_contact_relocation', 'confirmed_contact_handoff'):
                cfg['controller'].pop(key, None)
            normalized.append(encode(cfg))
        assert len(set(normalized)) == 1
        assert len({c['expected_initial_state_sha256'] for c in group}) == 1
    plan = {
        'version':f'contact-handoff-v1-{stage}', 'status':'FROZEN_BEFORE_EXECUTION',
        'scope':'One seed-2 development block, no independent reliability/generalization claim',
        'question': ('Separate the effects of relative pressure allocation and relocation' if stage == 'ablation'
                     else 'Does local confirmation with rollback retain useful pushers during contact replacement?'),
        'run_dir':f'runs/research/{MISSION}/{stage}', 'cases':cases, 'arms':list(arms),
        'conditions':previous['conditions'],
        'fixed_budget':{'episodes':expected, 'seed':2, 'frames_per_episode':frames, 'dt':.05,
                        'workers':8, 'internal_threads':1, 'candidate_versions':1,
                        'stop':'Unchanged HOLD + linear speed <=0.005 for 40 frames; resource cutoff otherwise',
                        'completion':'Unchanged settled + final Euclidean error <=0.12 m',
                        'extended_cutoff':'600 s; unfinished at cutoff is right-censored, not eventual impossibility',
                        'no_retry_until_success':True},
        'fairness':{'fixed':'initial positions, sensing, progress estimator, physics, safety/phase/stop thresholds, seed',
                    'changed':'Only declared arm switches and deployment variant; extended changes frame budget for both arms',
                    'unit':'paired geometry/direction condition in one development seed; frames are not replicates'},
        'analysis_rules':{'denominator':'Every planned/launched episode; retain errors, watchdogs and adverse cases',
                          'report':'All paired endpoint errors, completion, G500, settle/censor time, hard safety events, handoff reasons',
                          'compatibility':'Original/spacing replay arrays must match historical controls exactly; ablation references must match v1',
                          'continue_gate':'For main: net >=2 additional completions vs both controls, <=1 lost success vs either, no new hard safety violation; otherwise do not promote',
                          'extended':'Report separately; never use longer candidate runs against 150-s controls or restart a terminated run'},
        'historical_inputs':inputs, 'frozen_source_sha256':encode(source_hashes()),
        'freeze_script_sha256':sha(Path(__file__)), 'environment':environment_versions(),
    }
    if stage != 'ablation':
        design = ROOT / f'experiments/ccta2027/contact_handoff_v1_design.json'
        plan['candidate_design'] = read(design)
        plan['candidate_design_sha256'] = sha(design)
    if dry_run:
        print(json.dumps({'stage':stage, 'episodes':len(cases), 'source':plan['frozen_source_sha256']}))
        return
    dump(path, plan)
    load_protocol(path, 'development', ROOT)
    audit = ROOT / 'artifacts/migrations' / MISSION
    dump(audit / f'{stage}_freeze_receipt.json', {'protocol':str(path.relative_to(ROOT)), 'sha256':sha(path)})
    # Entry scripts/tests are outside the runner's runtime snapshot: protect them separately.
    for source in (Path(__file__), ROOT / 'tests/test_contact_handoff.py'):
        target = audit / 'execution_helpers' / stage / source.name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
    print(f'Frozen {stage}: {path}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('ablation', 'main', 'extended'))
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    freeze(args.stage, args.dry_run)
