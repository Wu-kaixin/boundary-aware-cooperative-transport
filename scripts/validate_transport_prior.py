"""Fixed-budget replication and new-start validation of the frozen 0.45 prior.

Uses the existing study runner, safety analysis and process monitor. No controller
defaults are modified. Historical cases retain their own execution identities.
"""
from __future__ import annotations

import argparse
import copy
import difflib
import gc
import hashlib
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys

import benchmark_workstation as monitor  # Set thread limits before numpy.
import numpy as np
import psutil
from dbact.geometry import signed_distance_to_polygon
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import (
    cached_result, environment_versions, source_hashes, verify_source_snapshot,
)

ROOT = Path(__file__).resolve().parents[1]
MISSION = '2026-11-02-transport-validation-v1'
AUDIT = ROOT / 'artifacts/migrations' / MISSION
OUT = ROOT / 'runs/maintenance' / MISSION
ANALYSIS = ROOT / 'artifacts/research' / MISSION
SEED = 20261102
METHODS = ('boundary_cvt', 'local_spacing', 'nearest_observed_target')
SETTINGS = {'reference': .25, 'candidate': .45}
NEW_NEAR = ('circle', 'ellipse_parallel', 'ellipse_perpendicular',
            'rectangle_parallel', 'l_reference', 'c_open_away',
            'u_tray_s0_r90', 'skew_box_s1_r90')
NEW_FAR = ('circle_pos0', 'skew_box_s0_r0_pos0', 'c_open_away_pos0', 'c_open_away_pos1')
OLD = ROOT / 'runs/research/2026-11-02-cargo-discovery-v1/near'
TUNING = ROOT / 'artifacts/migrations/2026-11-02-transport-tuning-v1'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def guard():
    assert Path.cwd().resolve() == ROOT
    assert Path(sys.prefix).resolve() == ROOT / '.venv'


def event(operation, **values):
    with (AUDIT / 'operations.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(dict(operation=operation, **values)) + '\n')


def prepare():
    guard()
    if AUDIT.exists():
        raise RuntimeError('Existing mission; inspect and continue without reinitializing')
    assert shutil.disk_usage(ROOT).free > 60 * 1024**3
    assert source_hashes() == read(TUNING / 'design.json')['source_sha256']
    for folder in (AUDIT, OUT, ANALYSIS):
        folder.mkdir(parents=True, exist_ok=False)
    originals = ['README.md', 'docs/DATA_INDEX.md', 'docs/results/README.md',
                 'experiments/ccta2027/README.md']
    protected = {}
    for name in originals:
        dest = AUDIT / 'backup' / name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / name, dest)
        protected[name] = sha(ROOT / name)
        assert sha(dest) == protected[name]
    for name, args in [('worktree.patch', ['diff', '--no-ext-diff', '--no-textconv', '--binary']),
                       ('index.patch', ['diff', '--no-ext-diff', '--no-textconv', '--cached', '--binary']),
                       ('status.txt', ['status', '--porcelain=v1', '--untracked-files=all']),
                       ('refs.txt', ['show-ref'])]:
        (AUDIT / 'backup' / name).write_bytes(subprocess.check_output(['git', '--no-optional-locks', *args], cwd=ROOT))
    state = dict(mission=MISSION, status='IN_PROGRESS', stage='PREPARED',
        head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        index_sha256=sha(ROOT / '.git/index'), protected_originals=protected,
        source_files=source_hashes(), environment=environment_versions(),
        manuscript_sha256={p.relative_to(ROOT).as_posix(): sha(p)
            for p in (ROOT / 'manuscript/ccta2027').rglob('*') if p.is_file()},
        resources=dict(physical_cores=psutil.cpu_count(logical=False), logical_cores=psutil.cpu_count(),
            affinity=psutil.Process().cpu_affinity(), memory=psutil.virtual_memory()._asdict(),
            disk=shutil.disk_usage(ROOT)._asdict()),
        constraints=['Simulation only', 'No new truth or attachment', 'Original success and safety thresholds',
            'No retuning or selective retries', 'No defaults, manuscript, dependencies or Git publication changes'])
    dump(AUDIT / 'mission_state.json', state)
    (AUDIT / 'CONTINUE.md').write_text('Prepared only. Freeze both protocols before any science run.\n', encoding='utf-8')
    event('prepare', protected=protected)
    print(json.dumps(dict(mission=MISSION, source=encode(source_hashes()))), flush=True)


def history_record(folder, case, row, analysis_file):
    receipt = read(folder / 'case_result.json')
    cached_result(folder, receipt['identity'])  # Every raw member, not only summary.
    assert read(folder / 'resolved_case.json') == case
    return dict(folder=folder.relative_to(ROOT).as_posix(), case=case, row=row,
        receipt_sha256=sha(folder / 'case_result.json'), analysis_file=str(analysis_file.relative_to(ROOT)),
        analysis_sha256=sha(analysis_file), identity=receipt['identity'])


def make_case(origin, setting, method, cohort, seed):
    case = copy.deepcopy(origin)
    condition = origin['stratum'] + '_' + origin['condition_id']
    case.update(condition_id=condition, case_id=condition + '_' + method + '_' + setting,
                seed=seed, split='validation', variant=method, arm=setting, frames=12000)
    case['config']['controller']['max_object_speed'] = SETTINGS[setting]
    case['factors'].update(cohort=cohort, origin_case=origin['case_id'], parameter_setting=setting,
                         original_parameter=.25, candidate_parameter=.45)
    return case


def initial_metadata(config, seed):
    env = SimulationEnvironment(copy.deepcopy(config), seed=seed)
    positions = np.array([a.position for a in env.agents])
    cargo = env.cargoes[0]
    distance = np.linalg.norm(positions[:, None] - positions[None, :], axis=-1)
    np.fill_diagonal(distance, np.inf)
    assert distance.min() >= config['controller']['d_min']
    clearance = signed_distance_to_polygon(positions, cargo.vertices)
    assert clearance.min() > config['controller']['robot_radius']
    returns = [len(env.controller.sensor.sense_view(a, env.cargoes, 0.)) for a in env.agents]
    task = env.tasks[cargo.object_id].as_dict()
    return dict(agent_initial_sha256=encode(positions.tolist()),
        physical_initial_sha256=encode(dict(robots=positions.tolist(), vertices=cargo.vertices.tolist(), task=task)),
        initial_own_scan_returns=returns, minimum_initial_boundary_distance_m=float(clearance.min()),
        minimum_initial_pair_distance_m=float(distance.min()), initial_centroid=cargo.position.tolist(), task=task)


def freeze(dry_run=False):
    guard()
    if (AUDIT / 'design.json').exists():
        raise RuntimeError('Frozen design exists: do not refreeze')
    state = read(AUDIT / 'mission_state.json')
    assert source_hashes() == state['source_files']
    near = read(ROOT / 'experiments/ccta2027/cargo_near_v1.json')['cases']
    far = read(ROOT / 'experiments/ccta2027/cargo_far_v1.json')['cases']
    baseline = [c for c in near if c['variant'] == 'boundary_cvt']
    assert len(baseline) == 33
    # Historical implementation differs only by the reviewed trace-off fix;
    # transport_arm always enables trace, and eight exact repeats are preserved.
    old_manifest = read(OLD / 'manifest.json')
    verify_source_snapshot(OLD, old_manifest['source_files'])
    changed = [p for p, h in old_manifest['source_files'].items() if state['source_files'].get(p) != h]
    assert changed == ['src/dbact/control/controller.py']
    old_controller = (OLD / 'source_snapshot/src/dbact/control/controller.py').read_text()
    current_controller = (ROOT / 'src/dbact/control/controller.py').read_text()
    expected = old_controller.replace('        if self.trace_enabled:\n            # A robot\'s own scan,',
        '        if self.trace_enabled or self.params.contact_patch_distance:\n            # A robot\'s own scan,', 1)
    expected = expected.replace('            self.last_scans = {agents[i].agent_id: scans[i] for i in range(len(agents))}',
        '            # The contact-distance controller also consumes these observations;\n'
        '            # enabling diagnostics must never change its information or commands.\n'
        '            # Store empty scans on a non-sensing frame rather than reuse stale data.\n'
        '            self.last_scans = {agents[i].agent_id: scans[i] for i in range(len(agents))}', 1)
    assert expected == current_controller
    assert 'env.controller.trace_enabled = True' in (ROOT / 'src/dbact_sim/experiments/deployment.py').read_text()
    old_analysis_file = ROOT / 'docs/results/cargo_discovery_v1/near_analysis.json'
    old_rows = read(old_analysis_file)['episodes']
    history = []
    for case in baseline:
        row = next(r for r in old_rows if r['case_path'] == (OLD / case['case_id']).relative_to(ROOT).as_posix())
        history.append(dict(setting='reference', condition=case['condition_id'],
            **history_record(OLD / case['case_id'], case, row, old_analysis_file)))
    reuse = {}
    for stage in ('screen', 'transfer'):
        af = ROOT / f'docs/results/transport_tuning_v1/{stage}_analysis.json'
        analysis = read(af)
        if stage == 'screen':
            assert len(analysis['exact_reference_reproduction']) == 8
            assert all(v['replay_exact'] and v['fields_exact'] for v in analysis['exact_reference_reproduction'].values())
        for row in analysis['episodes']:
            if row['arm'] != 'wider_motion_prior' or row['method'] != 'boundary_cvt' or not row['condition'].startswith('near_'):
                continue
            folder = ROOT / row['case_path']
            c = read(folder / 'resolved_case.json')
            name = row['condition'].removeprefix('near_')
            ref = next(x for x in baseline if x['condition_id'] == name)
            expected_config = copy.deepcopy(ref['config'])
            expected_config['controller']['max_object_speed'] = .45
            assert c['config'] == expected_config and c['seed'] == ref['seed'] and c['frames'] == ref['frames']
            assert c['expected_initial_state_sha256'] == ref['expected_initial_state_sha256']
            assert read(folder / 'case_result.json')['identity']['source_sha256'] == encode(state['source_files'])
            reuse[name] = history_record(folder, c, row, af)
            history.append(dict(setting='candidate', condition=name, **reuse[name]))
    assert len(reuse) == 8
    regression = [make_case(c, 'candidate', 'boundary_cvt', 'known_start_regression', 2)
                  for c in baseline if c['condition_id'] not in reuse]
    starts = []
    initial = {}
    for stratum, names, sources in [('near', NEW_NEAR, near), ('far', NEW_FAR, far)]:
        for name in names:
            origin = next(c for c in sources if c['condition_id'] == name and c['variant'] == 'boundary_cvt'
                          and (stratum == 'near' or c['arm'] == 'unknown_location'))
            base = copy.deepcopy(origin)
            if stratum == 'far':
                base['config']['agents'].update(center=[3.75, 1.05], jitter=.005)
            meta = initial_metadata(base['config'], SEED)
            assert meta['agent_initial_sha256'] != origin['expected_initial_state_sha256']
            if stratum == 'far':
                assert sum(meta['initial_own_scan_returns']) == 0
                assert 'experiment_initial_location_hint' not in base['config']
            initial[stratum + '_' + name] = meta
            for method in METHODS:
                for setting in SETTINGS:
                    case = make_case(base, setting, method, 'new_start_validation', SEED)
                    case['factors'].update(meta)
                    case['factors'].update(initial_tokens='empty', validation_seed=SEED,
                        initial_rule='Existing near scatter; far grid translated to [3.75,1.05] with Gaussian jitter sigma=.005m')
                    case['expected_initial_state_sha256'] = meta['agent_initial_sha256']
                    starts.append(case)
    assert len(regression) == 25 and len(starts) == 72
    prior_seeds = set()
    for p in (ROOT / 'experiments/ccta2027').glob('*.json'):
        prior_seeds.update(c['seed'] for c in read(p).get('cases', []) if 'seed' in c)
    assert SEED not in prior_seeds, 'New seed already used in a checked protocol'
    for cases in (regression, starts):
        random.Random(2026110201).shuffle(cases)  # Fixed run order, unrelated to simulation RNG.
    design = dict(mission=MISSION, source_files=state['source_files'], settings=SETTINGS,
        historical_comparisons=history, historical_source_changed=changed,
        historical_equivalence='Only trace-off scan fix; trace always ON in research. Eight exact scientific replays independently retained.',
        new_start_conditions=initial, seed=SEED, previously_declared_seeds=sorted(prior_seeds),
        methods=METHODS, initial_pairing='Identical physical initialization within six-arm condition blocks',
        budget=dict(new_regression=25, reused_regression_reference=33, reused_regression_candidate=8,
            new_start_episodes=72, maximum_new_episodes=97, seconds_per_case=600, workers=8,
            internal_threads=1, regression_wall_seconds=7200, new_starts_wall_seconds=7200),
        primary='Safe task completion, all scheduled starts retained; original task and hard-safety definitions',
        secondary=['G500 pass and every failure reason', 'Endpoint vector error', 'Physical duration and censoring',
                   'Constraint scaling count/rate', 'CONTACT_READY, actual contact, coverage and gap'],
        gates=dict(regression='Safe completions >= original and G500 >= original, zero hard-safety cases',
            new_start_cvt='No hard-safety cases; candidate net safe-completion gain >=1 and G500 not lower',
            method_advantage='At candidate setting CVT >=2 net safe completions over EACH control; all paired metrics reported'),
        statistical_scope='12 designed condition blocks with one fixed new seed, shared random numbers. Conditional out-of-selection initial-state check, not iid arbitrary-shape reliability. Near/far and known/new starts remain separate. No frame-level inference or confirmatory population p-values.',
        stopping='Run the two predeclared stages once, retain all failures. Stop only for identity/execution/resource/time failures; no replacement seeds, parameter mixtures or retries after scientific failure.',
        defaults='Unchanged in this mission even if gates pass',
        script_sha256=sha(__file__))
    if dry_run:
        print(json.dumps(dict(budget=design['budget'], conditions=list(initial))), flush=True)
        return
    assert (OUT / 'checks.exit.txt').read_text().strip() == '0'
    receipts = {}
    for stage, cases in [('regression', regression), ('new_starts', starts)]:
        path = ROOT / f'experiments/ccta2027/transport_validation_v1_{stage}.json'
        if path.exists():
            raise RuntimeError('Protocol exists')
        plan = dict(version='transport-validation-v1-' + stage, status='FROZEN_BEFORE_EXECUTION', cases=cases,
            run_dir=f'runs/research/{MISSION}/{stage}', frozen_source_sha256=encode(state['source_files']),
            environment=state['environment'], budget=design['budget'], design_scope=design['statistical_scope'])
        dump(path, plan)
        load_protocol(path, 'validation', ROOT)
        receipts[stage] = dict(path=path.relative_to(ROOT).as_posix(), sha256=sha(path), cases=len(cases))
    dump(AUDIT / 'design.json', design)
    dump(AUDIT / 'freeze_receipt.json', receipts)
    state.update(stage='BOTH_PROTOCOLS_FROZEN', protocols=receipts, budget=design['budget'])
    dump(AUDIT / 'mission_state.json', state)
    event('freeze', protocols=receipts, budget=design['budget'])
    print(json.dumps(receipts), flush=True)


def run(stage, dry_run=False, resume=False):
    guard()
    state = read(AUDIT / 'mission_state.json')
    receipt = read(AUDIT / 'freeze_receipt.json')[stage]
    protocol = ROOT / receipt['path']
    assert sha(protocol) == receipt['sha256']
    assert source_hashes() == state['source_files']
    assert environment_versions() == state['environment']
    plan = read(protocol)
    command = [sys.executable, '-m', 'dbact_sim', 'study', '--protocol', str(protocol),
               '--split', 'validation', '--workers', '8', '--out', str(ROOT / plan['run_dir'])]
    if dry_run:
        command += ['--dry-run']
    elif resume:
        command += ['--resume']
    elif (ROOT / plan['run_dir']).exists():
        raise RuntimeError('Output exists; inspect before a same-identity resume')
    label = stage + ('-dry-run' if dry_run else '-resume' if resume else '')
    monitor.OUT, monitor.AUDIT = OUT, AUDIT
    state.update(stage='RUNNING_' + label.upper())
    dump(AUDIT / 'mission_state.json', state)
    event('command_start', stage=label, command=command)
    result = monitor.run_command(label, command, 120 if dry_run else 7200, os.environ.copy())
    (OUT / (label + '.exit.txt')).write_text(str(result['exit_code']) + '\n')
    state.update(stage=label.upper() + '_FINISHED', last_execution=result['status'])
    dump(AUDIT / 'mission_state.json', state)
    event('command_finish', stage=label, result=result)
    if result['exit_code']:
        raise SystemExit(result['exit_code'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['prepare', 'freeze', 'run'])
    parser.add_argument('--stage', choices=['regression', 'new_starts'], default='regression')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    if args.action == 'prepare':
        prepare()
    elif args.action == 'freeze':
        freeze(args.dry_run)
    else:
        run(args.stage, args.dry_run, args.resume)
