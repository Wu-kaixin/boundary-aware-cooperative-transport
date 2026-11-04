"""Freeze and execute a bounded current-source validation; never tune outcomes."""
from __future__ import annotations

import argparse
import copy
import cProfile
import hashlib
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys

import benchmark_workstation as monitor  # Limits internal threads before numpy.
import numpy as np
import yaml
from dbact.geometry import signed_distance_to_polygon
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments import study
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'runs/research/2026-11-04-resource-noise-validation-v1'
PROTOCOL = ROOT / 'experiments/ccta2027/resource_noise_validation_v1.json'
DESIGN = ROOT / 'experiments/ccta2027/resource_noise_validation_v1_design.md'
BEST = ROOT / 'configs/research/best_known_transport_v1.yaml'
SEED = 2026110401
SHAPES = ('circle', 'ellipse_perpendicular', 'rectangle_parallel', 'l_reference', 'c_open_away')
METHODS = ('boundary_cvt', 'local_spacing', 'nearest_observed_target')
REGIMES = {'reference': (16, .01), 'robots8': (8, .01),
           'robots24': (24, .01), 'noise002': (16, .02)}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def guard():
    if Path.cwd().resolve() != ROOT or Path(sys.prefix).resolve() != ROOT / '.venv':
        raise RuntimeError('Run from repository root with its own .venv')


def helpers():
    names = [Path(__file__), DESIGN, ROOT / 'scripts/benchmark_workstation.py',
             ROOT / 'scripts/maintenance/windows_metrics.py']
    return {p.relative_to(ROOT).as_posix(): sha(p) for p in names}


def build_cases():
    source = read(ROOT / 'experiments/ccta2027/cargo_near_v1.json')['cases']
    base = yaml.safe_load(BEST.read_text(encoding='utf-8'))
    cases, initial = [], {}
    for shape in SHAPES:
        origin = next(c for c in source if c['condition_id'] == shape and c['variant'] == 'boundary_cvt')
        positions = {}
        for regime, (count, noise) in REGIMES.items():
            cfg = copy.deepcopy(base)
            cfg['cargoes'] = copy.deepcopy(origin['config']['cargoes'])
            cfg['agents']['count'] = count
            cfg['controller']['range_noise_std'] = noise
            env = SimulationEnvironment(copy.deepcopy(cfg), seed=SEED)
            robots = np.array([a.position for a in env.agents])
            if env.layout_notes:
                raise RuntimeError('Unexpected automatic layout adjustment')
            positions[regime] = robots
            pair = np.linalg.norm(robots[:, None] - robots[None, :], axis=-1)
            np.fill_diagonal(pair, np.inf)
            clearance = signed_distance_to_polygon(robots, env.cargoes[0].vertices)
            assert pair.min() >= cfg['controller']['d_min']
            assert clearance.min() > cfg['controller']['robot_radius']
            task = env.tasks['cargo_0'].as_dict()
            assert task['tolerance'] == .12
            condition = shape + '_' + regime
            meta = dict(condition=condition, shape_condition=shape, regime=regime,
                        robots=count, range_noise_std_m=noise, cohort='prospective_resource_noise',
                        agent_positions=robots.tolist(), agent_initial_sha256=encode(robots.tolist()),
                        task=task, physical_initial_sha256=encode(dict(robots=robots.tolist(),
                            vertices=env.cargoes[0].vertices.tolist(), task=task)),
                        min_initial_boundary_distance_m=float(clearance.min()),
                        min_initial_pair_distance_m=float(pair.min()),
                        area_m2=env.cargoes[0].area, mass_kg=env.cargoes[0].mass)
            initial[condition] = meta
            for method in METHODS:
                cases.append(dict(case_id=condition + '_' + method, condition_id=condition,
                    shape=origin['shape'], stratum='near', split='validation', gate=6,
                    seed=SEED, frames=12000, variant=method, arm=regime,
                    config=copy.deepcopy(cfg), expected_initial_state_sha256=meta['agent_initial_sha256'],
                    factors=copy.deepcopy(meta)))
        assert np.array_equal(positions['robots8'], positions['reference'][:8])
        assert np.array_equal(positions['reference'], positions['robots24'][:16])
        assert np.array_equal(positions['reference'], positions['noise002'])
    for regime in REGIMES:
        assert len({v['agent_initial_sha256'] for v in initial.values() if v['regime'] == regime}) == 1
    assert len(cases) == len({c['case_id'] for c in cases}) == 60
    random.Random(2026110402).shuffle(cases)
    return cases, initial


def freeze():
    if BASE.exists() or PROTOCOL.exists():
        raise RuntimeError('Existing identity: inspect it, never refreeze')
    prior = set()
    for path in (ROOT / 'experiments/ccta2027').glob('*.json'):
        obj = read(path)
        if isinstance(obj, dict):
            prior.update(c['seed'] for c in obj.get('cases', []) if 'seed' in c)
    assert SEED not in prior
    assert shutil.disk_usage(ROOT).free > 100 * 1024**3
    cases, initial = build_cases()
    source = study.source_hashes()
    BASE.mkdir(parents=True)
    (BASE / 'freeze').mkdir()
    (BASE / 'monitor').mkdir()
    plan = dict(version='resource-noise-validation-v1', status='FROZEN_BEFORE_EXECUTION',
        cases=cases, run_dir=(BASE / 'run').relative_to(ROOT).as_posix(),
        frozen_source_sha256=encode(source), environment=study.environment_versions(),
        statistical_scope='One unseen scene seed; descriptive designed shape/regime blocks, no IID inference',
        budget=dict(episodes=60, frames_per_case=12000, dt=.05, wall_seconds=14400),
        source_config_sha256=sha(BEST), design_sha256=sha(DESIGN), initial=initial)
    dump(PROTOCOL, plan)
    _, resolved = load_protocol(PROTOCOL, 'validation', ROOT)
    assert resolved == cases
    engineering = []
    # Fixed before execution: four regimes x two shapes, including 24 robots.
    for shape in ('rectangle_parallel', 'c_open_away'):
        for regime in REGIMES:
            c = copy.deepcopy(next(c for c in cases if c['factors']['shape_condition'] == shape
                              and c['arm'] == regime and c['variant'] == 'boundary_cvt'))
            c.update(frames=100, split='engineering')
            engineering.append(c)
    ep = BASE / 'freeze/engineering_protocol.json'
    dump(ep, dict(version='resource-noise-engineering-v1', cases=engineering,
                 frozen_source_sha256=encode(source), run_dir=str(BASE / 'engineering')))
    receipt = dict(protocol_sha256=sha(PROTOCOL), engineering_protocol_sha256=sha(ep),
        design_sha256=sha(DESIGN), source_files=source, helper_files=helpers(),
        environment=study.environment_versions(), protected_config_sha256=sha(BEST),
        prior_protocol_seed_count=len(prior), seed_absent_from_prior_protocols=True,
        head=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip())
    dump(BASE / 'freeze/receipt.json', receipt)
    for relative in source | helpers():
        target = BASE / 'freeze/source_snapshot' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, target)
    (BASE / 'freeze/packages.json').write_bytes(subprocess.check_output(
        [sys.executable, '-m', 'pip', 'list', '--format=json', '--disable-pip-version-check']))
    print(json.dumps(dict(status='FROZEN', cases=60, source_sha256=encode(source),
                         protocol_sha256=sha(PROTOCOL))), flush=True)


def verify():
    receipt = read(BASE / 'freeze/receipt.json')
    assert sha(PROTOCOL) == receipt['protocol_sha256']
    assert sha(BASE / 'freeze/engineering_protocol.json') == receipt['engineering_protocol_sha256']
    assert study.source_hashes() == receipt['source_files']
    assert helpers() == receipt['helper_files']
    assert study.environment_versions() == receipt['environment']
    assert sha(BEST) == receipt['protected_config_sha256']
    return receipt


def command(protocol, split, workers, out, resume=False):
    args = [sys.executable, '-m', 'dbact_sim', 'study', '--protocol', str(protocol),
            '--split', split, '--workers', str(workers), '--out', str(out)]
    return args + (['--resume'] if resume else [])


def measured(label, args, timeout):
    monitor.OUT, monitor.AUDIT = BASE / 'monitor', BASE / 'freeze'
    result = monitor.run_command(label, args, timeout, os.environ.copy())
    if result['status'] != 'COMPLETED' or result['exit_code'] != 0:
        raise RuntimeError('Execution stopped; preserve partial output: ' + label)
    verify()
    return result


def benchmark():
    verify()
    results = {}
    for workers in (1, 4, 8):
        folder = BASE / f'engineering/w{workers}'
        results[str(workers)] = measured(f'w{workers}', command(BASE / 'freeze/engineering_protocol.json',
                                        'engineering', workers, folder), 1200)
        summary = read(folder / 'summary.json')
        assert summary['source_integrity'] == 'PASS' and summary['completed_denominator'] == 8
        assert summary['execution_errors'] == 0
        for c in read(BASE / 'freeze/engineering_protocol.json')['cases']:
            casefolder = folder / c['case_id']
            row = read(casefolder / 'case_result.json')
            study.cached_result(casefolder, row['identity'])
            if workers == 1:
                continue
            reference = BASE / 'engineering/w1' / c['case_id']
            with np.load(casefolder / 'replay.npz', allow_pickle=False) as a, np.load(reference / 'replay.npz', allow_pickle=False) as b:
                assert a.files == b.files
                for key in a.files:
                    assert np.array_equal(a[key], b[key], equal_nan=True) if a[key].dtype.kind in 'fc' else np.array_equal(a[key], b[key])
            for name in ('paired_metrics.json', 'physics_substeps.json', 'summary.json', 'config.json'):
                assert read(casefolder / name) == read(reference / name), (c['case_id'], name)
    selected = int(min(results, key=lambda k: results[k]['wall_seconds_including_startup']))
    # Exercise receipt-checked resume with all eight cases complete; no replay.
    measured('resume_check', command(BASE / 'freeze/engineering_protocol.json', 'engineering', selected,
                                    BASE / f'engineering/w{selected}', resume=True), 120)
    dump(BASE / 'benchmark.json', dict(tiers=results, selected_workers=selected,
        exact_replays=True, scientific_extra_episodes=0,
        limitation='Fixed 100-step prefixes; startup-sensitive, not full-horizon optimization'))
    print(json.dumps(dict(selected_workers=selected, exact_replays=True)), flush=True)


def profile():
    receipt = verify()
    case = next(c for c in read(BASE / 'freeze/engineering_protocol.json')['cases']
                if c['condition_id'] == 'c_open_away_robots24')
    folder = BASE / 'engineering/profile'
    if folder.exists():
        raise RuntimeError('Preserve previous profile')
    identity = dict(schema='resource-noise-profile-v1', source_sha256=encode(receipt['source_files']),
                    case_sha256=encode(case), environment=receipt['environment'])
    profiler = cProfile.Profile()
    result = profiler.runcall(study.worker, (case, str(folder), identity))
    profiler.dump_stats(str(BASE / 'engineering/profile.pstats'))
    assert result['execution_status'] == 'COMPLETED'
    verify()
    print(json.dumps(dict(profile_case=case['case_id'], frames=100, scientific_episodes=0)))


def run():
    verify()
    workers = read(BASE / 'benchmark.json')['selected_workers']
    measured('science', command(PROTOCOL, 'validation', workers, BASE / 'run'), 14400)
    summary = read(BASE / 'run/summary.json')
    assert summary['completed_denominator'] == summary['planned_denominator'] == 60
    assert summary['source_integrity'] == 'PASS'
    dump(BASE / 'execution_outcome.json', dict(status='EXECUTION_COMPLETE',
         planned=60, recorded=60, errors=summary['execution_errors'],
         summary_sha256=sha(BASE / 'run/summary.json'), scientific_analysis='PENDING'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('freeze', 'benchmark', 'profile', 'run', 'verify'))
    args = parser.parse_args()
    guard()
    globals()[args.action]()
