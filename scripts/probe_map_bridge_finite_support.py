"""Current-scan observer for all 17 post-acquisition map-bridge conflicts."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

import argparse
import gzip
import json
from pathlib import Path
import shutil

import numpy as np

from research_io import ROOT, guard, read, sha
from trace_boundary_map import plain
from diagnose_pair_residuals import systems
from analyze_residual_coordination import decompose
from dbact.safety.filter import SafetyFilter, SafetyFilterParams
from dbact.sensing.finite_support import ScanSupport, support_rows

BASE = ROOT/'runs/research/2026-11-03-map-bridge-conflicts-v1'
REPLAY = BASE/'replay-capture-v2'
DESIGN = ROOT/'experiments/ccta2027/map_bridge_finite_observer_v1_design.md'
MODELS = ('full_bound', 'received_speed', 'zero_motion')


def packet_from_saved(value):
    return ScanSupport(np.asarray(value['points']).reshape(-1, 2), np.asarray(value['object_ids']),
        np.asarray(value['origin']), value['timestamp'], value['noise_allowance'],
        np.asarray(value['chords'], dtype=int).reshape(-1, 2), np.asarray(value['half_gaps']))


def observed_system(saved, timestamp, model):
    """Evaluation-only finite rows with unchanged hard constraints and scalars."""
    if model not in MODELS:
        raise ValueError(model)
    params = SafetyFilterParams(**saved['params'])
    own = saved['permitted_packets'].get(saved['agent_id'])
    if own is None or not len(own['points']) or abs(own['timestamp']-timestamp) > 1e-9:
        return dict(ready=False, reason='no_current_own_returns')
    selected = {key:packet_from_saved(saved['permitted_packets'][key])
        for key in dict.fromkeys([saved['agent_id'], *saved['neighbors']])
        if key in saved['permitted_packets'] and abs(saved['permitted_packets'][key]['timestamp']-timestamp) <= 1e-9}
    cover = params.object_velocity_bound
    support = support_rows(saved['position'], list(selected.values()), params, timestamp,
        cover, isolated_point_guards=True)
    info = dict(ready=support.ready, reason=support.reason, source_agents=list(selected),
        cover_motion_bound=cover, near_returns=support.near_returns,
        uncovered_returns=support.uncovered_returns, point_guard_rows=support.point_guard_rows)
    if not support.ready:
        return info
    if model == 'full_bound':
        speed = cover
    elif model == 'zero_motion':
        speed = 0.
    else:
        received = saved['point_velocities'] if saved['point_velocities'] is not None else [saved['object_velocity']]
        speed = float(np.min([cover, np.max(np.linalg.norm(np.asarray(received).reshape(-1, 2), axis=1))]))
    # Transport r_safe is the physical contact floor, without acquisition noise.
    h = support.lower_distance-params.r_safe
    normal_motion = np.full(len(h), speed)
    demand = normal_motion-params.gamma_obj*h
    b_obj = SafetyFilter(params)._cap_to_reachable(support.A, demand+params.rho)
    hard = np.asarray(saved['kinds']) != 'object'
    return dict(**info, A=np.vstack([np.asarray(saved['A'])[hard], support.A]).tolist(),
        b=np.r_[np.asarray(saved['b'])[hard], b_obj].tolist(),
        kinds=[*[k for k in saved['kinds'] if k != 'object'], *(['object']*len(h))],
        h=h.tolist(), normal_motion=normal_motion.tolist(), params=saved['params'],
        lower_distance=support.lower_distance.tolist(), hard_original_indices=np.flatnonzero(hard).tolist(),
        object_rows=len(h), cap_active=int(np.count_nonzero(b_obj < demand+params.rho-1e-15)))


def probe(output):
    import map_bridge_study as study
    from dbact_sim.experiments.deployment import dump
    guard()
    plan = study.check()
    receipt = read(REPLAY/'outcome.json')
    assert receipt['status'] == 'EXACT_RECONSTRUCTION'
    assert receipt['product_sha256'] == sha(REPLAY/'states.jsonl.gz')
    target = Path(output).resolve()
    if not target.is_relative_to((ROOT/'runs').resolve()):
        raise ValueError('Probe output must be a fresh directory under runs/')
    target.mkdir(parents=True, exist_ok=False)
    helpers = {name:sha(ROOT/name) for name in ('scripts/probe_map_bridge_finite_support.py',
        'scripts/analyze_residual_coordination.py', 'scripts/diagnose_pair_residuals.py',
        'scripts/qp_conflict_study.py', 'tests/test_map_bridge_finite_observer.py')}
    identity = dict(source_sha256=plan['frozen_source_sha256'], replay_product_sha256=receipt['product_sha256'],
        design_sha256=sha(DESIGN), helpers=helpers, new_physical_episodes=0,
        inference_limit='Empirical current-scan observer on 17 selected old states; received speed is not a certified actual speed bound')
    shutil.copy2(DESIGN, target/'design.md')
    for name in helpers:
        path = target/'source_snapshot'/name
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, path)
    dump(target/'started.json', identity)
    events = []
    try:
        with gzip.open(REPLAY/'states.jsonl.gz', 'rt', encoding='utf-8') as stream:
            states = [json.loads(line) for line in stream]
        selected = [s for s in states if s['target'] == 'agent_14']
        assert len(selected) == 17
        for state in selected:
            saved = state['agents'][state['target']]
            timestamp = (state['frame']-1)*saved['params']['dt']
            assert saved['params']['r_safe'] == .065
            for key, packet in saved['permitted_packets'].items():
                assert key in [saved['agent_id'], *saved['neighbors']]
                np.testing.assert_array_equal(packet['origin'], state['agents'][key]['position'])
            probes = []
            for model in MODELS:
                system = observed_system(saved, timestamp, model)
                comparisons = []
                if system['ready']:
                    for mode in ('capped_full', 'uncapped_full'):
                        b = systems(system)[mode]
                        comparisons.append(dict(mode=mode, b=b.tolist(),
                            **decompose(system['A'], b, system['kinds'], saved['params']['max_speed'])))
                probes.append(dict(model=model, system=system, comparisons=comparisons))
            events.append(dict(frame=state['frame'], target=state['target'], phase=state['phase'], probes=probes))
            print(json.dumps(dict(frame=state['frame'], probes=[dict(model=x['model'],
                ready=x['system']['ready'], reason=x['system']['reason'],
                feasible=[y['mode'] for y in x['comparisons'] if y['groups']['joint']['bounded']['feasible']]) for x in probes])), flush=True)
        summary = {}
        for model in MODELS:
            matches = [p for e in events for p in e['probes'] if p['model'] == model]
            summary[model] = dict(total=len(matches), ready=sum(x['system']['ready'] for x in matches),
                feasible={mode:sum(c['groups']['joint']['bounded']['feasible'] for p in matches
                    for c in p['comparisons'] if c['mode']==mode) for mode in ('capped_full','uncapped_full')})
        study.check()
        assert sha(DESIGN) == identity['design_sha256']
        assert helpers == {name:sha(ROOT/name) for name in helpers}
        dump(target/'probe.json', plain(dict(**identity, states=17, events=events, summary=summary)))
        dump(target/'outcome.json', dict(**identity, status='VERIFIED_OBSERVER_PROBE', states=17,
            probe_sha256=sha(target/'probe.json')))
        print(json.dumps(summary, indent=2), flush=True)
    except Exception as exc:
        dump(target/'failure.json', dict(**identity, status='PROBE_INCOMPLETE', completed_states=len(events), error=repr(exc)))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=BASE/'finite-observer')
    probe(parser.parse_args().output)
