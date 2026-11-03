"""Screen every robot at the captured frames before changing runtime behavior."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

import gzip
import json
from pathlib import Path
import shutil

import numpy as np

from research_io import ROOT, guard, read, sha
from probe_map_bridge_finite_support import observed_system, systems, REPLAY, BASE
from analyze_residual_coordination import bounded_check, decompose

DESIGN = ROOT/'experiments/ccta2027/map_bridge_neighborhood_observer_v1_design.md'
ERRATUM = ROOT/'experiments/ccta2027/map_bridge_finite_observer_v1_motion_cap_erratum.json'


def run():
    import map_bridge_study as study
    from dbact_sim.experiments.deployment import dump
    guard()
    plan = study.check()
    receipt = read(REPLAY/'outcome.json')
    assert receipt['status'] == 'EXACT_RECONSTRUCTION' and sha(REPLAY/'states.jsonl.gz') == receipt['product_sha256']
    target = BASE/'neighborhood-observer'
    target.mkdir(parents=True, exist_ok=False)
    helpers = {name:sha(ROOT/name) for name in ('scripts/probe_map_bridge_neighborhood.py',
        'scripts/probe_map_bridge_finite_support.py', 'scripts/analyze_residual_coordination.py',
        'scripts/diagnose_pair_residuals.py', 'scripts/qp_conflict_study.py')}
    identity = dict(source_sha256=plan['frozen_source_sha256'], replay_sha256=receipt['product_sha256'],
        design_sha256=sha(DESIGN), erratum_sha256=sha(ERRATUM), helpers=helpers, new_physical_episodes=0)
    for name in helpers:
        path = target/'source_snapshot'/name
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/name, path)
    shutil.copy2(DESIGN, target/'design.md')
    shutil.copy2(ERRATUM, target/'motion_cap_erratum.json')
    dump(target/'started.json', identity)
    records = []
    try:
        with gzip.open(REPLAY/'states.jsonl.gz', 'rt', encoding='utf-8') as stream:
            states = [json.loads(line) for line in stream]
        selected = [s for s in states if s['target'] == 'agent_14']
        assert len(selected) == 17
        for state in selected:
            assert len(state['agents']) == 16
            for aid, saved in state['agents'].items():
                assert saved['params']['object_velocity_bound'] == .2
                original = {}
                for mode in ('capped_full', 'uncapped_full'):
                    original[mode] = bounded_check(saved['A'], systems(saved)[mode], saved['params']['max_speed'])
                available = saved['diagnostic']['map_reference_status'] == 'fresh_reference'
                probes = []
                for model in ('full_bound', 'received_speed'):
                    changed = (observed_system(saved, (state['frame']-1)*saved['params']['dt'], model)
                        if available else dict(ready=False, reason='no_current_map_motion'))
                    cases = []
                    if changed['ready']:
                        for mode in ('capped_full', 'uncapped_full'):
                            b = systems(changed)[mode]
                            result = bounded_check(changed['A'], b, saved['params']['max_speed'])
                            cases.append(dict(mode=mode, **result,
                                newly_infeasible=original[mode]['feasible'] and not result['feasible'],
                                rescued=not original[mode]['feasible'] and result['feasible'],
                                decomposition=(decompose(changed['A'], b, changed['kinds'], saved['params']['max_speed'])
                                    if not result['feasible'] else None)))
                    probes.append(dict(model=model, system=changed, comparisons=cases))
                records.append(dict(frame=state['frame'], agent_id=aid, known_target=aid==state['target'],
                    phase=state['phase'], original=original, actual=saved['actual'],
                    true_gap=saved['true_gap'], original_status=saved['status'], probes=probes))
            print(json.dumps(dict(frame=state['frame'], robot_states=len(records))), flush=True)
        assert len(records) == 272 and sum(x['known_target'] for x in records) == 17
        counts = {}
        for model in ('full_bound', 'received_speed'):
            found = [p for x in records for p in x['probes'] if p['model'] == model]
            fallback = {}
            for p in found:
                if not p['system']['ready']:
                    why=p['system']['reason']; fallback[why]=fallback.get(why,0)+1
            counts[model] = dict(total=len(found), ready=sum(p['system']['ready'] for p in found), fallback=fallback,
                results={mode:dict(feasible=sum(c['feasible'] for p in found for c in p['comparisons'] if c['mode']==mode),
                    newly_infeasible=sum(c['newly_infeasible'] for p in found for c in p['comparisons'] if c['mode']==mode),
                    rescued=sum(c['rescued'] for p in found for c in p['comparisons'] if c['mode']==mode))
                    for mode in ('capped_full','uncapped_full')})
        study.check()
        assert helpers == {name:sha(ROOT/name) for name in helpers}
        assert sha(DESIGN) == identity['design_sha256'] and sha(ERRATUM) == identity['erratum_sha256']
        dump(target/'screen.json', dict(**identity, robot_states=272, records=records, counts=counts))
        dump(target/'outcome.json', dict(**identity, status='VERIFIED_NEIGHBORHOOD_OBSERVER', robot_states=272,
            screen_sha256=sha(target/'screen.json')))
        print(json.dumps(counts, indent=2), flush=True)
    except Exception as exc:
        dump(target/'failure.json', dict(**identity, status='SCREEN_INCOMPLETE', completed_states=len(records), error=repr(exc)))
        raise


if __name__ == '__main__':
    run()
