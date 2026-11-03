"""Independently cross-check unbounded half-plane diagnoses with HiGHS LP."""
from __future__ import annotations

import json
import shutil

import numpy as np
from scipy.optimize import linprog

from research_io import ROOT, guard, read, sha

BASE = ROOT/'runs/research/2026-11-03-map-bridge-conflicts-v1'


def feasible(A, b):
    A, b = np.asarray(A, dtype=float).reshape(-1, 2), np.asarray(b, dtype=float)
    result = linprog(np.zeros(2), A_ub=-A if len(b) else None,
        b_ub=-b if len(b) else None, bounds=[(None, None), (None, None)], method='highs')
    assert result.status in (0, 2), result.message
    report = dict(status=int(result.status), feasible=result.status==0)
    if result.status == 0:
        residual = float(np.min(A @ result.x-b)) if len(b) else 0.
        assert residual >= -1e-7
        report.update(u=result.x.tolist(), minimum_residual=residual)
    return report


def run():
    from dbact_sim.experiments.deployment import dump
    import map_bridge_study as study
    guard()
    plan = study.check()
    # Check inequality orientation and free variables before touching run data.
    assert feasible([[1., 0.]], [2.])['feasible']
    assert feasible([[-1., 0.]], [2.])['u'][0] <= -2.
    assert not feasible([[1., 0.], [-1., 0.]], [1., 0.])['feasible']
    report = read(BASE/'analysis/analysis.json')
    outcome = read(BASE/'analysis/outcome.json')
    assert sha(BASE/'analysis/analysis.json') == outcome['analysis_sha256']
    target = BASE/'unbounded-crosscheck'
    target.mkdir(parents=True, exist_ok=False)
    identity = dict(source_sha256=plan['frozen_source_sha256'], analysis_sha256=outcome['analysis_sha256'],
        script_sha256=sha(__file__), new_physical_episodes=0,
        scope='HiGHS numerical cross-check of every stored CLARABEL unbounded group and minimal conflict; not a formal proof')
    shutil.copy2(__file__, target/'audit_script.py')
    dump(target/'started.json', identity)
    checks = []
    try:
        for event in report['events']:
            for comparison in event['comparisons']:
                A, b = np.asarray(comparison['A']), np.asarray(comparison['b'])
                groups = [(name, data['indices'], data['unbounded']) for name, data in comparison['groups'].items()]
                conflict = comparison['minimal_conflict']
                if conflict is not None:
                    groups.append(('minimal_conflict', conflict['indices'], conflict['unbounded']))
                for name, indices, expected in groups:
                    cross = feasible(A[indices], b[indices])
                    assert cross['feasible'] == expected['feasible']
                    checks.append(dict(frame=event['frame'], geometry=comparison['geometry'],
                        mode=comparison['mode'], group=name, indices=indices, **cross))
        study.check()
        assert sha(__file__) == identity['script_sha256']
        dump(target/'checks.json', dict(**identity, all_agree=True, comparisons=len(checks), checks=checks))
        dump(target/'outcome.json', dict(**identity, status='VERIFIED_UNBOUNDED_CROSSCHECK',
            comparisons=len(checks), product_sha256=sha(target/'checks.json')))
        print(json.dumps(dict(status='VERIFIED_UNBOUNDED_CROSSCHECK', comparisons=len(checks))), flush=True)
    except Exception as exc:
        dump(target/'failure.json', dict(**identity, status='INCOMPLETE', verified=len(checks), error=repr(exc)))
        raise


if __name__ == '__main__':
    run()
