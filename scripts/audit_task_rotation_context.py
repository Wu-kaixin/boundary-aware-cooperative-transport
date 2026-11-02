"""Describe actual yaw in all 12 protected-setting cases and 4 acquisition cases.

Evaluator context only: these reused runs are not new validation. The material
point rotation term is not a registration-error or stopping certificate.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from research_io import ROOT, guard, read, sha
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, verify_source_snapshot


def analyze(out):
    guard()
    out = Path(out).resolve()
    if not out.is_relative_to(ROOT/'runs') or out.exists():
        raise ValueError('Choose a new output under runs/')
    cohorts = [
        ('protected_setting_12', 'experiments/ccta2027/transport_validation_v1_new_starts.json', 'validation'),
        ('initial_task_reference_4', 'experiments/ccta2027/task_reference_v1.json', 'development')]
    rows, sources = [], {}
    for cohort, name, split in cohorts:
        protocol = ROOT/name
        plan = read(protocol)
        batch = ROOT/plan['run_dir']
        manifest = read(batch/'manifest.json')
        identity = dict(schema='ccta-study-v2', protocol_sha256=sha(protocol), split=split,
            source_sha256=plan['frozen_source_sha256'], environment=plan['environment'])
        assert manifest['identity'] == identity
        assert encode(manifest['source_files']) == plan['frozen_source_sha256']
        verify_source_snapshot(batch, manifest['source_files'])
        cases = [c for c in plan['cases'] if
            (c['variant'] == 'boundary_cvt' and c['arm'] == 'candidate' if cohort == 'protected_setting_12'
             else c['arm'] == 'task_reference')]
        assert len(cases) == (12 if cohort == 'protected_setting_12' else 4)
        sources[cohort] = dict(protocol=name, identity=identity)
        for case in cases:
            folder = batch/case['case_id']
            result = cached_result(folder, {**identity, 'case_sha256': encode(case)})
            assert result['execution_status'] == 'COMPLETED'
            assert case['config']['controller']['max_object_speed'] == .45
            with np.load(folder/'replay.npz', allow_pickle=False) as z:
                angles = z['cargo/cargo_0/angles'].copy()
                radius = float(np.linalg.norm(z['cargo/cargo_0/local_vertices'], axis=1).max())
            yaw = angles-angles[0]
            rows.append(dict(cohort=cohort, condition=case['condition_id'], case_id=case['case_id'],
                samples=len(yaw), final_yaw_deg=float(np.degrees(yaw[-1])),
                peak_absolute_yaw_deg=float(np.degrees(np.abs(yaw).max())),
                true_material_point_rotation_term_max_m=float((2*np.abs(np.sin(yaw/2))*radius).max()),
                true_radius_m=radius, original_task_completed=result['task_completed'],
                original_endpoint_m=result['euclidean_endpoint_error'],
                receipt_sha256=sha(folder/'case_result.json')))
    out.mkdir(parents=True, exist_ok=False)
    summary = dict(scope=__doc__, script_sha256=sha(__file__), sources=sources, rows=rows,
        cohorts={name: dict(episodes=len(selected := [r for r in rows if r['cohort'] == name]),
            max_absolute_yaw_deg=max(r['peak_absolute_yaw_deg'] for r in selected),
            max_material_point_rotation_term_m=max(r['true_material_point_rotation_term_max_m'] for r in selected))
            for name, _, _ in cohorts}, new_physical_episodes=0)
    dump(out/'analysis.json', summary)
    print(summary['cohorts'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True)
    analyze(parser.parse_args().out)
