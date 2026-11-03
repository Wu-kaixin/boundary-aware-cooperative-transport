"""Publish saved-state diagnosis and an explicitly unvalidated runtime candidate."""
from __future__ import annotations

import csv
from importlib.metadata import version
import shutil

from research_io import ROOT, guard, read, sha
from local_pair_study import test_receipt
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import environment_versions, source_hashes, verify_source_snapshot


BASE = ROOT / 'runs/research/2026-11-03-map-bridge-conflicts-v1'
OBSERVER = ROOT / 'runs/research/2026-11-03-post-acquisition-support-v1/observer'
CHECKS = ROOT / 'runs/maintenance/2026-11-03-map-conflicts-publication-v1'
PUBLIC = ROOT / 'docs/results/map_bridge_conflicts_v1'


def table(name, rows):
    with (PUBLIC / name).open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    guard()
    closure = read(BASE / 'diagnosis_source_close.json')
    products = {'analysis': 'analysis.json', 'finite-observer': 'probe.json',
                'neighborhood-observer': 'screen.json', 'unbounded-crosscheck': 'checks.json'}
    raw_hashes = {}
    for folder, name in products.items():
        expected = closure['checks'][folder]
        assert sha(BASE / folder / 'outcome.json') == expected['receipt_sha256']
        assert sha(BASE / folder / name) == expected['product_sha256']
        raw_hashes[f'{folder}/{name}'] = expected['product_sha256']
    replay_path = BASE / 'replay-capture-v2'
    replay = read(replay_path / 'outcome.json')
    assert replay['status'] == 'EXACT_RECONSTRUCTION' and replay['verified_frames'] == 517
    assert len(replay['captured_frames']) == 18
    assert sha(replay_path / 'states.jsonl.gz') == replay['product_sha256']
    original_protocol = ROOT / 'experiments/ccta2027/map_reference_bridge_v1.json'
    assert sha(original_protocol) == replay['physical_protocol_sha256']
    original = read(original_protocol)
    verify_source_snapshot(ROOT / original['run_dir'], original['source_files'])
    assert encode(original['source_files']) == closure['source_sha256'] == replay['source_sha256']
    integrated_receipt = read(OBSERVER / 'outcome.json')
    assert integrated_receipt['status'] == 'EXACT_INTEGRATED_OBSERVER'
    assert integrated_receipt['robot_states'] == 288 and integrated_receipt['admitted'] == 181
    assert sha(OBSERVER / 'checks.json') == integrated_receipt['product_sha256']
    source = source_hashes()
    assert source == integrated_receipt['source_files']
    assert encode(source) == integrated_receipt['new_source_sha256']
    for name, digest in source.items():
        assert sha(OBSERVER / 'source_snapshot' / name) == digest
    protected = read(ROOT / 'docs/results/reference_surface_v1/protected_baseline.json')
    assert sha(ROOT / protected['canonical_config']) == protected['canonical_sha256'] == closure['protected_config_sha256']
    tests = test_receipt(CHECKS / 'core.xml')
    health = read(CHECKS / 'health.json')
    assert health['status'] == 'PASS'

    analysis = read(BASE / 'analysis/analysis.json')
    finite = read(BASE / 'finite-observer/probe.json')
    neighborhood = read(BASE / 'neighborhood-observer/screen.json')
    independent = read(BASE / 'unbounded-crosscheck/checks.json')
    assert analysis['selected_states'] == 18 and analysis['comparisons'] == 226
    assert finite['states'] == 17 and neighborhood['robot_states'] == 272
    assert independent['comparisons'] == 864 and independent['all_agree'] is True
    PUBLIC.mkdir(parents=True, exist_ok=False)

    comparisons, events = [], []
    for event in analysis['events']:
        geometry = event['geometry']
        faces = [] if geometry is None else geometry['face_points']
        edges = [edge for face in faces for edge in face['robot_relative_to_these_edges']]
        events.append(dict(frame=event['frame'],agent_id=event['target'],phase=event['phase'],
            true_gap_m=event['true_gap'],true_margin_m=event['true_margin'],
            original_h_min_m=min(event['h']),original_h_max_m=max(event['h']),
            original_scale=event['scale'],face_points=len(faces),
            all_associated_projections_outside=all(not e['projection_inside'] for e in edges) if edges else None,
            projection_t_min=min((e['projection_t'] for e in edges),default=None),
            projection_t_max=max((e['projection_t'] for e in edges),default=None),
            max_normal_error_deg=max((f['normal_error_deg'] for f in faces),default=None)))
        for comparison in event['comparisons']:
            groups, conflict = comparison['groups'], comparison['minimal_conflict']
            comparisons.append(dict(frame=event['frame'],agent_id=event['target'],
                geometry=comparison['geometry'],mode=comparison['mode'],
                joint_bounded_feasible=groups['joint']['bounded']['feasible'],
                joint_unbounded_feasible=groups['joint']['unbounded']['feasible'],
                object_bounded_feasible=groups['object']['bounded']['feasible'],
                hard_bounded_feasible=groups['agent']['bounded']['feasible'],
                minimal_conflict_indices='' if conflict is None else ' '.join(map(str,conflict['indices'])),
                cap_independent=None if conflict is None else conflict['cap_independent']))
    assert len(events) == 18 and len(comparisons) == 226
    table('all_18_states.csv', events)
    table('all_226_comparisons.csv', comparisons)

    probe_rows = []
    for event in finite['events']:
        for probe in event['probes']:
            for comparison in probe['comparisons']:
                probe_rows.append(dict(frame=event['frame'],agent_id=event['target'],model=probe['model'],
                    mode=comparison['mode'],ready=probe['system']['ready'],
                    object_rows=probe['system']['object_rows'],minimum_h_m=min(probe['system']['h']),
                    demand_speed_m_s=probe['system']['normal_motion'][0],
                    cover_bound_m_s=probe['system']['cover_motion_bound'],
                    joint_feasible=comparison['groups']['joint']['bounded']['feasible']))
    assert len(probe_rows) == 102
    table('all_17_observer_comparisons.csv', probe_rows)

    screen_rows = []
    for record in neighborhood['records']:
        for probe in record['probes']:
            system = probe['system']
            results = {item['mode']: item for item in probe['comparisons']}
            row = dict(frame=record['frame'],agent_id=record['agent_id'],known_target=record['known_target'],
                model=probe['model'],ready=system['ready'],reason=system['reason'],
                source_agents=' '.join(system.get('source_agents', [])),
                object_rows=system.get('object_rows'),
                minimum_h_m=min(system['h']) if system['ready'] else None,
                demand_speed_m_s=system['normal_motion'][0] if system['ready'] else None)
            for mode in ('capped_full', 'uncapped_full'):
                row[f'original_{mode}_feasible'] = record['original'][mode]['feasible']
                for field in ('feasible', 'newly_infeasible', 'rescued'):
                    row[f'{mode}_{field}'] = results.get(mode, {}).get(field)
            screen_rows.append(row)
    assert len(screen_rows) == 544
    table('all_272_neighborhood_states.csv', screen_rows)
    shutil.copy2(OBSERVER / 'checks.json', PUBLIC / 'integrated_288_checks.json')
    shutil.copy2(BASE / 'unbounded-crosscheck/checks.json', PUBLIC / 'unbounded_864_checks.json')
    shutil.copy2(ROOT / 'experiments/ccta2027/map_bridge_finite_observer_v1_motion_cap_erratum.json', PUBLIC / 'motion_cap_erratum.json')
    dump(PUBLIC / 'summary.json', dict(new_physical_episodes=0, verified_replay_frames=517,
        captured_robot_states=288,selected_states=18,comparisons=226,
        diagnosis_counts=analysis['counts'],observer_17=finite['summary'],
        neighborhood_272=neighborhood['counts'],unbounded_solver_crosschecks=864,
        integrated_robot_states=288,integrated_admitted=181,candidate_adopted=False,
        physical_protocol_status='NOT_FROZEN_NOT_EXECUTED',
        old_source_sha256=closure['source_sha256'],new_source_sha256=encode(source),
        limits=[analysis['inference_limit'],finite['inference_limit'],
            'Unknown boundary continuations and received motion are empirical assumptions.',
            'The acquisition state at frame402 is unresolved; the new candidate only replaces post-acquisition rows.']))
    dump(PUBLIC / 'engineering_checks.json', dict(core=tests,health=health,
        current_environment=environment_versions(),joblib=version('joblib'),
        canonical_config_sha256=protected['canonical_sha256'],current_source_sha256=encode(source),
        runtime_green=test_receipt(ROOT/'runs/maintenance/2026-11-03-post-acquisition-support-v1/runtime-green.xml'),
        prospective_gate_green=test_receipt(ROOT/'runs/maintenance/2026-11-03-post-acquisition-support-v1/study-green.xml'),
        physical_episodes=0,prospective_trajectory_auditor_status='NOT_EXECUTED'))
    dump(PUBLIC / 'provenance.json', dict(raw_base=BASE.relative_to(ROOT).as_posix(),raw_products=raw_hashes,
        original_protocol_sha256=sha(original_protocol),original_source_sha256=closure['source_sha256'],
        replay=replay,integrated_receipt=integrated_receipt,
        integrated_raw_directory=OBSERVER.relative_to(ROOT).as_posix(),
        diagnosis_source_close=closure,original_receipts_unchanged=True))
    dump(PUBLIC / 'publication.json', dict(new_physical_episodes=0,publisher_sha256=sha(__file__),
        products={path.name:sha(path) for path in sorted(PUBLIC.iterdir()) if path.is_file()},
        scope='Saved-state evidence and default-off implementation checkpoint. README/review tracked separately by Git.'))
    print(f'Published {len(list(PUBLIC.iterdir()))} evidence products; new physical episodes: 0', flush=True)


if __name__ == '__main__':
    main()
