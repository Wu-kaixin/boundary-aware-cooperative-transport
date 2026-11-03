"""Publish fixed-cohort packet geometry and all directional certificates."""
from __future__ import annotations

import csv
from pathlib import Path
import shutil

from research_io import ROOT, guard, read, sha
from dbact_sim.experiments.deployment import dump


GEOMETRY = ROOT / 'runs/research/2026-11-03-post-support-contact-packet-geometry-v1'
DIRECTION = ROOT / 'runs/research/2026-11-03-post-support-contact-direction-capability-v1'
VERIFY = ROOT / 'runs/maintenance/2026-11-03-contact-packet-geometry-v1/independent-check.json'
PUBLIC = ROOT / 'docs/results/post_support_contact_geometry_capability_v1'
MODELS = ('original_aggregate', 'finite_capped', 'finite_uncapped')


def table(path, rows):
    with path.open('x', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    guard()
    geometry, direction, verified = read(GEOMETRY / 'analysis.json'), read(DIRECTION / 'analysis.json'), read(VERIFY)
    assert verified['status'] == 'VERIFIED'
    assert sha(GEOMETRY / 'analysis.json') == verified['geometry_analysis_sha256'] == read(GEOMETRY / 'outcome.json')['analysis_sha256']
    assert sha(DIRECTION / 'analysis.json') == verified['direction_analysis_sha256'] == read(DIRECTION / 'outcome.json')['analysis_sha256']
    assert read(DIRECTION / 'outcome.json')['inconclusive'] == 0
    key = lambda row: (row['condition'], row['frame'], row['agent_id'])
    lookup = {key(row): row for row in direction['states']}
    rows, features, systems = [], [], []
    for state in geometry['states']:
        capability = lookup[key(state)]
        for name in ('admitted', 'guard', 'positive_force', 'declared_pusher'):
            assert state[name] == capability[name]
        feature = state.get('limiting_feature', {})
        row = {name: state[name] for name in ('condition', 'frame', 'agent_id', 'admitted', 'guard', 'positive_force', 'declared_pusher')}
        row.update(evaluation_precommand_signed_distance=state['evaluation_signed_distance'],
            evaluation_precommand_h=state['evaluation_h'], limiting_source=feature.get('source_agent'),
            limiting_own_source=feature.get('own_source'), limiting_kind=feature.get('kind'),
            limiting_distance=feature.get('distance'), limiting_structural=feature.get('structural'),
            limiting_noise=feature.get('noise'), limiting_h=feature.get('h'),
            limiting_normal_alignment=feature.get('normal_alignment_with_evaluation'),
            minimum_residual=state.get('minimum_uncapped_residual'),
            minimum_without_structural=state.get('minimum_without_structural'),
            minimum_without_noise=state.get('minimum_without_noise'),
            minimum_without_both=state.get('minimum_without_both'),
            original_goal_velocity=capability['original_goal_velocity'],
            counterfactual_selected_goal_velocity=capability['counterfactual_goal_velocity'])
        for model in MODELS:
            result = capability['systems'].get(model, {})
            row[model+'_classification'] = result.get('classification', 'NOT_ADMITTED')
            row[model+'_maximum_goal_velocity'] = result.get('maximum_goal_velocity')
            row[model+'_dual_upper_bound'] = result.get('dual_upper_bound')
            if result:
                systems.append(dict(condition=state['condition'], frame=state['frame'], agent_id=state['agent_id'],
                    positive_force=state['positive_force'], declared_pusher=state['declared_pusher'],
                    model=model, status=result['status'], classification=result['classification'],
                    maximum_goal_velocity=result.get('maximum_goal_velocity'), dual_upper_bound=result.get('dual_upper_bound'),
                    primal_dual_gap=result.get('primal_dual_gap'), minimum_residual=result.get('minimum_residual'),
                    speed=result.get('speed'), speed_limit=result['speed_limit'],
                    separator_upper_bound=result.get('separator_upper_bound')))
        rows.append(row)
        for f in state.get('rows', []):
            features.append(dict(condition=state['condition'], frame=state['frame'], agent_id=state['agent_id'],
                positive_force=state['positive_force'], declared_pusher=state['declared_pusher'],
                row_index=f['row_index'], source_agent=f['source_agent'], own_source=f['own_source'], kind=f['kind'],
                distance=f['distance'], structural=f['structural'], noise=f['noise'], h=f['h'],
                command_term=f['command_term'], distance_term=f['distance_term'], structural_term=f['structural_term'],
                noise_term=f['noise_term'], margin_term=f['margin_term'], motion_term=f['motion_term'],
                uncapped_residual=f['uncapped_residual'], normal_alignment=f['normal_alignment_with_evaluation'],
                lower_distance_minus_evaluation=f['lower_distance_minus_evaluation']))
    assert len(rows) == 128 and len(features) == 3813 and len(systems) == 336
    PUBLIC.mkdir(parents=True, exist_ok=False)
    table(PUBLIC / 'all_128_states.csv', rows)
    table(PUBLIC / 'all_17_positive_force_states.csv', [r for r in rows if r['positive_force']])
    table(PUBLIC / 'all_3813_object_rows.csv', features)
    table(PUBLIC / 'all_336_systems.csv', systems)
    dump(PUBLIC / 'direction_certificates.json', dict(input_analysis_sha256=sha(DIRECTION / 'analysis.json'),
        states=direction['states'], new_physical_episodes=0))
    dump(PUBLIC / 'summary.json', dict(geometry=geometry['counts'], direction=direction['counts'],
        original_control_frames_replayed=1749, new_physical_episodes=0,
        interpretation='Instantaneous fixed-state velocity capability; no new force, closed-loop causality or population reliability certificate.'))
    shutil.copy2(VERIFY, PUBLIC / 'verification.json')
    dump(PUBLIC / 'provenance.json', dict(geometry_directory=GEOMETRY.relative_to(ROOT).as_posix(),
        direction_directory=DIRECTION.relative_to(ROOT).as_posix(),
        geometry_sha256=sha(GEOMETRY / 'analysis.json'), direction_sha256=sha(DIRECTION / 'analysis.json'),
        contact_analysis_sha256=geometry['contact_analysis_sha256'],
        runtime_source_sha256=geometry['runtime_source_sha256'], protocol_sha256=geometry['protocol_sha256'],
        verification_sha256=sha(VERIFY), publisher_sha256=sha(__file__), new_physical_episodes=0))
    dump(PUBLIC / 'publication.json', dict(products={p.name: sha(p) for p in PUBLIC.iterdir() if p.is_file()}, new_physical_episodes=0))
    print(dict(states=len(rows), feature_rows=len(features), systems=len(systems)), flush=True)


if __name__ == '__main__':
    main()
