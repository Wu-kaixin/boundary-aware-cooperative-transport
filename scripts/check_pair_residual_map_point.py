"""Post-review, evaluation-only geometry check of the selected frame989 point."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from research_io import ROOT, guard, read, sha


def main(output):
    guard()
    import local_pair_study as study
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.study import cached_result
    from dbact_sim.experiments.deployment import encode
    plan = study.check_freeze()
    public = ROOT/'docs/results/pair_residual_v1'
    publication = read(public/'publication.json')
    witness_path = public/'residuals.json'
    assert sha(witness_path) == publication['outputs']['residuals.json']
    witness = next(w for w in read(witness_path) if w['frame'] == 989)
    case = next(c for c in plan['cases'] if c['case_id'] == 'far_skew_box_s0_r0_pos0_local_pair_candidate')
    folder = ROOT/plan['run_dir']/case['case_id']
    manifest = read(ROOT/plan['run_dir']/'manifest.json')
    result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
    assert result['execution_status'] == 'COMPLETED'
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    with np.load(folder/'replay.npz', allow_pickle=False) as saved:
        center = saved['cargo/cargo_0/centers'][988].copy()
        angle = float(saved['cargo/cargo_0/angles'][988])
        position = saved['agent_positions'][988, 5].copy()
    agent = witness['agent']
    assert env.agents[5].agent_id == agent['agent_id'] == 'agent_05'
    np.testing.assert_array_equal(position, agent['position'])
    env.cargoes[0].set_pose(center, angle)
    points = np.asarray(agent['aggregate_trace']['face_points'])
    gaps, _, feet = env.cargoes[0].signed_distance(points)
    own_gap, _, _ = env.cargoes[0].signed_distance(position.reshape(1, 2))
    assert abs(float(own_gap[0])-agent['true_gap']) < 1e-12
    distances = np.linalg.norm(points-position, axis=1)
    lower = own_gap[0]-distances
    assert len(points) == 1 and gaps[0] >= lower[0] > 0.
    row = read(folder/'paired_metrics.json')['steps'][988]
    diag = next(d for d in row['diagnostics'] if d['agent_id'] == 'agent_05')
    report = dict(status='VERIFIED_READ_ONLY_ADDENDUM', post_review=True, saved_frame=989,
        new_physical_episodes=0, new_controller_replay_frames=0, agent_id='agent_05',
        physical_protocol_sha256=sha(study.PROTOCOL), witness_sha256=sha(witness_path),
        original_receipt_sha256=sha(folder/'case_result.json'), script_sha256=sha(__file__),
        runtime_source_sha256=plan['frozen_source_sha256'],
        pre_command_cargo_position=center.tolist(), pre_command_cargo_angle=angle,
        robot_position=position.tolist(), selected_map_points=points.tolist(),
        robot_to_map_point_distances=distances.tolist(),
        selected_map_point_true_signed_clearances=gaps.tolist(),
        selected_map_point_nearest_true_points=feet.tolist(), triangle_distance_lower_bounds=lower.tolist(),
        diagnostic={k:diag[k] for k in ('reference_status', 'map_reference_status', 'reference_matches',
            'reference_residual_rms', 'reference_displacement', 'map_reference_displacement', 'reference_angle')},
        scope='Post-review direct evaluation of one already captured witness. Cargo geometry is truth for evaluation only. Point ancestry and reason for its displacement are not reconstructed here.')
    output = Path(output).resolve()
    assert output.is_relative_to(ROOT/'runs')
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(status=report['status'], point_true_distance=gaps[0],
        robot_to_point=distances[0], triangle_lower_bound=lower[0])), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=ROOT/'runs/maintenance/2026-11-03-pair-residual-v1/map-point-addendum.json')
    main(parser.parse_args().out)
