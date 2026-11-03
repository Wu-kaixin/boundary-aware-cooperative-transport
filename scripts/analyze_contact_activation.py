"""Publish exact activation diagnosis and eight conditional motion witnesses."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

from collections import Counter
import csv
import gzip
import json
import shutil
import sys

import numpy as np

from research_io import ROOT, guard, read, sha

BASE = ROOT / 'runs/research/2026-11-03-contact-activation-v1'
PUBLIC = ROOT / 'docs/results/contact_activation_v1'


def main():
    guard()
    assert not PUBLIC.exists() and not (BASE / 'analysis.json').exists()
    protocol = ROOT / 'experiments/ccta2027/acquisition_circulation_v1.json'
    original, replay_plan = read(protocol), read(BASE / 'replay_plan.json')
    for name, digest in replay_plan['files'].items():
        assert sha(ROOT / name) == digest
    for name, digest in original['source_files'].items():
        assert sha(ROOT / original['run_dir'] / 'source_snapshot' / name) == digest
    sys.path.insert(0, str(ROOT / original['run_dir'] / 'source_snapshot/src'))
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump
    from dbact.safety.qp2d import solve_min_norm_2d, solve_min_norm_2d_cvxpy
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon
    from matplotlib.lines import Line2D

    audit = read(BASE / 'audit/summary.json')
    assert len(audit['summaries']) == 12 and audit['status'] == 'EXACT_SAVED_PREDICATES'
    for name, digest in audit['products'].items():
        assert sha(BASE / 'audit' / name) == digest
    summaries, conditional, outcomes, plot_data = [], [], {}, {}
    for spec in replay_plan['cases']:
        alias = spec['alias']
        folder = BASE / alias
        outcome = read(folder / 'outcome.json')
        assert outcome['status'] == 'EXACT_RECONSTRUCTION' and outcome['verified_frames'] == spec['end_frame']
        assert outcome['source_sha256'] == original['frozen_source_sha256']
        assert outcome['protocol_sha256'] == sha(protocol)
        for name, digest in {**outcome['scripts'], **outcome['products']}.items():
            assert sha(folder / name) == digest
        case = next(c for c in original['cases'] if c['case_id'] == spec['case_id'])
        receipt = ROOT / original['run_dir'] / spec['case_id'] / 'case_result.json'
        assert sha(receipt) == outcome['original_receipt_sha256']
        env = SimulationEnvironment(case['config'], seed=case['seed'])
        goal = next(iter(env.controller.goal_directions.values()))
        threshold = env.controller.params.push_side_threshold
        counts = Counter()
        suppressed_modes = Counter()
        with gzip.open(folder / 'frames.jsonl.gz', 'rt', encoding='utf-8') as stream:
            verified = 0
            for line in stream:
                frame = json.loads(line)
                verified += 1
                assert frame['frame'] == verified
                if frame['phase'] != 'CONTACT_READY':
                    continue
                counts['frames'] += 1
                for obs in frame['observations'].values():
                    counts['robot_steps'] += 1
                    nav, mapping = obs['navigation'], obs['map_reference']
                    if obs['ready']:
                        counts['ready'] += 1
                        if nav is not None:
                            counts['ready_with_navigation'] += 1
                            observed = float(np.array(obs['nearest_normal']) @ np.array(nav['direction'])) <= -threshold
                            actual = float(np.array(obs['filter']['true_normal']) @ goal) <= -threshold
                            counts['observed_direction_eligible'] += int(observed)
                            counts['true_task_direction_eligible'] += int(actual)
                            counts['observed_reject_true_accept'] += int(not observed and actual)
                            counts['observed_accept_true_reject'] += int(observed and not actual)
                            counts['both_directions_reject'] += int(not observed and not actual)
                            if not observed and not actual:
                                counts['both_reject_and_redeploy_disabled'] += int(obs['redeploy_reason'] == 'contact_ready')
                    unavailable = mapping is not None and mapping['status'] == 'unavailable'
                    counts['map_unavailable'] += int(unavailable)
                    suppressed = (unavailable and np.any(np.asarray(obs['nominal']) != 0)
                                  and np.all(np.asarray(obs['filter']['nominal']) == 0))
                    counts['nonzero_nominal_suppressed'] += int(suppressed)
                    if suppressed:
                        suppressed_modes[obs['nominal_mode']] += 1
        assert verified == spec['end_frame']
        summaries.append(dict(case_id=spec['case_id'], alias=alias, replayed_frames=verified,
                              contact_ready=dict(counts), suppressed_modes=dict(suppressed_modes)))
        outcomes[alias] = outcome
        witnesses = read(folder / 'witnesses.json')
        first_ready = next(s for s in witnesses['snapshots'] if s['phase'] == 'CONTACT_READY')
        env.cargoes[0].set_pose(first_ready['true_center'], first_ready['true_angle'])
        plot_data[alias] = dict(snapshot=first_ready, polygon=env.cargoes[0].vertices.copy(),
                               goal=goal.copy(), radius=env.controller.params.robot_radius)
        if alias.endswith('candidate'):
            final = witnesses['snapshots'][-1]
            assert final['frame'] == 1200
            for aid, obs in final['observations'].items():
                if obs['map_reference']['status'] != 'unavailable':
                    continue
                q = final['geometry'][aid]['filter']
                A = np.asarray(q['A']).reshape(-1, 2)
                b = np.asarray(q['b_original'])
                nominal = np.asarray(obs['nominal'])
                assert np.all(np.asarray(q['nominal']) == 0) and not obs['nominal_push']
                sol = solve_min_norm_2d(nominal, A, b, q['params']['max_speed'])
                cross = solve_min_norm_2d_cvxpy(nominal, A, b, q['params']['max_speed'])
                assert sol.feasible and cross.feasible
                assert np.linalg.norm(sol.u) > 0 and np.linalg.norm(cross.u) > 0
                np.testing.assert_allclose(sol.u, cross.u, atol=2e-5, rtol=0)
                assert np.all(A @ cross.u >= b - 1e-7)
                assert np.linalg.norm(cross.u) <= q['params']['max_speed'] + 1e-7
                conditional.append(dict(alias=alias, case_id=spec['case_id'], frame=1200, agent_id=aid,
                    nominal_mode=obs['nominal_mode'], map_points=obs['map_points'], position=obs['position'],
                    nominal_before_override=nominal.tolist(), actual_nominal=q['nominal'], actual_command=q['command'],
                    row_kinds=q['row_kinds'], A=A.tolist(), b=b.tolist(), max_speed=q['params']['max_speed'],
                    feasible=True, conditional_command=sol.u.tolist(), independent_command=cross.u.tolist(),
                    independent_solver='CLARABEL', snapshot_sha256=outcome['products']['witnesses.json']))
    assert len(conditional) == 8
    report = dict(source_sha256=original['frozen_source_sha256'], protocol_sha256=sha(protocol),
        script_sha256=sha(__file__), new_physical_episodes=0,
        saved_cases=12, saved_transport_calls_verified=sum(r['checked_transport_calls'] for r in audit['summaries']),
        saved_predicate_scope='Final cage/push modes; reference overrides and early nominal branches reported separately',
        exact_replay_frames=sum(s['replayed_frames'] for s in summaries),
        replay_scope='Two full far translation baselines and two circulation prefixes to frame1200; not full600-second candidate replays',
        prefix_summaries=summaries, conditional_motion_witnesses=conditional,
        conditional_scope='One-step counterfactual using exactly the recorded original constraints and speed cap. Does not prove future safety, reference recovery or task completion.',
        full_saved_audit_sha256=sha(BASE / 'audit/summary.json'))
    dump(BASE / 'analysis.json', report)
    PUBLIC.mkdir(parents=True, exist_ok=False)
    shutil.copy2(BASE / 'analysis.json', PUBLIC / 'analysis.json')
    shutil.copy2(BASE / 'audit/summary.json', PUBLIC / 'saved_predicates.json')
    shutil.copy2(BASE / 'replay_plan.json', PUBLIC / 'replay_plan.json')
    dump(PUBLIC / 'replay_outcomes.json', outcomes)
    records = [dict(alias=s['alias'], replayed_frames=s['replayed_frames'], **s['contact_ready']) for s in summaries]
    with (PUBLIC / 'prefix_counts.csv').open('x', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    fig, axes = plt.subplots(2, 2, figsize=(10.5, 10.4))
    fig.subplots_adjust(left=.07, right=.98, bottom=.115, top=.91, wspace=.14, hspace=.23)
    for ax, alias in zip(axes.flat, ('c-baseline', 'c-candidate', 'skew-baseline', 'skew-candidate')):
        data = plot_data[alias]
        snap = data['snapshot']
        ax.add_patch(Polygon(data['polygon'], closed=True, facecolor='#ececec', edgecolor='black'))
        eligible = 0
        for aid, obs in snap['observations'].items():
            pos = np.asarray(obs['position'])
            unavailable = obs['map_reference'] is not None and obs['map_reference']['status'] == 'unavailable'
            productive = obs['ready'] and float(np.asarray(obs['filter']['true_normal']) @ data['goal']) <= -threshold
            eligible += productive
            color = '#1b9e77' if productive else ('#d95f02' if obs['ready'] else '#377eb8')
            ax.add_patch(Circle(pos, data['radius'], facecolor=color, edgecolor='white', linewidth=.7, alpha=.85))
            if unavailable:
                ax.scatter(*pos, marker='x', c='black', s=45, zorder=4)
            ax.annotate(aid.removeprefix('agent_'), pos, xytext=(4, 5), textcoords='offset points', fontsize=7)
        center = np.asarray(snap['true_center'])
        ax.annotate('', xy=center+.75*data['goal'], xytext=center,
                    arrowprops=dict(arrowstyle='->', color='black', lw=1.5))
        ax.set_title(f"{'C shape' if alias.startswith('c-') else 'Skew box'} | {'Translation' if alias.endswith('baseline') else 'Circulation'}\nFirst CONTACT_READY, frame {snap['frame']}; true-direction eligible: {eligible}", fontsize=10)
        ax.set(xlim=(0, 8), ylim=(0, 8), xlabel='x (m)', ylabel='y (m)', aspect='equal')
        ax.grid(alpha=.15)
    handles = [Line2D([], [], marker='o', linestyle='', color=color, label=label) for color, label in
               (('#1b9e77', 'Ready, productive true normal'), ('#d95f02', 'Ready, unproductive true normal'), ('#377eb8', 'Not contact-ready'))]
    handles.append(Line2D([], [], marker='x', linestyle='', color='black', label='Local map reference unavailable'))
    fig.legend(handles=handles, loc='lower center', ncols=2, frameon=False, fontsize=9, bbox_to_anchor=(.5, .025))
    fig.suptitle('Acquisition succeeded, but useful pushing positions were not populated\nTruth is used only by this observer; all16 robots and the full workspace are shown.', fontsize=12)
    for extension in ('png', 'pdf'):
        fig.savefig(PUBLIC / f'contact_positions.{extension}', dpi=180)
    plt.close(fig)
    dump(PUBLIC / 'publication.json', dict(script_sha256=sha(__file__),
        products={p.name:sha(p) for p in PUBLIC.iterdir()}, new_physical_episodes=0))
    print(json.dumps(dict(exact_frames=report['exact_replay_frames'], saved_calls=report['saved_transport_calls_verified'],
                          conditional_witnesses=len(conditional), summaries=summaries)), flush=True)


if __name__ == '__main__':
    main()
