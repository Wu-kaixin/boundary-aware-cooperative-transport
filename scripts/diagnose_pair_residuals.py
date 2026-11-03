"""Exact saved-state diagnosis of the two remaining local-pair failures."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

import argparse
from collections import Counter
from dataclasses import asdict
import gzip
import json
from pathlib import Path
import shutil
import time

import numpy as np

from research_io import ROOT, guard, read, sha
from trace_boundary_map import plain
from replay_qp_conflict import recorded

BASE = ROOT/'runs/research/2026-11-03-pair-residual-v1'
CHECKS = ROOT/'runs/maintenance/2026-11-03-pair-residual-v1'
PROTOCOL = ROOT/'experiments/ccta2027/pair_residual_v1.json'
DESIGN = ROOT/'experiments/ccta2027/pair_residual_v1_design.md'
CASES = dict(c=('far_c_open_away_pos0_local_pair_candidate', 1074),
             skew=('far_skew_box_s0_r0_pos0_local_pair_candidate', 989))
HELPERS = ('scripts/diagnose_pair_residuals.py', 'scripts/trace_boundary_map.py',
    'scripts/replay_qp_conflict.py', 'scripts/analyze_residual_coordination.py',
    'scripts/analyze_local_pair_feasibility.py', 'scripts/qp_conflict_study.py',
    'scripts/local_pair_study.py', 'tests/test_pair_residual_diagnostic.py')


def systems(snapshot):
    """Restore received motion/rho before reach-capping; never modify hard rows."""
    b = np.asarray(snapshot['b'], dtype=float)
    obj = np.asarray(snapshot['kinds']) == 'object'
    h, motion = np.asarray(snapshot['h']), np.asarray(snapshot['normal_motion'])
    if len(h) != int(obj.sum()) or len(motion) != len(h):
        raise ValueError('Object margins and normal motion must cover every object row')
    gamma, rho = snapshot['params']['gamma_obj'], snapshot['params']['rho']
    result = dict(capped_full=b.copy())
    for name, velocity, margin in (
        ('uncapped_full', motion, rho), ('uncapped_no_rho', motion, 0.),
        ('uncapped_zero_motion', np.zeros_like(motion), rho),
        ('uncapped_zero_motion_no_rho', np.zeros_like(motion), 0.)):
        rhs = b.copy()
        rhs[obj] = velocity-gamma*h+margin
        result[name] = rhs
    return result


def pair_system(first, second, mode):
    """Remove only two mutual half rows; all other received constraints remain."""
    if second['agent_id'] not in first['neighbors'] or first['agent_id'] not in second['neighbors']:
        raise ValueError('Pair must be a current mutual one-hop link')
    A = [np.asarray(x['A']).reshape(-1, 2) for x in (first, second)]
    b = [systems(x)[mode] for x in (first, second)]
    indices = [first['neighbors'].index(second['agent_id']), second['neighbors'].index(first['agent_id'])]
    delta = 2*(np.asarray(first['position'])-np.asarray(second['position']))
    for member, expected in enumerate((delta, -delta)):
        if not np.array_equal(A[member][indices[member]], expected):
            raise ValueError('Mutual row does not describe the current positions')
    if abs(b[0][indices[0]]-b[1][indices[1]]) > 1e-12:
        raise ValueError('Mutual half-responsibilities differ')
    keep = [np.arange(len(rhs)) != index for rhs, index in zip(b, indices)]
    joint_A = np.vstack([np.c_[A[0][keep[0]], np.zeros((keep[0].sum(), 2))],
        np.c_[np.zeros((keep[1].sum(), 2)), A[1][keep[1]]], np.r_[delta, -delta]])
    joint_b = np.r_[b[0][keep[0]], b[1][keep[1]], b[0][indices[0]]+b[1][indices[1]]]
    return joint_A, joint_b


def normal_components(nominal, transport, actual, normal):
    n, u, bias, applied = [np.asarray(x) for x in (normal, nominal, transport, actual)]
    return dict(nominal=float(u@n), transport=float(bias@n),
        coverage_residual=float((u-bias)@n), actual=float(applied@n))


def application_certificate(first, second, command, A, b):
    """Use the controller's actual domain even when no safety wall rows exist."""
    from dbact.geometry import clip_to_domain
    if first['domain'] != second['domain'] or first['params'] != second['params']:
        raise ValueError('Pair application contracts differ')
    positions = np.asarray([first['position'], second['position']])
    u = np.asarray(command).reshape(2, 2)
    params, domain = first['params'], first['domain']
    dt, speed = params['dt'], params['max_speed']
    applied = np.vstack([(clip_to_domain(p+dt*v, domain)-p)/dt for p,v in zip(positions,u)])
    delta, relative = positions[0]-positions[1], applied[0]-applied[1]
    hold = float(np.clip(-delta@relative/(relative@relative), 0., dt)) if relative@relative > 0 else 0.
    gap = float(np.linalg.norm(delta+hold*relative))
    residual = float(np.min(np.asarray(A)@applied.ravel()-b))
    return dict(applied_commands=applied.tolist(), minimum_hold_separation=gap,
        minimum_applied_residual=residual, application_valid=bool(residual >= -1e-7
            and max(np.linalg.norm(applied, axis=1)) <= speed+1e-7
            and gap >= params['d_min']-1e-7))


def freeze():
    guard()
    import local_pair_study as study
    from dbact_sim.experiments.deployment import dump
    plan = study.check_freeze()
    assert not BASE.exists() and not PROTOCOL.exists()
    tests = {name:study.test_receipt(CHECKS/name) for name in ('green-final.xml', 'core-final.xml')}
    paths = sorted(set(HELPERS) | set(plan['scripts']))
    identity = dict(status='FROZEN_BEFORE_DIAGNOSIS', new_physical_episodes=0,
        physical_protocol_sha256=sha(study.PROTOCOL), runtime_source_sha256=plan['frozen_source_sha256'],
        canonical_sha256=plan['canonical_sha256'], environment=plan['environment'],
        source_files=plan['source_files'], scripts={p:sha(ROOT/p) for p in paths},
        design_sha256=sha(DESIGN), cases=CASES, total_replay_frames=2063,
        selected_skew_saved_frames=[402, 989], phase_episodes=8, tests=tests)
    for path in [*identity['source_files'], *paths]:
        target = BASE/'freeze/source_snapshot'/path
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT/path, target)
    dump(PROTOCOL, identity)
    shutil.copy2(PROTOCOL, BASE/'freeze/protocol.json')
    shutil.copy2(DESIGN, BASE/'freeze/design.md')
    for name in tests: shutil.copy2(CHECKS/name, BASE/'freeze'/name)
    print(json.dumps(dict(status=identity['status'], replay_frames=2063, new_physics=0)), flush=True)


def check():
    import local_pair_study as study
    from dbact_sim.experiments.study import verify_source_snapshot
    original, frozen = study.check_freeze(), read(PROTOCOL)
    assert sha(PROTOCOL) == sha(BASE/'freeze/protocol.json')
    assert sha(DESIGN) == frozen['design_sha256'] == sha(BASE/'freeze/design.md')
    assert sha(study.PROTOCOL) == frozen['physical_protocol_sha256']
    assert original['source_files'] == frozen['source_files']
    assert {s:sha(ROOT/s) for s in frozen['scripts']} == frozen['scripts']
    verify_source_snapshot(BASE/'freeze', {**frozen['source_files'], **frozen['scripts']})
    for name, value in frozen['tests'].items():
        assert sha(CHECKS/name) == sha(BASE/'freeze'/name) == value['sha256']
    return original, frozen


def replay(alias, preflight=False):
    guard()
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result
    plan, frozen = check()
    case_id, complete_end = CASES[alias]
    end = min(20, complete_end) if preflight else complete_end
    target = BASE/('preflight' if preflight else 'replay')/alias
    target.mkdir(parents=True, exist_ok=False)
    case = next(x for x in plan['cases'] if x['case_id'] == case_id)
    folder = ROOT/plan['run_dir']/case_id
    manifest = read(ROOT/plan['run_dir']/'manifest.json')
    result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
    assert result['execution_status'] == 'COMPLETED'
    identity = dict(case_id=case_id, end_frame=end, new_physical_episodes=0, preflight=preflight,
        protocol_sha256=sha(PROTOCOL), original_receipt_sha256=sha(folder/'case_result.json'))
    dump(target/'started.json', identity)
    rows = read(folder/'paired_metrics.json')['steps']
    with np.load(folder/'replay.npz', allow_pickle=False) as z:
        saved = {key:z[key].copy() for key in ('times', 'agent_positions', 'cargo/cargo_0/centers',
            'cargo/cargo_0/angles', 'sensed_counts', 'sensed_points')}
    offsets = np.r_[0, np.cumsum(saved['sensed_counts'])]
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    c, safety = env.controller, env.controller.safety
    c.trace_enabled = True
    assert c.params.perception_every == c.params.planning_every == 1
    original_prepare, original_finish = safety.prepare_velocity, safety.finish_velocity
    original_nominal, original_transport, original_neighbors = c._nominal_command, c._transport_command, c._neighbor_indices
    ids = [a.agent_id for a in env.agents]
    captured, flow, problem_ids, witnesses = {}, {}, {}, []
    frame, call_index, verified, neighbors = 0, 0, 0, []

    def linked(agents):
        nonlocal neighbors
        neighbors = original_neighbors(agents)
        return neighbors

    def nominal(i, *args, **kwargs):
        aid = ids[i]
        flow[aid] = dict(transport_called=False, transport=[0., 0.])
        result = original_nominal(i, *args, **kwargs)
        flow[aid].update(raw_nominal=np.asarray(result[0]).copy())
        return result

    def transport(i, *args, **kwargs):
        result = original_transport(i, *args, **kwargs)
        flow[ids[i]].update(transport_called=True, transport=np.asarray(result[0]).copy())
        return result

    def prepare(position, nominal_velocity, neighbor_positions=(), **kwargs):
        nonlocal call_index
        i, call_index = call_index, call_index+1
        aid = ids[i]
        problem = original_prepare(position, nominal_velocity, neighbor_positions, **kwargs)
        problem_ids[id(problem)] = aid
        flow[aid]['nominal'] = np.asarray(nominal_velocity).copy()
        if alias == 'skew' and frame in frozen['selected_skew_saved_frames']:
            mask = np.asarray(problem.row_kinds) == 'object'
            support = kwargs.get('finite_support_trace') or {}
            trace = {}
            if support.get('applied'):
                motion = np.full(problem.object_count, kwargs['finite_support_motion_bound'])
            elif problem.object_count:
                A, b, free, h = safety._object_rows(position, kwargs['boundary_points'],
                    kwargs['boundary_normals'], kwargs['object_velocity'], kwargs.get('boundary_point_velocities'), trace=trace)
                np.testing.assert_array_equal(A, problem.A[mask])
                np.testing.assert_array_equal(b, problem.b_original[mask])
                np.testing.assert_array_equal(free, problem.b_no_margin[mask])
                np.testing.assert_array_equal(h, problem.h_object)
                motion = np.asarray(trace['demand'])+safety.params.gamma_obj*h
            else:
                motion = np.empty(0)
            captured[aid] = plain(dict(agent_id=aid, position=position,
                neighbors=[ids[j] for j in neighbors[i]], A=problem.A, b=problem.b_original,
                b_no_margin=problem.b_no_margin, kinds=problem.row_kinds, h=problem.h_object,
                normal_motion=motion, nominal=problem.u_nom, params=asdict(problem.parameters), domain=c.domain,
                support=support, aggregate_trace=trace, object_velocity=kwargs['object_velocity'],
                point_velocities=kwargs.get('boundary_point_velocities'),
                boundary_points=kwargs['boundary_points'], boundary_normals=kwargs['boundary_normals']))
        return problem

    def finish(problem, **kwargs):
        aid = problem_ids.pop(id(problem))
        result = original_finish(problem, **kwargs)
        flow[aid]['actual'] = result.velocity.copy()
        if aid in captured:
            captured[aid].update(status=result.status, scale=result.barrier_scale, actual=result.velocity.tolist())
        return result

    c._neighbor_indices, c._nominal_command, c._transport_command = linked, nominal, transport
    safety.prepare_velocity, safety.finish_velocity = prepare, finish
    started = time.perf_counter()
    try:
        with gzip.open(target/'components.jsonl.gz', 'wt', encoding='utf-8') as stream:
            for k in range(end):
                frame, call_index = k+1, 0
                captured.clear(); flow.clear(); problem_ids.clear()
                row = rows[k]
                cargo = env.cargoes[0]
                cargo.set_pose(saved['cargo/cargo_0/centers'][k], saved['cargo/cargo_0/angles'][k])
                for i, agent in enumerate(env.agents):
                    agent.position = saved['agent_positions'][k, i].copy()
                    agent.velocity = np.zeros(2) if k == 0 else np.asarray(rows[k-1]['commands'][i])
                commands = c.step(env.agents, env.cargoes, float(saved['times'][k]), env.dt)
                assert call_index == len(ids) and not problem_ids
                np.testing.assert_array_equal([v.velocity for v in commands], row['commands'])
                assert recorded([vars(d) for d in c.diagnostics]) == row['diagnostics'], ('diagnostics', frame)
                assert recorded(safety.stats.as_dict()) == row['solver_statistics'], ('statistics', frame)
                assert c.phase.name == row['phase'] and recorded(vars(c.phase_signals)) == row['phase_guards']
                assert recorded(c.phase_monitor.as_dict()) == row['phase_response']
                np.testing.assert_array_equal(c.map_snapshot(env.map_snapshot_agent).points,
                    saved['sensed_points'][offsets[k+1]:offsets[k+2]])
                # Truth below is evaluation only, after every command is fixed.
                gaps, normals, _ = cargo.signed_distance(saved['agent_positions'][k])
                data = []
                for i, diag in enumerate(c.diagnostics):
                    aid = ids[i]
                    item = flow[aid]
                    overridden = not np.array_equal(item['raw_nominal'], item['nominal'])
                    data.append(dict(agent_id=aid, mode=diag.mode, push=diag.push_side,
                        transport_called=item['transport_called'], nominal_overridden=overridden,
                        true_gap=float(gaps[i]), normal=normals[i].tolist(),
                        components=normal_components(item['nominal'], item['transport'], item['actual'], normals[i])))
                    if aid in captured:
                        captured[aid].update(true_gap=float(gaps[i]), true_normal=normals[i].tolist())
                stream.write(json.dumps(plain(dict(frame=frame, phase=c.phase.name, agents=data)),
                    allow_nan=False, separators=(',', ':'))+'\n')
                if captured:
                    witnesses.append(dict(frame=frame, phase=c.phase.name,
                        reference_status=c.initial_reference_status, agents=plain(captured)))
                verified += 1
                if verified % 100 == 0:
                    print(json.dumps(dict(case=alias, verified=verified, phase=c.phase.name)), flush=True)
        dump(target/'witnesses.json', witnesses)
        check()
        dump(target/'outcome.json', dict(**identity, status='EXACT_RECONSTRUCTION',
            verified_frames=verified, elapsed_seconds=time.perf_counter()-started,
            products={name:sha(target/name) for name in ('components.jsonl.gz', 'witnesses.json')}))
        print(json.dumps(dict(case=alias, verified=verified, status='EXACT_RECONSTRUCTION')), flush=True)
    except Exception as exc:
        dump(target/'failure.json', dict(**identity, status='INVALID_RECONSTRUCTION',
            verified_frames=verified, failing_frame=frame, error=repr(exc)))
        raise


def analyze():
    guard()
    from analyze_residual_coordination import decompose, phase_contact
    from analyze_local_pair_feasibility import paired_solutions
    from dbact_sim.experiments.deployment import dump, encode
    from dbact_sim.experiments.study import cached_result
    plan, frozen = check()
    out = BASE/'analysis'
    out.mkdir(parents=True, exist_ok=False)
    phases, residuals, pairs, goals = [], [], [], {}
    try:
        for alias, (_, expected) in CASES.items():
            target = BASE/'replay'/alias
            outcome = read(target/'outcome.json')
            assert outcome['status'] == 'EXACT_RECONSTRUCTION' and outcome['verified_frames'] == expected
            assert not outcome['preflight'] and outcome['protocol_sha256'] == sha(PROTOCOL)
            for name, digest in outcome['products'].items(): assert sha(target/name) == digest
        witnesses = read(BASE/'replay/skew/witnesses.json')
        assert [w['frame'] for w in witnesses] == [402, 989]
        for witness in witnesses:
            actors = witness['agents']
            first = actors['agent_05']
            assert first['status'] == 'scaled_barrier'
            speed = first['params']['max_speed']
            modes = []
            for mode, b in systems(first).items():
                result = decompose(first['A'], b, first['kinds'], speed)
                modes.append(dict(mode=mode, b=b.tolist(), **result))
            residuals.append(dict(frame=witness['frame'], phase=witness['phase'],
                agent=first, systems=modes, true_margin=first['true_gap']-first['params']['r_safe']))
            for aid in first['neighbors']:
                second = actors[aid]
                if first['agent_id'] not in second['neighbors']:
                    raise AssertionError('Unexpected asymmetric graph in frozen no-dropout data')
                assert second['params'] == first['params']
                for mode in ('capped_full', 'uncapped_full'):
                    A, b = pair_system(first, second, mode)
                    solved = paired_solutions(A, b, np.r_[first['nominal'], second['nominal']], speed)
                    for solver in ('clarabel', 'slsqp'):
                        cert = solved[solver]
                        if not cert['valid']: continue
                        cert.update(application_certificate(first, second, cert['command'], A, b))
                    pairs.append(dict(frame=witness['frame'], peer=aid, mode=mode,
                        peer_original_status=second['status'], peer_support=second['support'],
                        A=A.tolist(), b=b.tolist(), **solved))
            print(json.dumps(dict(frame=witness['frame'], individual={m['mode']:m['groups']['joint']['bounded']['feasible'] for m in modes},
                peer_count=len(first['neighbors']))), flush=True)
        manifest = read(ROOT/plan['run_dir']/'manifest.json')
        for case in plan['cases']:
            folder = ROOT/plan['run_dir']/case['case_id']
            result = cached_result(folder, {**manifest['identity'], 'case_sha256':encode(case)})
            goals[case['case_id']] = np.asarray(result['goal_direction'])
            phases.append(phase_contact(case, folder, result))
            print(json.dumps(dict(phase_case=case['case_id'], frames=phases[-1]['frames'])), flush=True)
        c_case = CASES['c'][0]
        original_rows = read(ROOT/plan['run_dir']/c_case/'paired_metrics.json')['steps']
        component_counts = {}
        with gzip.open(BASE/'replay/c/components.jsonl.gz', 'rt', encoding='utf-8') as stream:
            for line in stream:
                frame = json.loads(line)
                row = original_rows[frame['frame']-1]
                assert frame['phase'] == row['phase']
                positive = {aid for aid, force in zip(row['contact_ids'], row['contact_forces'])
                    if float(np.asarray(force)@goals[c_case]) > 1e-9}
                counts = component_counts.setdefault(frame['phase'], Counter())
                for agent in frame['agents']:
                    counts['robot_steps'] += 1
                    if agent['push'] or not agent['transport_called'] or agent['nominal_overridden']: continue
                    counts['nonpush_component_eligible'] += 1
                    components = agent['components']
                    inward = components['nominal'] < -1e-9
                    counts['nonpush_inward_nominal'] += int(inward)
                    counts['nonpush_inward_actual'] += int(components['actual'] < -1e-9)
                    counts['nonpush_inward_coverage_outward_bias'] += int(inward and components['coverage_residual'] < -1e-9 and components['transport'] >= -1e-9)
                    counts['inward_nominal_and_positive_first_sample'] += int(inward and agent['agent_id'] in positive)
        check()
        report = dict(status='VERIFIED_DIAGNOSTIC', protocol_sha256=sha(PROTOCOL),
            runtime_source_sha256=frozen['runtime_source_sha256'], new_physical_episodes=0,
            controller_replay_frames=sum(x[1] for x in CASES.values()), residuals=residuals, pairs=pairs,
            phases=phases, phase_frames=sum(x['frames'] for x in phases),
            physics_substeps=sum(x['substep_samples'] for x in phases),
            c_components={k:dict(v) for k,v in component_counts.items()},
            component_scope='Pre-command truth normal; eligible means transport called and nominal not subsequently overridden. Positive contacts are first physics substep at post-command robot positions. Association, not causal attribution.')
        dump(out/'analysis.json', report)
        dump(out/'outcome.json', dict(status=report['status'], analysis_sha256=sha(out/'analysis.json'),
            new_physical_episodes=0, controller_replay_frames=report['controller_replay_frames'],
            phase_frames=report['phase_frames'], physics_substeps=report['physics_substeps'], pair_problems=len(pairs)))
        print(json.dumps(read(out/'outcome.json')), flush=True)
    except Exception as exc:
        dump(out/'failure.json', dict(status='INVALID_DIAGNOSTIC', error=repr(exc),
            completed_residuals=len(residuals), completed_pair_problems=len(pairs), completed_phases=len(phases)))
        raise


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=['freeze', 'replay', 'analyze'])
    parser.add_argument('--case', choices=CASES)
    parser.add_argument('--preflight', action='store_true')
    args = parser.parse_args()
    if args.command == 'replay':
        if args.case is None: parser.error('--case is required for replay')
        replay(args.case, args.preflight)
    elif args.command == 'freeze': freeze()
    else: analyze()
