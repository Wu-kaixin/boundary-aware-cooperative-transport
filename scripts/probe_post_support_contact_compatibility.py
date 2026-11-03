"""Exactly replay completed controls and inspect finite rows at force events."""
from __future__ import annotations

import os
for _key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[_key]='1'

from collections import Counter
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import traceback

import numpy as np

from research_io import ROOT,guard,read,sha
from replay_qp_conflict import recorded
from trace_boundary_map import plain
import post_acquisition_support_study as study
from dbact.control.post_support import filter_inputs
from dbact.safety.filter import SafetyFilter,SafetyFilterParams
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.study import cached_result


BASE=ROOT/'runs/research/2026-11-03-post-support-contact-compatibility-v1'
DESIGN=ROOT/'experiments/ccta2027/post_support_contact_compatibility_v1_design.md'


def selection(rows):
    eligible=[i for i,r in enumerate(rows) if r['initial_reference']['status']=='frozen'
              and r['phase'] not in ('BRAKE','HOLD') and r['positive_force_sum']>0.]
    if not eligible:
        return {}
    first=eligible[0]
    peak=max(eligible,key=lambda i:rows[i]['positive_force_sum'])
    selected={}
    selected.setdefault(first,[]).append('first_positive_force')
    selected.setdefault(peak,[]).append('maximum_positive_force')
    return selected


def replay(case,folder,selected):
    rows=read(folder/'paired_metrics.json')['steps']
    if not selected:
        return dict(condition=case['condition_id'],verified_frames=0,selection={},events=[],missing=True)
    end=max(selected)+1
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        saved={key:z[key].copy() for key in ('times','agent_positions',
            'cargo/cargo_0/centers','cargo/cargo_0/angles','sensed_counts','sensed_points','task/cargo_0/direction')}
    offsets=np.r_[0,np.cumsum(saved['sensed_counts'])]
    env=SimulationEnvironment(case['config'],seed=case['seed'])
    controller=env.controller
    controller.trace_enabled=True
    original=controller.safety.prepare_velocity
    ids=[agent.agent_id for agent in env.agents]
    frame,call_index=0,0
    captured={}
    events=[]
    goal=np.asarray(saved['task/cargo_0/direction']).reshape(2)

    def prepare(position,nominal_velocity,neighbor_positions=(),**kwargs):
        nonlocal call_index
        i,call_index=call_index,call_index+1
        problem=original(position,nominal_velocity,neighbor_positions,**kwargs)
        if frame-1 not in selected:
            return problem
        aid=ids[i]
        neighbor_ids=[]
        for point in neighbor_positions:
            matches=[a.agent_id for a in env.agents if np.array_equal(a.position,point)]
            assert len(matches)==1 and matches[0]!=aid
            neighbor_ids.append(matches[0])
        current,trace=filter_inputs(controller,aid,neighbor_ids,float(saved['times'][frame-1]))
        counter=SafetyFilter(SafetyFilterParams(**asdict(problem.parameters)))
        alternative=counter.prepare_velocity(position,nominal_velocity,neighbor_positions,**kwargs,**current)
        hard=np.asarray(problem.row_kinds)!='object'
        alternative_hard=np.asarray(alternative.row_kinds)!='object'
        np.testing.assert_array_equal(problem.A[hard],alternative.A[alternative_hard])
        np.testing.assert_array_equal(problem.b_original[hard],alternative.b_original[alternative_hard])
        result=counter.finish_velocity(alternative)
        captured[aid]=dict(original=problem,alternative=alternative,result=result,trace=trace)
        return problem

    controller.safety.prepare_velocity=prepare
    for k,row in enumerate(rows[:end]):
        frame,call_index=k+1,0
        captured.clear()
        env.cargoes[0].set_pose(saved['cargo/cargo_0/centers'][k],saved['cargo/cargo_0/angles'][k])
        for i,agent in enumerate(env.agents):
            agent.position=saved['agent_positions'][k,i].copy()
            agent.velocity=np.zeros(2) if k==0 else np.asarray(rows[k-1]['commands'][i])
        commands=controller.step(env.agents,env.cargoes,float(saved['times'][k]),env.dt)
        assert call_index==len(ids)
        np.testing.assert_array_equal([v.velocity for v in commands],row['commands'])
        assert recorded([vars(d) for d in controller.diagnostics])==row['diagnostics'],('diagnostics',frame)
        assert recorded(controller.safety.stats.as_dict())==row['solver_statistics']
        assert controller.phase.name==row['phase'] and recorded(vars(controller.phase_signals))==row['phase_guards']
        assert recorded(controller.phase_monitor.as_dict())==row['phase_response']
        np.testing.assert_array_equal(controller.map_snapshot(env.map_snapshot_agent).points,
            saved['sensed_points'][offsets[k+1]:offsets[k+2]])
        if k in selected:
            forces=Counter()
            for aid,force in zip(row['contact_ids'],row['contact_forces']):
                forces[aid]+=max(0.,float(np.asarray(force)@goal))
            states=[]
            for i,aid in enumerate(ids):
                item=captured[aid]
                old,new,result,trace=item['original'],item['alternative'],item['result'],item['trace']
                u=np.asarray(row['commands'][i])
                mask=np.asarray(new.row_kinds)=='object'
                capped=new.A[mask]@u-new.b_original[mask]
                uncapped=(new.A[mask]@u-(trace['demand_speed']-new.parameters.gamma_obj*new.h_object+new.parameters.rho)
                          if trace['applied'] else None)
                states.append(dict(agent_id=aid,positive_physical_force=float(forces[aid]),
                    declared_pusher=row['diagnostics'][i]['push_side'],support=trace,
                    original_command=u.tolist(),counterfactual_command=result.velocity.tolist(),
                    original_goal_velocity=float(u@goal),counterfactual_goal_velocity=float(result.velocity@goal),
                    counterfactual_status=result.status,counterfactual_barrier_scale=result.barrier_scale,
                    original_command_min_capped_object_residual=float(capped.min()) if len(capped) else None,
                    original_command_min_uncapped_object_residual=None if uncapped is None else float(uncapped.min()),
                    original_rows=dict(A=old.A,b=old.b_original,h=old.h_object,kinds=old.row_kinds),
                    counterfactual_rows=dict(A=new.A,b=new.b_original,h=new.h_object,kinds=new.row_kinds)))
            events.append(dict(saved_frame=frame,recorded_frame=row['frame'],selection_rules=selected[k],
                phase=row['phase'],positive_force_sum=row['positive_force_sum'],states=states))
        if frame%200==0:
            print(json.dumps(dict(condition=case['condition_id'],verified_frames=frame)),flush=True)
    return plain(dict(condition=case['condition_id'],verified_frames=end,
        selection={str(k+1):v for k,v in selected.items()},events=events,missing=False))


def main():
    guard()
    plan=study.check()
    manifest=read(study.OUT/'manifest.json')
    assert manifest['identity']==study.identity(plan,study.PROTOCOL)
    cases=[c for c in plan['cases'] if c['arm']=='map_bridge_control']
    assert len(cases)==4
    schedule=[]
    for case in cases:
        folder=study.OUT/case['case_id']
        result=cached_result(folder,{**manifest['identity'],'case_sha256':encode(case)})
        assert result['execution_status']=='COMPLETED'
        rows=read(folder/'paired_metrics.json')['steps']
        schedule.append(dict(case=case,selection=selection(rows),receipt_sha256=sha(folder/'case_result.json')))
    identity=dict(protocol_sha256=sha(study.PROTOCOL),runtime_source_sha256=plan['frozen_source_sha256'],
        script_sha256=sha(__file__),design_sha256=sha(DESIGN),
        serialization_helper_sha256=sha(ROOT/'scripts/trace_boundary_map.py'),new_physical_episodes=0,
        all_four_conditions=[c['condition_id'] for c in cases],schedule=schedule)
    BASE.mkdir(parents=True,exist_ok=False)
    shutil.copy2(__file__,BASE/'probe_script.py')
    shutil.copy2(DESIGN,BASE/'design.md')
    shutil.copy2(ROOT/'scripts/trace_boundary_map.py',BASE/'serialization_helper.py')
    dump(BASE/'started.json',identity)
    results=[]
    try:
        for item in schedule:
            case=item['case']
            folder=study.OUT/case['case_id']
            assert sha(folder/'case_result.json')==item['receipt_sha256']
            result=replay(case,folder,item['selection'])
            results.append(result)
            dump(BASE/(case['condition_id']+'.json'),result)
        study.check()
        assert sha(__file__)==identity['script_sha256'] and sha(DESIGN)==identity['design_sha256']
        assert sha(ROOT/'scripts/trace_boundary_map.py')==identity['serialization_helper_sha256']
        dump(BASE/'analysis.json',dict(**identity,results=results,
            scope='Exact control replay and finite-support counterfactual at preselected force events. No new physical outcomes.'))
        dump(BASE/'outcome.json',dict(status='VERIFIED_CONTACT_COMPATIBILITY',conditions=4,
            verified_frames=sum(r['verified_frames'] for r in results),analysis_sha256=sha(BASE/'analysis.json')))
    except Exception:
        dump(BASE/'failure.json',dict(status='INCOMPLETE',completed_conditions=len(results),traceback=traceback.format_exc()))
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,default=BASE)
    BASE=parser.parse_args().out.resolve()
    if not BASE.is_relative_to((ROOT/'runs').resolve()):
        raise ValueError('Use a fresh output directory under runs/')
    main()
