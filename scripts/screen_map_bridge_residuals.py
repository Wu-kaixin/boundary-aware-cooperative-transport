"""Post-study screen of every scaled state; no replay or new physics."""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from research_io import ROOT,guard,read,sha
from map_bridge_study import BASE,OUT,PROTOCOL,ARMS,check
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump,encode
from dbact_sim.experiments.study import cached_result


def screen(out):
    guard()
    plan=check()
    report=read(BASE/'analysis.json')
    assert sha(BASE/'analysis.json')==read(BASE/'analysis_outcome.json')['analysis_sha256']
    out=out.resolve()
    assert out.is_relative_to(ROOT/'runs') and not out.exists()
    out.mkdir(parents=True)
    manifest=read(OUT/'manifest.json')
    events=[]
    for row in report['episodes']:
        if row['arm']!=ARMS[1] or not row['scaling']['events']:
            continue
        case=next(c for c in plan['cases'] if c['case_id']==row['case_id'])
        folder=OUT/case['case_id']
        cached_result(folder,{**manifest['identity'],'case_sha256':encode(case)})
        steps=read(folder/'paired_metrics.json')['steps']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            positions=z['agent_positions'].copy()
            centers=z['cargo/cargo_0/centers'].copy()
            angles=z['cargo/cargo_0/angles'].copy()
        env=SimulationEnvironment(case['config'],seed=case['seed'])
        ids=[a.agent_id for a in env.agents]
        for event in row['scaling']['events']:
            frame=event['frame']; k=frame-1
            cargo=env.cargoes[0]
            cargo.set_pose(centers[k],angles[k])
            for aid in event['agents']:
                diag=next(d for d in steps[k]['diagnostics'] if d['agent_id']==aid)
                gap=float(cargo.signed_distance(positions[k,ids.index(aid)][None,:])[0][0])
                displacement=np.asarray(diag['map_reference_displacement'])
                actual=centers[k]-centers[0]
                bridge=diag['map_reference_bridge']
                raw=None if bridge is None else np.asarray(bridge['raw_displacement'])
                events.append(dict(case_id=case['case_id'],saved_frame=frame,agent_id=aid,
                    phase=steps[k]['phase'],capture_state=event['capture_state'],
                    true_gap_m=gap,true_displacement=actual.tolist(),
                    map_displacement_error_m=(displacement-actual).tolist(),
                    raw_aggregate_error_m=None if raw is None else (raw-actual).tolist(),
                    map_displacement_error_norm_m=float(np.linalg.norm(displacement-actual)),
                    true_angle_rad=float(angles[k]-angles[0]),diagnostic=diag))
    expected=sum(r['scaling']['scaled_robot_steps'] for r in report['episodes'] if r['arm']==ARMS[1])
    assert len(events)==expected
    report=dict(status='VERIFIED_POST_STUDY_SCREEN',protocol_sha256=sha(PROTOCOL),
        raw_analysis_sha256=sha(BASE/'analysis.json'),script_sha256=sha(__file__),
        new_physical_episodes=0,new_replay_frames=0,scaled_robot_steps=len(events),events=events,
        scope='All candidate scaled states from the completed eight-episode study. Pre-command truth is '
              'evaluation only. Map/reference errors do not identify the selected safety row or prove '
              'the cause of infeasibility; a frozen exact row replay is the next discriminating step.')
    dump(out/'screen.json',report)
    dump(out/'outcome.json',dict(status=report['status'],screen_sha256=sha(out/'screen.json'),
        scaled_robot_steps=len(events)))
    for e in events:
        print({k:e[k] for k in ('saved_frame','agent_id','phase','true_gap_m','map_displacement_error_m','raw_aggregate_error_m')})


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    screen(parser.parse_args().out)
