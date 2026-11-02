"""One frozen same-input test of innovation trimming; no physical intervention."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from dbact.cpu_budget import limit_blas_threads
from dbact.control.controller import _merge_scans
from dbact.control.observed_reference import ObservedReferenceTracker
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import dump
from dbact_sim.experiments.study import cached_result
from prepare_cargo_discovery import ROOT, MISSION, AUDIT, sha

def main():
    limit_blas_threads(1)
    folder = ROOT/'runs/research/2026-11-02-contact-consistency-v1/run/c_open_away_consistent_aggregate'
    out = ROOT/'artifacts/research'/MISSION/'shadow'
    out.mkdir(exist_ok=False)
    receipt = json.loads((folder/'case_result.json').read_text())
    cached_result(folder, receipt['identity'])
    case = json.loads((folder/'resolved_case.json').read_text())
    steps = json.loads((folder/'paired_metrics.json').read_text())['steps']
    with np.load(folder/'replay.npz', allow_pickle=False) as z:
        data = {k:z[k].copy() for k in ('times','agent_positions','cargo/cargo_0/centers',
                                      'cargo/cargo_0/angles','task/cargo_0/start')}
    # Fixed before evaluating the candidate. No second variant if this fails.
    gate = dict(minimum_relative_MAE_reduction=.25, maximum_final_displacement_error_m=.12,
                minimum_fresh_feedback_fraction=.90, wall_seconds=1800,
                window='from first historical reference bootstrap to termination',
                paired_MAE='frames on which original and candidate both have a fresh team reading')
    dump(out/'frozen_gate.json', dict(gate=gate,source_sha256={p:sha(ROOT/p) for p in (
        'src/dbact/control/observed_reference.py','scripts/audit_cargo_reference.py')},
        input_receipt_sha256=sha(folder/'case_result.json'), products=receipt['products']))
    env = SimulationEnvironment(case['config'], seed=case['seed'])
    c = env.controller
    assert c.params.perception_every == 1
    trackers = [{},{}]
    estimates = np.full((2,len(steps),2),np.nan)
    errors = []
    started = time.perf_counter()
    bootstrap = min(k for k,s in enumerate(steps) if any(d['reference_status']=='bootstrap_only' for d in s['diagnostics']))
    exact = 0
    for k,s in enumerate(steps):
        if time.perf_counter()-started > gate['wall_seconds']:
            raise TimeoutError('Fixed offline budget')
        t = data['times'][k]
        env.cargoes[0].set_pose(data['cargo/cargo_0/centers'][k],data['cargo/cargo_0/angles'][k])
        for i,a in enumerate(env.agents): a.position=data['agent_positions'][k,i].copy()
        scans = [c.sensor.sense_view(a,env.cargoes,t) for a in env.agents]
        neighbors = c._neighbor_indices(env.agents)
        votes = [[],[]]
        for i,a in enumerate(env.agents):
            d=s['diagnostics'][i]
            assert len(scans[i]) == d['own_scan_points'], (k,i)
            if k < bootstrap: continue
            batch,_ = _merge_scans(scans,[i]+neighbors[i])
            for arm in (0,1):
                tracker = trackers[arm].get(a.agent_id)
                if d['reference_status']=='bootstrap_only':
                    tracker = ObservedReferenceTracker(gate=c.params.registration_gate,
                        normal_cosine=c.params.registration_normal_cosine,max_speed=c.params.max_object_speed,
                        max_yaw_rate=c.params.max_object_yaw_rate,velocity_filter=c.params.object_velocity_filter,
                        radial_readout=c.params.reference_radial_pivot,range_noise_std=c.params.range_noise_std,
                        innovation_trim=bool(arm))
                    tracker.initialize(batch,d['reference_bootstrap_offset'],t)
                    trackers[arm][a.agent_id]=tracker
                    reading=None
                elif tracker is not None:
                    reading=tracker.update(batch,t)
                else:
                    reading=None
                if arm == 0 and tracker is not None:
                    assert tracker.reason == d['reference_status'], (k,i,tracker.reason,d['reference_status'])
                    if reading is not None:
                        np.testing.assert_array_equal(reading.displacement,d['reference_displacement'])
                        np.testing.assert_array_equal(reading.velocity,d['reference_velocity'])
                        assert reading.matches == d['reference_matches']
                        exact += 1
                if reading is not None: votes[arm].append(reading.displacement)
        for arm in (0,1):
            if votes[arm]: estimates[arm,k]=np.median(votes[arm],axis=0)
    truth = data['cargo/cargo_0/centers'][:-1]-data['task/cargo_0/start']
    window = np.arange(len(steps)) > bootstrap
    available = np.isfinite(estimates).all(axis=2)
    paired = window & available.all(axis=0)
    errors = np.linalg.norm(estimates-truth[None,:,:],axis=2)
    original, candidate = [float(e[paired].mean()) for e in errors]
    fresh = float(available[1,window].mean())
    final = float(errors[1,-1]) if available[1,-1] else None
    passed = bool(candidate <= .75*original and fresh >= .90 and final is not None and final <= .12)
    np.savez_compressed(out/'same_input.npz',time=data['times'][:-1],truth=truth,estimates=estimates)
    result=dict(status='PASS' if passed else 'FAIL',exact_historical_accepted_readings=exact,
        frames=len(steps),paired_frames=int(paired.sum()),original_MAE_m=original,candidate_MAE_m=candidate,
        fractional_reduction=1-candidate/original,fresh_fraction=fresh,final_error_m=final,
        wall_seconds=time.perf_counter()-started,gate=gate,
        decision='Run one 18-case closed-loop comparison' if passed else 'Stop candidate expansion; common four-method batch retains previous corrected controller',
        scope='Same physical trajectory and local inputs; selected development case, not closed-loop improvement or independent samples')
    dump(out/'analysis.json',result)
    print(json.dumps(result),flush=True)

if __name__=='__main__':main()
