"""Reproduce the descriptive audit of the frozen far-field hard-safety failure."""
import argparse
import json
import numpy as np
from dbact.geometry import signed_distance_to_polygon, rotate
from dbact.control.phase import Phase
from prepare_cargo_discovery import ROOT, MISSION, sha


def audit(dry_run=False):
    folder=ROOT/'runs/research'/MISSION/'far/c_open_away_pos1_initial_location_hint'
    metrics=json.loads((folder/'summary.json').read_text())['cargoes']['cargo_0']['g500']['metrics']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        t,xy,angles,local,robots,phase=[z[k].copy() for k in ('times',
            'cargo/cargo_0/centers','cargo/cargo_0/angles','cargo/cargo_0/local_vertices',
            'agent_positions','phase')]
    dist=np.array([signed_distance_to_polygon(robots[k],rotate(local,angles[k])+xy[k]) for k in range(len(t))])
    series=json.loads((ROOT/'artifacts/research'/MISSION/'far-case-analysis/c_open_away_pos1_initial_location_hint.json').read_text())['series']
    first=int(np.flatnonzero(np.min(dist,axis=1)<0)[0]);idx=int(np.argmin(dist[first]))
    worst=np.unravel_index(np.argmin(dist),dist.shape)
    record=dict(case='c_open_away_pos1_initial_location_hint',
        scope='Post-physics saved frames only; original contract includes its own full-record metrics. No rerun or rule change.',
        first_saved_center_inside=dict(frame=first,time_s=float(t[first]),agent_index=idx,
            signed_clearance_m=float(dist[first,idx]),phase=Phase(int(phase[first])).name,
            preceding_control_estimation_error_m=series['navigation_error'][first-1]),
        worst_saved_center_inside=dict(frame=int(worst[0]),time_s=float(t[worst[0]]),
            agent_index=int(worst[1]),signed_clearance_m=float(dist[worst])),
        original_contract={k:metrics[k] for k in ('min_signed_clearance','max_penetration',
            'penetration_budget','min_inter_agent_distance','solver_fallbacks','solver_infeasible')},
        inputs={(folder/n).relative_to(ROOT).as_posix():sha(folder/n) for n in ('replay.npz','summary.json','resolved_case.json')},
        conclusion='Hard safety violation retained. This descriptive temporal audit does not isolate a causal estimation or contact-control fault.')
    output=ROOT/'artifacts/research'/MISSION/'far_safety_failure.json'
    if output.exists():
        assert json.loads(output.read_text())==record, 'Preserve conflicting audit; do not overwrite'
    elif not dry_run:
        output.write_text(json.dumps(record,indent=2),encoding='utf-8')
    print(json.dumps(record))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run',action='store_true')
    audit(parser.parse_args().dry_run)
