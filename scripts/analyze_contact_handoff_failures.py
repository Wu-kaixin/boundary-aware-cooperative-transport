"""Read the frozen v1 failures before separating allocation and relocation."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from dbact_sim.experiments.deployment import dump
from dbact_sim.experiments.study import cached_result

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / 'runs/research/2026-11-01-dynamic-contact-v1/run'
OUT = ROOT / 'artifacts/research/2026-11-01-contact-handoff-v1/offline'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def mean_window(steps, start, end):
    selected = steps[max(0,start):min(len(steps),end)]
    if not selected:
        return None
    return {
        'frames':len(selected),
        'mean_contact_count':float(np.mean([s['contacts'] for s in selected])),
        'mean_push_flags':float(np.mean([sum(d['push_side'] for d in s['diagnostics']) for s in selected])),
        'mean_forward_force_N':float(np.mean([s['forward_force'] for s in selected])),
        'mean_true_forward_speed_m_s':float(np.mean([s['true_velocity'][0] for s in selected])),
    }


def main():
    plan=read(OLD/'resolved_protocol.json')
    results=[]; inputs={}
    for condition in plan['conditions']:
        name=condition['condition']; groups={}
        for arm in ('legacy','feedback_fix','dynamic_contact'):
            folder=OLD/f'{name}_{arm}'
            receipt=read(folder/'case_result.json')
            cached_result(folder,receipt['identity'])
            inputs[str((folder/'case_result.json').relative_to(ROOT))]=hashlib.sha256((folder/'case_result.json').read_bytes()).hexdigest()
            groups[arm]=read(folder/'paired_metrics.json')
        candidate=groups['dynamic_contact']['steps']; fixed=groups['feedback_fix']['steps']
        events=[]
        for k,s in enumerate(candidate):
            for i,d in enumerate(s['diagnostics']):
                if d['contact_redeploy_reason']!='committed_new':
                    continue
                previous=candidate[k-1] if k else None
                events.append({'frame':s['frame'],'agent_id':d['agent_id'],
                               'previous_push_flag':previous['diagnostics'][i]['push_side'] if previous else None,
                               'previous_recorded_physical_contact':d['agent_id'] in previous['contact_ids'] if previous else None,
                               'before_0_5s':mean_window(candidate,k-10,k),
                               'after_1s':mean_window(candidate,k+1,k+21)})
        first_difference=next((a['frame'] for a,b in zip(fixed,candidate)
                               if a['commands']!=b['commands']),None)
        first_move=events[0]['frame'] if events else None
        first_suspend=next((s['frame'] for s in candidate if s['phase_response']['push_suspended']),None)
        pre_move_frames=max(0,(first_move or 0)-1)
        summaries={arm:{k:d['summary'][k] for k in ('task_completed','settled','watchdog','euclidean_endpoint_error','events')}
                   for arm,d in groups.items()}
        results.append({'condition':name,'summaries':summaries,'first_command_difference_vs_fix':first_difference,
                        'first_redeployment':first_move,'allocation_diverges_before_moving':bool(first_difference and first_move and first_difference<first_move),
                        'first_push_suspension':first_suspend,'relocations':len(events),
                        'departing_previous_pushers':sum(e['previous_push_flag'] is True for e in events),
                        'departing_previous_physical_contacts':sum(e['previous_recorded_physical_contact'] is True for e in events),
                        'before_first_move_candidate':mean_window(candidate,pre_move_frames-20,pre_move_frames),
                        'same_clock_feedback_fix':mean_window(fixed,pre_move_frames-20,pre_move_frames),
                        'events':events})
        del groups,candidate,fixed
    conclusion={'scope':'Post-hoc diagnostic of nine fixed seed-2 conditions; event windows are descriptive, not isolated interventions',
                'cases':results,'inputs':inputs,
                'analysis_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'time_limit_limit':'Existing settled-outside-target failures already terminated in HOLD; adding a watchdog budget alone cannot restart their controller',
                'next_ablation':{'conditions':['circle','rectangle_parallel','c_open_away'],
                                 'arms':['feedback_fix','allocation_only','relocation_only','joint_v1'],
                                 'episodes':12,'seed':2,'frames':3000,
                                 'held_fixed':'safety feedback enabled in all four arms, all sensing/physics/phase/criteria as v1'}}
    dump(OUT/'failure_analysis.json',conclusion)
    lines=['| Condition | First command divergence | First move | Departing prior pushers | Prior physical contacts |',
           '|---|---:|---:|---:|---:|']
    for r in results:
        lines.append(f"| {r['condition']} | {r['first_command_difference_vs_fix']} | {r['first_redeployment']} | {r['departing_previous_pushers']}/{r['relocations']} | {r['departing_previous_physical_contacts']}/{r['relocations']} |")
    (OUT/'failure_table.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('\n'.join(lines))


if __name__=='__main__':
    main()
