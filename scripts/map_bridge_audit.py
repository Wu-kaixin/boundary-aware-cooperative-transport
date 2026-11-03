"""Audit every recorded map bridge against actual fresh, permitted messages."""
from __future__ import annotations

from collections import Counter

import numpy as np

from research_io import read
from dbact.control.initial_task_reference import geometric_median


def audit(folder,case):
    steps=read(folder/'paired_metrics.json')['steps']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        positions=z['agent_positions'].copy()
    previous,applied,started={},{},{}
    counts=Counter()
    max_continuous_step=0.
    resets=[]
    assert case['config']['controller'].get('communication_dropout_prob',0.)==0.
    for k,row in enumerate(steps):
        diags={d['agent_id']:d for d in row['diagnostics']}
        ids=list(diags)
        fresh={aid:d for aid,d in diags.items() if d.get('reference_displacement') is not None
               and all(x is not None for x in d['reference_displacement'])}
        for i,aid in enumerate(ids):
            d=diags[aid]
            trace=d['map_reference_bridge']
            if trace is None:
                assert d['map_reference_status']=='unavailable'
                previous[aid]={}
                counts['unavailable']+=1
                continue
            status=trace['status']
            counts[status]+=1
            if status=='stationary':
                assert d['map_reference_status']=='stationary_acquisition'
                np.testing.assert_array_equal(trace['displacement'],[0.,0.])
                assert not trace['reset_history']
                previous[aid],applied[aid],started[aid]={},np.zeros(2),False
                continue
            permitted=[ids[j] for j in range(len(ids)) if i==j or
                np.linalg.norm(positions[k,i]-positions[k,j])<=case['config']['controller']['comm_range']]
            current={a:np.asarray(fresh[a]['reference_displacement']) for a in permitted if a in fresh}
            assert set(trace['current'])==set(current)==set(trace['voters'])==set(d['map_reference_voters'])
            for a in current:
                np.testing.assert_array_equal(trace['current'][a],current[a])
            prior=previous.get(aid,{})
            assert set(prior)==set(trace['previous'])
            for a in prior:
                np.testing.assert_array_equal(trace['previous'][a],prior[a])
            common=[a for a in current if a in prior]
            assert set(common)==set(trace['common_voters'])
            if common:
                expected_delta=geometric_median(np.array([current[a]-prior[a] for a in common]))
                expected=applied[aid]+expected_delta
                assert status=='continuous' and not trace['reset_history']
                max_continuous_step=max(max_continuous_step,float(np.linalg.norm(expected_delta)))
                velocity=geometric_median(np.array([fresh[a]['reference_velocity'] for a in common]))
            else:
                expected=geometric_median(np.array(list(current.values())))
                expected_delta=expected-applied.get(aid,np.zeros(2))
                velocity=geometric_median(np.array([fresh[a]['reference_velocity'] for a in current]))
                reset=started.get(aid,False)
                assert trace['reset_history'] is reset
                assert status==('reanchored' if reset else 'anchored')
                if reset:
                    assert trace['retained_cells_before_alignment']==0
                    resets.append(dict(frame=k+1,agent_id=aid,discarded_cells=trace['discarded_cells']))
            np.testing.assert_allclose(trace['delta'],expected_delta,atol=1e-12,rtol=0.)
            np.testing.assert_allclose(trace['displacement'],expected,atol=1e-12,rtol=0.)
            np.testing.assert_allclose(d['map_reference_displacement'],expected,atol=1e-12,rtol=0.)
            np.testing.assert_allclose(d['map_reference_velocity'],velocity,atol=1e-12,rtol=0.)
            previous[aid],applied[aid],started[aid]=current,expected,True
    assert sum(counts.values())==len(steps)*len(ids)
    return dict(verified=True,frames=len(steps),agent_steps=sum(counts.values()),counts=dict(counts),
        maximum_continuous_step_m=max_continuous_step,resets=resets,
        scope='All fresh-message membership and bridge arithmetic; reset cell counts are runtime diagnostics, '
              'with object-specific discard behavior independently covered by regression tests. No geometry certificate.')
