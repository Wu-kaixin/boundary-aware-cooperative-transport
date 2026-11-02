"""Signed accounting of a verified historical reconstruction, not a causal proof."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from dbact_sim.experiments.deployment import dump


def median_weights(raw, live):
    weights = np.zeros_like(raw)
    for k in range(len(raw)):
        ids = np.flatnonzero(live[k])
        if not len(ids):
            continue
        for axis in range(2):
            order = ids[np.argsort(raw[k,ids,axis],kind="stable")]
            weights[k,order[(len(ids)-1)//2],axis] += .5
            weights[k,order[len(ids)//2],axis] += .5
    return weights


def stationary_analysis(folder):
    with np.load(folder/'reconstruction.npz',allow_pickle=False) as z:
        raw,truth,angle,q,m=[z[k] for k in ('raw','truth','angle','quality','matches')]
    weights=median_weights(raw,(q==2)&(m>0))
    delta=np.diff(raw,axis=0)
    stationary=(np.linalg.norm(np.diff(truth,axis=0),axis=1)<1e-14)&(np.abs(np.diff(angle))<1e-14)
    motion=1.24*(weights[1:]*delta).sum(axis=1)
    switch=1.24*(np.diff(weights,axis=0)*raw[:-1]).sum(axis=1)
    result={'definition':'Both recorded translation and yaw increments <1e-14; numerical stationary diagnostic, not sensor truth available to control',
        'stationary_intervals':int(stationary.sum()),'total_intervals':len(stationary),
        'seconds':float(stationary.sum()*.05),
        'weighted_motion_on_stationary_xy_m':motion[stationary].sum(axis=0).tolist(),
        'voter_change_on_stationary_xy_m':switch[stationary].sum(axis=0).tolist(),
        'agent04_raw_stationary_drift_xy_m':delta[stationary,4].sum(axis=0).tolist(),
        'agent04_stationary_path_length_m':float(np.linalg.norm(delta[stationary,4],axis=1).sum())}
    path=folder/'stationary_drift.json'
    if path.exists():
        assert json.loads(path.read_text())==result, 'Existing derivative changed'
    else:dump(path,result)
    print(json.dumps(result,indent=2))


def analyze(folder):
    if (folder/'signed_accounting.json').exists():
        raise RuntimeError('Preserve previous analysis')
    with np.load(folder/'reconstruction.npz',allow_pickle=False) as z:
        raw,reg,fusion,truth,team,t,q,m,angle = [z[k].copy() for k in
            ('raw','registration','fusion','truth','team_navigation','time','quality','matches','angle')]
    live = (q==2)&(m>0)
    w = median_weights(raw,live)
    valid = np.flatnonzero(live.any(axis=1))
    assert np.all(np.diff(valid)==1), 'Unavailable gaps need explicit accounting'
    first = valid[0]
    weights = w[first:]
    base, tr, tm = raw[first:],truth[first:],team[first:]
    np.testing.assert_allclose((1.24*weights*base).sum(axis=1),tm,atol=1e-14,rtol=0)
    parts=np.zeros((len(valid),5,2))
    parts[0,0] = tm[0]-tr[0]
    parts[1:,1] = (weights[1:]*reg[first+1:]).sum(axis=1)-np.diff(tr,axis=0)
    parts[1:,2] = (weights[1:]*fusion[first+1:]).sum(axis=1)
    parts[1:,3] = .24*(weights[1:]*np.diff(base,axis=0)).sum(axis=1)
    parts[1:,4] = 1.24*(np.diff(weights,axis=0)*base[:-1]).sum(axis=1)
    accum=np.cumsum(parts,axis=0)
    residual=float(np.max(np.abs(accum.sum(axis=1)-(tm-tr))))
    if residual > 1e-11: raise RuntimeError('Signed accounting failed')
    intervals=[]
    for lo,hi in ((0,30),(30,150),(150,300),(300,450),(450,600)):
        take=(t[first:]>=lo)&(t[first:]<hi)
        intervals.append(dict(start_s=lo,end_s=hi,components_xy_m=parts[take].sum(axis=0).tolist()))
    result=dict(identity='Verified reconstructed inputs; same historical trajectory, not new experiments',
        residual_max_m=residual,labels=['initial error','registration minus true increment','fusion','fixed gain','voter switch'],
        final_components_xy_m=accum[-1].tolist(),final_signed_error_xy_m=(tm[-1]-tr[-1]).tolist(),
        final_raw_median_xy_m=(weights[-1]*base[-1]).sum(axis=0).tolist(),
        final_gain_contribution_xy_m=(.24*weights[-1]*base[-1]).sum(axis=0).tolist(),
        final_truth_xy_m=tr[-1].tolist(),final_supervisor_displacement_xy_m=tm[-1].tolist(),
        final_angle_rad=float(angle[-1]),intervals=intervals,
        voter_switch_frames=int(np.count_nonzero(np.any(np.diff(weights,axis=0)!=0,axis=(1,2)))),
        per_agent_final_error_xy_m=(1.24*raw[-1]-truth[-1]).tolist(),
        caveat='Coordinate-median ties ordered by agent ID. Component allocation depends on tie rule; this is an additive identity, not isolated causal effects. Full-rank matching does not certify absolute accuracy.')
    dump(folder/'signed_accounting.json',result)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size':8,'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,axes=plt.subplots(3,1,figsize=(7.16,6.6),layout='constrained')
    axes[0].plot(t,truth[:,0],label='Physical x displacement (evaluation)')
    axes[0].plot(t,team[:,0],label='Shared controller x estimate',ls='--')
    axes[0].plot(t,truth[:,1],label='Physical y displacement',alpha=.8)
    axes[0].plot(t,team[:,1],label='Shared controller y estimate',ls=':')
    axes[0].set_ylabel('Displacement (m)');axes[0].legend(ncol=2,fontsize=7)
    for a,axis in enumerate(axes[1:]):
        for j,label in enumerate(result['labels']):axis.plot(t[first:],accum[:,j,a],label=label,lw=.9)
        axis.plot(t[first:],tm[:,a]-tr[:,a],'k--',label='sum = signed error',lw=1.2)
        axis.set_ylabel(('x' if a==0 else 'y')+' cumulative error (m)')
        axis.grid(alpha=.2)
    axes[1].legend(ncol=2,fontsize=7);axes[-1].set_xlabel('Physical time (s)')
    for ext in ('pdf','svg','png'):fig.savefig(folder/f'signed_accounting.{ext}',dpi=200)
    plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--folder',type=Path,required=True)
    p.add_argument('--stationary-only',action='store_true');args=p.parse_args()
    stationary_analysis(args.folder) if args.stationary_only else analyze(args.folder)
