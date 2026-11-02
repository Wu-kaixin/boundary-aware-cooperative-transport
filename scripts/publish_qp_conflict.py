"""Publish the full diagnosis, plus post-hoc linear infeasibility witnesses."""
from __future__ import annotations

import csv
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np
from scipy.optimize import linprog

import qp_conflict_study as study
from research_io import guard,read,sha
from dbact_sim.experiments.deployment import dump


def main():
    guard()
    plan = study.check()
    public = study.PUBLIC
    analysis = read(public/'analysis.json')
    assert sha(public/'analysis.json')==sha(study.BASE/'analysis.json')
    assert analysis['all_selected_independent_agree']
    names = ('all_scaled_robot_steps.csv','four_cases.csv','linear_witnesses.json',
             'velocity_constraints.png','velocity_constraints.pdf','publication.json')
    if any((public/n).exists() for n in names):
        raise RuntimeError('Preserve existing publication')
    records = []
    for key in plan['prefixes']:
        replay = read(study.BASE/key/'reconstruction.json')
        expected = next(s for s in analysis['cases'] if s['case_id']==key)
        assert sha(study.BASE/key/'reconstruction.json')==expected['reconstruction_sha256']
        for e in replay['events']:
            records.append(dict(case_id=key,frame=e['frame'],agent_id=e['agent_id'],phase=e['phase'],
                acquisition_status=e['acquisition_status'],scale=e['scale'],observed_min_h_m=e['minimum_h'],
                true_h_m=e['true_h_m'],true_signed_distance_m=e['true_signed_distance_m'],
                raw_estimated_speed_m_s=float(np.linalg.norm(e['object_velocity'])),
                true_speed_m_s=float(np.linalg.norm(e['true_velocity'])),
                min_normal_alignment=e['min_normal_alignment'],**e['feasibility']))
    assert len(records)==analysis['scaled_robot_steps']==953
    with (public/names[0]).open('x',newline='',encoding='utf-8') as out:
        writer=csv.DictWriter(out,fieldnames=list(records[0]))
        writer.writeheader();writer.writerows(records)
    compact = [dict(case_id=c['case_id'],frames=c['verified_frames'],scaled_robot_steps=c['scaled_robot_steps'],
        **c['feasible_counts'],observed_h_negative=c['observed_h_negative'],true_h_negative=c['true_h_negative'],
        min_scale=c['min_scale']) for c in analysis['cases']]
    with (public/names[1]).open('x',newline='',encoding='utf-8') as out:
        writer=csv.DictWriter(out,fieldnames=list(compact[0]))
        writer.writeheader();writer.writerows(compact)
    certificates = {}
    for key,w in analysis['witnesses'].items():
        case_id = key.split('/')[0]
        with np.load(study.ROOT/plan['original_run_dir']/case_id/'replay.npz',allow_pickle=False) as z:
            positions,ids = z['agent_positions'][w['frame']-1],z['agent_ids']
        neighbors = []
        for q in w['neighbor_positions']:
            matches=np.flatnonzero(np.all(positions==q,axis=1))
            assert len(matches)==1
            neighbors.append(str(ids[matches[0]]))
        A,b=np.asarray(w['A']),np.asarray(w['b_no_margin'])
        linear=linprog(-b,A_eq=np.vstack([A.T,np.ones(len(b))]),b_eq=[0.,0.,1.],
            bounds=[(0,None)]*len(b),method='highs')
        certificate=None
        if linear.success and float(b@linear.x)>1e-7:
            weights=linear.x
            residual=float(np.max(np.abs(weights@A)))
            gap=float(weights@b)
            assert residual<1e-9 and float(weights.min())>=0
            labels=neighbors+['wall']*w['row_kinds'].count('wall')+['object']*w['row_kinds'].count('object')
            assert len(labels)==len(b)
            certificate=dict(nonnegative_weights=weights.tolist(),normal_cancellation_residual=residual,
                positive_rhs_gap=gap,rows=[dict(index=int(i),kind=w['row_kinds'][i],
                    neighbor_or_object=labels[i],weight=float(weights[i]),A=A[i].tolist(),b=float(b[i]))
                    for i in np.flatnonzero(weights>1e-12)],
                interpretation='lambda >= 0, lambda.T A = 0, lambda.T b > 0: contradictory even without a speed cap; floating-point witness with residual reported')
        without=w['without_speed_cap']
        assert (certificate is not None)==(not without['feasible']),key
        nearest_sample=float(np.min(np.linalg.norm(np.asarray(w['boundary_points'])-w['position'],axis=1)))
        certificates[key]=dict(frame=w['frame'],agent_id=w['agent_id'],linear_certificate=certificate,
            without_speed_cap=without,nearest_map_sample_distance_m=nearest_sample,
            true_boundary_distance_m=w['true_signed_distance_m'],
            nearest_map_sample_boundary_distance_lower_bound_m=max(0.,w['true_signed_distance_m']-nearest_sample))
    dump(public/names[2],dict(scope='Post-hoc algebraic explanation of already selected frozen witnesses; no new replay or physics',
        formula='distance(map_sample,true_boundary) >= distance(robot,true_boundary) - distance(robot,map_sample)',
        witnesses=certificates,analysis_sha256=sha(public/'analysis.json'),script_sha256=sha(__file__)))
    fig,axes=plt.subplots(2,2,figsize=(9,8),sharex=True,sharey=True)
    fig.subplots_adjust(left=.08,right=.97,bottom=.17,top=.91,wspace=.24,hspace=.27)
    selected=[]
    titles=('C: translation baseline','C: rejected geometry','Skew: translation baseline','Skew: rejected geometry')
    for ax,(case_id,title) in zip(axes.flat,zip(plan['prefixes'],titles)):
        choices=[w for k,w in analysis['witnesses'].items() if k.startswith(case_id+'/')]
        w=min(choices,key=lambda x:(x['scale'],x['frame']))
        selected.append(dict(case_id=case_id,frame=w['frame'],agent=w['agent_id']))
        A,b,kinds=np.asarray(w['A']),np.asarray(w['b_no_margin']),np.asarray(w['row_kinds'])
        cap=w['max_speed']
        axis=np.linspace(-cap,cap,401)
        X,Y=np.meshgrid(axis,axis)
        U=np.c_[X.ravel(),Y.ravel()]
        in_circle=np.linalg.norm(U,axis=1)<=cap+1e-7
        for mask,color in ((kinds!='object','#0072b2'),(kinds=='object','#e69f00')):
            feasible=in_circle & np.all(U@A[mask].T>=b[mask]-1e-7,axis=1)
            ax.contourf(X,Y,feasible.reshape(X.shape).astype(float),levels=[.5,1.5],colors=[color],alpha=.35)
        theta=np.linspace(0,2*np.pi,400)
        ax.plot(cap*np.cos(theta),cap*np.sin(theta),'k-',lw=.8)
        ax.scatter(*w['u_nominal'],marker='^',color='#777777',s=35,zorder=5)
        ax.scatter(*w['u_returned'],marker='o',color='#009e73',edgecolors='black',s=38,zorder=6)
        ax.axhline(0,color='#aaaaaa',lw=.5);ax.axvline(0,color='#aaaaaa',lw=.5)
        ax.set_xlim(-.38,.38);ax.set_ylim(-.38,.38);ax.set_aspect('equal')
        ax.set_title(f"{title}\nframe {w['frame']}, {w['agent_id']}, scale {w['scale']:.4g}",fontsize=10)
        ax.set_xlabel('x velocity (m/s)');ax.set_ylabel('y velocity (m/s)')
    fig.suptitle('Separate feasible velocity sets have no joint feasible solution',fontsize=12)
    handles=[Patch(facecolor='#0072b2',alpha=.35,label='Agent / wall rows + speed cap'),
        Patch(facecolor='#e69f00',alpha=.35,label='Object row (without rho) + speed cap'),
        Line2D([],[],marker='^',linestyle='none',color='#777777',label='Nominal'),
        Line2D([],[],marker='o',linestyle='none',color='#009e73',label='Returned after scaling')]
    fig.legend(handles=handles,loc='lower center',ncols=2,frameon=False,fontsize=9,bbox_to_anchor=(.5,.055))
    fig.text(.5,.015,'Illustrative grid; feasibility checked by both QP solvers on 11 unique selected witnesses. No new physical episodes.',ha='center',fontsize=8)
    for suffix in ('png','pdf'):
        fig.savefig(public/f'velocity_constraints.{suffix}',dpi=180)
    plt.close(fig)
    dump(public/'publication.json',dict(script_sha256=sha(__file__),analysis_sha256=sha(public/'analysis.json'),
        protocol_sha256=sha(study.PROTOCOL),outputs={n:sha(public/n) for n in names if n!='publication.json'},
        plotted_witnesses=selected,new_physical_episodes=0))
    print(json.dumps(dict(rows=len(records),witness_labels=len(certificates),
        linear_contradiction_labels=sum(c['linear_certificate'] is not None for c in certificates.values()))),flush=True)


if __name__=='__main__':
    main()
