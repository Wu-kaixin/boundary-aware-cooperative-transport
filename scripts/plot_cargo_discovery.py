"""Programmatic figures from frozen inputs/verified outputs, never synthetic runs."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Rectangle,Circle
from dbact.geometry import rotate
from dbact_sim.experiments.deployment import dump
from prepare_cargo_discovery import ROOT,MISSION,sha
from freeze_cargo_discovery import METHODS

OUT=ROOT/'artifacts/research'/MISSION/'figures'
LABELS={'boundary_cvt':'Boundary CVT','local_spacing':'Spacing',
        'nearest_observed_target':'Nearest','detection_density_cvt':'Detection CVT'}
COLORS=['#0072B2','#E69F00','#009E73','#CC79A7']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'pdf.fonttype':42,
                     'ps.fonttype':42,'svg.fonttype':'none'})
def read(p):return json.loads(p.read_text(encoding='utf-8'))

def save(fig,name,caption,inputs):
    OUT.mkdir(exist_ok=True)
    paths=[]
    for extension in ('pdf','svg','png'):
        p=OUT/(name+'.'+extension)
        if p.exists():raise RuntimeError('Figure already exists; preserve it: '+str(p))
        fig.savefig(p,dpi=180,bbox_inches='tight');paths.append(p)
    plt.close(fig)
    dump(OUT/(name+'.json'),dict(caption=caption,source_sha256=sha(Path(__file__)),
        inputs={p.relative_to(ROOT).as_posix():sha(p) for p in inputs},
        outputs={p.relative_to(ROOT).as_posix():sha(p) for p in paths}))

def preflight():
    protocol=ROOT/'experiments/ccta2027/cargo_near_v1.json';plan=read(protocol)
    chosen=[c for c in plan['cases'] if c['arm']=='boundary_cvt' and c['factors']['cohort']=='new_cargo_24' and c['factors']['angle_deg']==0]
    fig,axes=plt.subplots(3,4,figsize=(7.15,5.1),layout='constrained')
    for ax,case in zip(axes.flat,chosen):
        vertices=np.asarray(case['config']['cargoes'][0]['vertices'])-[4,4]
        ax.add_patch(Polygon(vertices,fc='#CADCE7',ec='#263F50',lw=.8))
        ax.arrow(-1.3,-1.5,.7,0,head_width=.08,color='black',length_includes_head=True)
        ax.set(xlim=(-1.9,1.9),ylim=(-1.8,1.8),aspect='equal',xticks=[-1,0,1],yticks=[-1,0,1])
        ax.set_title(case['condition_id'].removesuffix('_r0').replace('_',' '),fontsize=8)
        ax.grid(alpha=.18);ax.tick_params(labelsize=6)
    fig.suptitle('Twelve new rigid cargo geometries; each tested at 0 and 90 degrees',fontsize=10)
    save(fig,'cargo_geometry_atlas',
        'Predeclared development cargo outlines. Each has polygon area 2.05875 m2 and mass 4.1175 kg. '
        'All twelve are tested in two orientations under four deployment rules. The arrows mark the +x task direction. '
        'These are protocol geometries, not transport outcomes or draws from an arbitrary-shape distribution.',[protocol])
    shadow=ROOT/'artifacts/research'/MISSION/'shadow'
    with np.load(shadow/'same_input.npz',allow_pickle=False) as z:
        t,truth,estimate=[z[k].copy() for k in ('time','truth','estimates')]
    fig,axes=plt.subplots(2,1,figsize=(7.15,4.3),sharex=True,layout='constrained')
    axes[0].plot(t,truth[:,0],color='black',label='Evaluation truth')
    for i,(name,color,ls) in enumerate(zip(['Original reference','Innovation-trim candidate'],COLORS,['-','--'])):
        axes[0].plot(t,estimate[i,:,0],c=color,ls=ls,lw=.9,label=name)
        axes[1].plot(t,estimate[i,:,0]-truth[:,0],c=color,ls=ls,lw=.9)
    axes[0].axhline(1.2,color='.5',ls=':',label='Task displacement')
    axes[0].set_ylabel('x displacement (m)');axes[0].legend(ncol=2,fontsize=7)
    axes[1].set(xlabel='Physical time (s)',ylabel='Signed x error (m)')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('C opening away: same-input diagnostic; candidate gate failed',fontsize=10)
    save(fig,'reference_negative_result',
        'Historical C-away trajectory with two readouts on identical local scan inputs. Gaps denote no admitted reading. '
        'No smoothing, extrapolation or new physics. The candidate reduced paired vector MAE by 7.48%, but retained '
        '0.715 m final vector error and failed the predeclared gate. Frames are repeated measurements of one development trajectory.',
        [shadow/'same_input.npz',shadow/'analysis.json',shadow/'frozen_gate.json'])

def near():
    path=ROOT/'artifacts/research'/MISSION/'near-analysis/analysis.json';a=read(path)
    protocol=ROOT/'experiments/ccta2027/cargo_near_v1.json';plan=read(protocol)
    conditions=[m['condition'] for m in plan['conditions']]
    lookup={(r['condition'],r['arm']):r for r in a['episodes']}
    fig,ax=plt.subplots(figsize=(7.15,10.3),layout='constrained')
    status_color={'S':'#CCE5EF','F':'#F2D8BE','H':'#E8BDCC','E':'#CCCCCC'}
    for i,c in enumerate(conditions):
        for j,arm in enumerate(METHODS):
            r=lookup[c,arm]
            status='E' if r['execution_status']!='COMPLETED' else 'H' if r['hard_safety'] else 'S' if r['completed'] else 'F'
            ax.add_patch(Rectangle((j-.5,i-.5),1,1,fc=status_color[status],ec='white',lw=.8))
            value=r.get('endpoint_error_m');text=status if value is None else f'{status} {value:.3f}'
            ax.text(j,i,text,ha='center',va='center',fontsize=7)
    ax.set(xlim=(-.5,3.5),ylim=(32.5,-.5),xticks=range(4),yticks=range(33))
    ax.set_xticklabels([LABELS[m] for m in METHODS],fontsize=8)
    ax.set_yticklabels([f'{i+1:02d}  '+c.replace('_',' ') for i,c in enumerate(conditions)],fontsize=7)
    ax.xaxis.tick_top();ax.tick_params(length=0)
    ax.axhline(8.5,c='black',lw=1)
    ax.set_title('S = safe completion; F = task failure; H = hard-safety violation; E = execution error\nCells show final Euclidean error (m); all 132 planned starts',fontsize=8,pad=25)
    save(fig,'near_all_outcomes',
        'All 33 paired near-field development conditions under the same repaired transport pipeline. The first nine '
        'retain historical initial states; the remaining 24 use a common new-geometry annulus. S denotes task completion '
        'with no original hard-safety failure, F a task failure, H a hard-safety failure, and E an execution exception. '
        'Entries are final Euclidean goal errors, not population success probabilities.',[protocol,path])
    fig,axes=plt.subplots(3,1,figsize=(7.15,6.7),sharex=True,layout='constrained')
    for j,(method,color,marker) in enumerate(zip(METHODS,COLORS,['o','s','^','D'])):
        rows=[lookup[c,method] for c in conditions];x=np.arange(1,34)+(j-1.5)*.16
        axes[0].scatter(x,[r.get('endpoint_error_m',np.nan) for r in rows],c=color,marker=marker,s=16,label=LABELS[method])
        for xx,r in zip(x,rows):
            axes[1].scatter(xx,r.get('seconds',np.nan),c=color,marker=marker if r['completed'] else 'x',s=16)
        axes[2].scatter(x,[np.nan if r.get('ready_gap_upper_m') is None else r['ready_gap_upper_m'] for r in rows],c=color,marker=marker,s=16)
    axes[0].axhline(.12,c='.4',ls=':',lw=.8);axes[0].set_ylabel('Endpoint error (m)');axes[0].legend(ncol=4,fontsize=7)
    axes[1].set_ylabel('Physical duration (s)');axes[1].set_title('x = not completed; this duration is not a successful completion time',fontsize=8)
    axes[2].set_ylabel('Uncovered arc at ready (m)');axes[2].set_title('360-sample upper bound; absent points mean CONTACT_READY not reached',fontsize=8)
    axes[2].set_xlabel('Condition number in the complete outcome matrix');axes[2].set_xticks(range(1,34));axes[2].tick_params(labelsize=6)
    for ax in axes:ax.grid(alpha=.2)
    save(fig,'near_paired_metrics',
        'Paired task error, physical duration and sampled uncovered boundary arc at first CONTACT_READY. Every planned '
        'condition is retained. Failed or censored durations use x markers; missing readiness measurements are not filled '
        'with zeros. Arc measurements use 360 equal-arclength samples, a 0.42 m observer radius and only external robot centers. '
        'They are independent deployment diagnostics, not caging or transport guarantees.',[protocol,path])

def far():
    path=ROOT/'artifacts/research'/MISSION/'far-analysis/analysis.json';a=read(path)
    protocol=ROOT/'experiments/ccta2027/cargo_far_v1.json';plan=read(protocol)
    rows=a['episodes'];fig,axes=plt.subplots(1,3,figsize=(7.15,4.5),sharey=True,layout='constrained')
    for i,r in enumerate(rows):
        color=COLORS[0] if r['arm']=='unknown_location' else COLORS[1]
        axes[0].scatter(r.get('first_detection_seconds',np.nan),i,c=color,s=22)
        axes[1].scatter(r.get('seconds',np.nan),i,c=color,marker='o' if r['completed'] else 'x',s=22)
        axes[2].scatter(r.get('endpoint_error_m',np.nan),i,c=color,s=22)
    axes[2].axvline(.12,c='.4',ls=':')
    axes[0].set_yticks(range(len(rows)),[r['condition'].replace('_',' ')+' / '+('unknown' if r['arm']=='unknown_location' else 'hint') for r in rows],fontsize=6)
    axes[0].invert_yaxis()
    for ax,label in zip(axes,['First detection (s)','Duration (s); x = fail','Endpoint error (m)']):ax.set_xlabel(label,fontsize=7);ax.grid(alpha=.2)
    fig.suptitle('Six identical-physics pairs: unknown location versus one initial location packet',fontsize=9)
    save(fig,'far_location_pairs',
        'Far-field location-information ablation on three geometries and two positions. Both arms have zero initial sensor '
        'returns and identical physical states; the hint arm alone receives a one-shot initial location token with unchanged TTL. '
        'Detection, task completion and final error are reported separately. No geometry prior or ongoing pose oracle is supplied.',[protocol,path])
    fig,axes=plt.subplots(3,3,figsize=(7.15,7.4),layout='constrained')
    for row,name in enumerate(('circle_pos0','skew_box_s0_r0_pos0','c_open_away_pos0')):
        case=next(c for c in plan['cases'] if c['condition_id']==name and c['arm']=='unknown_location')
        result=next(r for r in rows if r['condition']==name and r['arm']=='unknown_location')
        folder=ROOT/plan['run_dir']/case['case_id']
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            t,xy,yaw,local,robots=[z[k].copy() for k in ('times','cargo/cargo_0/centers','cargo/cargo_0/angles','cargo/cargo_0/local_vertices','agent_positions')]
            goal=z['task/cargo_0/goal_point'].copy()
        contact=result.get('first_contact_seconds')
        middle=None if contact is None else min(int(round(contact/.05))+1,len(t)-1)
        for col,k in enumerate((0,middle,len(t)-1)):
            ax=axes[row,col];ax.set(xlim=(0,8),ylim=(0,8),aspect='equal');ax.tick_params(labelsize=6)
            if k is None:ax.text(.5,.5,'No physical contact',transform=ax.transAxes,ha='center');continue
            ax.add_patch(Polygon(rotate(local,yaw[k])+xy[k],fc='#D5DFE4',ec='#35434A',lw=.8))
            for p in robots[k]:ax.add_patch(Circle(p,.13,fc=COLORS[0],ec='white',lw=.3))
            ax.plot(xy[:k+1,0],xy[:k+1,1],c='black',lw=.7)
            ax.add_patch(Circle(goal,.12,fill=False,ec='black',lw=.8))
            ax.set_title(f'{name.replace("_"," ")}\nt={t[k]:.2f}s'+('\n'+result['outcome'] if col==2 else ''),fontsize=6.5)
            ax.grid(alpha=.15)
    fig.suptitle('Preselected unknown-location cases: initial / first contact / actual terminal state',fontsize=9)
    save(fig,'far_full_process',
        'Position-0 cases selected before outcomes: circle, skew box and C opening away. Columns show the initial state, '
        'first recorded actual contact and actual terminal state. Cargo geometry and its black trajectory are evaluation '
        'truth used only for visualization. Missing contact is explicitly labelled; terminal failures are not replaced by successful frames.',[protocol,path])

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['preflight','near','far'])
    globals()[p.parse_args().stage]()
