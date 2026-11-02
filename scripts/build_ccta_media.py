"""Offline publication assets from immutable census receipts; no simulation calls.

Run from the repository root. Outputs must be new; --videos-only may add videos
to a previously generated figure directory, but never replaces an existing video.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.lines import Line2D
import numpy as np
from analyze_multishape_census import Replay, visual, rows_for, read, METHODS, SHORT, CID
from dbact_sim.visualization.animation import ensure_mp4_export_available

ROOT = Path(__file__).resolve().parents[1]
SOURCE_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
COLORS = ['#0072B2', '#D55E00', '#009E73']
MARKERS = ['o', 's', '^']
NAMES = ['Circle', 'Ellipse parallel', 'Ellipse perpendicular', 'Rectangle parallel',
         'Rectangle perpendicular', 'L reference', 'L rotated', 'C toward', 'C away']
plt.rcParams.update({'font.size':8, 'axes.titlesize':8, 'axes.labelsize':8,
                     'legend.fontsize':7, 'pdf.fonttype':42, 'ps.fonttype':42,
                     'svg.fonttype':'none', 'font.family':'DejaVu Sans'})

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()

def dump(p,obj):
    p.write_text(json.dumps(obj,indent=2,ensure_ascii=False),encoding='utf-8')

def save(fig,out,name):
    for ext in ('pdf','svg','png'):
        fig.savefig(out/f'{name}.{ext}',dpi=220)
    plt.close(fig)

def bounds(replays):
    points=[]
    for r in replays:
        points.extend([r.data['agent_positions'].reshape(-1,2),
                       r.data[f'task/{CID}/goal_point'].reshape(1,2)])
    p=np.concatenate(points); low=p.min(0)-.3; high=p.max(0)+.3
    center=(low+high)/2; radius=max(high-low)/2
    return center-radius,center+radius

def scene(r,fig,ax,limits):
    v=visual(r,fig,ax)
    lo,hi=limits;ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]))
    ax.set_xlabel('');ax.set_ylabel('')
    ax.tick_params(labelsize=7)
    for a in [*v.push_arrows,*v.goal_arrows.values()]:
        a.set_mutation_scale(5);a.set_linewidth(.6)
    return v

def keyframes(r):
    d=r.data;ev=r.result['events']
    def phase(p):
        k=ev[p]['frame']
        if k is None:return None
        if not 0 <= k+1 < len(d['times']):raise ValueError(f'Invalid event index: {p}={k}')
        return k+1
    mapped=np.flatnonzero(d['sensed_counts']>0)
    steps=read(r.folder/'paired_metrics.json')['steps']
    assert np.array_equal(d[f'cargo/{CID}/contacts'][1:],[s['contacts'] for s in steps])
    contact=np.flatnonzero(d[f'cargo/{CID}/contacts']>0)
    start=phase('TRANSPORT');stop=phase('BRAKE')
    middle=None if start is None else (start+(stop if stop is not None else len(d['times'])-1))//2
    return [int(mapped[0]) if len(mapped) else None,phase('CONTACT_READY'),
            int(contact[0]) if len(contact) else None,middle,len(d['times'])-1]

def stages(batch,plan,out):
    names=plan['visual_selection']['overview_conditions']
    replays=[Replay(batch/(n+'_boundary_cvt')) for n in names]
    lim=bounds(replays);receipt=[]
    titles=['First saved map','Geometric ready','First actual contact','Transport midpoint','Actual termination']
    fig,axs=plt.subplots(5,5,figsize=(7.1,7.7))
    for row,(name,r) in enumerate(zip(names,replays)):
        for col,(k,title) in enumerate(zip(keyframes(r),titles)):
            ax=axs[row,col]
            if k is None: ax.axis('off');ax.text(.5,.5,'NOT REACHED',ha='center',transform=ax.transAxes,fontsize=7)
            else:
                scene(r,fig,ax,lim).update(k);ax.set_xticks([]);ax.set_yticks([])
                text=f'{r.trace.times[k]:.2f} s'
                if col==4:text+=f"\n{'COMPLETE' if r.result['task_completed'] else 'FAILED'}"
                ax.set_title(text,fontsize=7,pad=2)
            if col==0:ax.set_ylabel(name.replace('_parallel','').replace('_reference','').replace('c_open_toward','C toward'),fontsize=8)
            receipt.append({'case':name+'_boundary_cvt','event':title,'frame':k,'time_s':None if k is None else float(r.trace.times[k])})
    for ax,title in zip(axs[0],titles):ax.text(.5,1.35,title,ha='center',transform=ax.transAxes,fontsize=7)
    fig.subplots_adjust(left=.07,right=.995,bottom=.06,top=.93,hspace=.45,wspace=.12)
    fig.text(.5,.02,'Event-indexed columns: actual contact can precede geometric readiness. One development seed (2).',ha='center',fontsize=7)
    save(fig,out,'five_shape_stages')
    r=replays[0];fig,axs=plt.subplots(1,5,figsize=(7.1,2.05))
    for ax,k,title in zip(axs,keyframes(r),titles):
        if k is None:ax.axis('off');continue
        scene(r,fig,ax,lim).update(k);ax.set_xticks([]);ax.set_yticks([])
        ax.set_title(f'{title}\n{r.trace.times[k]:.2f} s',fontsize=7)
    fig.subplots_adjust(left=.015,right=.99,bottom=.18,top=.82,wspace=.12)
    fig.text(.5,.04,'Circle / CVT / seed 2: contact and readiness occurred, but the final task failed.',ha='center',fontsize=7)
    save(fig,out,'full_process');dump(out/'keyframes.json',receipt)

def figures(batch,plan,out):
    rows=rows_for(batch,plan)  # checks every listed immutable product
    for r in rows:
        replay=Replay(batch/(r['condition']+'_'+r['method']))
        d=replay.data
        r['robot_total_path_m']=float(np.linalg.norm(np.diff(d['agent_positions'],axis=0),axis=2).sum())
        steps=read(replay.folder/'paired_metrics.json')['steps']
        dt=float(np.median(np.diff(d['times'])))
        commands=np.array([s['commands'] for s in steps])
        r['command_squared_integral_m2_per_s']=float(np.square(commands).sum()*dt)
        r['success_time_s']=r['seconds'] if r['completed'] else None
        r['endpoint_longitudinal_residual_m']=float((d[f'cargo/{CID}/centers'][-1]-d[f'task/{CID}/goal_point'])@d[f'task/{CID}/direction'])
        direction=d[f'task/{CID}/direction'];perp=np.array([-direction[1],direction[0]])
        r['endpoint_lateral_residual_m']=float((d[f'cargo/{CID}/centers'][-1]-d[f'task/{CID}/goal_point'])@perp)
        r['case_path']=replay.folder.relative_to(ROOT).as_posix()
    with (out/'all_27_episodes.csv').open('w',newline='',encoding='utf-8') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    dump(out/'all_27_episodes.json',rows)
    conditions=[c['condition'] for c in plan['conditions']]
    fig,axs=plt.subplots(1,3,figsize=(7.1,4.1),gridspec_kw={'width_ratios':[1.35,1,1]})
    for i,c in enumerate(conditions):
        for j,m in enumerate(METHODS):
            r=next(r for r in rows if r['condition']==c and r['method']==m);y=i+(j-1)*.23
            for ax,key in zip(axs,['endpoint_error_m','seconds','robot_total_path_m']):
                ax.scatter(r[key],y,marker=MARKERS[j],s=26,edgecolors=COLORS[j],facecolors=COLORS[j] if r['completed'] else 'none',linewidths=1)
            if not r['settled']:axs[1].text(r['seconds']+2,y,'W',fontsize=6,va='center')
    for ax in axs:
        ax.set_yticks(range(9));ax.invert_yaxis();ax.grid(axis='x',alpha=.2)
    axs[0].set_yticklabels(NAMES);axs[1].set_yticklabels([]);axs[2].set_yticklabels([])
    for ax,label in zip(axs,['Endpoint error [m]','Actual duration [s]','Total robot path [m]']):ax.set_xlabel(label)
    axs[0].axvline(.12,color='black',ls=':',lw=.8);axs[1].set_xlim(-5,170)
    handles=[Line2D([],[],color=COLORS[j],marker=MARKERS[j],ls='',label=SHORT[m]) for j,m in enumerate(METHODS)]
    fig.legend(handles=handles,loc='upper center',ncol=3,frameon=False)
    fig.subplots_adjust(left=.23,right=.985,bottom=.15,top=.89,wspace=.23)
    fig.text(.61,.015,'Filled: completed; open: failed; W: watchdog. B=1 shared development block; no CI.',ha='center',fontsize=6.8)
    save(fig,out,'method_comparison')
    fig,axs=plt.subplots(2,2,figsize=(7.1,4.9))
    keys=['min_robot_distance_m','max_penetration_m','command_squared_integral_m2_per_s','barrier_scalings']
    labels=['Minimum robot distance [m]','Maximum physical penetration [m]','Command effort proxy [m²/s]','Barrier scalings [count]']
    for ax,key,label in zip(axs.flat,keys,labels):
        for j,m in enumerate(METHODS):
            vals=[next(r[key] for r in rows if r['condition']==c and r['method']==m) for c in conditions]
            ax.plot(np.arange(9)+(j-1)*.12,vals,marker=MARKERS[j],color=COLORS[j],ls='',ms=4,label=SHORT[m])
        ax.set_xticks(range(9),range(1,10));ax.set_ylabel(label);ax.grid(alpha=.2)
    fig.legend(handles=handles,loc='upper center',ncol=3,frameon=False)
    fig.subplots_adjust(left=.10,right=.98,bottom=.12,top=.9,hspace=.38,wspace=.35)
    fig.text(.5,.025,'Condition IDs 1–9 follow the complete results table. Discrete safety observations; no continuous guarantee.',ha='center',fontsize=7)
    save(fig,out,'safety_resources')
    selected=['circle','rectangle_parallel','c_open_away'];fig,axs=plt.subplots(3,3,figsize=(7.1,5.2))
    alignment={}
    for j,name in enumerate(selected):
        r=Replay(batch/(name+'_boundary_cvt'));d=r.data;t=d['times'][:-1]
        truth=(d[f'cargo/{CID}/centers'][:-1]-d[f'task/{CID}/start'])@d[f'task/{CID}/direction']
        estimate=d[f'cargo/{CID}/progress_estimate'][1:]
        steps=read(r.folder/'paired_metrics.json')['steps']
        assert len(steps)==len(t)
        assert np.allclose(estimate,[s['estimated_progress'] for s in steps],atol=1e-12,rtol=0)
        axs[0,j].plot(t,truth,color='black',lw=1,label='Evaluation truth')
        axs[0,j].plot(t,estimate,color=COLORS[0],ls='--',lw=1,label='Controller team estimate')
        axs[0,j].axhline(1.2,color='gray',ls=':',lw=.7)
        axs[0,j].set_title(name.replace('_',' '))
        lateral=d[f'cargo/{CID}/centers'][:-1,1]-d[f'task/{CID}/start'][1]
        axs[1,j].plot(t,estimate-truth,color=COLORS[0],lw=1,label='Signed progress error')
        axs[1,j].plot(t,lateral,color=COLORS[1],ls='--',lw=1,label='True cross-track')
        axs[1,j].axhline(0,color='gray',lw=.5)
        axs[2,j].step(d['times'][1:],[s['contacts'] for s in steps],where='post',color='black',lw=.8)
        for phase,color,style in [('CONTACT_READY','#777777','--'),('TRANSPORT','#009E73',':'),('BRAKE','#D55E00','-.'),('HOLD','#CC79A7','-')]:
            k=r.result['events'][phase]['frame']
            if k is not None:
                for ax in axs[:,j]:ax.axvline(k*.05,color=color,ls=style,lw=.7)
        for ax in axs[:,j]:ax.grid(alpha=.15);ax.set_xlim(0,r.end)
        axs[2,j].set_xlabel('Physical time [s]')
        alignment[name]={'estimate_source':'replay progress_estimate[1:]','truth_source':'replay centers[:-1]',
                         'control_times':'replay times[:-1]','contact_report_times':'replay times[1:] (after robot move, before cargo integration)',
                         'frames':len(t),'smoothing':'none','downsampling':'none'}
    axs[0,0].set_ylabel('Progress [m]');axs[1,0].set_ylabel('Signed error / offset [m]');axs[2,0].set_ylabel('Actual contacts')
    axs[0,0].legend(loc='upper left',fontsize=6);axs[1,0].legend(loc='lower left',fontsize=6)
    fig.subplots_adjust(left=.095,right=.965,bottom=.14,top=.94,wspace=.35,hspace=.25)
    fig.legend(handles=[Line2D([],[],color=c,ls=ls,lw=.8,label=name) for name,c,ls in [('READY','#777777','--'),('TRANSPORT','#009E73',':'),('BRAKE','#D55E00','-.'),('HOLD','#CC79A7','-')]],loc='lower center',ncol=4,frameon=False,title='CVT, seed 2; all control samples, no smoothing',fontsize=7,title_fontsize=7)
    save(fig,out,'aligned_mechanism');dump(out/'alignment.json',alignment)
    with (ROOT/'docs/results/progress_local_v2/comparison_results.csv').open() as f:v2=list(csv.DictReader(f))
    fig,ax=plt.subplots(figsize=(3.4,2.5))
    ids=list(dict.fromkeys(r['case_id'] for r in v2))
    for j,arm in enumerate(['baseline','certificate_only','anchored_readout']):
        ax.plot(np.arange(3)+(j-1)*.12,[float(next(r['endpoint_error'] for r in v2 if r['case_id']==c and r['arm']==arm))*1000 for c in ids],ls='',marker=MARKERS[j],color=COLORS[j],label=arm.replace('_',' '),ms=5,mfc='none' if j==1 else COLORS[j])
    ax.axhline(120,color='gray',ls=':',lw=.8);ax.set(xticks=range(3),xticklabels=['R2','L8','C8'],ylabel='Endpoint error [mm]')
    fig.legend(*ax.get_legend_handles_labels(),fontsize=6,loc='upper center',ncol=3,frameon=False);ax.grid(axis='y',alpha=.2)
    fig.subplots_adjust(left=.19,right=.98,bottom=.17,top=.8);save(fig,out,'v2_negative')
    stages(batch,plan,out)

def video(batch,ids,path,overview=False):
    if path.exists():raise FileExistsError(path)
    replays=[Replay(batch/c) for c in ids];limits=bounds(replays)
    fig,axs=plt.subplots(2 if overview else 1,3,figsize=(15,10 if overview else 5.5))
    axs=np.asarray(axs).ravel();renderers=[];texts=[]
    for ax,r,name in zip(axs,replays,ids):
        renderers.append(scene(r,fig,ax,limits));ax.set_title(name.replace('_',' '),fontsize=10)
        texts.append(ax.text(0,-.12,'',transform=ax.transAxes,fontsize=8,va='top'))
    if overview:
        axs[-1].axis('off');axs[-1].text(.02,.95,'Five preselected CVT reference conditions\n\nOrange object: evaluation geometry\nTeal: agent_00 fused map (not raw rays)\nRings: geometric readiness\n\nSeed 2; task +x, distance 1.2 m\nSimulated control: 20 Hz\nVideo: 10 fps, nominal 2.5x\nPhysical samples: 0.25 s; no interpolation\nEnded cases hold their actual final state\n\nFailures remain visible.',va='top',fontsize=11)
    fig.subplots_adjust(left=.035,right=.985,top=.9,bottom=.24 if not overview else .12,wspace=.23,hspace=.65)
    banner=fig.suptitle('',fontsize=13)
    end=max(r.end for r in replays);timeline=list(np.arange(0,end+1e-8,.25))
    if timeline[-1]<end-1e-8:timeline.append(end)
    writer=animation.FFMpegWriter(fps=10,codec='libx264',bitrate=2600,extra_args=['-pix_fmt','yuv420p','-movflags','+faststart'])
    render_seconds=0.;started=time.perf_counter()
    with writer.saving(fig,str(path),dpi=100):
        for t in timeline:
            tick=time.perf_counter();banner.set_text(f'Physical time {t:.2f} s | seed 2 | task +x, 1.2 m | 2.5x playback')
            for r,v,txt in zip(replays,renderers,texts):
                k=max(0,int(np.searchsorted(r.trace.times,min(t,r.end)+1e-8,side='right')-1));v.update(k)
                d=r.data;true=float((d[f'cargo/{CID}/centers'][k]-d[f'task/{CID}/start'])@d[f'task/{CID}/direction'])
                estimate=float(d[f'cargo/{CID}/progress_estimate'][k]);err=float(np.linalg.norm(d[f'cargo/{CID}/centers'][k]-d[f'task/{CID}/goal_point']))
                status=f"ENDED at {r.end:.2f}s: {'COMPLETE' if r.result['task_completed'] else 'FAILED'}" if t>=r.end else 'RUNNING'
                txt.set_text(f'State t={d["times"][k]:.2f}s | phase {r.trace.phase_labels[k]}\n{status}\nGoal error={err:.3f}m | evaluation truth progress={true:.3f}m\nTeam estimate={estimate:.3f}m @ control t={max(0,d["times"][k]-.05):.2f}s\nActual contacts={int(d[f"cargo/{CID}/contacts"][k])} | seed=2 | task +x')
            fig.canvas.draw();render_seconds+=time.perf_counter()-tick;writer.grab_frame()
        fig.savefig(path.with_suffix('.png'),dpi=90)
    elapsed=time.perf_counter()-started;plt.close(fig)
    from imageio_ffmpeg import count_frames_and_secs
    count,seconds=count_frames_and_secs(str(path));assert count==len(timeline)
    return {'video':path.name,'cases':ids,'sha256':sha(path),'generator_sha256':SOURCE_SHA256,'frames':count,'playback_seconds':seconds,'physical_end_s':end,
            'control_hz':20,'playback_fps':10,'sample_interval_s':.25,'nominal_speedup':2.5,
            'wall_seconds':elapsed,'end_to_end_frames_per_second':count/elapsed,'explicit_canvas_draw_seconds':render_seconds,
            'timing_caveat':'writer includes additional figure drawing and encoder pipe waits; remainder is not pure encoding time',
            'decode':'all frames counted successfully','replay_hashes':{c:sha(batch/c/'replay.npz') for c in ids}}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--batch',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--videos-only',action='store_true');p.add_argument('--resume-videos',action='store_true');p.add_argument('--dry-run',action='store_true');a=p.parse_args()
    batch=a.batch.resolve();out=a.out.resolve()
    if not batch.is_relative_to(ROOT/'runs') or not out.is_relative_to(ROOT/'artifacts') or out==batch:raise ValueError('Paths must be repository runs input and artifacts output')
    plan=read(batch/'resolved_protocol.json')
    assert len(plan['cases'])==27, 'Only the frozen 27-case census is supported'
    for c in plan['cases']:
        assert c['seed']==2 and c['config']['dt']==.05, 'Unexpected seed or time step'
        with np.load(batch/c['case_id']/'replay.npz',allow_pickle=False) as z:
            assert np.array_equal(z[f'task/{CID}/direction'],[1.,0.])
            assert np.isclose(float(z[f'task/{CID}/distance']),1.2,rtol=0,atol=1e-12)
    assert sha(ROOT/'experiments/ccta2027/multishape_census_v1.json')=='0d6f4701dabfe4850ec487378a4ffd4db273e275697f61aa5db435ea3624b39b'
    if a.dry_run:print(json.dumps({'cases':len(plan['cases']),'videos':10,'new_physics':0,'output':str(out)}));return
    if not a.videos_only:
        out.mkdir(parents=True,exist_ok=False);figures(batch,plan,out)
        inputs={}
        for case in plan['cases']:
            folder=batch/case['case_id'];rec=read(folder/'case_result.json')
            inputs[str((folder/'case_result.json').relative_to(ROOT))]=sha(folder/'case_result.json')
            for rel in rec['products']:inputs[str((folder/rel).relative_to(ROOT))]=sha(folder/rel)
        for rel in ['docs/results/progress_local_v2/comparison_results.csv','scripts/analyze_multishape_census.py','src/dbact_sim/visualization/renderer.py']:
            inputs[rel]=sha(ROOT/rel)
        dump(out/'provenance.json',{'source_inputs':inputs,'generator_sha256':SOURCE_SHA256,'new_physics_episodes':0,'statistics':'B=1 shared seed block; descriptive, no CI','fonts':'DejaVu Sans; PDF Type42; SVG text','figure_width_in':{'double':7.1,'single':3.4}})
    else:
        ensure_mp4_export_available();out.mkdir(parents=True,exist_ok=True);receipts=[]
        previous=read(out/'video_receipts.json') if a.resume_videos and (out/'video_receipts.json').exists() else []
        groups=[('overview',[c+'_boundary_cvt' for c in plan['visual_selection']['overview_conditions']],True)]
        groups.extend((c['condition'],[c['condition']+'_'+m for m in METHODS],False) for c in plan['conditions'])
        for name,ids,overview in groups:
            existing=next((r for r in previous if r['video']==name+'.mp4'),None)
            if existing:
                from imageio_ffmpeg import count_frames_and_secs
                assert existing['cases']==ids and sha(out/existing['video'])==existing['sha256']
                assert all(sha(batch/c/'replay.npz')==h for c,h in existing['replay_hashes'].items())
                assert count_frames_and_secs(str(out/existing['video']))[0]==existing['frames']
                receipts.append(existing);print('Verified '+name,flush=True);continue
            receipts.append(video(batch,ids,out/(name+'.mp4'),overview));dump(out/'video_receipts.json',receipts);print(name,flush=True)
        ffmpeg=plt.rcParams['animation.ffmpeg_path'];gif=out/'overview.gif'
        if gif.exists():raise FileExistsError(gif)
        subprocess.run([ffmpeg,'-n','-i',str(out/'overview.mp4'),'-vf','fps=6,scale=720:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse','-loop','0',str(gif)],check=True,capture_output=True)
    print('DONE',flush=True)

if __name__=='__main__':main()
