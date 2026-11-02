"""Physical-time synchronized offline videos of preselected cases, including failures."""
import argparse
import json
from pathlib import Path
import subprocess
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
from matplotlib.patches import Polygon,Circle
import imageio_ffmpeg
from dbact.control.phase import Phase
from dbact.geometry import rotate
from dbact_sim.experiments.deployment import dump
from prepare_cargo_discovery import ROOT,MISSION,sha
from freeze_cargo_discovery import METHODS
from plot_cargo_discovery import LABELS,COLORS

def read(p):return json.loads(p.read_text(encoding='utf-8'))

def render(stage):
    protocol=ROOT/f'experiments/ccta2027/cargo_{stage}_v1.json';plan=read(protocol)
    analysis_path=ROOT/'artifacts/research'/MISSION/(stage+'-analysis/analysis.json')
    analysis=read(analysis_path)
    data_path=analysis_path.with_name('plot_data.json');series=read(data_path)
    if stage=='near':
        names=['skew_box_s0_r0_'+method for method in METHODS]
        grid=(2,2);size=(12.8,10.8)
    else:
        names=[name+'_'+arm for name in ('circle_pos0','skew_box_s0_r0_pos0','c_open_away_pos0')
               for arm in ('unknown_location','initial_location_hint')]
        grid=(3,2);size=(12.8,15.6)
    out=ROOT/'artifacts/research'/MISSION/'media';out.mkdir(exist_ok=True)
    video=out/(stage+'_synchronized.mp4');gif=out/(stage+'_preview.gif')
    if video.exists() or gif.exists():raise RuntimeError('Media already exists; preserve it')
    panes=[];inputs={protocol.relative_to(ROOT).as_posix():sha(protocol),
        analysis_path.relative_to(ROOT).as_posix():sha(analysis_path),data_path.relative_to(ROOT).as_posix():sha(data_path)}
    fig=plt.figure(figsize=size,layout='constrained')
    outer=fig.add_gridspec(*grid)
    for slot,name in zip(outer,names):
        # Keep the scientific state visible: HUD text gets its own strip rather
        # than covering the initially far-away cargo or the searching robots.
        inner=slot.subgridspec(2,1,height_ratios=[1.,.27],hspace=.03)
        ax=fig.add_subplot(inner[0]); info_ax=fig.add_subplot(inner[1])
        info_ax.set_axis_off()
        case=next(c for c in plan['cases'] if c['case_id']==name)
        row=next(r for r in analysis['episodes'] if r['condition']==case['condition_id'] and r['arm']==case['arm'])
        folder=ROOT/plan['run_dir']/name
        with np.load(folder/'replay.npz',allow_pickle=False) as z:
            saved={k:z[k].copy() for k in ('times','phase','agent_positions','cargo/cargo_0/centers',
                'cargo/cargo_0/angles','cargo/cargo_0/local_vertices','task/cargo_0/start','task/cargo_0/goal_point')}
        inputs[(folder/'replay.npz').relative_to(ROOT).as_posix()]=sha(folder/'replay.npz')
        ax.set(xlim=(0,8),ylim=(0,8),aspect='equal',xlabel='x (m)',ylabel='y (m)');ax.grid(alpha=.18)
        patch=Polygon(saved['cargo/cargo_0/local_vertices']+saved['cargo/cargo_0/centers'][0],fc='#CAD7DE',ec='#3E505D',lw=1)
        ax.add_patch(patch)
        robots=[]
        for position in saved['agent_positions'][0]:
            disk=Circle(position,.13,fc=COLORS[0],ec='white',lw=.4)
            ax.add_patch(disk);robots.append(disk)
        trail,=ax.plot([],[],c='.2',lw=.9)
        ax.add_patch(Circle(saved['task/cargo_0/goal_point'],.12,fill=False,ec='black'))
        hud=info_ax.text(.01,.98,'',transform=info_ax.transAxes,va='top',fontsize=8)
        name_label=LABELS[case['variant']] if stage=='near' else case['arm'].replace('_',' ')
        ax.set_title(case['condition_id'].replace('_',' ')+' | '+name_label+' | seed 2 | task +x',fontsize=9)
        panes.append((saved,series[name],row,patch,robots,trail,hud))
    longest=max(p[0]['times'][-1] for p in panes)
    # One physical second per frame, 10 playback frames/s. No interpolation:
    # each pane uses its latest real saved state at or before the common time.
    schedule=np.arange(0,longest+1e-9,1.)
    if schedule[-1]<longest:schedule=np.r_[schedule,longest]
    title=fig.suptitle('',fontsize=11)
    ffmpeg=imageio_ffmpeg.get_ffmpeg_exe();matplotlib.rcParams['animation.ffmpeg_path']=ffmpeg
    writer=FFMpegWriter(fps=10,codec='libx264',extra_args=['-crf','23','-pix_fmt','yuv420p','-threads','4'])
    started=time.perf_counter()
    with writer.saving(fig,str(video),dpi=100):
        for physical_time in schedule:
            title.set_text(f'Physical-time comparison: t = {physical_time:.2f} s | replay at 10x | cargo = evaluation truth')
            for saved,small,row,patch,robots,trail,hud in panes:
                k=min(len(saved['times'])-1,int(np.searchsorted(saved['times'],physical_time,side='right')-1))
                t=saved['times'][k];xy=saved['cargo/cargo_0/centers'][k]
                patch.set_xy(rotate(saved['cargo/cargo_0/local_vertices'],saved['cargo/cargo_0/angles'][k])+xy)
                for disk,position in zip(robots,saved['agent_positions'][k]):disk.center=position
                trail.set_data(saved['cargo/cargo_0/centers'][:k+1].T)
                estimated=np.asarray(small['estimated_displacement'][k-1],dtype=float) if k else np.full(2,np.nan)
                estimate='unavailable' if not np.isfinite(estimated).all() else f'({estimated[0]:.3f}, {estimated[1]:.3f}) m'
                error=np.linalg.norm(xy-saved['task/cargo_0/goal_point']);truth=xy-saved['task/cargo_0/start']
                ended=physical_time>=saved['times'][-1]
                hud.set_text(f'{"ENDED" if ended else "RUNNING"} | state t={t:.2f} s | {Phase(int(saved["phase"][k])).name}\n'
                    f'Goal error (truth): {error:.3f} m\nTruth displacement: ({truth[0]:.3f}, {truth[1]:.3f}) m\n'
                    f'Last control estimate (t-dt): {estimate}\n'+(row['outcome'] if ended else ''))
            writer.grab_frame()
    plt.close(fig)
    wall=time.perf_counter()-started
    # Light preview keeps the whole video with reduced frame rate and resolution.
    subprocess.run([ffmpeg,'-v','error','-i',str(video),'-vf',
        'fps=3,scale=640:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse',
        '-loop','0',str(gif)],check=True)
    dump(out/(stage+'_media.json'),dict(stage=stage,cases=names,selection='Fixed first skew geometry / position-0 three-geometry pairs; outcomes not used for selection',
        simulation_dt_s=.05,simulation_hz=20.,physical_sample_interval_s=1.,playback_fps=10.,nominal_speedup=10.,
        rendered_frames=len(schedule),render_wall_seconds=wall,render_frames_per_second=len(schedule)/wall,
        alignment='Shared physical time, latest real state; terminal pane frozen and labelled ENDED with true terminal time. Estimate is previous control readout, not truth.',
        input_sha256=inputs,source_sha256=sha(Path(__file__)),
        output_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in (video,gif)}))
    print(json.dumps(dict(stage=stage,frames=len(schedule),wall_seconds=wall,video=str(video))))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('stage',choices=['near','far']);render(p.parse_args().stage)
