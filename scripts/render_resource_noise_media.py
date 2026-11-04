"""Render preselected measured trajectories, synchronized by physical time."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import time

import resource_noise_validation as study
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np
import imageio_ffmpeg
from analyze_multishape_census import Replay, draw_stages, CID
from build_ccta_media import bounds, scene
from dbact_sim.experiments.deployment import dump
from dbact_sim.visualization.animation import ensure_mp4_export_available


def video(ids, path, overview=False):
    if path.exists():
        raise FileExistsError(path)
    batch = study.BASE / 'run'
    replays = [Replay(batch / c) for c in ids]
    limits = bounds(replays)
    fig, axes = plt.subplots(2 if overview else 1,3,figsize=(15,10 if overview else 5.5))
    axes = np.asarray(axes).ravel()
    renderers, texts = [], []
    for ax,r,c in zip(axes,replays,ids):
        renderers.append(scene(r,fig,ax,limits))
        label = c.replace('ellipse_perpendicular','Ellipse perp.').replace('rectangle_parallel','Rectangle parallel')
        label = label.replace('nearest_observed_target','Nearest').replace('boundary_cvt','CVT').replace('local_spacing','Spacing')
        ax.set_title(label.replace('_',' '),fontsize=10)
        texts.append(ax.text(0,-.12,'',transform=ax.transAxes,fontsize=8,va='top'))
    if overview:
        axes[-1].axis('off')
        axes[-1].text(.02,.95,'Five preselected CVT reference conditions\n\nOrange: evaluation geometry\nTeal: agent_00 fused map, not raw laser\nRings: geometric readiness, not contact\n\nSeed 2026110401; +x goal, 1.2 m\n16 robots; range sigma 0.01 m\n20 Hz control; 10 fps video; 2.5x speed\nSamples every 0.25 s, no interpolation\nEnded cases hold their measured final state\nFailures remain visible.\n\nScalar estimate is an auxiliary logged signal;\ncurrent navigation uses vector displacement.',va='top',fontsize=10)
    fig.subplots_adjust(left=.035,right=.985,top=.9,bottom=.24 if not overview else .12,wspace=.23,hspace=.65)
    banner = fig.suptitle('',fontsize=13)
    end = max(r.end for r in replays)
    timeline = list(np.arange(0,end+1e-8,.25))
    if timeline[-1] < end-1e-8:
        timeline.append(end)
    writer = animation.FFMpegWriter(fps=10,codec='libx264',bitrate=2600,
        extra_args=['-threads','4','-pix_fmt','yuv420p','-movflags','+faststart'])
    started = time.perf_counter()
    with writer.saving(fig,str(path),dpi=100):
        for t in timeline:
            banner.set_text(f'Physical time {t:.2f} s | seed 2026110401 | task +x, 1.2 m | 2.5x playback')
            for r,v,txt in zip(replays,renderers,texts):
                k=max(0,int(np.searchsorted(r.trace.times,min(t,r.end)+1e-8,side='right')-1))
                v.update(k)
                d=r.data
                true=float((d[f'cargo/{CID}/centers'][k]-d[f'task/{CID}/start'])@d[f'task/{CID}/direction'])
                estimate=float(d[f'cargo/{CID}/progress_estimate'][k])
                err=float(np.linalg.norm(d[f'cargo/{CID}/centers'][k]-d[f'task/{CID}/goal_point']))
                status=f"ENDED at {r.end:.2f}s: {'COMPLETE' if r.result['task_completed'] else 'FAILED'}" if t>=r.end else 'RUNNING'
                txt.set_text(f'State t={d["times"][k]:.2f}s | phase {r.trace.phase_labels[k]}\n{status}\nGoal error={err:.3f}m | true projected progress={true:.3f}m\nAuxiliary scalar estimate={estimate:.3f}m @ previous control\nActual contacts={int(d[f"cargo/{CID}/contacts"][k])} | G500={r.result["g500_pass"]} (terminal)')
            writer.grab_frame()
        fig.savefig(path.with_suffix('.png'),dpi=90)
    plt.close(fig)
    count,seconds=imageio_ffmpeg.count_frames_and_secs(str(path))
    assert count==len(timeline)
    return dict(video=path.name,cases=ids,frames=count,playback_seconds=seconds,physical_end_s=end,
        control_hz=20,playback_fps=10,nominal_speedup=2.5, wall_seconds=time.perf_counter()-started,
        video_sha256=study.sha(path), replay_sha256={c:study.sha(batch/c/'replay.npz') for c in ids},
        information='Evaluation geometry is offline only; auxiliary scalar estimate is not the vector navigation input',
        decoder='imageio_ffmpeg.count_frames_and_secs: entire video decoded')


def main(out):
    study.guard(); study.verify()
    assert study.read(study.BASE/'run/summary.json')['completed_denominator']==60
    out=out.resolve()
    if not out.is_relative_to(study.BASE) or out.exists():
        raise ValueError('Use a new media directory under the study')
    out.mkdir(parents=True)
    ensure_mp4_export_available()
    refs=[s+'_reference' for s in study.SHAPES]
    draw_stages(study.BASE/'run', {'visual_selection':{'overview_conditions':refs}}, out)
    groups=[('five_shape_overview',[c+'_boundary_cvt' for c in refs],True)]
    groups += [(shape+'_three_methods',[shape+'_reference_'+m for m in study.METHODS],False)
               for shape in ('rectangle_parallel','c_open_away')]
    receipts=[]
    for name,ids,overview in groups:
        receipts.append(video(ids,out/(name+'.mp4'),overview))
        dump(out/'video_receipts.json',receipts)
        print(json.dumps(receipts[-1]),flush=True)
    ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
    gif=out/'five_shape_overview.gif'
    subprocess.run([ffmpeg,'-v','error','-i',str(out/'five_shape_overview.mp4'),
        '-vf','fps=10,scale=900:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse',
        '-loop','0',str(gif)],check=True)
    from PIL import Image
    with Image.open(gif) as im:
        frames=im.n_frames
        for k in range(frames):
            im.seek(k);im.load()
    assert frames==receipts[0]['frames']
    dump(out/'media_manifest.json',dict(videos=receipts,gif_frames=frames,
        generator_sha256=study.sha(Path(__file__)),
        artifacts={p.name:study.sha(p) for p in out.iterdir() if p.is_file()}))


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    main(ap.parse_args().out)
