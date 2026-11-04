"""Post hoc display of fixed validation outcomes; no controller changes or new episodes."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import animation
import imageio_ffmpeg
import resource_noise_validation as study
from analyze_resource_noise_validation import episode
from analyze_multishape_census import Replay, CID
from build_ccta_media import bounds, scene
from dbact_sim.experiments.deployment import dump
from dbact_sim.visualization.animation import ensure_mp4_export_available


def mark_events(ax, row):
    for name, style in [('ready_s', ':'), ('brake_s', '--'), ('hold_s', '-')]:
        if row[name] is not None:
            ax.axvline(row[name], color='.55', ls=style, lw=.65, zorder=0)


def export(fig, out, name):
    for ext in ('pdf', 'png', 'svg'):
        fig.savefig(out / (name + '.' + ext), dpi=200)
    plt.close(fig)


def failure_video(ids, out):
    ensure_mp4_export_available()
    replays = [Replay(study.BASE / 'run' / c) for c in ids]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.5))
    renderers, texts = [], []
    for ax, r, title in zip(axes, replays, ['CVT', 'Spacing', 'Nearest observed target']):
        renderers.append(scene(r, fig, ax, bounds(replays)))
        ax.set_title(title)
        texts.append(ax.text(0, -.12, '', transform=ax.transAxes, fontsize=8, va='top'))
    fig.subplots_adjust(left=.04, right=.985, top=.86, bottom=.25, wspace=.22)
    banner = fig.suptitle('', fontsize=12)
    fig.text(.5, .015, 'Post hoc diagnostic selection: C-away, 8 robots, seed 2026110401. '
             'Orange: offline truth; teal: agent 00 fused map. Ended trajectories hold their last real state.',
             ha='center', fontsize=9)
    end = max(r.end for r in replays)
    timeline = list(np.arange(0, end+1e-7, 1.0))
    if timeline[-1] < end-1e-7:
        timeline.append(end)
    path = out / 'c_away_8_robots_full_comparison.mp4'
    writer = animation.FFMpegWriter(fps=10, codec='libx264', bitrate=2600,
        extra_args=['-threads', '4', '-pix_fmt', 'yuv420p', '-movflags', '+faststart'])
    started = time.perf_counter()
    with writer.saving(fig, str(path), dpi=100):
        for t in timeline:
            banner.set_text(f'Physical time {t:.2f} s | complete measured horizon | 1 s sampling, 10x playback')
            for r, visual, txt in zip(replays, renderers, texts):
                k = max(0, int(np.searchsorted(r.trace.times, min(t, r.end)+1e-8, side='right')-1))
                visual.update(k)
                d = r.data
                err = float(np.linalg.norm(d[f'cargo/{CID}/centers'][k]-d[f'task/{CID}/goal_point']))
                ended = t >= r.end-1e-7
                status = (f"ENDED {r.end:.2f}s: " + ('COMPLETE' if r.result['task_completed'] else 'FAILED')) if ended else 'RUNNING'
                txt.set_text(f'State t={r.trace.times[k]:.2f}s | phase {r.trace.phase_labels[k]}\n{status}\n'
                    f'Goal error={err:.3f}m | actual contacts={int(d[f"cargo/{CID}/contacts"][k])}\n'
                    f'G500={r.result["g500_pass"]} (terminal verdict)')
            writer.grab_frame()
        fig.savefig(out / 'c_away_8_robots_full_comparison.png', dpi=100)
    plt.close(fig)
    count, duration = imageio_ffmpeg.count_frames_and_secs(str(path))
    assert count == len(timeline)
    return dict(file=path.name, cases=ids, frames=count, playback_seconds=duration,
        physical_end_s=end, sampling_s=1.0, fps=10, speedup=10,
        selection='Post hoc: includes both incomplete current-source cases and their matched CVT control',
        wall_seconds=time.perf_counter()-started, sha256=study.sha(path),
        replay_sha256={c:study.sha(study.BASE/'run'/c/'replay.npz') for c in ids},
        verification='Entire MP4 decoded and frame count checked')


def main(out):
    study.guard(); study.verify()
    out = out.resolve()
    if out.exists() or not out.is_relative_to(study.BASE):
        raise ValueError('Use a fresh output directory under the study')
    data_path = study.BASE / 'analysis_v1/analysis.json'
    data = study.read(data_path)
    assert len(data['episodes']) == 60
    out.mkdir(parents=True)
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':7, 'pdf.fonttype':42,
        'ps.fonttype':42, 'svg.fonttype':'none', 'axes.spines.top':False, 'axes.spines.right':False})
    fig, axes = plt.subplots(5, 3, figsize=(7.1, 7.7), layout='constrained')
    for i, shape in enumerate(study.SHAPES):
        cid = shape+'_reference_boundary_cvt'
        tr = data['series'][cid]
        row = next(r for r in data['episodes'] if r['case_id']==cid)
        a, b, c = axes[i]
        t = tr['control_time']
        a.plot(t, tr['true_progress'], color='#0072B2', label='True, pre-physics')
        a.plot(t, tr['estimated_navigation_progress'], color='#D55E00', ls='--', label='Navigation estimate')
        a.axhline(1.2, color='.6', lw=.6)
        b.plot(tr['time'], np.linalg.norm(np.asarray(tr['xy'])-tr['goal'], axis=1), color='#0072B2')
        b.axhline(.12, color='.4', ls=':', lw=.7)
        c.plot(t, tr['forward_force'], color='#009E73', lw=.6)
        a.set_ylabel(shape.replace('_',' ')+'\nProgress (m)', fontsize=6.5)
        b.set_ylabel('Goal error (m)'); c.set_ylabel('Projected force (N)')
        for ax in (a, b, c):
            mark_events(ax, row); ax.set_xlabel('Physical time (s)')
    axes[0, 0].legend(fontsize=5.5, loc='upper left')
    fig.suptitle('Five preselected reference CVT tasks | dotted READY, dashed BRAKE, solid HOLD\n'
                 'Control estimates align with pre-physics truth; force is the subsequent contact report.', fontsize=8)
    export(fig, out, 'reference_navigation')
    ids = ['c_open_away_robots8_'+m for m in study.METHODS]
    plan = study.read(study.PROTOCOL); manifest = study.read(study.BASE/'run/manifest.json')
    diag = {}
    fig, axes = plt.subplots(2, 3, figsize=(7.1, 3.9), layout='constrained')
    for j, cid in enumerate(ids):
        case = next(c for c in plan['cases'] if c['case_id']==cid)
        row, tr = episode(case, study.BASE/'run'/cid, manifest)
        diag[cid] = dict(row=row, series=tr)
        axes[0,j].plot(tr['control_time'], tr['true_progress'], color='#0072B2', lw=.8)
        axes[0,j].plot(tr['control_time'], tr['estimated_navigation_progress'], color='#D55E00', ls='--', lw=.8)
        axes[0,j].axhline(1.2, color='.5', lw=.6)
        axes[1,j].plot(tr['time'], np.linalg.norm(np.asarray(tr['xy'])-tr['goal'],axis=1), color='#0072B2', lw=.8)
        axes[1,j].axhline(.12, color='.5', ls=':', lw=.8)
        axes[0,j].set_title(['CVT: complete at 571.55 s','Spacing: BRAKE watchdog','Nearest: off-target HOLD'][j],fontsize=7)
        axes[0,j].set_ylabel('Projected progress (m)'); axes[1,j].set_ylabel('Goal error (m)')
        for ax in axes[:,j]:
            mark_events(ax,row); ax.set_xlabel('Physical time (s)')
    fig.suptitle('C-away, 8 robots: post hoc failure diagnosis, all samples, distinct real horizons\n'
                 'Blue: evaluation truth; orange dashed: navigation estimate. Gray events: READY / BRAKE / HOLD.',fontsize=8)
    export(fig,out,'c8_failure_diagnostics')
    dump(out/'failure_series.json',diag)
    receipt = failure_video(ids,out)
    dump(out/'diagnostics_manifest.json',dict(analysis_sha256=study.sha(data_path),
        generator_sha256=study.sha(Path(__file__)), video=receipt,
        selection='Reference CVT conditions preselected; C-away/8-robot diagnostic selected after outcomes; no new physics',
        artifacts={p.name:study.sha(p) for p in out.iterdir() if p.is_file()}))
    print(json.dumps(receipt),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    main(ap.parse_args().out)
