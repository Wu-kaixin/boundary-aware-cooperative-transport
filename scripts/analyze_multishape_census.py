"""Tables, measured keyframes and physical-time videos for the frozen census."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from dbact.contracts import ClosedLoopContract
from dbact_sim.experiments.deployment import dump
from dbact_sim.experiments.study import cached_result
from dbact_sim.trace import SimulationTrace
from dbact_sim.visualization.renderer import ResearchVisualizer
from dbact_sim.visualization.animation import ensure_mp4_export_available

METHODS=("boundary_cvt","nearest_observed_target","local_spacing")
SHORT={"boundary_cvt":"CVT","nearest_observed_target":"Nearest","local_spacing":"Spacing"}
CID="cargo_0"
plt.rcParams.update({"font.size":8,"axes.titlesize":8,"pdf.fonttype":42})


def read(path):return json.loads(path.read_text())


def rows_for(batch,plan):
    rows=[]
    for case in plan["cases"]:
        folder=batch/case["case_id"]
        receipt=read(folder/"case_result.json")
        result=cached_result(folder,receipt["identity"])  # every raw product hash
        if result["execution_status"]!="COMPLETED":
            raise RuntimeError(f"Execution error retained: {case['case_id']}; include error in denominator")
        summary=read(folder/"summary.json"); cargo=summary["cargoes"][CID]; gate=cargo["g500"]
        verdict=ClosedLoopContract(**gate["gates"]).evaluate(gate["metrics"])
        assert verdict.success==gate["success"] and verdict.reasons==gate["failure_reasons"]
        steps=read(folder/"paired_metrics.json")["steps"]
        with np.load(folder/"replay.npz",allow_pickle=False) as replay:
            peak_map_signal=float(np.max(replay["map_coverage"]))
            peak_rotation=float(np.max(np.abs(np.rad2deg(replay[f"cargo/{CID}/angles"]-replay[f"cargo/{CID}/angles"][0]))))
        dt=case["config"]["dt"]
        first_contact=next((s["frame"] for s in steps if s["contacts"]>0),None)
        first_motion=next((s["frame"] for s in steps if s["true_progress"]>1e-9),None)
        if result["task_completed"]:failure="COMPLETED"
        elif result["contact_ready_frame"] is None:failure="DEPLOYMENT_NOT_READY"
        elif first_contact is None:failure="NO_PHYSICAL_CONTACT_RECORDED"
        elif first_motion is None:failure="NO_POSITIVE_PROGRESS_RECORDED"
        elif result["settled"]:failure="SETTLED_OUTSIDE_ENDPOINT_TOLERANCE"
        else:failure="WATCHDOG_"+steps[-1]["phase"]
        row={"condition":case["condition_id"],"shape":case["shape"],"method":case["variant"],"seed":case["seed"],
             "completed":result["task_completed"],"g500":result["g500_pass"],"quality_ok":verdict.metrics["quality_ok"],
             "settled":result["settled"],"termination":result["termination"]["terminated_by"],
             "frames":result["termination"]["frames_run"],"seconds":dt*result["termination"]["frames_run"],
             "endpoint_error_m":result["euclidean_endpoint_error"],"signed_progress_error_m":result["final_distance_error"],
             "max_cross_track_m":result["max_cross_track"],"rotation_deg":result["rotation_deg"],
             "max_boundary_coverage":cargo["max_strict_coverage"],"final_boundary_coverage":cargo["final_strict_coverage"],
             "max_shared_map_coverage_signal":peak_map_signal,"max_abs_rotation_deg":peak_rotation,
             "max_physical_contacts":cargo["max_contacts"],"final_true_progress_m":cargo["J"],
             "final_estimated_progress_previous_control_m":result["final_estimated_progress"],
             "first_contact_report_frame":first_contact,"first_positive_progress_frame":first_motion,
             "physical_contact_ready_frame":result["physical_contact_ready"]["frame"],
             "failure_stage":failure,"g500_reasons":" | ".join(verdict.reasons),
             "quality_warnings":" | ".join(verdict.metrics["quality_reasons"]),
             "min_robot_distance_m":result["min_inter_agent_distance"],
             "min_signed_clearance_m":result["min_signed_clearance"],
             "max_penetration_m":cargo["max_penetration"],
             "barrier_scalings":gate["metrics"]["barrier_scalings"],
             "solver_fallbacks":gate["metrics"]["solver_fallbacks"],"solver_infeasible":gate["metrics"]["solver_infeasible"],
             "initial_state_sha256":result["initial_state_sha256"],"config_sha256":result["config_sha256"],
             "case_path":folder.as_posix()}
        for phase,event in result["events"].items():row[phase.lower()+"_frame"]=event["frame"]
        rows.append(row)
    for condition in plan["conditions"]:
        group=[r for r in rows if r["condition"]==condition["condition"]]
        assert len(group)==3 and len({r["initial_state_sha256"] for r in group})==1 and len({r["config_sha256"] for r in group})==1
    return rows


def write_tables(batch,plan,out):
    rows=rows_for(batch,plan)
    with (out/"all_27_episodes.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    counts={m:{"completed":sum(r["completed"] for r in rows if r["method"]==m),
               "g500":sum(r["g500"] for r in rows if r["method"]==m),
               "quality_ok":sum(r["quality_ok"] for r in rows if r["method"]==m),
               "settled":sum(r["settled"] for r in rows if r["method"]==m),"denominator":9} for m in METHODS}
    dump(out/"analysis.json",{"counts":counts,"rows":rows,"conditions":plan["conditions"],
         "scope":"One fixed development seed, nine geometry/orientation conditions; no reliability inference",
         "time_alignment":"Replay state is after physics; estimated progress at displayed time t comes from control t-dt. Contact report is start-of-hold penalty force. No aligned estimation-error claim from replay.",
         "quality_reconstruction":"Re-evaluated unchanged ClosedLoopContract on stored metrics/gates to recover diagnostic quality_reasons omitted by original summary serialization. Original verdict verified, raw files unchanged.",
         "safety":"Discrete recorded clearances/scaling/fallbacks only; no continuous safety or hardware guarantee"})
    lines=["| Condition | Method | Deploy | Contact | Complete | G500 | Endpoint m | Time s | Failure |",
           "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(f"| {r['condition']} | {SHORT[r['method']]} | {r['contact_ready_frame'] is not None} | {r['first_contact_report_frame'] is not None} | {r['completed']} | {r['g500']} | {r['endpoint_error_m']:.4f} | {r['seconds']:.2f} | {r['failure_stage']} |")
    (out/"capability_table.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    print(json.dumps(counts),flush=True)
    return rows


def draw_comparison(rows,plan,out):
    fig,axes=plt.subplots(1,2,figsize=(7,3.6),gridspec_kw={"width_ratios":[2,1.2]})
    colors=["#176b87","#d57a28","#6554a4"]
    for i,c in enumerate(plan["conditions"]):
        for k,m in enumerate(METHODS):
            r=next(r for r in rows if r["condition"]==c["condition"] and r["method"]==m)
            y=i+(k-1)*.2
            axes[0].scatter(r["endpoint_error_m"],y,color=colors[k],marker="o" if r["completed"] else "x",s=22)
            axes[1].scatter(r["seconds"],y,color=colors[k],marker="o" if r["settled"] else ">",s=22)
    axes[0].axvline(.12,color="gray",ls="--",lw=.8)
    axes[0].set(xlabel="Endpoint error [m]; circle=completed",yticks=range(9),yticklabels=[c["condition"].replace("_"," ") for c in plan["conditions"]])
    axes[1].set(xlabel="Stop / watchdog [s]",yticks=range(9),yticklabels=[])
    for ax in axes:ax.invert_yaxis();ax.grid(axis="x",alpha=.2)
    for k,m in enumerate(METHODS):axes[1].scatter([],[],color=colors[k],label=SHORT[m])
    fig.legend(*axes[1].get_legend_handles_labels(),fontsize=7,loc="upper center",bbox_to_anchor=(.69,1),ncol=3,frameon=False)
    fig.tight_layout(rect=(0,0,1,.94));fig.savefig(out/"method_comparison.pdf");fig.savefig(out/"method_comparison.png",dpi=180);plt.close(fig)


def write_tex_tables(rows,plan,out):
    """Numeric manuscript inputs from the same rows used by CSV and plots."""
    names={"circle":"Circle","ellipse_parallel":"Ellipse parallel","ellipse_perpendicular":"Ellipse perp.",
           "rectangle_parallel":"Rect. parallel","rectangle_perpendicular":"Rect. perp.",
           "l_reference":"L reference","l_rotated":"L rotated","c_open_toward":"C toward","c_open_away":"C away"}
    macros=[];lines=[]
    for method,prefix in zip(METHODS,("Cvt","Nearest","Spacing")):
        for key,suffix in (("completed","Complete"),("g500","G")):
            count=sum(r[key] for r in rows if r["method"]==method)
            macros.append("\\newcommand{\\"+prefix+suffix+"}{"+str(count)+"}")
    for condition in plan["conditions"]:
        name=condition["condition"];values=[]
        for method in METHODS:
            row=next(r for r in rows if r["condition"]==name and r["method"]==method)
            values.append(f"{row['endpoint_error_m']:.3f} / {'T' if row['completed'] else 'F'}")
        lines.append(names[name]+" & "+" & ".join(values)+r" \\")
    (out/"census_numbers.tex").write_text("\n".join(macros)+"\n")
    (out/"census_rows.tex").write_text("\n".join(lines)+"\n")
    table=[r"\begin{table}[t]",
           r"\caption{All nine conditions. Entries: endpoint error in meters / spatial completion (T or F). One fixed seed per condition; three methods share initial states. Full records additionally contain stage times, safety and G500 reasons.}",
           r"\label{tab:census}",r"\centering\footnotesize",r"\begin{tabular}{lccc}\toprule",
           r"Condition & CVT & Nearest & Spacing\\\midrule",*lines,r"\bottomrule\end{tabular}",r"\end{table}"]
    (out/"census_table.tex").write_text("\n".join(table)+"\n")


class Replay:
    def __init__(self,folder):
        self.folder=folder;self.trace=SimulationTrace.from_replay(folder/"replay.npz")
        with np.load(folder/"replay.npz",allow_pickle=False) as f:self.data={k:f[k].copy() for k in f.files}
        self.result=read(folder/"case_result.json")["result"]
        self.end=float(self.trace.times[-1])
    def hud(self,index,physical_time):
        d=self.data;t=self.trace.times[index]
        true=float((d[f"cargo/{CID}/centers"][index]-d[f"task/{CID}/start"])@d[f"task/{CID}/direction"])
        est=float(d[f"cargo/{CID}/progress_estimate"][index])
        err=float(np.linalg.norm(d[f"cargo/{CID}/centers"][index]-d[f"task/{CID}/goal_point"]))
        end="" if physical_time<self.end else f"  ENDED: complete={self.result['task_completed']} G500={self.result['g500_pass']}"
        return f"t={t:.2f}s  {self.trace.phase_labels[index]}{end}\ntrue={true:.3f}m  estimate(previous control)={est:.3f}m\nendpoint={err:.3f}m  physical contacts={int(d[f'cargo/{CID}/contacts'][index])}"


def visual(replay,fig,ax):
    renderer=ResearchVisualizer(replay.trace,view_mode="paper",figure=fig,world_ax=ax,show_cage=False,show_map=True,show_sensor=False)
    ax.get_legend().remove();ax.set(xlim=(0,8),ylim=(0,8));ax.tick_params(labelsize=6)
    return renderer


def draw_stages(batch,plan,out):
    names=plan["visual_selection"]["overview_conditions"]
    fig,axes=plt.subplots(5,3,figsize=(7,7.9));receipt=[]
    for i,name in enumerate(names):
        replay=Replay(batch/(name+"_boundary_cvt"));events=replay.result["events"]
        # Supervisor events are zero-based control indices; trace[0] is the
        # initial state, so the resulting measured state is trace[event + 1].
        def measured(phase):
            value=events[phase]["frame"]
            return None if value is None else min(value+1,replay.trace.frame_count-1)
        start=measured("TRANSPORT");stop=measured("BRAKE")
        frames=[measured("CONTACT_READY"),None if start is None else (start+(stop if stop is not None else replay.trace.frame_count-1))//2,replay.trace.frame_count-1]
        for k,frame in enumerate(frames):
            ax=axes[i,k];stage=("Deployment ready","Transport interval","Actual final")[k]
            if frame is None:
                ax.axis("off");ax.text(.5,.5,f"{name}\n{stage}: NOT REACHED",ha="center",va="center",transform=ax.transAxes,fontsize=7)
            else:
                renderer=visual(replay,fig,ax);renderer.update(frame)
                for arrow in [*renderer.push_arrows,*renderer.goal_arrows.values()]:
                    arrow.set_mutation_scale(5);arrow.set_linewidth(.7)
                for collection in ax.collections:
                    if collection.get_label()=="target":collection.set_sizes([35])
                title=f"{name.replace('_',' ')} | {replay.trace.times[frame]:.2f}s"
                if k==2:title+=f"\ncomplete={replay.result['task_completed']}; error={replay.result['euclidean_endpoint_error']:.3f}m"
                ax.set_title(title,fontsize=6.5);ax.set_xlabel("");ax.set_ylabel("")
            receipt.append({"case":name+"_boundary_cvt","stage":stage,"frame":frame,
                            "replay_sha256":hashlib.sha256((replay.folder/"replay.npz").read_bytes()).hexdigest()})
    fig.subplots_adjust(left=.045,right=.99,bottom=.055,top=.91,hspace=.52,wspace=.12)
    fig.suptitle("Deployment ready                  Transport interval                    Actual termination",fontsize=9,y=.985)
    fig.text(.5,.012,"Orange: true scene (evaluation); teal dots: agent_00 fused map; rings: geometric readiness, not physical contact.",ha="center",fontsize=6.5)
    fig.savefig(out/"five_shape_stages.pdf");fig.savefig(out/"five_shape_stages.png",dpi=200);plt.close(fig)
    # A fixed circle overview shows early information, not only final enclosure.
    replay=Replay(batch/(names[0]+"_boundary_cvt"));events=replay.result["events"]
    frames=[0,*[None if events[p]["frame"] is None else events[p]["frame"]+1 for p in ("ENCLOSE","CONTACT_READY","TRANSPORT")],replay.trace.frame_count-1]
    fig,axes=plt.subplots(1,5,figsize=(7,1.85))
    for ax,frame,title in zip(axes,frames,("Initial","Boundary map","Deployment","Contact / transport","Final")):
        if frame is None:ax.axis("off");ax.text(.5,.5,"Not reached",ha="center");continue
        renderer=visual(replay,fig,ax);renderer.update(frame)
        for arrow in [*renderer.push_arrows,*renderer.goal_arrows.values()]:
            arrow.set_mutation_scale(5);arrow.set_linewidth(.7)
        for collection in ax.collections:
            if collection.get_label()=="target":collection.set_sizes([25])
        ax.set_title(f"{title}\nt={replay.trace.times[frame]:.2f}s",fontsize=6.5);ax.set_xlabel("");ax.set_ylabel("")
        receipt.append({"case":names[0]+"_boundary_cvt","stage":title,"frame":frame,
                        "replay_sha256":hashlib.sha256((replay.folder/"replay.npz").read_bytes()).hexdigest()})
    fig.subplots_adjust(left=.03,right=.995,bottom=.04,top=.80,wspace=.17)
    fig.savefig(out/"full_process.pdf");fig.savefig(out/"full_process.png",dpi=200);plt.close(fig)
    dump(out/"keyframe_receipt.json",receipt)


def make_video(batch,case_ids,path,overview=False):
    replays=[Replay(batch/name) for name in case_ids]
    fig,axes=plt.subplots(2 if overview else 1,3,figsize=(15,9 if overview else 5.2))
    axes=np.asarray(axes).reshape(-1);renderers=[];texts=[]
    for ax,replay,name in zip(axes,replays,case_ids):
        renderers.append(visual(replay,fig,ax));ax.set_title(name.replace("_boundary_cvt"," CVT").replace("_"," "),fontsize=9,pad=8)
        texts.append(ax.text(.0,-.20 if overview else -.11,"",transform=ax.transAxes,fontsize=7.7,va="top"))
    if overview:axes[-1].axis("off");axes[-1].text(.02,.88,"All five fixed reference conditions\nOriginal boundary CVT feedback\n\nOrange: true scene (offline display)\nTeal: agent_00 fused boundary map\nRings: geometric-ready agents\nHUD contacts: actual physics record\n\nTruth: post-physics state\nEstimate: previous control update\nEnded runs hold their last measured state\n\nFixed seed 2; all failures retained\n2.5x playback; 10 fps; stride 5",va="top",fontsize=10)
    fig.subplots_adjust(left=.04,right=.985,top=.88,bottom=.18 if not overview else .11,wspace=.25,hspace=.47)
    banner=fig.suptitle("",fontsize=13)
    duration=max(r.end for r in replays)
    timeline=np.arange(0,duration+1e-8,.25).tolist()
    if timeline[-1]<duration-1e-8:timeline.append(duration)
    writer=animation.FFMpegWriter(fps=10,codec="h264",bitrate=2600,extra_args=["-pix_fmt","yuv420p","-movflags","+faststart"])
    with writer.saving(fig,str(path),dpi=100):
        for t in timeline:
            banner.set_text(f"DBACT full task | physical time {t:.2f}s | 2.5x playback | one development seed")
            for replay,renderer,text in zip(replays,renderers,texts):
                index=int(np.searchsorted(replay.trace.times,min(t,replay.end)+1e-8,side="right")-1)
                renderer.update(max(0,index));text.set_text(replay.hud(max(0,index),t))
            writer.grab_frame()
        fig.savefig(path.with_suffix(".png"),dpi=100)
    plt.close(fig)
    return {"video":path.name,"cases":case_ids,"duration_physical_s":duration,"rendered_frames":len(timeline),
            "fps":10,"stride":5,"nominal_playback_factor":2.5,
            "video_sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
            "replay_sha256":{name:hashlib.sha256((batch/name/"replay.npz").read_bytes()).hexdigest() for name in case_ids}}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch",type=Path,required=True);parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--videos",action="store_true")
    parser.add_argument("--resume-videos",action="store_true",help="Verify completed video receipts and continue only missing videos")
    args=parser.parse_args()
    plan=read(args.batch/"resolved_protocol.json");args.out.mkdir(parents=True,exist_ok=True)
    if args.videos:
        ensure_mp4_export_available();receipts=[]
        previous=read(args.out/"video_receipts.json") if args.resume_videos and (args.out/"video_receipts.json").is_file() else []
        groups=[("multishape_overview",[n+"_boundary_cvt" for n in plan["visual_selection"]["overview_conditions"]],True)]
        groups += [(c["condition"],[c["condition"]+"_"+m for m in METHODS],False) for c in plan["conditions"]]
        for name,cases,overview in groups:
            path=args.out/(name+".mp4")
            if path.exists():
                saved=next((r for r in previous if r["video"]==path.name),None)
                if not saved or saved["cases"]!=cases:
                    raise ValueError(f"Video exists without a matching resume receipt, preserve it: {path}")
                for case,expected in saved["replay_sha256"].items():
                    if hashlib.sha256((args.batch/case/"replay.npz").read_bytes()).hexdigest()!=expected:
                        raise ValueError(f"Changed replay: {case}")
                current=hashlib.sha256(path.read_bytes()).hexdigest()
                if saved.get("video_sha256",current)!=current:raise ValueError(f"Changed video: {path}")
                from imageio_ffmpeg import count_frames_and_secs
                actual_frames,_=count_frames_and_secs(str(path))
                if actual_frames!=saved["rendered_frames"]:raise ValueError(f"Incomplete video: {path}")
                # Early receipts predate output hashing. Decode/count validation
                # establishes current completeness; do not claim a prior hash.
                if "video_sha256" not in saved:saved["output_hash_recorded_after_decode_validation"]=True
                saved["video_sha256"]=current;receipts.append(saved)
                print("Verified existing: "+name,flush=True);continue
            receipts.append(make_video(args.batch,cases,path,overview))
            completed={r["video"] for r in receipts}
            dump(args.out/"video_receipts.json",receipts+[r for r in previous if r["video"] not in completed])
            print(name,flush=True)
        dump(args.out/"video_receipts.json",receipts)
    else:
        rows=write_tables(args.batch,plan,args.out);write_tex_tables(rows,plan,args.out)
        draw_comparison(rows,plan,args.out);draw_stages(args.batch,plan,args.out)


if __name__=="__main__":main()
