"""Offline, signed accounting of the frozen v2 logs; no controller changes."""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from dbact_sim.experiments.deployment import dump

LABELS = ("raw map - truth", "fixed gain", "held readout", "candidate readout", "median voter change")


def analyze(folder):
    metrics = json.loads((folder / "paired_metrics.json").read_text())
    task = metrics["summary"]["task"]["cargo_0"]
    direction = np.array(task["direction"])
    raw, gain, held, candidate, values, weights = [], [], [], [], [], []
    times, truth, team, positions = [], [], [], []
    registration, fusion, local_feedback = [], [], []
    path = folder / "registration.jsonl.gz"
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        for index, row in enumerate(map(json.loads, stream)):
            assert row["control_frame"] == index
            assert metrics["steps"][index]["frame"] == index + 1
            agents = sorted(row["agents"], key=lambda a:a["agent"])
            u = np.array([np.dot(a["m_post"],direction) for a in agents])
            g = np.array([a["gain"] for a in agents])
            original = np.array([a["original"] for a in agents])
            r = np.array([a["returned"] for a in agents])
            eligible = [i for i,a in enumerate(agents) if a["quality"] != "unavailable"] or list(range(len(agents)))
            ordered = sorted(eligible, key=lambda i:(r[i],agents[i]["agent"]))
            w = np.zeros(len(agents)); n=len(ordered)
            w[ordered[(n-1)//2]] += .5; w[ordered[n//2]] += .5
            raw.append(u); gain.append((g-1)*u); held.append(original-g*u); candidate.append(r-original)
            values.append(r); weights.append(w)
            times.append(row["time"]); truth.append(row["truth"]["cargo_0"]["progress"])
            team.append(row["team_progress"]["cargo_0"])
            positions.append(row["truth"]["cargo_0"]["position"])
            local_feedback.append([a["local_feedback"] for a in agents])
            for key, dest in (("registration",registration),("fusion",fusion)):
                dest.append([np.dot(a[key]["translation"],direction) if a[key] else 0. for a in agents])
    assert len(times) == len(metrics["steps"])
    u,g,h,c,r,w = map(np.asarray,(raw,gain,held,candidate,values,weights))
    t,j,team = map(np.asarray,(times,truth,team))
    err = team-j
    reconstructed = np.sum(w*r,axis=1)
    median_residual=float(np.max(np.abs(reconstructed-team)))
    if median_residual>1e-12:
        raise ValueError("Cannot reconstruct actual team voters; do not infer missing votes")
    levels=np.stack([np.sum(w*(u-j[:,None]),axis=1),np.sum(w*g,axis=1),
                     np.sum(w*h,axis=1),np.sum(w*c,axis=1)],axis=1)
    increments=np.zeros((len(t),5)); increments[0,:4]=levels[0]
    for k,a in enumerate((u,g,h,c)):
        da=np.diff(a,axis=0)
        if k==0: da-=np.diff(j)[:,None]
        increments[1:,k]=np.sum(w[1:]*da,axis=1)
    increments[1:,4]=np.sum(np.diff(w,axis=0)*r[:-1],axis=1)
    accumulated=np.cumsum(increments,axis=0)
    residual=float(np.max(np.abs(accumulated.sum(axis=1)-err)))
    identity=float(np.max(np.abs(np.diff(u,axis=0)-np.asarray(registration)[1:]-np.asarray(fusion)[1:])))
    if residual>1e-12 or identity>1e-12:
        raise ValueError("Signed accounting or map translation identity failed")
    steps=metrics["steps"]
    hold=next((k for k,s in enumerate(steps) if s["phase"]=="HOLD"),None)
    hold_info=None
    if hold is not None:
        goal=np.array(task["goal_point"])
        hold_info={"control_frame":hold,"time_s":float(t[hold]),"estimated_progress_m":float(team[hold]),
                   "true_progress_m":float(j[hold]),"target_m":task["target_distance"],
                   "true_endpoint_error_m":float(np.linalg.norm(np.array(positions[hold])-goal)),
                   "true_speed_m_s":float(np.linalg.norm(steps[hold-1]["true_velocity"])) if hold else 0.,
                   "truth_short_of_target":bool(j[hold]<task["target_distance"]),
                   "outside_endpoint_tolerance":bool(np.linalg.norm(np.array(positions[hold])-goal)>task["tolerance"])}
    # Local loop.holding is not persisted; the input alone does not prove its
    # hidden hysteretic state. Keep this limitation instead of fabricating events.
    info={"frames":len(t),"median_reconstruction_max_residual":median_residual,
          "cumulative_identity_max_residual":residual,"map_translation_max_residual":identity,
          "final_signed_error_m":float(err[-1]),"final_level_components_m":dict(zip(LABELS[:4],levels[-1].tolist())),
          "final_accumulated_components_m":dict(zip(LABELS,accumulated[-1].tolist())),
          "voter_weight_change_frames":int(np.count_nonzero(np.any(np.diff(w,axis=0)!=0,axis=1))),
          "nonzero_held_readout_agent_frames":int(np.count_nonzero(np.abs(h)>1e-12)),
          "first_shared_HOLD":hold_info,"local_loop_holding":"NOT_RECORDED: private hysteretic state cannot be identified exactly",
          "g500":json.loads((folder/"summary.json").read_text())["cargoes"]["cargo_0"]["g500"],
          "input_sha256":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (path,folder/"paired_metrics.json",folder/"summary.json")}}
    return info,(t,err,accumulated)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch",type=Path,required=True); parser.add_argument("--out",type=Path,required=True)
    args=parser.parse_args(); args.out.mkdir(parents=True,exist_ok=False)
    plt.rcParams.update({"font.size":8,"pdf.fonttype":42})
    report={"interpretation":"Exact signed accounting along endogenous observed trajectories, not causal component effects; baseline and certificate passivity established separately. Median ties use agent-ID order: weights and component allocation are not uniquely identified by the team scalar, and voter-change counts include this tie rule.",
            "conclusion":"No new implementation defect established. Accumulated map bias and fixed-gain/hold/voter effects are testable hypotheses; current candidate search is closed.","cases":{}}
    for case in sorted(p for p in args.batch.iterdir() if (p/"baseline").is_dir()):
        data={}; report["cases"][case.name]={}
        for arm in ("baseline","certificate_only","anchored_readout"):
            info,arrays=analyze(case/arm); report["cases"][case.name][arm]=info;data[arm]=arrays
        fig,axes=plt.subplots(2,1,figsize=(6.8,5),layout="constrained")
        for arm,(t,err,_) in data.items(): axes[0].plot(t,1000*err,label=arm)
        axes[0].axhline(0,color="k",lw=.5);axes[0].set_ylabel("Signed team error [mm]");axes[0].legend(fontsize=7)
        t,err,parts=data["baseline"]
        for i,label in enumerate(LABELS):axes[1].plot(t,1000*parts[:,i],label=label)
        axes[1].plot(t,1000*err,"k--",label="sum / actual error")
        axes[1].set(xlabel="Physical time [s]",ylabel="Baseline cumulative terms [mm]")
        axes[1].legend(fontsize=7,ncol=2);fig.suptitle(case.name.replace("development_", "").replace("_boundary_cvt", ""))
        for ax in axes:
            ax.grid(alpha=.2)
            hold=report["cases"][case.name]["baseline"]["first_shared_HOLD"]
            if hold:ax.axvline(hold["time_s"],color="gray",ls=":",lw=.8)
        axes[0].text(.98,.03,"Dotted line: baseline first HOLD",transform=axes[0].transAxes,fontsize=7,
                     ha="right",bbox={"facecolor":"white","edgecolor":"none","alpha":.9})
        fig.savefig(args.out/(case.name+".png"),dpi=160);fig.savefig(args.out/(case.name+".pdf"));plt.close(fig)
    dump(args.out/"signed_error_analysis.json",report)
    print(json.dumps({c:{a:r["final_signed_error_m"] for a,r in arms.items()} for c,arms in report["cases"].items()},indent=2))


if __name__=="__main__":main()
