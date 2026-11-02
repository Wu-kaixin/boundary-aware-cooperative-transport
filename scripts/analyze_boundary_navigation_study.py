"""Audit the fixed six-method batch; evaluate both horizons from saved data only."""
from __future__ import annotations
import csv
import argparse
import json
from pathlib import Path
import numpy as np

from dbact.contracts import ClosedLoopContract
from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, verify_source_snapshot
from freeze_boundary_navigation_study import ROOT, MISSION, ARMS, read, sha
from analyze_contact_handoff_study import replay_prefix_equal
from analyze_dynamic_contact_study import old_fields_equal


def at_horizon(times, positions, goal, seconds, termination, completed):
    """No interpolation, no success inferred from briefly passing through target."""
    times = np.asarray(times)
    k = min(len(times)-1, int(np.searchsorted(times, seconds + 1e-8, side='right'))-1)
    ended = termination <= seconds + 1e-8
    return {"time": float(times[k]), "endpoint_error_m": float(np.linalg.norm(positions[k]-goal)),
            "completed": bool(ended and completed), "terminated": bool(ended), "index": k}


def analysis_output(requested=None):
    allowed = (ROOT/'artifacts/research'/MISSION).resolve()
    out = Path(requested).resolve() if requested is not None else allowed
    if not out.is_relative_to(allowed):
        raise ValueError('Analysis output must stay inside this mission artifact directory')
    if (out/'analysis.json').exists():
        raise RuntimeError('Analysis exists; preserve its identity and choose a new --out directory')
    return out


def analyze(requested_out=None):
    out = analysis_output(requested_out)
    plan = read(ROOT/"experiments/ccta2027/boundary_navigation_v1.json")
    batch = ROOT/plan["run_dir"]
    manifest, summary = read(batch/"manifest.json"), read(batch/"summary.json")
    if summary.get("source_integrity") != "PASS" or summary["completed_denominator"] != 54:
        raise RuntimeError("Incomplete execution; preserve outputs and report the unfinished budget")
    verify_source_snapshot(batch, manifest["source_files"])
    assert encode(manifest["source_files"]) == plan["frozen_source_sha256"]
    for name, digest in plan["historical_inputs"].items():
        if sha(ROOT/name) != digest: raise RuntimeError(f"Historical input changed: {name}")
    out.mkdir(parents=True, exist_ok=True)
    rows, trajectories, compatibility, input_hashes = [], {}, {}, dict(plan["historical_inputs"])
    mechanism = {}
    for case in plan["cases"]:
        print(f"Auditing {case['case_id']}", flush=True)
        folder = batch/case["case_id"]
        receipt = read(folder/"case_result.json")
        assert receipt["identity"] == {**manifest["identity"], "case_sha256":encode(case)}
        r = cached_result(folder, receipt["identity"])
        input_hashes[(folder/"case_result.json").relative_to(ROOT).as_posix()] = sha(folder/"case_result.json")
        for name, digest in receipt["products"].items():
            input_hashes[(folder/name).relative_to(ROOT).as_posix()] = digest
        row = dict(condition=case["condition_id"], arm=case["arm"], seed=case["seed"],
            execution_status=r["execution_status"], wall_seconds=r["wall_seconds"],
            completed=bool(r.get("task_completed")), completed_150=False, g500=bool(r.get("g500_pass")),
            error=r.get("error", ""))
        if r["execution_status"] != "COMPLETED":
            rows.append(row); continue
        steps = read(folder/"paired_metrics.json")["steps"]
        with np.load(folder/"replay.npz", allow_pickle=False) as z:
            times, centers = z["times"].copy(), z["cargo/cargo_0/centers"].copy()
            physical_speeds = z["cargo/cargo_0/speed"].copy()
            start, goal = z["task/cargo_0/start"].copy(), z["task/cargo_0/goal_point"].copy()
            directions = z["task/cargo_0/direction"].copy()
        seconds = case["config"]["dt"]*r["termination"]["frames_run"]
        h150 = at_horizon(times, centers, goal, 150., seconds, row["completed"])
        h600 = at_horizon(times, centers, goal, 600., seconds, row["completed"])
        assert abs(h600["endpoint_error_m"]-r["euclidean_endpoint_error"]) < 1e-12
        assert bool(r["settled"] and h600["endpoint_error_m"] <= case["config"]["task"]["tolerance"]) == row["completed"]
        physical_ready = ((np.linalg.norm(centers-goal,axis=1) <= case['config']['task']['tolerance'])
                          & (physical_speeds <= r['termination']['settle_speed']))
        terminal_streak = 0
        for flag in physical_ready[:0:-1]:
            if not flag: break
            terminal_streak += 1
        gate = read(folder/"summary.json")["cargoes"]["cargo_0"]["g500"]
        contract = ClosedLoopContract(**gate["gates"])
        verdict = contract.evaluate(gate["metrics"])
        assert verdict.success == gate["success"] and verdict.reasons == gate["failure_reasons"]
        m, hard = gate["metrics"], []
        if m["min_inter_agent_distance"] < m["d_min"] - contract.d_min_tolerance: hard.append("separation")
        if m["min_signed_clearance"] < 0: hard.append("inside_object")
        if m["max_penetration"] > m["penetration_budget"]: hard.append("penetration_budget")
        if m["solver_fallbacks"] or m["solver_infeasible"]: hard.append("solver_failure")
        last = steps[-1]
        nav = [d for d in last["diagnostics"] if d.get("navigation_voters")]
        median_estimate = (np.median([d["navigation_displacement"] for d in nav],axis=0).tolist() if nav else None)
        nav_times, nav_error, true_goal_error, nav_position_error = [], [], [], []
        live_counts, selected_counts = [], []
        for index, step in enumerate(steps):
            diagnostics = step['diagnostics']
            live = [d for d in diagnostics if d.get('navigation_voters')]
            live_counts.append(len(live))
            selected_counts.append(sum(d['push_side'] for d in diagnostics))
            if live:
                # Diagnostics are the command's pre-physics feedback. Compare
                # against center[index], not the following post-physics center.
                estimate = np.median([d['navigation_displacement'] for d in live],axis=0)
                truth = centers[index]-start
                nav_times.append(float(times[index]))
                nav_error.append(float(np.linalg.norm(goal-start-estimate)))
                true_goal_error.append(float(np.linalg.norm(goal-centers[index])))
                nav_position_error.append(float(np.linalg.norm(estimate-truth)))
        cumulative_vector_messages = sum(sum(v != d['agent_id'] for v in d.get("navigation_voters", "").split('|'))
            for s in steps for d in s["diagnostics"] if d.get("navigation_voters"))
        row.update(seconds=seconds, completion_seconds=seconds if row["completed"] else None,
            completed_150=h150["completed"], endpoint_error_150_m=h150["endpoint_error_m"],
            endpoint_error_m=h600["endpoint_error_m"], settled=bool(r["settled"]),
            outcome="COMPLETED" if row["completed"] else "SETTLED_OFF_TARGET" if r["settled"] else "CENSORED_600S",
            outcome_150="COMPLETED" if h150["completed"] else "SETTLED_OFF_TARGET" if h150["terminated"] and r["settled"] else "CENSORED_150S",
            posthoc_physical_target_and_low_speed_tail_s=terminal_streak*case['config']['dt'],
            posthoc_goal_static_without_hold=bool(terminal_streak>=r['termination']['settle_frames'] and last['phase']!='HOLD'),
            final_phase=last["phase"], true_progress_m=float((centers[-1]-start)@directions),
            cross_track_final_m=float((centers[-1]-start)@np.array([-directions[1],directions[0]])),
            max_cross_track_m=r["max_cross_track"], estimated_progress_m=r["final_estimated_progress"],
            rotation_deg=r['rotation_deg'],
            final_navigation_vote_median=median_estimate, hard_safety="|".join(hard),
            navigation_logged_frames=len(nav_times),
            median_voted_position_error_mean_m=float(np.mean(nav_position_error)) if nav_position_error else None,
            median_voted_position_error_final_m=nav_position_error[-1] if nav_position_error else None,
            final_median_voted_goal_error_m=nav_error[-1] if nav_error else None,
            final_controller_remaining_m=float(case['config']['task']['distance_min']-last['phase_guards']['progress'])
                if 'progress' in last.get('phase_guards',{}) else None,
            barrier_scalings=m["barrier_scalings"], min_separation_m=m["min_inter_agent_distance"],
            max_penetration_m=m["max_penetration"], min_clearance_m=m["min_signed_clearance"],
            solver_infeasible=m["solver_infeasible"], solver_fallbacks=m["solver_fallbacks"],
            g500_reasons=" | ".join(verdict.reasons), navigation_neighbor_vote_uses=cumulative_vector_messages,
            final_true_speed=float(np.linalg.norm(last["true_velocity"])),
            mean_positive_contacts=float(np.mean([s["positive_contacts"] for s in steps])),
            mean_push_flags=float(np.mean([sum(d["push_side"] for d in s["diagnostics"]) for s in steps])))
        rows.append(row)
        trajectories[case["case_id"]] = dict(times=times,xy=centers-start,goal=goal-start,cut=h150["index"])
        mechanism[case['case_id']] = dict(time=nav_times,estimated_goal_error=nav_error,
            true_goal_error=true_goal_error,median_voted_position_error=nav_position_error,
            caveat='Median of logged local one-hop feedbacks, not the shared supervisor median; diagnostic frame series are not independent samples')
        if case["arm"] in ("cvt", "spacing"):
            oldname = case["factors"]["source_case"]
            if case["arm"] == "spacing": oldname = oldname.replace("boundary_cvt", "local_spacing")
            reference = ROOT/"runs/research/2026-10-31-multishape-census-v1/run"/oldname
            oldsteps = read(reference/"paired_metrics.json")["steps"]
            arrays = replay_prefix_equal(reference/"replay.npz", folder/"replay.npz")
            compatibility[case["case_id"]] = dict(arrays=arrays,
                old_fields_exact=old_fields_equal(oldsteps, steps[:len(oldsteps)]))
        del steps
    counts = {arm:dict(completed_150=sum(r["completed_150"] for r in rows if r["arm"]==arm),
        completed_600=sum(r["completed"] for r in rows if r["arm"]==arm),denominator=9,
        g500=sum(r["g500"] for r in rows if r["arm"]==arm),
        errors=sum(r["execution_status"]!="COMPLETED" for r in rows if r["arm"]==arm),
        barrier_scaling_events=sum(r.get('barrier_scalings',0) for r in rows if r['arm']==arm),
        hard_safety_episodes=sum(bool(r.get("hard_safety")) for r in rows if r["arm"]==arm)) for arm in ARMS}
    names = [c["condition_id"] for c in plan["cases"] if c["arm"]=="cvt"]
    names = names[1:]+names[:1]  # C-away last for the common five-shape ordering
    lookup = {(r["condition"],r["arm"]):r for r in rows}
    paired = {}
    for arm in list(ARMS)[2:]:
        paired[arm] = {}
        for ref in ("cvt", "spacing"):
            gain=[n for n in names if lookup[n,arm]["completed"] and not lookup[n,ref]["completed"]]
            loss=[n for n in names if lookup[n,ref]["completed"] and not lookup[n,arm]["completed"]]
            unsafe=[n for n in names if lookup[n,arm].get("hard_safety") and not lookup[n,ref].get("hard_safety")]
            paired[arm][ref] = dict(gained=gain,lost=loss,new_hard_safety=unsafe,net=len(gain)-len(loss))
    verified = all(v["old_fields_exact"] and all(v["arrays"].values()) for v in compatibility.values())
    report = dict(mission=MISSION,counts=counts,paired=paired,compatibility=compatibility,
        compatibility_pass=verified,all_episodes=rows,source_integrity="PASS",historical_integrity="PASS",
        scope="Same reused development seed=2; conditions are paired, frames are not samples; no default promotion",
        communication="Vote-use counts are logged solver inputs, not measured wire traffic; two displacement floats per received vector message",
        posthoc_diagnostic="Physical target + low-speed tail was added after observing a completed C-away record; not a success redefinition, promotion gate or predeclared primary outcome",
        analysis_source_sha256=sha(Path(__file__)))
    dump(out/"analysis.json",report)
    dump(out/"navigation_diagnostics.json",mechanism)
    dump(out/"provenance.json",dict(inputs=input_hashes,execution_source_sha256=plan["frozen_source_sha256"],
        analysis_source_sha256=sha(Path(__file__)),helper_sources={name:sha(ROOT/"scripts"/name) for name in
            ("freeze_boundary_navigation_study.py","analyze_contact_handoff_study.py","analyze_dynamic_contact_study.py")}))
    with (out/"all_episodes.csv").open("w",newline="",encoding="utf8") as f:
        w=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)));w.writeheader();w.writerows(rows)
    table=["| Condition | "+" | ".join(ARMS)+" |","|---|"+"---|"*len(ARMS)]
    for name in names:
        cells=[]
        for arm in ARMS:
            r=lookup[name,arm]
            cells.append(f"{r.get('outcome','ERROR')} / {r.get('endpoint_error_m',float('nan')):.4f} m")
        table.append("| "+name+" | "+" | ".join(cells)+" |")
    (out/"capability_table.md").write_text("\n".join(table)+"\n",encoding="utf8")
    plot(rows,names,trajectories,out)
    plot_feedback(names, mechanism, out)
    print(json.dumps({"counts":counts,"paired":paired,"compatibility_pass":verified},indent=2))
    if not verified: raise RuntimeError("Historical control reproduction failed; do not claim fair comparison")


def plot(rows,names,trajectories,out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    plt.rcParams.update({"font.size":7,"pdf.fonttype":42,"svg.fonttype":"none"})
    colors=("#0072B2","#E69F00","#CC79A7","#56B4E9","#009E73","#D55E00")
    markers=('o','s','D','v','^','P')
    fig,axs=plt.subplots(1,2,figsize=(7.16,3.35),sharey=True)
    for ax,horizon in zip(axs,(150,600)):
        for j,arm in enumerate(ARMS):
            group={r['condition']:r for r in rows if r['arm']==arm}
            for k,name in enumerate(names):
                r=group[name]; error=r.get('endpoint_error_150_m' if horizon==150 else 'endpoint_error_m')
                if error is None: continue
                done=r['completed_150' if horizon==150 else 'completed']
                ax.scatter(k+(j-2.5)*.11,error,s=18,marker=markers[j],edgecolors=colors[j],
                    facecolors=colors[j] if done else 'none',label=arm if k==0 else None)
        ax.axhline(.12,color='k',lw=.7,ls=':');ax.set_title(f'{horizon} s observation; filled = completed')
        ax.set_xticks(range(9),[n.replace('_',' ') for n in names],rotation=60,ha='right',fontsize=6)
        ax.grid(axis='y',alpha=.2)
    axs[0].set_ylabel('2D endpoint error (m)')
    fig.legend(*axs[0].get_legend_handles_labels(),loc='upper center',ncol=3,frameon=False)
    fig.tight_layout(rect=(0,0,1,.87))
    for ext in ('pdf','svg','png'): fig.savefig(out/f'endpoint_comparison.{ext}',dpi=220)
    plt.close(fig)
    fig,axs=plt.subplots(3,3,figsize=(7.16,6.0))
    for ax,name in zip(axs.flat,names):
        for j,arm in enumerate(ARMS):
            t=trajectories.get(f'{name}_{arm}')
            if t is None: continue
            ax.plot(t['xy'][:,0],t['xy'][:,1],color=colors[j],lw=.8,label=arm,ls=('-', '--',':','-.','-','--')[j])
            ax.scatter(*t['xy'][-1],s=13,marker=markers[j],color=colors[j])
            if t['cut']<len(t['xy'])-1: ax.scatter(*t['xy'][t['cut']],s=10,marker='x',color=colors[j])
        ax.add_patch(Circle((1.2,0),.12,fill=False,ec='k',ls=':',lw=.6))
        ax.scatter(0,0,s=10,c='k');ax.set_title(name.replace('_',' '));ax.set_aspect('equal',adjustable='datalim')
        ax.set_xlabel('Task-axis displacement (m)');ax.set_ylabel('Lateral displacement (m)');ax.grid(alpha=.2)
    fig.legend(*axs.flat[0].get_legend_handles_labels(),loc='upper center',ncol=3,frameon=False)
    fig.tight_layout(rect=(0,0,1,.93))
    for ext in ('pdf','svg','png'): fig.savefig(out/f'complete_trajectories.{ext}',dpi=220)
    plt.close(fig)


def plot_feedback(names,mechanism,out):
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(3,3,figsize=(7.16,5.7))
    for ax,name in zip(axs.flat,names):
        for arm,color in [('vector_cvt','#009E73'),('vector_spacing','#D55E00')]:
            series=mechanism.get(f'{name}_{arm}',{})
            t=series.get('time',[])
            if not t: continue
            ax.plot(t,series['true_goal_error'],color=color,lw=.8,label=f'{arm} truth')
            ax.plot(t,series['estimated_goal_error'],color=color,lw=.7,ls='--',label=f'{arm} estimate')
        ax.axhline(.12,color='k',ls=':',lw=.6)
        ax.set_title(name.replace('_',' '));ax.set_xlabel('Physical time (s)');ax.set_ylabel('2D goal error (m)')
        ax.grid(alpha=.2)
    fig.legend(*axs.flat[0].get_legend_handles_labels(),loc='upper center',ncol=2,frameon=False)
    fig.tight_layout(rect=(0,0,1,.91))
    for ext in ('pdf','svg','png'):fig.savefig(out/f'estimated_and_true_goal_error.{ext}',dpi=220)
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, help='New directory inside artifacts/research/2026-11-01-boundary-navigation-v1')
    analyze(parser.parse_args().out)
