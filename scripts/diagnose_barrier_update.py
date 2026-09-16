"""Parallel barrier-update diagnosis for C5 / L11 / L17 (no polar observer).

Top-level worker for Windows spawn.  Does not rewrite completed hashed
certificate-repair runs.  Output lives under the new artifact root.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact.barrier_diagnosis import (  # noqa: E402
    decompose_h,
    explain_agent_object_rows,
    hold_min_true_clearance,
)
from dbact.cpu_budget import (  # noqa: E402
    choose_outer_workers,
    detect_cpu_environment,
    dump_json,
    limit_blas_threads,
    ResourceSampler,
)
from dbact.theorem_mode import TheoremModeAbort, oracle_boundary_view  # noqa: E402

SHAPES = {
    "l_shape": ROOT / "configs/sim/theorem/static_l_shape_n16_oracle.yaml",
    "rectangle": ROOT / "configs/sim/theorem/static_rectangle_n16_oracle.yaml",
    "c_shape": ROOT / "configs/sim/theorem/static_c_shape_n16_oracle.yaml",
}

CASES = [
    {"shape": "c_shape", "seed": 5, "agent": 14, "fail": list(range(228, 233)), "window": (198, 250)},
    {"shape": "l_shape", "seed": 11, "agent": 15, "fail": list(range(238, 241)), "window": (208, 260)},
    {"shape": "l_shape", "seed": 17, "agent": 14, "fail": list(range(318, 323)), "window": (288, 340)},
]

STAGE_C = [
    {"shape": "l_shape", "seed": 2, "agent": 0, "fail": [], "window": (0, 80)},
    {"shape": "c_shape", "seed": 5, "agent": 14, "fail": list(range(228, 250)), "window": (200, 280)},
    {"shape": "l_shape", "seed": 11, "agent": 15, "fail": list(range(220, 260)), "window": (200, 280)},
    {"shape": "l_shape", "seed": 17, "agent": 14, "fail": list(range(290, 340)), "window": (280, 360)},
]

STAGE_D = [
    {"shape": shape, "seed": seed, "agent": 0, "fail": [], "window": (0, 40)}
    for shape in ("l_shape", "rectangle", "c_shape")
    for seed in (2, 5, 8, 11, 17, 23)
]

# Frozen-method independent set. Declared before use; not used to design the cover.
STAGE_E = [
    {"shape": shape, "seed": seed, "agent": 0, "fail": [], "window": (0, 40)}
    for shape in ("l_shape", "rectangle", "c_shape")
    for seed in (29, 31, 37)
]


def _dump(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, default=_default), encoding="utf-8")
    os.replace(tmp, path)


def _default(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    raise TypeError(type(obj))


def load_cfg(shape: str) -> dict:
    return yaml.safe_load(SHAPES[shape].read_text(encoding="utf-8"))


def diagnose_case(payload: dict) -> dict:
    """Windows-spawn worker: one independent 600-step replay with barrier traces."""
    limit_blas_threads(1)
    os.environ["DBACT_CVT_WORKERS"] = str(int(payload.get("cvt_workers", 1)))
    from dbact_sim.environment import SimulationEnvironment

    shape = payload["shape"]
    seed = int(payload["seed"])
    agent = int(payload["agent"])
    fail = [int(x) for x in payload["fail"]]
    w0, w1 = int(payload["window"][0]), int(payload["window"][1])
    frames = int(payload["frames"])
    out_dir = Path(payload["out_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    cfg = payload["cfg"]
    started = time.perf_counter()
    env = SimulationEnvironment(cfg, seed=seed)
    vertices = env.cargoes[0].vertices.copy()
    filt = env.controller.safety
    dt = float(cfg["dt"])
    r_safe = float(env.controller.params.r_safe)
    rho = float(env.controller.params.rho)
    gamma = float(env.controller.params.gamma_obj)
    n_agents = int(cfg["agents"]["count"])
    mode_used = str(env.controller.safety.params.object_row_mode)

    rows = []
    window_frames = []
    k0_fail_frames = []
    prev_obj = [None] * n_agents
    abort = None
    completed = 0
    qp_status = {}

    try:
        for k in range(frames):
            p = np.vstack([ag.position for ag in env.agents])
            cmds = env.controller.step(env.agents, env.cargoes, env.t, env.dt)
            rec = env.controller.last_theorem_record
            results = list(env.controller._last_filter_results)
            view = oracle_boundary_view(env.cargoes, spacing=env.controller.params.theorem_oracle_spacing)
            env.controller.apply_commands(env.agents, cmds, env.dt)
            env.engine.step(env.cargoes, env.agents, env.dt)
            env.t += env.dt
            completed = k + 1
            p_next = np.vstack([ag.position for ag in env.agents])
            u_cmd = rec.u_command.copy()
            for st in rec.solver_status:
                qp_status[str(st)] = qp_status.get(str(st), 0) + 1

            zr = [bool(fr.zero_input_feasible_with_rho) for fr in results]
            zhard = [bool(fr.zero_input_feasible) for fr in results]
            in_k0 = all(str(s) == "optimal" for s in rec.solver_status) and all(zr)
            bad = [i for i in range(n_agents) if (not zr[i]) or str(rec.solver_status[i]) != "optimal"]
            if not in_k0:
                k0_fail_frames.append(k)

            in_window = w0 <= k <= w1
            dump_agents = set(bad)
            if in_window:
                dump_agents.add(agent)
            if k in fail:
                dump_agents.update(range(n_agents))

            frame_pack = {
                "frame": k,
                "in_K0": bool(in_k0),
                "k0_bad_agents": bad,
                "solver_status": list(rec.solver_status),
                "hold_min_pair": rec.hold_min_pair_distance,
                "hold_min_object_clearance": rec.hold_min_object_clearance,
                "agents": {},
            }
            next_obj = [None] * n_agents
            for i in range(n_agents):
                expl = explain_agent_object_rows(
                    filt, p[i], view.points, view.normals, vertices
                )
                next_obj[i] = expl["trace"] if expl["trace"] else None
                if i not in dump_agents:
                    continue
                fr = results[i]
                decomp = decompose_h(p[i], p_next[i], u_cmd[i], dt, prev_obj[i], expl["trace"] or {})
                hold = hold_min_true_clearance(p[i], u_cmd[i], vertices, dt)
                kinds = list(fr.row_kinds or [])
                A = np.asarray(fr.A_rows, dtype=float) if fr.A_rows is not None else np.empty((0, 2))
                b_orig = np.asarray(fr.b_original, dtype=float) if fr.b_original is not None else np.empty(0)
                b_eff = np.asarray(fr.b_effective, dtype=float) if fr.b_effective is not None else np.empty(0)
                u = np.asarray(fr.velocity, dtype=float)
                resid = (A @ u - b_orig) if len(A) and len(b_orig) == len(A) else np.empty(0)
                frame_pack["agents"][str(i)] = {
                    "position": p[i].tolist(),
                    "position_next": p_next[i].tolist(),
                    "u": u.tolist(),
                    "u_nom": None if fr.u_nominal is None else np.asarray(fr.u_nominal).tolist(),
                    "status": fr.status,
                    "zero_input_feasible": bool(fr.zero_input_feasible),
                    "zero_input_feasible_with_rho": bool(fr.zero_input_feasible_with_rho),
                    "row_kinds": kinds,
                    "b_original": b_orig.tolist(),
                    "b_effective": b_eff.tolist(),
                    "residual_original": resid.tolist(),
                    "h_object_qp": None if fr.h_object is None else np.asarray(fr.h_object).tolist(),
                    "explain": expl,
                    "decompose": decomp,
                    "hold_true": hold,
                    "true_clearance_minus_r_safe": float(expl["h_true"]),
                    "qp_object_residual_min": float(fr.object_residual_min),
                    "qp_agent_residual_min": float(fr.agent_residual_min),
                    "qp_wall_residual_min": float(fr.wall_residual_min),
                }
            prev_obj = next_obj
            if dump_agents:
                window_frames.append(frame_pack)
            rows.append(
                {
                    "frame": k,
                    "in_K0": bool(in_k0),
                    "k0_bad_agents": bad,
                    "zhard_false": [i for i, z in enumerate(zhard) if not z],
                    "zr_false": [i for i, z in enumerate(zr) if not z],
                    "h_obj_target": frame_pack["agents"].get(str(agent), {}).get("explain", {}).get("h_aggregate")
                    if str(agent) in frame_pack["agents"]
                    else None,
                    "h_true_target": frame_pack["agents"].get(str(agent), {}).get("true_clearance_minus_r_safe")
                    if str(agent) in frame_pack["agents"]
                    else None,
                    "J_proxy_speed2": float(np.sum(np.sum(u_cmd * u_cmd, axis=1))),
                    "hold_min_object": rec.hold_min_object_clearance,
                    "qp_all_optimal": all(str(s) == "optimal" for s in rec.solver_status),
                }
            )
    except TheoremModeAbort as exc:
        abort = exc.as_dict()

    target_series = []
    for pack in window_frames:
        ag = pack["agents"].get(str(agent))
        if ag is None:
            continue
        expl = ag["explain"]
        decomp = ag["decompose"]
        target_series.append(
            {
                "frame": pack["frame"],
                "in_K0": pack["in_K0"],
                "position": ag["position"],
                "u": ag["u"],
                "u_nom": ag["u_nom"],
                "h_aggregate": expl.get("h_aggregate"),
                "h_true": expl.get("h_true"),
                "h_gap": expl.get("h_agg_minus_h_true"),
                "feature": expl.get("feature"),
                "support": expl.get("support"),
                "zero_in_F_hard": expl.get("zero_in_F_hard"),
                "zero_in_F_rho": expl.get("zero_in_F_rho"),
                "translation_defect": (expl.get("trace") or {}).get("translation_defect"),
                "alignment": (expl.get("trace") or {}).get("alignment_sum_gn_over_G"),
                "n_bar": (expl.get("trace") or {}).get("n_bar"),
                "offset": (expl.get("trace") or {}).get("offset_from_n_k"),
                "anchor": (expl.get("trace") or {}).get("anchor"),
                "n_near": (expl.get("trace") or {}).get("n_near"),
                "n_same_face": (expl.get("trace") or {}).get("n_same_face"),
                "cap_active": (expl.get("trace") or {}).get("cap_active"),
                "rhs_uncapped": (expl.get("trace") or {}).get("rhs_uncapped"),
                "rhs_capped": (expl.get("trace") or {}).get("rhs_capped"),
                "rhs_no_margin_uncapped": (expl.get("trace") or {}).get("rhs_no_margin_uncapped"),
                "decompose": decomp,
                "hold_true": ag.get("hold_true"),
                "status": ag.get("status"),
            }
        )

    summary = {
        "shape": shape,
        "seed": seed,
        "target_agent": agent,
        "declared_fail_frames": fail,
        "observed_k0_fail_frames": k0_fail_frames,
        "frames_requested": frames,
        "frames_completed": completed,
        "abort": abort,
        "qp_status_counts": qp_status,
        "r_safe": r_safe,
        "rho": rho,
        "gamma_obj": gamma,
        "rho_over_gamma": rho / gamma if gamma else None,
        "object_row_mode": mode_used,
        "dt": dt,
        "wall_seconds": time.perf_counter() - started,
        "cvt_workers": int(os.environ.get("DBACT_CVT_WORKERS", "1")),
        "pid": os.getpid(),
    }
    _dump(out_dir / "summary.json", summary)
    _dump(out_dir / "step_index.json", rows)
    _dump(out_dir / "target_series.json", target_series)
    _dump(out_dir / "window_frames.json", window_frames)
    marker = {
        "shape": shape,
        "seed": seed,
        "source": "diagnose_barrier_update",
        "completed": completed,
        "abort": abort,
    }
    _dump(out_dir / "COMPLETE.json", marker)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=ROOT / "artifacts" / "static_main_theorem_closure_2026-09-16")
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--serial", action="store_true")
    parser.add_argument("--subdir", type=str, default="barrier_diagnosis")
    parser.add_argument("--stage-c", action="store_true")
    parser.add_argument("--stage-d", action="store_true")
    parser.add_argument("--stage-e", action="store_true")
    parser.add_argument("--only", nargs="*", default=None, help="Restrict to keys like l_shape:2")
    args = parser.parse_args()
    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    diag = out / args.subdir
    diag.mkdir(parents=True, exist_ok=True)
    if args.stage_e:
        cases = STAGE_E
    elif args.stage_d:
        cases = STAGE_D
    elif args.stage_c:
        cases = STAGE_C
    else:
        cases = CASES
    if args.only:
        want = {(part.split(":")[0], int(part.split(":")[1])) for part in args.only}
        cases = [c for c in cases if (c["shape"], int(c["seed"])) in want]
        if not cases:
            raise SystemExit("empty case list after --only")

    env = detect_cpu_environment(worker_marker="diagnose_barrier_update")
    dump_json(out / "cpu_environment.json", env.as_dict())
    n_tasks = len(cases)
    plan = choose_outer_workers(n_tasks, env)
    if args.serial:
        plan["outer_workers"] = 1
        plan["inner_cvt_workers"] = 1
        plan["reason"] = "user_forced_serial"
    dump_json(diag / "cpu_plan.json", plan)
    dump_json(out / "cpu_plan_diagnosis.json", plan)
    print(
        f"[cpu] physical={env.physical_cpus} logical={env.logical_cpus} "
        f"usable={env.usable_cpus} outer={plan['outer_workers']} cvt={plan['inner_cvt_workers']} "
        f"existing={env.existing_workers} reason={plan['reason']}",
        flush=True,
    )

    payloads = []
    for case in cases:
        payloads.append(
            {
                **case,
                "cfg": load_cfg(case["shape"]),
                "frames": int(args.frames),
                "out_dir": str(diag / f"{case['shape']}_seed{case['seed']}"),
                "cvt_workers": plan["inner_cvt_workers"],
            }
        )
    sampler = ResourceSampler(diag / "resource_usage.csv", interval_s=1.0)
    sampler.start()
    t0 = time.perf_counter()
    workers = int(plan["outer_workers"])
    if workers <= 1 or args.serial:
        summaries = [diagnose_case(p) for p in payloads]
    else:
        summaries = [None] * len(payloads)
        with ProcessPoolExecutor(max_workers=workers) as pool:
            fmap = {pool.submit(diagnose_case, p): i for i, p in enumerate(payloads)}
            for fut in as_completed(fmap):
                i = fmap[fut]
                summaries[i] = fut.result()
                print(json.dumps(summaries[i], default=str), flush=True)
    wall = time.perf_counter() - t0
    sampler.stop()
    dump_json(
        diag / "parallel_benchmark.json",
        {
            "n_cases": len(payloads),
            "outer_workers": workers,
            "inner_cvt_workers": plan["inner_cvt_workers"],
            "wall_seconds": wall,
            "usable_cpus": env.usable_cpus,
            "physical_cpus": env.physical_cpus,
            "logical_cpus": env.logical_cpus,
            "reason": plan["reason"],
        },
    )
    dump_json(diag / "summaries.json", summaries)
    print(json.dumps({"out": str(diag), "wall_seconds": wall, "n": len(summaries)}, indent=2), flush=True)


if __name__ == "__main__":
    limit_blas_threads(1)
    main()
