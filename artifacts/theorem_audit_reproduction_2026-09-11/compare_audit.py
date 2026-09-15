"""Item-by-item comparison of reproduced audit vs packaged reference."""
from __future__ import annotations

import hashlib
import json
import math
import os
import platform
import subprocess
import sys
from pathlib import Path

import numpy as np
import yaml

REF = Path(r"e:\boundary-aware-cooperative-transport\artifacts\theorem_audit_2026-09-11_source\DBACT_audit\results")
NEW = Path(r"e:\boundary-aware-cooperative-transport\artifacts\theorem_audit_reproduction_2026-09-11\reproduced_results")
REPO = Path(r"e:\dbact-proof-audit-source")
OUT = Path(r"e:\boundary-aware-cooperative-transport\artifacts\theorem_audit_reproduction_2026-09-11")


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


def rel_err(a, b):
    a, b = float(a), float(b)
    denom = max(abs(a), abs(b), 1e-30)
    return abs(a - b) / denom


def flatten(obj, prefix=""):
    if isinstance(obj, dict):
        for k, v in obj.items():
            yield from flatten(v, f"{prefix}.{k}" if prefix else k)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            yield from flatten(v, f"{prefix}[{i}]")
    else:
        yield prefix, obj


def compare_json(name, numeric_tol=0.0):
    a = json.loads((REF / name).read_text(encoding="utf-8"))
    b = json.loads((NEW / name).read_text(encoding="utf-8"))
    fa, fb = dict(flatten(a)), dict(flatten(b))
    keys = sorted(set(fa) | set(fb))
    rows = []
    for k in keys:
        if k not in fa:
            rows.append(dict(key=k, status="missing_in_reference", ref=None, new=fb[k]))
            continue
        if k not in fb:
            rows.append(dict(key=k, status="missing_in_reproduction", ref=fa[k], new=None))
            continue
        va, vb = fa[k], fb[k]
        if isinstance(va, (int, float)) and isinstance(vb, (int, float)) and not (
            isinstance(va, bool) or isinstance(vb, bool)
        ):
            if math.isnan(float(va)) and math.isnan(float(vb)):
                status = "match"
            elif va == vb:
                status = "match"
            elif abs(float(va) - float(vb)) <= numeric_tol:
                status = "within_tol"
            else:
                status = "numeric_diff"
            rows.append(dict(key=k, status=status, ref=va, new=vb,
                             abs_err=abs(float(va) - float(vb)),
                             rel_err=rel_err(va, vb)))
        else:
            rows.append(dict(key=k, status="match" if va == vb else "value_diff",
                             ref=va, new=vb))
    return rows


def csv_compare(seed):
    a = np.genfromtxt(REF / f"seed_{seed}" / "residuals.csv", delimiter=",", names=True)
    b = np.genfromtxt(NEW / f"seed_{seed}" / "residuals.csv", delimiter=",", names=True)
    fields = [n for n in a.dtype.names if n in b.dtype.names]
    n = min(len(a), len(b))
    out = dict(n_ref=len(a), n_new=len(b), fields=fields, per_field={}, first_diff=None)
    for f in fields:
        xa, xb = np.asarray(a[f], dtype=float)[:n], np.asarray(b[f], dtype=float)[:n]
        both_nan = np.isnan(xa) & np.isnan(xb)
        finite = ~(np.isnan(xa) | np.isnan(xb))
        abs_e = np.zeros(n)
        abs_e[finite] = np.abs(xa[finite] - xb[finite])
        abs_e[both_nan] = 0.0
        mismatch = ~both_nan & (np.isnan(xa) | np.isnan(xb) | (abs_e > 0))
        idx = int(np.argmax(mismatch)) if mismatch.any() else None
        out["per_field"][f] = dict(
            max_abs=float(np.max(abs_e)),
            mean_abs=float(np.mean(abs_e)),
            last_abs=float(abs_e[-1]),
            n_mismatch=int(np.sum(mismatch)),
            first_mismatch_index=idx,
            first_mismatch_time=float(a["time"][idx]) if idx is not None else None,
            ref_at_first=float(xa[idx]) if idx is not None else None,
            new_at_first=float(xb[idx]) if idx is not None else None,
        )
    # overall first residual mismatch excluding time
    first = None
    for i in range(n):
        for f in fields:
            if f == "time":
                continue
            va, vb = float(a[f][i]), float(b[f][i])
            if math.isnan(va) and math.isnan(vb):
                continue
            if va != vb:
                first = dict(index=i, time=float(a["time"][i]), field=f, ref=va, new=vb,
                             abs_err=abs(va - vb) if not (math.isnan(va) or math.isnan(vb)) else None)
                break
        if first:
            break
    out["first_any_diff"] = first
    return out


def trajectory_compare(seed):
    a = np.load(REF / f"seed_{seed}" / "trajectory.npz")
    b = np.load(NEW / f"seed_{seed}" / "trajectory.npz")
    report = {}
    for key in ["positions", "vertices", "velocities"]:
        xa, xb = a[key], b[key]
        report[key] = dict(shape_ref=list(xa.shape), shape_new=list(xb.shape))
        n = min(len(xa), len(xb))
        diffs = np.linalg.norm((xa[:n] - xb[:n]).reshape(n, -1), axis=1)
        first = int(np.argmax(diffs > 0)) if np.any(diffs > 0) else None
        report[key].update(
            max_norm=float(np.max(diffs)),
            mean_norm=float(np.mean(diffs)),
            first_nonzero_frame=first,
            first_nonzero_norm=float(diffs[first]) if first is not None else 0.0,
            last_norm=float(diffs[-1]),
        )
    report["direction_ref"] = a["direction"].tolist()
    report["direction_new"] = b["direction"].tolist()
    report["direction_abs"] = float(np.linalg.norm(a["direction"] - b["direction"]))
    report["dt_ref"] = float(a["dt"])
    report["dt_new"] = float(b["dt"])
    return report


def object_speed(seed, path):
    data = np.load(path / f"seed_{seed}" / "trajectory.npz")
    vertices = data["vertices"]
    dt = float(data["dt"])
    # centroid of polygon vertices as a proxy for object translation
    centroids = vertices.mean(axis=1)
    dxy = np.diff(centroids, axis=0)
    speed = np.linalg.norm(dxy, axis=1) / dt
    # yaw from first edge
    e0 = vertices[:, 1, :] - vertices[:, 0, :]
    yaw = np.arctan2(e0[:, 1], e0[:, 0])
    dyaw = np.diff(np.unwrap(yaw)) / dt
    return dict(
        max_translation_speed=float(np.max(speed)),
        mean_translation_speed=float(np.mean(speed)),
        max_yaw_rate=float(np.max(np.abs(dyaw))),
        exceeds_max_object_speed_0p25=bool(np.max(speed) > 0.25),
        n_frames_over_0p25=int(np.sum(speed > 0.25)),
    )


def safety_compare(seed):
    a = json.loads((REF / f"seed_{seed}" / "safety_diagnostics.json").read_text(encoding="utf-8"))
    b = json.loads((NEW / f"seed_{seed}" / "safety_diagnostics.json").read_text(encoding="utf-8"))
    n = min(len(a), len(b))
    first = None
    clip_ref = sum(x["clipping_agents"] for x in a)
    clip_new = sum(x["clipping_agents"] for x in b)
    for i in range(n):
        for k in a[i]:
            va, vb = a[i][k], b[i][k]
            if va != vb:
                first = dict(frame=i, time=a[i].get("time"), key=k, ref=va, new=vb)
                break
        if first:
            break
    return dict(n_ref=len(a), n_new=len(b), clip_events_ref=clip_ref, clip_events_new=clip_new,
                first_diff=first)


def independent_constants():
    N, R, kc, u, ds, sigma, phi0, dt, grid = 16, 0.8, 0.9, 0.35, 0.28, 0.2, 0.001, 0.05, 20
    width = height = 8.0
    L = 7.2
    gamma_agent = 6.0
    r0 = min(R, ds / 2, width / 2, height / 2)
    mlo = phi0 * math.pi * r0 * r0 / 4
    mhi = math.pi * R * R * (phi0 + L)
    a = kc / (2 * mhi)
    d_terms = math.sqrt(N) * (kc * 2 * R + max(kc * R - u, 0) + 2 * u)
    d_alt = math.sqrt(N) * (u + kc * R)
    d = min(d_terms, d_alt)
    CK = 2 * math.pi * sigma ** 2
    LK = math.sqrt(2) * math.pi ** 1.5 * sigma
    M = phi0 * width * height + CK * L
    h = 2 * R / (grid - 1)
    rho = math.sqrt(2) * h / 2
    return dict(
        r0=r0, m_minus=mlo, m_plus=mhi, a_minus=a, alpha=a / 2,
        d_terms_b0=d_terms, d_speed_alternative=d_alt, dbar=d,
        CK=CK, LK=LK, M_total_upper=M,
        beta_static=d * d / (2 * a),
        B_gradient_static=d * d / (a * a),
        B_gradient_geometry_cell=4 * N * mhi * mhi * R * R,
        B_gradient_geometry_total=4 * R * R * M * M,
        B_centroid_static=d * d / (a * a * 4 * mlo * mlo),
        B_centroid_geometry=N * R * R,
        ratio_to_cell_geometry=(1 + u / (kc * R)) ** 2,
        ratio_to_total_geometry=(d * d / (a * a)) / (4 * R * R * M * M),
        eta_m_floor=phi0 * (4 * math.pi * R * rho + math.pi * rho * rho),
        eta_over_m_minus=(phi0 * (4 * math.pi * R * rho + math.pi * rho * rho)) / mlo,
        gamma_dt=gamma_agent * dt,
        safety_neighbor_radius=2 * u / gamma_agent + math.sqrt((2 * u / gamma_agent) ** 2 + ds ** 2),
        finite_T_static_base=lambda H0, T: H0 / ((a / 2) * T) + (d * d) / (a * a),
    )


def env_info():
    import numpy
    import scipy
    import matplotlib
    info = dict(
        python=sys.version,
        platform=platform.platform(),
        machine=platform.machine(),
        processor=platform.processor(),
        numpy=numpy.__version__,
        scipy=scipy.__version__,
        matplotlib=matplotlib.__version__,
    )
    try:
        import yaml as _yaml
        info["pyyaml"] = _yaml.__version__
    except Exception as e:
        info["pyyaml"] = str(e)
    for name in ["pymunk", "shapely", "cvxpy", "osqp"]:
        try:
            mod = __import__(name)
            info[name] = getattr(mod, "__version__", getattr(mod, "version", "imported"))
        except Exception as e:
            info[name] = f"not_imported: {e}"
    try:
        info["numpy_show_config"] = numpy.show_config(mode="dicts")
    except TypeError:
        info["numpy_show_config"] = "unavailable"
    info["env_threads"] = {k: os.environ.get(k) for k in
                           ["OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
                            "NUMEXPR_NUM_THREADS"]}
    cfg = REPO / "configs/sim/d/l_shape_closed_loop.yaml"
    blob = subprocess.check_output(["git", "-C", str(REPO), "show",
                                    "HEAD:configs/sim/d/l_shape_closed_loop.yaml"])
    info["config_worktree_sha256"] = hashlib.sha256(cfg.read_bytes()).hexdigest()
    info["config_blob_sha256"] = hashlib.sha256(blob).hexdigest()
    info["config_worktree_crlf"] = cfg.read_bytes().count(b"\r\n")
    info["config_blob_crlf"] = blob.count(b"\r\n")
    info["git_commit"] = subprocess.check_output(["git", "-C", str(REPO), "rev-parse", "HEAD"],
                                                 text=True).strip()
    info["git_dirty"] = bool(subprocess.check_output(
        ["git", "-C", str(REPO), "status", "--porcelain"], text=True).strip())
    info["current_workspace_commit"] = subprocess.check_output(
        ["git", "-C", r"e:\boundary-aware-cooperative-transport", "rev-parse", "HEAD"],
        text=True).strip()
    try:
        info["pip_freeze"] = subprocess.check_output(
            [sys.executable, "-m", "pip", "freeze"], text=True).strip().splitlines()
    except Exception as e:
        info["pip_freeze_error"] = str(e)
    return info


def main():
    summary = {}
    summary["json_files"] = {}
    for name, tol in [
        ("constants.json", 0.0),
        ("counterexamples.json", 0.0),
        ("observer_validation.json", 0.0),
        ("provenance.json", 0.0),
        ("run_audit_summary.json", 0.0),
    ]:
        rows = compare_json(name, numeric_tol=tol)
        diffs = [r for r in rows if r["status"] not in ("match", "within_tol")]
        summary["json_files"][name] = dict(
            n_keys=len(rows), n_diff=len(diffs), diffs=diffs,
            exact_matches=sum(1 for r in rows if r["status"] == "match"),
        )
    summary["seeds"] = {}
    for seed in [2, 5, 8]:
        a = json.loads((REF / f"seed_{seed}" / "audit_summary.json").read_text(encoding="utf-8"))
        b = json.loads((NEW / f"seed_{seed}" / "audit_summary.json").read_text(encoding="utf-8"))
        rows = []
        fa, fb = dict(flatten(a)), dict(flatten(b))
        for k in sorted(set(fa) | set(fb)):
            if k not in fa or k not in fb:
                continue
            va, vb = fa[k], fb[k]
            if isinstance(va, (int, float)) and isinstance(vb, (int, float)) and not isinstance(va, bool):
                rows.append(dict(key=k, ref=va, new=vb, abs_err=abs(float(va) - float(vb)),
                                 rel_err=rel_err(va, vb), match=va == vb))
            else:
                rows.append(dict(key=k, ref=va, new=vb, match=va == vb))
        summary["seeds"][str(seed)] = dict(
            audit_summary_diffs=[r for r in rows if not r.get("match")],
            residuals=csv_compare(seed),
            trajectory=trajectory_compare(seed),
            safety=safety_compare(seed),
            object_speed_ref=object_speed(seed, REF),
            object_speed_new=object_speed(seed, NEW),
        )
        sim_a = json.loads((REF / f"seed_{seed}" / "simulation_summary.json").read_text(encoding="utf-8"))
        sim_b = json.loads((NEW / f"seed_{seed}" / "simulation_summary.json").read_text(encoding="utf-8"))
        g500_keys = ["g500"] if "g500" in sim_a else []
        summary["seeds"][str(seed)]["g500_ref"] = sim_a.get("g500") or sim_a.get("contracts")
        summary["seeds"][str(seed)]["g500_new"] = sim_b.get("g500") or sim_b.get("contracts")
        summary["seeds"][str(seed)]["final_phase_ref"] = sim_a.get("phases")
        summary["seeds"][str(seed)]["final_phase_new"] = sim_b.get("phases")
        # common top-level numeric fields
        common_num = {}
        for k, va in sim_a.items():
            if k in sim_b and isinstance(va, (int, float)) and not isinstance(va, bool):
                vb = sim_b[k]
                if isinstance(vb, (int, float)):
                    common_num[k] = dict(ref=va, new=vb, abs_err=abs(float(va) - float(vb)))
        summary["seeds"][str(seed)]["simulation_numeric"] = common_num

    c_script = json.loads((NEW / "constants.json").read_text(encoding="utf-8"))
    c_indep = independent_constants()
    indep_cmp = {}
    for k, v in c_indep.items():
        if callable(v):
            continue
        if k in c_script:
            indep_cmp[k] = dict(script=c_script[k], independent=v,
                                abs_err=abs(float(c_script[k]) - float(v)))
    summary["independent_constants"] = indep_cmp
    # finite-time static expressions using reproduced H(0)
    finite = {}
    for seed, H0 in zip([2, 5, 8], [None, None, None]):
        data = np.genfromtxt(NEW / f"seed_{seed}" / "residuals.csv", delimiter=",", names=True)
        H0 = float(data["H"][0])
        T = float(data["time"][-1])
        alpha = c_script["alpha"]
        B = c_script["B_gradient_static"]
        finite[str(seed)] = dict(H0=H0, T=T, finite_T_static=H0 / (alpha * T) + B)
    summary["finite_T_static_from_reproduced_H0"] = finite
    summary["environment"] = env_info()
    dump(OUT / "comparison.json", summary)

    # compact human table
    lines = ["seed,metric,ref,new,abs_err,rel_err"]
    for seed in [2, 5, 8]:
        s = summary["seeds"][str(seed)]
        for row in s["audit_summary_diffs"]:
            if "abs_err" in row:
                lines.append(f"{seed},{row['key']},{row['ref']},{row['new']},{row['abs_err']},{row['rel_err']}")
            else:
                lines.append(f"{seed},{row['key']},{row['ref']},{row['new']},,")
    (OUT / "comparison_summary.csv").write_text("\n".join(lines), encoding="utf-8")
    print("wrote comparison.json")
    for seed in [2, 5, 8]:
        t = summary["seeds"][str(seed)]["trajectory"]["positions"]
        r = summary["seeds"][str(seed)]["residuals"]
        print(f"seed {seed}: first pos diff frame={t['first_nonzero_frame']} "
              f"norm={t['first_nonzero_norm']:.3e} last={t['last_norm']:.3e} "
              f"first residual={r['first_any_diff']}")


if __name__ == "__main__":
    main()
