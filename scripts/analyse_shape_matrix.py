#!/usr/bin/env python
"""v2 - read the decisive matrix and answer the five questions it was run to answer.

    python scripts/analyse_shape_matrix.py runs/v2_shape_matrix

The matrix itself is deliberately dumb: it runs every case and writes everything
down. This script is where the interpretation lives, and it is kept separate so
that re-reading the data cannot change it.

Five questions, each answered against a measured quantity rather than an
impression of the table:

1. does performance degrade systematically with object diameter;
2. does concavity degrade it;
3. when alpha rises, is the binding constraint transport authority, cross-track
   growth, or enclosure;
4. is there a shape family that fails systematically;
5. what strength of "arbitrary shape" claim the numbers actually support.

On the headline ratio
--------------------
J / diameter is reported because it is the quantity that makes v1 and CODEX
comparable at all. It is *not* a free measure of goodness: the task distance is
L = alpha * diameter, so a team that simply reaches its target scores
J / diameter ~ alpha, and at alpha = 0.1 no controller can score 0.5 no matter
how well it works. The decisive threshold therefore has to be read per alpha, and
the quantity that carries the generalisation claim across alphas is J / L -- did
the team reach the distance it was asked to cover -- together with the success
rate. Both are reported. Reading J / diameter alone, pooled over alpha, would be
a way of making the answer depend on the mix of alphas in the matrix.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np

try:
    from scipy import stats as sps
except Exception:  # pragma: no cover - scipy is a hard dependency of the repo
    sps = None


def wilson(successes: int, trials: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if trials <= 0:
        return (0.0, 1.0)
    p = successes / trials
    d = 1.0 + z * z / trials
    c = (p + z * z / (2.0 * trials)) / d
    h = z * math.sqrt(p * (1.0 - p) / trials + z * z / (4.0 * trials * trials)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def column(rows: list[dict], key: str) -> np.ndarray:
    return np.asarray(
        [float(r[key]) for r in rows if r.get(key) is not None and np.isfinite(float(r[key]))],
        dtype=float,
    )


def paired(rows: list[dict], x: str, y: str) -> tuple[np.ndarray, np.ndarray]:
    xs, ys = [], []
    for r in rows:
        a, b = r.get(x), r.get(y)
        if a is None or b is None:
            continue
        a, b = float(a), float(b)
        if np.isfinite(a) and np.isfinite(b):
            xs.append(a)
            ys.append(b)
    return np.asarray(xs), np.asarray(ys)


def correlate(rows: list[dict], x: str, y: str) -> dict:
    """Pearson and Spearman with p-values. Spearman is the one to trust here:
    the relationships need not be linear and n is small."""
    xs, ys = paired(rows, x, y)
    if len(xs) < 4 or np.std(xs) < 1e-12 or np.std(ys) < 1e-12:
        return {"n": int(len(xs)), "insufficient": True}
    out = {"n": int(len(xs))}
    if sps is not None:
        pr = sps.pearsonr(xs, ys)
        sr = sps.spearmanr(xs, ys)
        slope = sps.linregress(xs, ys)
        out.update(
            pearson_r=float(pr[0]),
            pearson_p=float(pr[1]),
            spearman_rho=float(sr.correlation),
            spearman_p=float(sr.pvalue),
            slope=float(slope.slope),
            intercept=float(slope.intercept),
        )
    else:
        out.update(pearson_r=float(np.corrcoef(xs, ys)[0, 1]))
    return out


def stat(values: np.ndarray) -> str:
    if len(values) == 0:
        return "     n/a"
    return f"{values.mean():6.3f}+-{values.std(ddof=1) if len(values) > 1 else 0.0:5.3f}"


def rate_line(rows: list[dict]) -> str:
    n = len(rows)
    k = sum(1 for r in rows if r.get("success"))
    lo, hi = wilson(k, n)
    return f"{k:2d}/{n:2d} [{lo:.2f},{hi:.2f}]"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("root", nargs="?", default="runs/v2_shape_matrix")
    parser.add_argument("--out", default=None, help="Write the report here as well as to stdout.")
    args = parser.parse_args()

    root = Path(args.root)
    payload = json.loads((root / "monte_carlo.json").read_text(encoding="utf-8"))
    records = payload["records"]
    manifest = payload["manifest"]
    alphas = sorted({float(r["alpha"]) for r in records})
    shapes = sorted({r["shape"] for r in records})

    lines: list[str] = []

    def emit(text: str = "") -> None:
        lines.append(text)
        print(text)

    emit("=" * 100)
    emit("v2 DECISIVE MATRIX -- v1 controller x 12 shape families x scale-relative distance")
    emit("=" * 100)
    emit(f"git sha        : {manifest['git_sha']}  (dirty={manifest['git_dirty']})")
    emit(f"config         : {manifest['config_path']}")
    emit(f"config sha256  : {manifest['config_sha256']}")
    emit(f"seeds          : {manifest['seeds']}")
    emit(f"alphas         : {manifest['alphas']}")
    emit(f"shape scale    : x{manifest['shape_scale_factor']:.4f} (anchored on l_shape = v1's cargo)")
    emit(f"distance rule  : {manifest['task_distance_rule']}")
    emit(f"screening      : {manifest['screening']}")
    emit(f"episodes       : {len(records)}")
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("1. HEADLINE")
    emit("-" * 100)
    jd = column(records, "J_over_diameter")
    jl = column(records, "J_over_target")
    ok = sum(1 for r in records if r.get("success"))
    lo, hi = wilson(ok, len(records))
    elig = [r for r in records if r.get("runtime_domain_eligible")]
    elig_ok = sum(1 for r in elig if r.get("success"))
    elo, ehi = wilson(elig_ok, len(elig)) if elig else (0.0, 1.0)
    plo, phi = wilson(len(elig), len(records))
    emit(f"J/diameter  (all cases, pooled over alpha) : {jd.mean():.3f} +- {jd.std(ddof=1):.3f}   "
         f"median {np.median(jd):.3f}   range [{jd.min():.3f}, {jd.max():.3f}]")
    emit(f"J/L         (reached the asked distance)   : {jl.mean():.3f} +- {jl.std(ddof=1):.3f}   "
         f"median {np.median(jl):.3f}   range [{jl.min():.3f}, {jl.max():.3f}]")
    emit(f"P(eligible)                : {len(elig)}/{len(records)} = {len(elig)/len(records):.3f}  "
         f"Wilson95 [{plo:.3f}, {phi:.3f}]")
    emit(f"P(success | eligible)      : {elig_ok}/{len(elig)} = "
         f"{elig_ok/len(elig) if elig else float('nan'):.3f}  Wilson95 [{elo:.3f}, {ehi:.3f}]")
    emit(f"P(success)  unconditional  : {ok}/{len(records)} = {ok/len(records):.3f}  "
         f"Wilson95 [{lo:.3f}, {hi:.3f}]")
    emit(f"solver: fallbacks={sum(int(r.get('solver_fallbacks') or 0) for r in records)}  "
         f"infeasible={sum(int(r.get('solver_infeasible') or 0) for r in records)}  "
         f"barrier scalings={sum(int(r.get('barrier_scalings') or 0) for r in records)}  "
         f"rho relaxations={sum(int(r.get('margin_relaxations') or 0) for r in records)}")
    emit(f"termination: HOLD reached {sum(1 for r in records if r.get('hold_frame') is not None)}/{len(records)}  "
         f"watchdog {sum(1 for r in records if r.get('terminated_by') == 'watchdog')}")
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("2. PER ALPHA  (J/diameter is bounded by ~alpha by construction; read J/L for reach)")
    emit("-" * 100)
    emit(f"{'alpha':>6} {'n':>4} {'success':>16} {'J/d':>14} {'J/L':>14} {'eff':>14} "
         f"{'dir err deg':>14} {'xtrack/d':>14} {'peak cover':>14}")
    for a in alphas:
        rows = [r for r in records if float(r["alpha"]) == a]
        emit(f"{a:6.2f} {len(rows):4d} {rate_line(rows):>16} "
             f"{stat(column(rows,'J_over_diameter')):>14} {stat(column(rows,'J_over_target')):>14} "
             f"{stat(column(rows,'efficiency')):>14} {stat(column(rows,'direction_error_deg')):>14} "
             f"{stat(column(rows,'max_cross_track_over_diameter')):>14} "
             f"{stat(column(rows,'max_strict_coverage')):>14}")
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("3. PER SHAPE  (pooled over alpha and seed)")
    emit("-" * 100)
    emit(f"{'shape':>18} {'n':>4} {'diam':>7} {'concav':>7} {'success':>16} {'J/d':>14} {'J/L':>14} "
         f"{'eff':>14} {'peak cover':>14}")
    for s in shapes:
        rows = [r for r in records if r["shape"] == s]
        d = column(rows, "diameter_m")
        c = column(rows, "concavity_ratio")
        emit(f"{s:>18} {len(rows):4d} {d.mean() if len(d) else float('nan'):7.2f} "
             f"{c.mean() if len(c) else float('nan'):7.3f} {rate_line(rows):>16} "
             f"{stat(column(rows,'J_over_diameter')):>14} {stat(column(rows,'J_over_target')):>14} "
             f"{stat(column(rows,'efficiency')):>14} {stat(column(rows,'max_strict_coverage')):>14}")
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("4. PER SHAPE x ALPHA  (success out of 5 seeds; J/L mean)")
    emit("-" * 100)
    header = f"{'shape':>18}" + "".join(f"{'a=' + f'{a:.1f}':>18}" for a in alphas)
    emit(header)
    for s in shapes:
        cells = []
        for a in alphas:
            rows = [r for r in records if r["shape"] == s and float(r["alpha"]) == a]
            k = sum(1 for r in rows if r.get("success"))
            jlv = column(rows, "J_over_target")
            cells.append(f"{k}/{len(rows)} J/L={jlv.mean() if len(jlv) else float('nan'):5.2f}")
        emit(f"{s:>18}" + "".join(f"{c:>18}" for c in cells))
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("5. QUESTION 1 -- does performance degrade systematically with object DIAMETER?")
    emit("-" * 100)
    emit("Correlations are computed WITHIN each alpha. Pooling across alpha would mix the")
    emit("designed factor into the answer, because larger objects are given longer targets.")
    for a in alphas:
        rows = [r for r in records if float(r["alpha"]) == a]
        emit(f"  alpha={a:.2f}")
        for metric in ("J_over_target", "efficiency", "max_cross_track_over_diameter",
                       "max_strict_coverage", "completion_time_s"):
            c = correlate(rows, "diameter_m", metric)
            if c.get("insufficient"):
                emit(f"    diameter vs {metric:32s} : insufficient data (n={c['n']})")
            else:
                emit(f"    diameter vs {metric:32s} : rho={c['spearman_rho']:+.3f} p={c['spearman_p']:.4f}  "
                     f"(pearson r={c['pearson_r']:+.3f}, slope={c['slope']:+.4f}/m, n={c['n']})")
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("6. QUESTION 2 -- does CONCAVITY degrade performance?")
    emit("-" * 100)
    emit("concavity_ratio = 1 - area/area(convex hull); 0 for convex outlines.")
    for a in alphas:
        rows = [r for r in records if float(r["alpha"]) == a]
        emit(f"  alpha={a:.2f}")
        for metric in ("J_over_target", "efficiency", "max_strict_coverage", "completion_time_s"):
            c = correlate(rows, "concavity_ratio", metric)
            if c.get("insufficient"):
                emit(f"    concavity vs {metric:31s} : insufficient data (n={c['n']})")
            else:
                emit(f"    concavity vs {metric:31s} : rho={c['spearman_rho']:+.3f} p={c['spearman_p']:.4f}  "
                     f"(n={c['n']})")
    convex = [r for r in records if float(r.get("concavity_ratio") or 0.0) <= 0.01]
    concave = [r for r in records if float(r.get("concavity_ratio") or 0.0) > 0.01]
    emit(f"  convex  (ratio<=0.01): {rate_line(convex)}  J/L {stat(column(convex,'J_over_target'))}  "
         f"peak cover {stat(column(convex,'max_strict_coverage'))}")
    emit(f"  concave (ratio> 0.01): {rate_line(concave)}  J/L {stat(column(concave,'J_over_target'))}  "
         f"peak cover {stat(column(concave,'max_strict_coverage'))}")
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("7. QUESTION 3 -- when alpha rises, WHAT binds: authority, cross-track, or enclosure?")
    emit("-" * 100)
    emit(f"{'alpha':>6} {'reach J/L':>14} {'stalled':>9} {'never armed':>12} {'encl t/o':>9} "
         f"{'xtrack m':>14} {'xtrack/d':>14} {'peak cover':>14} {'barrier':>9} {'watchdog':>9}")
    for a in alphas:
        rows = [r for r in records if float(r["alpha"]) == a]
        comp = Counter(r.get("failure_class") for r in rows)
        emit(f"{a:6.2f} {stat(column(rows,'J_over_target')):>14} "
             f"{comp.get('TRANSPORT_STALL', 0):9d} {comp.get('TRANSPORT_NEVER_ARMED', 0):12d} "
             f"{comp.get('ENCLOSURE_TIMEOUT', 0):9d} "
             f"{stat(column(rows,'max_cross_track')):>14} "
             f"{stat(column(rows,'max_cross_track_over_diameter')):>14} "
             f"{stat(column(rows,'max_strict_coverage')):>14} "
             f"{int(sum(int(r.get('barrier_scalings') or 0) for r in rows)):9d} "
             f"{sum(1 for r in rows if r.get('terminated_by') == 'watchdog'):9d}")
    emit()
    emit("  gate attribution across alpha (which g500 reason fired, counted per case):")
    for a in alphas:
        rows = [r for r in records if float(r["alpha"]) == a]
        reasons = Counter()
        for r in rows:
            for reason in (r.get("failure_reasons") or []):
                key = reason.split(":")[0][:60]
                reasons[key] += 1
        emit(f"    alpha={a:.2f}: " + (", ".join(f"{k} x{v}" for k, v in reasons.most_common(6)) or "none"))
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("8. QUESTION 4 -- does any SHAPE FAMILY fail systematically?")
    emit("-" * 100)
    flagged = []
    for s in shapes:
        rows = [r for r in records if r["shape"] == s]
        k = sum(1 for r in rows if r.get("success"))
        lo_s, hi_s = wilson(k, len(rows))
        comp = Counter(r.get("failure_class") for r in rows if not r.get("success"))
        note = ", ".join(f"{name} x{n}" for name, n in comp.most_common(3)) or "-"
        emit(f"  {s:>18}: {k:2d}/{len(rows):2d} [{lo_s:.2f},{hi_s:.2f}]   {note}")
        # "Systematic" = the upper Wilson bound sits below the pooled rate, i.e.
        # this family is worse than the matrix in a way the sample size supports.
        if hi_s < ok / len(records):
            flagged.append((s, k, len(rows), hi_s))
    emit()
    if flagged:
        emit("  families whose Wilson UPPER bound is below the pooled success rate "
             f"({ok/len(records):.3f}) -- i.e. worse than the matrix, not just unlucky:")
        for s, k, n, hi_s in flagged:
            emit(f"    {s}: {k}/{n}, upper bound {hi_s:.3f}")
    else:
        emit("  no family's Wilson upper bound falls below the pooled rate: no family is")
        emit("  separated from the matrix at this sample size. That is a statement about")
        emit("  power as much as about the controller -- 15 episodes per family.")
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("9. REJECTIONS AND INFEASIBILITY")
    emit("-" * 100)
    pre = [r for r in records if not r.get("domain_eligible")]
    rt = [r for r in records if r.get("domain_eligible") and not r.get("runtime_domain_eligible")]
    emit(f"  rejected before the run (admissibility predicates) : {len(pre)}/{len(records)}")
    emit(f"  rejected at run time (map never reached epsilon)   : {len(rt)}/{len(records)}")
    reasons = Counter(x for r in records for x in (r.get("certificate_failures") or []))
    for name, n in reasons.most_common():
        emit(f"    {name:44s} x{n}")
    emit(f"  construction failures : {sum(1 for r in records if r.get('construction_error'))}")
    for r in records:
        if r.get("construction_error"):
            emit(f"    {r['case_id']}: {r['construction_error']}")
    emit(f"  map gap: mean {column(records,'runtime_map_gap_m').mean():.4f} m "
         f"against a required {records[0].get('runtime_map_gap_required_m')} m")
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("10. THE FIVE WORST CASES  (by J/diameter -- not a highlight reel)")
    emit("-" * 100)
    worst = sorted(records, key=lambda r: (float(r.get("J_over_diameter") or 0.0), r["case_id"]))[:5]
    for r in worst:
        emit(f"  {r['case_id']}")
        emit(f"     d={r.get('diameter_m'):.2f} m  L={r.get('target_distance_m'):.2f} m  "
             f"J={r.get('J'):.3f}  J/d={r.get('J_over_diameter'):.3f}  J/L={r.get('J_over_target'):.3f}")
        emit(f"     eff={r.get('efficiency'):.3f}  dir_err={r.get('direction_error_deg')}  "
             f"xtrack={r.get('max_cross_track'):.3f}  peak_cover={r.get('max_strict_coverage'):.3f}")
        emit(f"     phase={r.get('final_phase')}  frames={r.get('frames_run')}  "
             f"end={r.get('terminated_by')}  class={r.get('failure_class')}")
        for reason in (r.get("failure_reasons") or [])[:4]:
            emit(f"       - {reason[:110]}")
    emit()

    # ------------------------------------------------------------------ #
    emit("-" * 100)
    emit("11. FAILURE TAXONOMY (all cases)")
    emit("-" * 100)
    for name, n in Counter(r.get("failure_class") for r in records).most_common():
        emit(f"  {name:26s} {n:4d}  ({n / len(records):.3f})")
    emit()

    if args.out:
        Path(args.out).write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
