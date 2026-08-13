#!/usr/bin/env python
"""T6 - the publication artefact pipeline: figures, and a manifest that indexes them.

    python scripts/generate_publication_artifacts.py --out artifacts/publication

Ported from CODEX's ``generate_publication_artifacts.py``, with its two load-bearing pieces
kept: the Wilson interval on every reported proportion, and **PNG plus PDF for every
figure** -- raster for a draft, vector for a submission, produced from the same call so the
two cannot drift.

What this reads, and what it refuses to do
------------------------------------------
Every figure is drawn from a file already committed under ``docs/results/``. This script
runs no episodes. That separation is the same one ``render_closed_loop.py`` enforces for the
animation, and for the same reason: a figure that re-runs the physics charges plotting to
the simulation clock and makes "the numbers in the paper" a different set from "the numbers
in the repository".

If a source file is missing the figure is **skipped and recorded as skipped** in the
manifest, with the path it wanted. It is not drawn from a default, and it is not silently
omitted -- a missing panel that leaves no trace is how a figure set comes to describe a
different experiment than the one that ran.

Closed-loop frames are **not** drawn here. They are rendered by v1's
``dbact_sim.replay`` through ``scripts/render_closed_loop.py``, which keeps the v1 phase
palette and the "draw the robot's own map, not the true outline" principle. This script
only records in the manifest which run directory holds them.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from dbact.provenance import git_sha  # noqa: E402

RESULTS = ROOT / "docs" / "results"

MATRIX_CSV = RESULTS / "v2_shape_matrix" / "episodes.csv"
CONTROL_CSV = RESULTS / "v2_control_explore_gain0" / "episodes.csv"
BASELINE_JSON = RESULTS / "v2_baseline_12seed" / "g500_sweep.json"
SE2_JSON = RESULTS / "se2" / "se2_ablation.json"
T3_CONE_JSON = RESULTS / "t3" / "lateral_authority.json"
T3_ARC_JSON = RESULTS / "t3" / "push_arc_ablation.json"
T5_JSON = RESULTS / "t5" / "robustness_ablation.json"
T6_JSON = RESULTS / "t6" / "distance_ablation.json"
T7_JSON = RESULTS / "t7" / "explore_gain_profile.json"

#: The v1 phase palette, so a figure produced here and a frame produced by
#: ``dbact_sim.visualization`` cannot disagree about which colour BRAKE is.
PHASE_ORDER = ["SEARCH", "DISCOVER", "ENCLOSE", "CONTACT_READY", "TRANSPORT", "BRAKE", "HOLD"]


def relative(path: Path) -> str:
    """Repo-relative where possible, absolute otherwise.

    The manifest is meant to be readable from a checkout, so paths inside the tree are
    recorded relative to it. An output directory outside the tree -- a scratch run, a
    tmpdir in a test -- is recorded absolute rather than crashing the generator after the
    figures have already been drawn.
    """
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def wilson(successes: int, trials: int, z: float = 1.959963984540054) -> list[float] | None:
    """Wilson score interval. Every proportion in the manifest carries one.

    A normal approximation on 0/15 gives [0, 0], which reads as proof of impossibility.
    The Wilson form does not, which is why it is the one used.
    """
    if trials <= 0:
        return None
    p = successes / trials
    denominator = 1.0 + z * z / trials
    centre = (p + z * z / (2.0 * trials)) / denominator
    half = z * math.sqrt(p * (1.0 - p) / trials + z * z / (4.0 * trials * trials)) / denominator
    return [max(0.0, centre - half), min(1.0, centre + half)]


class Artifacts:
    """Figure sink that writes PNG and PDF together and records what it wrote."""

    def __init__(self, out: Path) -> None:
        self.out = out
        self.out.mkdir(parents=True, exist_ok=True)
        self.figures: list[dict] = []
        self.skipped: list[dict] = []

    def save(self, fig, name: str, caption: str, sources: list[Path]) -> None:
        paths = []
        for suffix in ("png", "pdf"):
            path = self.out / f"{name}.{suffix}"
            fig.savefig(path, dpi=200, bbox_inches="tight")
            paths.append(relative(path))
        plt.close(fig)
        self.figures.append(
            {
                "name": name,
                "caption": caption,
                "files": paths,
                "sources": [relative(p) for p in sources],
            }
        )
        print(f"  wrote {name}.png / .pdf", flush=True)

    def skip(self, name: str, reason: str, wanted: list[Path]) -> None:
        self.skipped.append(
            {"name": name, "reason": reason, "wanted": [str(p) for p in wanted]}
        )
        print(f"  SKIP  {name}: {reason}", flush=True)


def read_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def truthy(row: dict, key: str) -> bool:
    return str(row.get(key, "")).strip().lower() in ("true", "1")


def numeric(rows: list[dict], key: str) -> np.ndarray:
    out = []
    for row in rows:
        value = row.get(key)
        if value is None or str(value).strip() == "":
            continue
        try:
            v = float(value)
        except ValueError:
            continue
        if np.isfinite(v):
            out.append(v)
    return np.asarray(out, dtype=float)


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None


# --------------------------------------------------------------------------- #
# figures
# --------------------------------------------------------------------------- #


def fig_success_by_shape(art: Artifacts, rows: list[dict]) -> None:
    shapes = sorted({r["shape"] for r in rows})
    counts = [(s, sum(1 for r in rows if r["shape"] == s and truthy(r, "success")),
               sum(1 for r in rows if r["shape"] == s)) for s in shapes]
    counts.sort(key=lambda c: c[1] / max(c[2], 1))

    fig, ax = plt.subplots(figsize=(8, 4.5))
    y = np.arange(len(counts))
    rates = [s / max(n, 1) for _, s, n in counts]
    intervals = [wilson(s, n) for _, s, n in counts]
    lo = [r - (iv[0] if iv else r) for r, iv in zip(rates, intervals)]
    hi = [(iv[1] if iv else r) - r for r, iv in zip(rates, intervals)]
    ax.barh(y, rates, color="#4878a8", xerr=[lo, hi], error_kw={"ecolor": "#333", "capsize": 3})
    ax.set_yticks(y)
    ax.set_yticklabels([f"{s}  ({k}/{n})" for s, k, n in counts])
    ax.set_xlabel("contract success rate, with 95% Wilson interval")
    ax.set_xlim(0, 1)
    ax.axvline(54 / 180, color="#c04040", ls="--", lw=1, label="pooled P(success) = 0.300")
    ax.legend(loc="lower right", fontsize=8)
    ax.set_title("Contract success by shape family (180 episodes)")
    art.save(fig, "01_success_by_shape",
             "Contract success rate per shape family, 5 seeds x 3 alpha each, with Wilson "
             "intervals. Two families score 0/15. The interval on 0/15 is not [0,0].",
             [MATRIX_CSV])


def fig_j_over_diameter_by_alpha(art: Artifacts, rows: list[dict]) -> None:
    alphas = sorted({float(r["alpha"]) for r in rows})
    data = [numeric([r for r in rows if float(r["alpha"]) == a], "J_over_diameter") for a in alphas]

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(10, 4.2))
    ax.boxplot(data, tick_labels=[f"{a:.1f}" for a in alphas], showmeans=True)
    ax.set_xlabel("alpha = L / diameter")
    ax.set_ylabel("J / diameter")
    ax.set_title("Normalised displacement rises with alpha")
    ax.axhline(0.0, color="#888", lw=0.8)

    cross = [numeric([r for r in rows if float(r["alpha"]) == a], "max_cross_track_over_diameter")
             for a in alphas]
    ax2.boxplot(cross, tick_labels=[f"{a:.1f}" for a in alphas], showmeans=True)
    ax2.set_xlabel("alpha = L / diameter")
    ax2.set_ylabel("max cross-track / diameter")
    ax2.set_title("...and so does normalised lateral error")
    art.save(fig, "02_j_over_diameter_by_alpha",
             "Displacement generalises with scale-relative task distance; lateral error "
             "degrades with it. Enclosure timeouts are identically zero across all three alpha.",
             [MATRIX_CSV])


def fig_phase_durations(art: Artifacts, rows: list[dict]) -> None:
    stages = [
        ("detect", "first_detection_frame", None),
        ("enclose", "contact_ready_frame", "first_detection_frame"),
        ("arm transport", "transport_frame", "contact_ready_frame"),
        ("brake", "brake_frame", "transport_frame"),
        ("hold", "hold_frame", "brake_frame"),
    ]
    series, labels = [], []
    for label, end, start in stages:
        values = []
        for row in rows:
            e = row.get(end)
            if e is None or str(e).strip() == "":
                continue
            s = 0.0 if start is None else row.get(start)
            if start is not None and (s is None or str(s).strip() == ""):
                continue
            values.append(float(e) - float(s or 0.0))
        if values:
            series.append(np.asarray(values))
            labels.append(f"{label}\n(n={len(values)})")

    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.boxplot(series, tick_labels=labels, showmeans=True)
    ax.set_ylabel("frames")
    ax.set_yscale("log")
    ax.set_title("Phase durations, censored: only episodes that reached each phase appear")
    art.save(fig, "03_phase_durations",
             "Per-phase frame counts. Each box contains only the episodes that reached that "
             "phase, so the sample shrinks left to right and these are censored durations, "
             "not a completion-time distribution.",
             [MATRIX_CSV])


def fig_direction_progress(art: Artifacts, rows: list[dict]) -> None:
    j = numeric(rows, "J_over_diameter")
    err = numeric(rows, "direction_error_deg")
    n = min(len(j), len(err))
    fig, ax = plt.subplots(figsize=(6.4, 4.6))
    success = np.asarray([truthy(r, "success") for r in rows])[:n]
    ax.scatter(err[:n][~success], j[:n][~success], s=18, c="#c07070", label="failed", alpha=0.75)
    ax.scatter(err[:n][success], j[:n][success], s=18, c="#3a7d44", label="passed", alpha=0.85)
    ax.axvline(20.0, color="#c04040", ls="--", lw=1, label="direction gate 20 deg")
    ax.set_xlabel("direction error (deg)")
    ax.set_ylabel("J / diameter")
    ax.set_title("Directional progress against direction error")
    ax.legend(fontsize=8)
    art.save(fig, "04_direction_progress",
             "Almost every episode is inside the explicit 20 degree direction gate; the "
             "binding gate is cross-track, which the identity makes a 5.9 degree requirement.",
             [MATRIX_CSV])


def fig_cross_track_identity(art: Artifacts, cone: dict) -> None:
    runs = cone["runs"]
    pred = np.asarray([r["identity_prediction"] for r in runs])
    obs = np.asarray([r["max_cross_track"] for r in runs])
    fig, ax = plt.subplots(figsize=(5.6, 5.2))
    lim = float(max(pred.max(), obs.max())) * 1.1
    ax.plot([0, lim], [0, lim], color="#888", ls="--", lw=1, label="y = x")
    ax.scatter(pred, obs, s=34, c="#4878a8")
    ax.axhline(0.15, color="#c04040", ls=":", lw=1, label="cross-track gate 0.15 m")
    ax.set_xlabel("J sin(direction error)  (m)")
    ax.set_ylabel("measured max cross-track  (m)")
    ax.set_title(f"The two gates are one gate (r = {cone['identity']['correlation']:.3f})")
    ax.legend(fontsize=8)
    ax.set_xlim(0, lim)
    ax.set_ylim(0, lim)
    art.save(fig, "05_cross_track_identity",
             "max cross-track = J sin(direction error), correlation 0.981 over 12 seeds. An "
             "absolute 0.15 m cross-track gate is therefore a direction-error gate whose "
             "threshold depends on how far the object travelled.",
             [T3_CONE_JSON])


def fig_reachable_cone(art: Artifacts, cone: dict, arc: dict | None) -> None:
    runs = cone["runs"]
    half = np.asarray([r["cone_half_width_mean_deg"] for r in runs])
    err = np.asarray([r["direction_error_deg"] for r in runs])
    ncols = 1 if arc is None else 2
    fig, axes = plt.subplots(1, ncols, figsize=(5.6 * ncols, 4.4), squeeze=False)
    ax = axes[0][0]
    ax.scatter(half, err, s=34, c="#4878a8")
    ax.set_xlabel("reachable cone half-width (deg)")
    ax.set_ylabel("direction error (deg)")
    ax.set_title(f"Observational: r = {np.corrcoef(half, err)[0, 1]:+.3f}")
    ax.axhline(cone["implied_direction_gate_deg_mean"], color="#c04040", ls="--", lw=1,
               label="what the gate demands")
    ax.legend(fontsize=8)

    if arc is not None:
        ax2 = axes[0][1]
        taus = arc["thresholds"]
        dirs = [a["distributions"]["direction_error_deg"]["mean"] for a in arc["arms"]]
        cross = [a["distributions"]["max_cross_track"]["mean"] for a in arc["arms"]]
        ax2.plot(taus, dirs, "o-", color="#4878a8", label="direction error (deg)")
        ax2b = ax2.twinx()
        ax2b.plot(taus, cross, "s--", color="#c07040", label="max cross-track (m)")
        ax2.set_xlabel("push-set membership threshold tau (narrower cone to the right)")
        ax2.set_ylabel("direction error (deg)")
        ax2b.set_ylabel("max cross-track (m)")
        ax2.set_title("Controlled: narrowing the cone makes both worse")
        lines = ax2.get_lines() + ax2b.get_lines()
        ax2.legend(lines, [l.get_label() for l in lines], fontsize=8, loc="upper left")
    art.save(fig, "06_reachable_cone",
             "Left: observationally, a wider reachable cone accompanies worse aim, which "
             "would refute authority saturation. Right: the controlled test narrows the cone "
             "directly and every measure degrades, so the correlation is a confound with the "
             "sampled goal direction and v1's explanation stands.",
             [p for p in (T3_CONE_JSON, T3_ARC_JSON) if p.exists()])


def fig_rotation(art: Artifacts, rows: list[dict]) -> None:
    rot = numeric(rows, "rotation_deg")
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    ax.hist(rot, bins=40, color="#4878a8", edgecolor="white")
    ax.axvline(15.0, color="#c04040", ls="--", lw=1, label="yaw gate +-15 deg")
    ax.axvline(-15.0, color="#c04040", ls="--", lw=1)
    ax.set_xlabel("cargo rotation over the episode (deg)")
    ax.set_ylabel("episodes")
    ax.set_title("Cargo rotation. Almost all of it is inside the yaw gate")
    ax.legend(fontsize=8)
    art.save(fig, "07_cargo_rotation",
             "Cargo net rotation. The distribution is tight around zero, which is why the "
             "SE(2) boundary-point velocity term had nothing to capture on this baseline.",
             [MATRIX_CSV])


def fig_force_and_contacts(art: Artifacts, timeseries: Path) -> None:
    rows = read_csv(timeseries)
    t = numeric(rows, "time")
    fx = numeric(rows, "net_force_x")
    fy = numeric(rows, "net_force_y")
    torque = numeric(rows, "net_torque")
    contacts = numeric(rows, "contacts")
    n = min(len(t), len(fx), len(fy), len(torque), len(contacts))
    force = np.hypot(fx[:n], fy[:n])

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(8, 5.6), sharex=True)
    ax.plot(t[:n], force, color="#4878a8", lw=1.0, label="|net force| (N)")
    ax.plot(t[:n], np.abs(torque[:n]), color="#c07040", lw=1.0, label="|net torque| (N m)")
    ax.set_ylabel("magnitude")
    ax.legend(fontsize=8)
    ax.set_title("Net contact wrench and contact count, representative run")
    ax2.plot(t[:n], contacts[:n], color="#3a7d44", lw=1.0)
    ax2.set_xlabel("time (s)")
    ax2.set_ylabel("contacts")
    art.save(fig, "08_net_wrench_and_contacts",
             "Net contact force magnitude, net torque magnitude and contact count over one "
             "episode. The torque trace is the quantity a transport claim has to keep small: "
             "a team that spins the object into its displacement is not transporting it.",
             [timeseries])


def fig_safety(art: Artifacts, rows: list[dict]) -> None:
    clearance = numeric(rows, "min_signed_clearance")
    penetration = numeric(rows, "max_penetration")
    separation = numeric(rows, "min_inter_agent_distance")
    d_min = numeric(rows, "d_min")

    fig, (ax, ax2, ax3) = plt.subplots(1, 3, figsize=(12, 3.8))
    ax.hist(clearance, bins=30, color="#4878a8", edgecolor="white")
    ax.axvline(0.0, color="#c04040", ls="--", lw=1)
    ax.set_xlabel("min signed clearance (m)")
    ax.set_title("Robots never inside the cargo")

    ax2.hist(penetration, bins=30, color="#4878a8", edgecolor="white")
    budget = float(np.median(numeric(rows, "penetration_budget"))) if rows else 0.0
    ax2.axvline(budget, color="#c04040", ls="--", lw=1, label=f"budget {budget:.3f} m")
    ax2.set_xlabel("max penetration (m)")
    ax2.set_title("Penetration against budget")
    ax2.legend(fontsize=8)

    n = min(len(separation), len(d_min))
    slack = separation[:n] - d_min[:n]
    ax3.hist(slack, bins=30, color="#4878a8", edgecolor="white")
    ax3.axvline(0.0, color="#c04040", ls="--", lw=1, label="d_min")
    ax3.set_xlabel("min inter-agent distance - d_min (m)")
    ax3.set_title(f"Separation slack ({int(np.sum(slack < -1e-6))} episodes below)")
    ax3.legend(fontsize=8)
    art.save(fig, "09_safety_distances",
             "Clearance, penetration and inter-agent separation over all 180 episodes. One "
             "episode breaches d_min by 76 mm and is reported as SOLVER_FAILURE, because "
             "classify returns on its first match.",
             [MATRIX_CSV])


def fig_perception_error(art: Artifacts, se2: dict) -> None:
    arm = se2["arms"]["off"]
    d = arm["distributions"]
    terms = [
        ("normal_error_deg", "normal error (deg)", 30.0),
        ("normal_projection_error_mps", "barrier velocity error (m/s)", 0.02),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2))
    for ax, (prefix, label, declared) in zip(axes, terms):
        stats = [("mean", "mean"), ("p50", "p50"), ("p95", "p95"), ("p99", "p99"), ("max", "max")]
        values, labels = [], []
        for key, name in stats:
            entry = d.get(f"{prefix}_{key}")
            if entry and "mean" in entry:
                values.append(entry["mean"])
                labels.append(name)
        ax.bar(labels, values, color="#4878a8")
        ax.axhline(declared, color="#c04040", ls="--", lw=1.2,
                   label=f"declared premise {declared}")
        ax.set_yscale("log")
        ax.set_ylabel(label)
        ax.legend(fontsize=8)
    breach = d.get("velocity_breach_fraction", {}).get("mean")
    axes[1].set_title(f"{breach:.1%} of cells exceed the premise" if breach else "")
    axes[0].set_title("Perception error against the declared premises")
    art.save(fig, "10_perception_error",
             "The six-term audit against the two declared error premises. The barrier-visible "
             "velocity error exceeds its declared bound -- which is also rho -- for the "
             "majority of measured cells, so the ISSf premise does not hold on these runs.",
             [SE2_JSON])


def fig_failure_composition(art: Artifacts, rows: list[dict], control: list[dict] | None) -> None:
    def composition(source: list[dict]) -> Counter:
        return Counter(r.get("failure_class", "?") for r in source)

    matrix = composition(rows)
    order = [k for k, _ in matrix.most_common()]
    fig, ax = plt.subplots(figsize=(8.6, 4.4))
    x = np.arange(len(order))
    width = 0.38 if control else 0.7
    ax.bar(x - (width / 2 if control else 0), [matrix[k] for k in order], width,
           color="#4878a8", label=f"matrix (n={len(rows)})")
    if control:
        ctrl = composition(control)
        ax.bar(x + width / 2, [ctrl.get(k, 0) for k in order], width,
               color="#c07040", label=f"explore_gain=0 control (n={len(control)})")
    ax.set_xticks(x)
    ax.set_xticklabels(order, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("episodes")
    ax.set_title("Failure composition, most structural cause first")
    ax.legend(fontsize=8)
    art.save(fig, "11_failure_composition",
             "One label per episode, assigned by the first match in classify's ordering. An "
             "episode that failed two ways appears once, so this is 'the most structural "
             "cause found' rather than a partition of failure modes.",
             [p for p in (MATRIX_CSV, CONTROL_CSV) if p.exists()])


def fig_conditional_domain(art: Artifacts, rows: list[dict]) -> None:
    eligible = [r for r in rows if truthy(r, "runtime_domain_eligible")]
    rejected = [r for r in rows if not truthy(r, "runtime_domain_eligible")]
    groups = [
        ("all", sum(1 for r in rows if truthy(r, "success")), len(rows)),
        ("eligible", sum(1 for r in eligible if truthy(r, "success")), len(eligible)),
        ("rejected", sum(1 for r in rejected if truthy(r, "success")), len(rejected)),
    ]
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    x = np.arange(len(groups))
    rates = [k / max(n, 1) for _, k, n in groups]
    intervals = [wilson(k, n) for _, k, n in groups]
    lo = [r - (iv[0] if iv else r) for r, iv in zip(rates, intervals)]
    hi = [(iv[1] if iv else r) - r for r, iv in zip(rates, intervals)]
    ax.bar(x, rates, color=["#888", "#4878a8", "#c07040"],
           yerr=[lo, hi], error_kw={"ecolor": "#333", "capsize": 4})
    ax.set_xticks(x)
    ax.set_xticklabels([f"{name}\n{k}/{n}" for name, k, n in groups])
    ax.set_ylabel("contract success rate")
    ax.set_ylim(0, 1)
    ax.set_title("Conditioning on eligibility does not raise the success rate")
    art.save(fig, "12_conditional_domain",
             "P(success | eligible) = 0.275 sits below the unconditional 0.300, and 13 of the "
             "31 rejected episodes succeeded. Fisher's exact gives p = 0.13, so the predicate "
             "is not shown to be informative rather than shown to be anti-informative.",
             [MATRIX_CSV])


def fig_runtime(art: Artifacts, rows: list[dict], profile: dict | None) -> None:
    fps = numeric(rows, "fps")
    ncols = 1 if profile is None else 2
    fig, axes = plt.subplots(1, ncols, figsize=(5.6 * ncols, 4.0), squeeze=False)
    ax = axes[0][0]
    ax.hist(fps, bins=30, color="#4878a8", edgecolor="white")
    ax.set_xlabel("frames per second")
    ax.set_ylabel("episodes")
    ax.set_title("Matrix runtime, 16 robots")
    if profile is not None:
        ax2 = axes[0][1]
        arms = [("explore_gain 0", profile["explore_gain_0"]), ("explore_gain 6", profile["explore_gain_6"])]
        ax2.bar([a for a, _ in arms], [s["fps_mean"] for _, s in arms],
                yerr=[s["fps_sd"] for _, s in arms], color=["#4878a8", "#c07040"],
                error_kw={"ecolor": "#333", "capsize": 4})
        ax2.set_ylabel("frames per second")
        ratio = profile["paired_cost_ratio"]
        ax2.set_title(f"Paired per-frame cost ratio {ratio['mean']:.3f} +- {ratio['sd']:.3f}")
    art.save(fig, "13_runtime",
             "Machine-dependent empirical runtime. Not a bound, not a complexity result, and "
             "not transferable: one machine, one Python, one BLAS.",
             [p for p in (MATRIX_CSV, T7_JSON) if p.exists()])


def fig_robustness(art: Artifacts, t5: dict) -> None:
    names = list(t5["arms"])
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(12, 4.4))
    x = np.arange(len(names))
    passes = [t5["arms"][n]["pass"] for n in names]
    totals = [t5["arms"][n]["total"] for n in names]
    rates = [p / max(t, 1) for p, t in zip(passes, totals)]
    intervals = [wilson(p, t) for p, t in zip(passes, totals)]
    lo = [r - (iv[0] if iv else r) for r, iv in zip(rates, intervals)]
    hi = [(iv[1] if iv else r) - r for r, iv in zip(rates, intervals)]
    colours = ["#c04040" if t5["arms"][n]["measured_out_of_domain"] else "#4878a8" for n in names]
    ax.bar(x, rates, color=colours, yerr=[lo, hi], error_kw={"ecolor": "#333", "capsize": 3})
    ax.set_xticks(x)
    ax.set_xticklabels([f"{n}\n{p}/{t}" for n, p, t in zip(names, passes, totals)],
                       rotation=30, ha="right", fontsize=7)
    ax.set_ylabel("contract success rate")
    ax.set_ylim(0, 1)
    ax.set_title("Robustness arms (red = measured out-of-domain)")

    pseudo = [(t5["arms"][n]["distributions"].get("pseudo_frontier_per_frame") or {}).get("mean")
              for n in names]
    have = [(n, v) for n, v in zip(names, pseudo) if v is not None]
    if have:
        ax2.bar(np.arange(len(have)), [v for _, v in have], color="#c07040")
        ax2.set_xticks(np.arange(len(have)))
        ax2.set_xticklabels([n for n, _ in have], rotation=30, ha="right", fontsize=7)
        ax2.set_ylabel("provably spurious frontier targets per frame")
        ax2.set_title("Pseudo-frontier rate after the map closes")
    art.save(fig, "14_robustness",
             "Six degradation arms over 12 seeds. Bars are red where the six-term audit "
             "measured a breach of the declared error premise -- including the nominal arm, "
             "because the baseline config already runs at the 10 mm noise the plan designates "
             "out-of-domain. Right: frontier targets emitted after the pooled map already "
             "satisfies epsilon, every one of which is provably spurious.",
             [T5_JSON])


def fig_distance_ablation(art: Artifacts, t6: dict) -> None:
    per = t6["per_alpha"]
    alphas = sorted(float(k) for k in per)
    keys = [f"{a:.1f}" for a in alphas]
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))
    ax.errorbar(alphas, [per[k]["J_over_diameter"]["mean"] for k in keys],
                yerr=[per[k]["J_over_diameter"]["sd"] for k in keys],
                fmt="o-", color="#4878a8", capsize=3, label="J / diameter")
    ax.plot(alphas, alphas, ls="--", color="#888", lw=1, label="perfect tracking")
    ax.set_xlabel("alpha = L / diameter")
    ax.set_ylabel("J / diameter")
    ax.set_title("Displacement against demand, 12 seeds per alpha")
    ax.legend(fontsize=8)

    rates = [per[k]["success_rate"] for k in keys]
    intervals = [per[k]["success_wilson95"] for k in keys]
    lo = [r - iv[0] for r, iv in zip(rates, intervals)]
    hi = [iv[1] - r for r, iv in zip(rates, intervals)]
    ax2.errorbar(alphas, rates, yerr=[lo, hi], fmt="s-", color="#c07040", capsize=3)
    ax2.set_xlabel("alpha = L / diameter")
    ax2.set_ylabel("contract success rate")
    ax2.set_ylim(0, 1)
    ax2.set_title("Success against demand, with Wilson intervals")
    art.save(fig, "15_distance_ablation",
             "Task distance swept scale-relative on one shape at 12 seeds per level, "
             "extending the matrix's three alpha levels to five. CODEX's fixed metric "
             "distances are deliberately not inherited.",
             [T6_JSON])


# --------------------------------------------------------------------------- #


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="artifacts/publication")
    parser.add_argument("--timeseries", default="",
                        help="safety_timeseries.csv from a representative run, for the "
                             "net-wrench and contact-count figure.")
    parser.add_argument("--replay-run", default="",
                        help="Run directory whose replay.npz holds the closed-loop frames. "
                             "Recorded in the manifest; rendered by render_closed_loop.py.")
    args = parser.parse_args()

    art = Artifacts(ROOT / args.out if not Path(args.out).is_absolute() else Path(args.out))
    print(f"writing to {art.out}")

    matrix = read_csv(MATRIX_CSV) if MATRIX_CSV.exists() else None
    control = read_csv(CONTROL_CSV) if CONTROL_CSV.exists() else None
    cone = load_json(T3_CONE_JSON)
    arc = load_json(T3_ARC_JSON)
    se2 = load_json(SE2_JSON)
    t5 = load_json(T5_JSON)
    t6 = load_json(T6_JSON)
    profile = load_json(T7_JSON)

    if matrix:
        fig_success_by_shape(art, matrix)
        fig_j_over_diameter_by_alpha(art, matrix)
        fig_phase_durations(art, matrix)
        fig_direction_progress(art, matrix)
        fig_rotation(art, matrix)
        fig_safety(art, matrix)
        fig_failure_composition(art, matrix, control)
        fig_conditional_domain(art, matrix)
        fig_runtime(art, matrix, profile)
    else:
        for name in ("01_success_by_shape", "02_j_over_diameter_by_alpha", "03_phase_durations",
                     "04_direction_progress", "07_cargo_rotation", "09_safety_distances",
                     "11_failure_composition", "12_conditional_domain", "13_runtime"):
            art.skip(name, "matrix episodes.csv missing", [MATRIX_CSV])

    if cone:
        fig_cross_track_identity(art, cone)
        fig_reachable_cone(art, cone, arc)
    else:
        art.skip("05_cross_track_identity", "lateral_authority.json missing", [T3_CONE_JSON])
        art.skip("06_reachable_cone", "lateral_authority.json missing", [T3_CONE_JSON])

    if se2:
        fig_perception_error(art, se2)
    else:
        art.skip("10_perception_error", "se2_ablation.json missing", [SE2_JSON])

    timeseries = Path(args.timeseries) if args.timeseries else None
    if timeseries and timeseries.exists():
        fig_force_and_contacts(art, timeseries)
    else:
        art.skip("08_net_wrench_and_contacts",
                 "no safety_timeseries.csv given; pass --timeseries",
                 [timeseries or Path("<representative run>/safety_timeseries.csv")])

    if t5:
        fig_robustness(art, t5)
    else:
        art.skip("14_robustness", "robustness_ablation.json missing", [T5_JSON])

    if t6:
        fig_distance_ablation(art, t6)
    else:
        art.skip("15_distance_ablation", "distance_ablation.json missing", [T6_JSON])

    manifest = {
        "generator": "scripts/generate_publication_artifacts.py",
        "git_sha": git_sha(ROOT),
        "figure_formats": ["png", "pdf"],
        "figures": art.figures,
        "skipped": art.skipped,
        "closed_loop_frames": {
            "run": args.replay_run or None,
            "renderer": "scripts/render_closed_loop.py via dbact_sim.replay",
            "note": (
                "Rendered by v1's replay pipeline, not by this script: it keeps the v1 phase "
                "palette and draws one robot's own map beside the true outline rather than "
                "reconstructing a surface from ground truth."
            ),
        },
        "sources": {
            name: {"path": relative(path), "present": path.exists()}
            for name, path in (
                ("matrix", MATRIX_CSV), ("explore_gain_control", CONTROL_CSV),
                ("baseline_12seed", BASELINE_JSON), ("se2", SE2_JSON),
                ("lateral_authority", T3_CONE_JSON), ("push_arc", T3_ARC_JSON),
                ("robustness", T5_JSON), ("distance", T6_JSON), ("explore_gain_profile", T7_JSON),
            )
        },
        "non_claims": [
            "No figure here is a bound. The finite-time bound reports available=false; see "
            "scripts/derive_finite_time_bound.py.",
            "Completion times are right-censored (42 of 180 matrix episodes hit the watchdog), "
            "so no completion-time distribution in these figures may be read as a bound.",
            "Runtime figures are machine-dependent stopwatch readings.",
            "Eligibility is a conservative filter, not an operational envelope: see figure 12.",
        ],
    }
    (art.out / "publication_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"\n{len(art.figures)} figures, {len(art.skipped)} skipped")
    print(f"wrote {art.out / 'publication_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
