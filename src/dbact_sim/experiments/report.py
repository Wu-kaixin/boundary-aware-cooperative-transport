"""Export concise paper figures from recorded cases, without resimulation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from dbact_sim.experiments.evidence import _bool_count, episode_row, pair_rows, seed_cluster_bootstrap

METHODS = ("boundary_cvt", "nearest_observed_target", "local_spacing")
COLORS = ("#24649b", "#db872b", "#418568")


def save(fig, folder: Path, name: str):
    fig.savefig(folder / f"{name}.png", dpi=200, bbox_inches="tight")
    fig.savefig(folder / f"{name}.pdf", bbox_inches="tight")
    plt.close(fig)


def render(run: Path, out: Path) -> None:
    payload = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    results = payload["results"]
    out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})

    # All nine seed/shape cases in each static arm, including unsafe controls.
    arms = ("local_samples", "local_segments", "oracle")
    labels = ("Original local", "Segment + gap guard", "Oracle reference")
    fig, axes = plt.subplots(1, 2, figsize=(9.5, 3.8))
    for index, (arm, color) in enumerate(zip(arms, COLORS)):
        rows = [r for r in results if r["split"] == "static" and r["arm"] == arm]
        x = index + np.linspace(-0.12, 0.12, len(rows))
        axes[0].scatter(x, [1000 * r["clearance_path_min"] for r in rows], color=color, s=26)
        axes[1].scatter(x, [r["H_final"] / r["H_initial"] for r in rows], color=color, s=26)
    axes[0].axhline(65, color="#b53d42", linestyle="--", linewidth=1, label="Declared 65 mm threshold")
    axes[0].legend(fontsize=8)
    axes[0].set_ylabel("Minimum whole-hold clearance (mm)")
    axes[1].set_ylabel("Final / initial deployment cost")
    for ax in axes:
        ax.set_xticks(range(3), [f"{label}\nn=9" for label in labels], fontsize=8)
    fig.suptitle("Static deployment: safety and quality reported together")
    fig.tight_layout()
    save(fig, out, "static_deployment")

    # Three populations remain separate. Reused-seed groups have no binomial CI.
    cohorts = ("development", "generalization", "robustness")
    fig, ax = plt.subplots(figsize=(9, 4))
    x = np.arange(3)
    for index, (method, color) in enumerate(zip(METHODS, COLORS)):
        values, lows, highs, counts = [], [], [], []
        for cohort in cohorts:
            rows = [r for r in results if r["split"] == cohort and r["variant"] == method]
            rate = _bool_count(rows, "task_completed")
            value = rate["rate"] or 0.0
            values.append(value)
            lows.append(value - rate["low"] if rate["low"] is not None else np.nan)
            highs.append(rate["high"] - value if rate["high"] is not None else np.nan)
            counts.append(f"{rate['k']}/{rate['n']}")
        positions = x + (index - 1) * 0.24
        ax.bar(positions, values, 0.24, color=color, label=method, yerr=[lows, highs], capsize=3)
        for position, value, count in zip(positions, values, counts):
            ax.text(position, value + 0.025, count, ha="center", fontsize=8)
    ax.set_xticks(x, cohorts)
    ax.set_ylim(0, 1)
    ax.set_ylabel("Task completion rate")
    ax.legend(fontsize=8, loc="upper right")
    ax.set_title("All failures retained; intervals only for distinct-seed groups")
    fig.tight_layout()
    save(fig, out, "task_completion")

    episodes = [episode_row(r, r["split"]) for r in results if r["split"] == "generalization"]
    fig, ax = plt.subplots(figsize=(7.6, 4))
    pair_labels = []
    for index, method in enumerate(METHODS[1:]):
        pairs = pair_rows(episodes, METHODS[0], method)
        observed = [p for p in pairs if p["ready_difference_seconds"] is not None]
        values = [p["ready_difference_seconds"] for p in observed]
        stats = seed_cluster_bootstrap(pairs)
        ax.scatter(index + np.linspace(-0.12, 0.12, len(values)), values, color=COLORS[index + 1], s=26)
        if stats["mean"] is not None:
            error = [[stats["mean"] - stats["low"]], [stats["high"] - stats["mean"]]] if stats["low"] is not None else None
            ax.errorbar(index, stats["mean"], yerr=error, color="black", marker="D", capsize=5)
        pair_labels.append(f"{'Nearest observed target' if index == 0 else 'Local spacing'}\n{len(observed)}/{len(pairs)} complete pairs")
    ax.axhline(0, color="gray", linestyle="--", linewidth=1)
    ax.set_xticks([0, 1], pair_labels)
    ax.set_ylabel("CONTACT_READY time: CVT minus comparator (s)")
    ax.set_title("Generalization; censored pairs omitted from differences, counted above")
    fig.tight_layout()
    save(fig, out, "paired_preparation")

    rows = [r for r in results if r["split"] == "ablation" and r["variant"] != METHODS[0]]
    reference = {r["shape"]: r for r in results if r["split"] == "ablation" and r["variant"] == METHODS[0]}
    labels, readiness, endpoint = [], [], []
    for row in rows:
        base = reference[row["shape"]]
        labels.append(f"{row['shape']}: {row['variant']}")
        readiness.append(None if row["contact_ready_seconds"] is None or base["contact_ready_seconds"] is None
                         else row["contact_ready_seconds"] - base["contact_ready_seconds"])
        endpoint.append(100 * (row["euclidean_endpoint_error"] - base["euclidean_endpoint_error"]))
    fig, axes = plt.subplots(1, 2, figsize=(11, 6), sharey=True)
    y = np.arange(len(rows))
    for ax, values, label, color in zip(axes, (readiness, endpoint),
                                      ("Preparation difference (s)", "Endpoint-error difference (cm)"), COLORS):
        valid = [(i, v) for i, v in enumerate(values) if v is not None]
        ax.barh([i for i, _ in valid], [v for _, v in valid], color=color)
        ax.axvline(0, color="gray", linewidth=1)
        ax.set_xlabel(label)
    axes[0].set_yticks(y, labels, fontsize=7)
    fig.suptitle("Target-generation ablations minus full CVT; one seed per shape")
    fig.tight_layout()
    save(fig, out, "target_ablations")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    render(args.run, args.out)


if __name__ == "__main__":
    main()
