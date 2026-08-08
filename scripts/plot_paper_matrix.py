"""Create the standard paper comparison figure from ``all_runs.csv``."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default="runs/paper_matrix/all_runs.csv")
    parser.add_argument("--output", default="runs/paper_matrix/method_comparison.png")
    args = parser.parse_args()

    with Path(args.input).open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError("No experiment rows found")

    grouped: dict[str, list[dict]] = {}
    for row in rows:
        label = row.get("config") or row.get("method") or "unknown"
        grouped.setdefault(label, []).append(row)
    labels = list(grouped)

    def stats(key: str) -> tuple[np.ndarray, np.ndarray]:
        means, stds = [], []
        for label in labels:
            values = [float(row[key]) for row in grouped[label] if row.get(key) not in {None, "", "None"}]
            means.append(float(np.mean(values)) if values else np.nan)
            stds.append(float(np.std(values, ddof=1)) if len(values) > 1 else 0.0)
        return np.asarray(means), np.asarray(stds)

    coverage, coverage_std = stats("final_coverage")
    success, success_std = stats("success")
    x = np.arange(len(labels))
    width = 0.36
    fig, ax = plt.subplots(figsize=(max(7.0, 0.9 * len(labels)), 4.2))
    ax.bar(x - width / 2, coverage, width, yerr=coverage_std, label="Final coverage", color="#1769aa")
    ax.bar(x + width / 2, success, width, yerr=success_std, label="Success rate", color="#d97706")
    ax.set_ylim(0.0, 1.05)
    ax.set_ylabel("mean ± std")
    ax.set_xticks(x, labels, rotation=25, ha="right")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(frameon=False)
    fig.tight_layout()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
