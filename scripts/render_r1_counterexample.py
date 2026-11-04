"""Render the saved finite-ray counterexample; never rerun the analysis."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from research_io import ROOT, guard, read, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    if not out.is_relative_to(ROOT / "runs"):
        raise ValueError("Use a new figure directory under runs/")
    guard()
    data = read(args.analysis)
    example = data["counterexample"]
    out.mkdir(parents=True, exist_ok=False)
    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "pdf.fonttype": 42,
            "svg.fonttype": "none",
            "svg.hashsalt": "dbact-r1-v1",
        }
    )
    fig, axes = plt.subplots(
        2, 1, figsize=(9.2, 4.8), layout="constrained", sharex=True, sharey=True
    )
    fig.get_layout_engine().set(rect=(0.0, 0.085, 1.0, 0.915))
    origin = np.asarray(example["origin"])
    directions = np.asarray(example["directions"])
    for index, (ax, key, title) in enumerate(
        zip(
            axes,
            ["flat_vertices", "hidden_vertices"],
            [
                "A  Flat boundary: true clearance 100 mm",
                "B  Hidden protrusion: true clearance 20 mm",
            ],
        )
    ):
        vertices = (np.asarray(example[key]) - origin) * 1000
        ax.add_patch(
            Polygon(vertices, facecolor="#c7d5df", edgecolor="#334e68", lw=1.4)
        )
        if index:
            ax.add_patch(
                Polygon(vertices[4:], facecolor="#e69f00", edgecolor="#b66a00", lw=1.4)
            )
        for ray_index in [0, 1]:
            endpoint = directions[ray_index] * example["ray_ranges"][ray_index] * 1000
            ax.plot(
                [0, endpoint[0]], [0, endpoint[1]], color="#0072b2", lw=1.15, zorder=4
            )
            ax.scatter(*endpoint, color="#0072b2", s=32, zorder=6)
        ax.scatter(0, 0, color="#182b3a", s=40, zorder=6)
        ax.text(0, -3.4, "Sensor / robot centre", ha="left", va="top", fontsize=9)
        ax.text(
            52,
            8.8,
            "Adjacent rays: 5 degrees",
            color="#0072b2",
            ha="center",
            fontsize=9,
        )
        if index:
            tip = vertices[5]
            ax.annotate(
                "Unobserved feature",
                xy=tip,
                xytext=(40, -8),
                color="#875000",
                arrowprops={"arrowstyle": "->", "color": "#875000"},
                fontsize=9,
            )
        ax.set(xlim=(-4, 114), ylim=(-11, 15), ylabel="y (mm)", title=title)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=0.15, lw=0.6)
    axes[-1].set_xlabel("x relative to sensor (mm)")
    fig.suptitle("Same finite-ray observations; different true clearance", fontsize=13)
    fig.text(
        0.5,
        0.016,
        "Synthetic geometry only. The hidden world violates the assumed initial safety clearance.",
        ha="center",
        fontsize=9,
        color="#444444",
    )
    for suffix in ["png", "pdf", "svg"]:
        fig.savefig(
            out / ("finite_ray_counterexample." + suffix), dpi=300, bbox_inches="tight"
        )
    plt.close(fig)
    caption = (
        "Finite-ray ambiguity in a synthetic geometry example, without a physical rollout. "
        "The two polygons produce the same nearest-hit ranges for all 72 scan directions "
        "(numerical difference below 1e-12 m); only the two adjacent rays around the hidden feature are shown. "
        "The unchanged support gate admits both with a minimum lower-distance estimate of 65.626 mm, "
        "while the true clearances are 100 and 20 mm. The hidden world already violates the 65 mm "
        "initial safety clearance. This example refutes a general geometric lower-bound interpretation "
        "of finite-ray half-gap support; it does not refute an invariance theorem that assumes certified "
        "initial safety and valid whole-interval coverage. Equal physical axis scales are used.\n"
    )
    with (out / "caption.txt").open("x", encoding="utf-8") as f:
        f.write(caption)
    shutil.copy2(__file__, out / Path(__file__).name)
    receipt = {
        "analysis_sha256": sha(args.analysis),
        "renderer_sha256": sha(__file__),
        "source": "Saved counterexample coordinates, not a new physical experiment",
        "files": {p.name: sha(p) for p in out.iterdir() if p.is_file()},
    }
    with (out / "provenance.json").open("x", encoding="utf-8") as f:
        json.dump(receipt, f, indent=2)
    print(json.dumps({"output": str(out), "files": list(receipt["files"])}))


if __name__ == "__main__":
    main()
