"""Publish compact evidence from fresh runs; never infer success from exit code.

Run after the consolidation gates. Raw trajectories remain in ignored runs/.
"""
from __future__ import annotations

import hashlib
import json
import platform
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import scipy

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/results/consolidation"
ASSETS = ROOT / "docs/assets"


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    manifest = {"python": platform.python_version(), "numpy": np.__version__,
                "scipy": scipy.__version__, "files": {}, "figures": {},
                "scope": "static oracle-map theory validation; Gates 5 and 6 not certified"}

    def remember(path):
        key = path.relative_to(ROOT).as_posix()
        manifest["files"][key] = digest(path)
        return key

    def copy(source, target, inputs=()):
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        remember(source)
        remember(target)
        if inputs:
            manifest["figures"][target.relative_to(ROOT).as_posix()] = {
                "source": source.relative_to(ROOT).as_posix(),
                "inputs": {remember(p): digest(p) for p in inputs},
            }

    lines = ["Generated from fresh JSON/CSV outputs; all seeds and failures retained.", "",
             "| Run | Frames | Termination | G500 | Failure reasons |",
             "| --- | ---: | --- | --- | --- |"]
    near = []
    for name, seed, asset in [("near", 2, "closed_loop_d_seed2.gif"),
                               ("near", 4, "closed_loop_d_seed4.gif"),
                               ("near", 8, "closed_loop_d_seed8.gif"),
                               ("search", 7, "search_d_seed7.gif")]:
        folder = ROOT / "runs/readme" / (f"near_seed_{seed}" if name == "near" else "search_seed7")
        summary = read(folder / "summary.json")
        cargo = next(iter(summary["cargoes"].values()))
        gate = cargo["g500"]
        lines.append(f"| {name} seed {seed} | {summary['steps']} | {summary['timing']['terminated_by']} | "
                     f"{'PASS' if gate['success'] else 'FAIL'} | {'; '.join(gate['failure_reasons']) or '—'} |")
        near.append({"run": name, "seed": seed, "summary": summary})
        copy(folder / "summary.json", OUT / f"{name}_seed{seed}.json")
        copy(folder / "closed_loop.gif", ASSETS / asset,
             [folder / "replay.npz", folder / "summary.json"])
    manifest["closed_loop_commits"] = sorted({r["summary"]["provenance"]["git_sha"] for r in near})

    sweep_path = ROOT / "runs/readme/d_sweep/g500_sweep.json"
    sweep = read(sweep_path)
    copy(sweep_path, OUT / "g500_sweep.json")
    lines += ["", f"Near-field sweep seeds `{sweep['seeds']}`: **{sweep['g500_pass']}/{sweep['g500_total']} G500 pass**.",
              "", "| Metric | Mean ± population SD | Min–max |", "| --- | ---: | ---: |"]
    for key, stat in sweep["distributions"].items():
        if stat:
            lines.append(f"| {key} | {stat['mean']:.6g} ± {stat['sd']:.6g} | {stat['min']:.6g}–{stat['max']:.6g} |")
    for folder, file in [("d10_diag", "diagnosis.json"), ("d10_ab", "ab.json"), ("d10_enc", "gate.json")]:
        copy(ROOT / "runs/readme" / folder / file, OUT / f"{folder}.json")
    for folder, file, asset, data in [
        ("d10_diag", "figA_segments.png", "d10-post-detection-stages.png", "diagnosis.json"),
        ("d10_diag", "figB_coverage.png", "d10-coverage-and-gap.png", "diagnosis.json"),
        ("d10_enc", "figF_gate_tradeoff.png", "d10-gate-tradeoff.png", "gate.json"),
    ]:
        source = ROOT / "runs/readme" / folder
        copy(source / file, ASSETS / asset, [source / data])

    matrix = ROOT / "runs/paper/jeh_matrix"
    summary = read(matrix / "summary.json")
    copy(matrix / "summary.json", OUT / "jeh_summary.json")
    copy(matrix / "summary.csv", OUT / "jeh_summary.csv")
    copy(matrix / "prior_screen.json", OUT / "prior_screen.json")
    manifest["static_commits"] = sorted({r["code_sha"] for r in summary["cases"]})
    lines += ["", "**Static oracle-map theory validation** (3 shapes × 9 seeds × 600 frames).",
              "The bounds below are the declared rigorous prior quantities; plotted J/E/H use numerical observer estimates.",
              "", "| Shape | B_E | B_J_prior | B_J_geom | Complete | K0 entire horizon |",
              "| --- | ---: | ---: | ---: | ---: | ---: |"]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    checks = {}
    for col, shape in enumerate(("l_shape", "rectangle", "c_shape")):
        rows = [r for r in summary["cases"] if r["shape"] == shape]
        first = rows[0]
        lines.append(f"| {shape} | {first['B_E_rigorous']:.10g} | {first['B_J_prior_certificate']:.10g} | "
                     f"{first['B_J_geom']:.10g} | {sum(r['complete_30s'] for r in rows)}/{len(rows)} | "
                     f"{sum(r['theorem_conditions_hold_full_horizon'] for r in rows)}/{len(rows)} |")
        case = matrix / "runs" / shape / "n20/seed_2"
        remember(case / "trajectory.npz")
        data = np.load(case / "trajectory.npz", allow_pickle=False)
        p = data["positions"]
        vertices = data["vertices"]
        ax = axes[0, col]
        ax.fill(vertices[:, 0], vertices[:, 1], color="0.75")
        for i in range(p.shape[1]):
            ax.plot(p[:, i, 0], p[:, i, 1], lw=0.7, alpha=0.65)
        ax.scatter(p[0, :, 0], p[0, :, 1], facecolors="none", edgecolors="C0", s=22, label="initial")
        ax.scatter(p[-1, :, 0], p[-1, :, 1], c="C3", s=18, label="final")
        ax.set(title=f"{shape}, seed 2", xlabel="x (m)", ylabel="y (m)", aspect="equal")
        ax.legend(fontsize=7)
        ax = axes[1, col]
        for key in ("J", "E", "H_star"):
            ax.plot(np.arange(len(data[key])) * 0.05, data[key], label=key)
        ax.set(xlabel="Time (s)", ylabel="Numerical observer metric")
        ax.legend(fontsize=7)
        if shape != "c_shape":
            serial = ROOT / "runs/paper/jeh_serial_pair/runs" / shape / "n20/seed_2/trajectory.npz"
            other = np.load(serial, allow_pickle=False)
            remember(serial)
            delta = {key: float(np.max(np.abs(other[key] - data[key]))) for key in ("J", "E", "H_star", "positions")}
            checks[shape] = {"max_absolute_difference": delta, "pass": all(v <= 1e-12 for v in delta.values())}
    fig.suptitle("Static oracle-map theory validation — numerical observer trajectories")
    fig.tight_layout()
    figure = ASSETS / "static-deployment-comparison.png"
    fig.savefig(figure, dpi=160)
    plt.close(fig)
    remember(figure)
    manifest["figures"][figure.relative_to(ROOT).as_posix()] = {
        "inputs": {p: h for p, h in manifest["files"].items() if p.endswith("trajectory.npz")},
        "seed": 2, "label": "static oracle-map theory validation"}
    (OUT / "serial_parallel_consistency.json").write_text(json.dumps(checks, indent=2), encoding="utf-8")
    lines += ["", f"Serial/parallel full 600-frame comparison (L-shape and rectangle, seed 2, tolerance 1e-12): "
              f"**{'PASS' if all(v['pass'] for v in checks.values()) else 'FAIL'}**.", "",
              "![Static deployment comparison](docs/assets/static-deployment-comparison.png)", "",
              "[Experiment manifest and source hashes](docs/results/consolidation/manifest.json).",
              "[Full compact results](docs/results/consolidation/)."]
    queue = ROOT / "runs/consolidation/readme_manifest.json"
    manifest["execution"] = read(queue)
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    block = "\n".join(lines)
    (OUT / "results.md").write_text(block + "\n", encoding="utf-8")
    for path in ROOT.glob("README*.md"):
        text = path.read_text(encoding="utf-8")
        start, end = "<!-- CONSOLIDATION_RESULTS_START -->", "<!-- CONSOLIDATION_RESULTS_END -->"
        if start in text:
            before, rest = text.split(start, 1)
            _, after = rest.split(end, 1)
            path.write_text(before + start + "\n" + block + "\n" + end + after, encoding="utf-8")


if __name__ == "__main__":
    main()
