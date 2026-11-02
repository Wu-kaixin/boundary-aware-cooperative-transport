"""Generate this development study's tables and figures from preserved records."""

import csv
import hashlib
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from dbact.sensing.task_observability import (
    directional_certificate,
    select_additional_normal,
)

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "runs/research/2026-10-31-readiness-v1"
FIG = ROOT / "manuscript/ccta2027/generated"
FIG.mkdir(exist_ok=True)
matplotlib.rcParams.update(
    {
        "pdf.fonttype": 42,
        "font.size": 8,
        "axes.labelsize": 8,
        "axes.titlesize": 8,
        "xtick.labelsize": 7,
        "ytick.labelsize": 7,
    }
)


def csv_write(path, rows):
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def file_sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def historical():
    rows = []
    for shape in ("rectangle", "l_shape", "c_shape"):
        for seed in (2, 5, 8):
            p = (
                ROOT
                / f"runs/ccta2027/draft_v2/run/development_{shape}_seed{seed}_boundary_cvt"
            )
            case = json.loads((p / "case_result.json").read_text())
            r = case["result"]
            steps = json.loads((p / "paired_metrics.json").read_text())["steps"]
            start = next(
                i for i, s in enumerate(steps) if s["phase"] == "CONTACT_READY"
            )
            first = steps[start]
            later = steps[min(start + 20, len(steps) - 1)]
            with np.load(p / "replay.npz", allow_pickle=False) as replay:
                keys = list(replay.files)
            row = {
                "case_id": p.name,
                "shape": shape,
                "seed": seed,
                "task_completed": r["task_completed"],
                "geometric_ready_control_index": r["contact_ready_frame"],
                "first_ready_logged_frame": first["frame"],
                "physical_ready": not r["physical_contact_ready"]["right_censored"],
                "delta_progress_next20": later["true_progress"]
                - first["true_progress"],
                "final_progress_bias": steps[-1]["estimated_progress"]
                - steps[-1]["true_progress"],
                "endpoint_error": r["euclidean_endpoint_error"],
                "source_sha256": case["identity"]["source_sha256"],
                "trajectory": str((p / "replay.npz").relative_to(ROOT)),
                "trajectory_sha256": file_sha256(p / "replay.npz"),
                "replay_keys": ";".join(keys),
            }
            rows.append(row)
    csv_write(OUT / "historical_cases.csv", rows)
    fig, ax = plt.subplots(figsize=(6.4, 2.6))
    for success, marker, label in [
        (True, "o", "Completed"),
        (False, "x", "Not completed"),
    ]:
        selected = [r for r in rows if r["task_completed"] == success]
        ax.scatter(
            [r["final_progress_bias"] for r in selected],
            [r["endpoint_error"] for r in selected],
            marker=marker,
            label=label,
        )
    ax.set(
        xlabel="Final estimated minus true progress (m)", ylabel="Endpoint error (m)"
    )
    ax.legend()
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(FIG / "historical_bias.pdf")
    plt.close(fig)
    latex = []
    for row in rows:
        shape = {"rectangle": "Rectangle", "l_shape": "L-shape", "c_shape": "C-shape"}[
            row["shape"]
        ]
        latex.append(
            f"{shape} & {row['seed']} & {row['delta_progress_next20']:.4f} & "
            f"{row['final_progress_bias']:+.4f} & {row['endpoint_error']:.4f} & "
            + ("Yes" if row["task_completed"] else "No")
            + r" \\"
        )
    (FIG / "historical_rows.tex").write_text("\n".join(latex), encoding="utf-8")
    return rows


def observation():
    rows = []
    truth = np.array([0.02, 0.03])
    candidates = np.array([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0], [-1.0, 1.0]])
    candidates /= np.linalg.norm(candidates, axis=1)[:, None]
    for geometry in ("parallel", "near_parallel", "diverse"):
        angles = (
            np.zeros(8)
            if geometry == "parallel"
            else np.tile(np.deg2rad([-2, 2]), 4)
            if geometry == "near_parallel"
            else np.tile([0, np.pi / 2], 4)
        )
        initial = np.column_stack((np.cos(angles), np.sin(angles)))
        for angle in (0, 45, 90):
            d = np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))])
            for noise in (0.0, 0.001):
                epsilon = np.random.default_rng(101).normal(0, noise, 12)
                choice, _ = select_additional_normal(initial, candidates, d)
                for mode in ("repeat", "certificate_only", "targeted_probe"):
                    extra = (
                        initial[0] if mode != "targeted_probe" else candidates[choice]
                    )
                    a = np.vstack((initial, np.tile(extra, (4, 1))))
                    b = a @ truth + epsilon
                    h = a.T @ a
                    estimate = np.linalg.solve(
                        h + 0.001 * np.trace(h) * np.eye(2), a.T @ b
                    )
                    cert = directional_certificate(a, d)
                    eigen, axes = np.linalg.eigh(h)
                    coarse = eigen[0] / eigen[-1] > 1e-3 or abs(axes[:, -1] @ d) >= 0.5
                    rows.append(
                        {
                            "geometry": geometry,
                            "angle": angle,
                            "noise_std": noise,
                            "mode": mode,
                            "measurements": 12,
                            "coarse_accept": bool(coarse),
                            "identifiable": cert["identifiable"],
                            "null_component": cert["null_component"],
                            "amplification": cert["amplification"],
                            "ridge_progress_error": float(d @ estimate - d @ truth),
                            "gain_progress_error": float(
                                1.24 * (d @ estimate) - d @ truth
                            ),
                            "candidate_index": int(choice)
                            if mode == "targeted_probe"
                            else -1,
                        }
                    )
    csv_write(OUT / "observation_conditions.csv", rows)
    for i in range(0, len(rows), 3):
        assert rows[i]["gain_progress_error"] == rows[i + 1]["gain_progress_error"]
    report = {
        mode: {
            "conditions": 18,
            "identifiable": sum(r["identifiable"] for r in rows if r["mode"] == mode),
            "coarse_false_accept": sum(
                r["coarse_accept"] and not r["identifiable"]
                for r in rows
                if r["mode"] == mode
            ),
            "mean_abs_ridge_error": float(
                np.mean(
                    [abs(r["ridge_progress_error"]) for r in rows if r["mode"] == mode]
                )
            ),
            "mean_abs_gain_error": float(
                np.mean(
                    [abs(r["gain_progress_error"]) for r in rows if r["mode"] == mode]
                )
            ),
        }
        for mode in ("repeat", "certificate_only", "targeted_probe")
    }
    (OUT / "observation_summary.json").write_text(json.dumps(report, indent=2))
    fig, axes = plt.subplots(1, 2, figsize=(3.4, 2.0))
    for ax, key, title in zip(
        axes,
        ["mean_abs_ridge_error", "mean_abs_gain_error"],
        ["Ridge", "Ridge + gain 1.24"],
    ):
        ax.bar(
            ["Repeat", "Cert.", "Target"],
            [report[m][key] * 1000 for m in report],
            color=["#697c92", "#697c92", "#167b83"],
        )
        ax.set(title=title, ylabel="Mean abs. error (mm)")
    fig.tight_layout()
    fig.savefig(FIG / "observation_probe.pdf")
    plt.close(fig)
    return report


def paired():
    results = json.loads((OUT / "paired/results.json").read_text())
    rows = []
    for receipt in results:
        assert receipt["status"] == "EXECUTED", receipt
        r = receipt["result"]
        rows.append(
            {
                "case_id": receipt["case_id"],
                "mode": receipt["mode"],
                "endpoint_error": r["euclidean_endpoint_error"],
                "completed": r["task_completed"],
                "settled": r["settled"],
                "g500": r["g500_pass"],
                "frames": r["termination"]["frames_run"],
                "ready_frame": r["contact_ready_frame"],
                "initial_hash": r["initial_state_sha256"],
                "config_hash": r["config_sha256"],
            }
        )
    csv_write(OUT / "paired_results.csv", rows)
    fig, ax = plt.subplots(figsize=(3.4, 2.1))
    names = sorted({r["case_id"] for r in rows})
    x = np.arange(3)
    for mode, shift, color in [
        ("baseline", -0.18, "#697c92"),
        ("oracle_progress_only", 0.18, "#b9653a"),
    ]:
        ax.bar(
            x + shift,
            [
                next(
                    r["endpoint_error"]
                    for r in rows
                    if r["case_id"] == n and r["mode"] == mode
                )
                for n in names
            ],
            0.34,
            label="Baseline" if mode == "baseline" else "Progress oracle",
            color=color,
        )
    ax.set_xticks(
        x,
        [
            n.replace("development_", "")
            .replace("_boundary_cvt", "")
            .replace("c_shape_seed", "C-shape ")
            .replace("l_shape_seed", "L-shape ")
            .replace("rectangle_seed", "Rectangle ")
            for n in names
        ],
        fontsize=7,
    )
    ax.set_ylabel("Endpoint error (m)")
    ax.legend(
        fontsize=7,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        ncol=2,
        frameon=False,
    )
    fig.tight_layout()
    fig.savefig(FIG / "paired_oracle.pdf")
    plt.close(fig)
    for n in names:
        pair = [r for r in rows if r["case_id"] == n]
        assert (
            pair[0]["initial_hash"] == pair[1]["initial_hash"]
            and pair[0]["config_hash"] == pair[1]["config_hash"]
        )
    latex = []
    for n in names:
        pair = {r["mode"]: r for r in rows if r["case_id"] == n}
        b, o = pair["baseline"], pair["oracle_progress_only"]
        label = (
            n.replace("development_", "").replace("_boundary_cvt", "").replace("_", " ")
        )
        latex.append(
            f"{label} & {b['endpoint_error']:.4f} & {o['endpoint_error']:.4f} & "
            f"{int(b['completed'])}/{int(o['completed'])} & {int(b['g500'])}/{int(o['g500'])}"
            + r" \\"
        )
    (FIG / "paired_rows.tex").write_text("\n".join(latex), encoding="utf-8")
    return rows


if __name__ == "__main__":
    if "--paired" in sys.argv:
        print(json.dumps(paired(), indent=2))
    else:
        print(
            json.dumps(
                {"historical": historical(), "observation": observation()}, indent=2
            )
        )
