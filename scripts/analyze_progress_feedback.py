"""Read-only study analysis; derived tables never replace original receipts."""

import argparse
import csv
import gzip
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np

from dbact.control.task_progress_readout import anchored_candidate

ROOT = Path(__file__).resolve().parents[1]
STUDY = ROOT / "runs/research/2026-10-31-progress-local-v2"


def publication(rows):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    matplotlib.rcParams.update(
        {
            "pdf.fonttype": 42,
            "font.size": 8,
            "axes.titlesize": 8,
            "axes.labelsize": 8,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
        }
    )
    target = ROOT / "manuscript/ccta2027/generated"
    ids = [
        "development_rectangle_seed2_boundary_cvt",
        "development_l_shape_seed8_boundary_cvt",
        "development_c_shape_seed8_boundary_cvt",
    ]
    labels = ["Rectangle 2", "L-shape 8", "C-shape 8"]
    latex = []
    fig, axes = plt.subplots(3, 1, figsize=(3.4, 4.0), sharex=True)
    for ax, cid, label in zip(axes, ids, labels):
        group = {r["arm"]: r for r in rows if r["case_id"] == cid}
        base, local = group["baseline"], group["anchored_readout"]
        latex.append(
            f"{label} & {base['endpoint_error']:.4f} & {local['endpoint_error']:.4f} & "
            f"{base['mean_abs_aligned_team_error']:.4f} & {local['mean_abs_aligned_team_error']:.4f} & "
            f"{int(base['completed'])}/{int(local['completed'])}" + r" \\"
        )
        for arm, color, style, name in [
            ("baseline", "#697c92", "-", "Baseline"),
            ("certificate_only", "#c29c24", ":", "Certificate only"),
            ("anchored_readout", "#b9653a", "-", "Anchored readout"),
        ]:
            times = []
            errors = []
            path = STUDY / "comparison" / cid / arm / "registration.jsonl.gz"
            with gzip.open(path, "rt", encoding="utf-8") as stream:
                for line in stream:
                    r = json.loads(line)
                    key = next(iter(r["truth"]))
                    times.append(r["time"])
                    errors.append(r["team_progress"][key] - r["truth"][key]["progress"])
            ax.plot(times, errors, color=color, ls=style, lw=1, label=name)
        ax.set_title(label)
        ax.set_ylabel("Bias (m)")
        ax.axhline(0, color="black", lw=0.4)
        ax.grid(alpha=0.15)
    axes[-1].set_xlabel("Control time (s)")
    axes[0].legend(
        fontsize=6,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.25),
        ncol=3,
        frameon=False,
    )
    fig.tight_layout()
    fig.savefig(target / "progress_local_errors.pdf")
    plt.close(fig)
    (target / "progress_local_rows.tex").write_text("\n".join(latex), encoding="utf-8")


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def analyze_case(folder, receipt, config):
    r = receipt["result"]
    cid = next(iter(r["task"]))
    d = np.asarray(r["goal_direction"])
    steps = json.loads((folder / "paired_metrics.json").read_text())["steps"]
    errors = []
    own_errors = []
    candidate_errors = []
    admitted_original_errors = []
    deltas = []
    reasons = Counter()
    ranks = Counter()
    identifiable = 0
    registrations = 0
    coarse_false_accepts = 0
    accounting_max = 0.0
    identity_max = 0.0
    normals = 0
    trimmed = 0
    brake = None
    aligned_count = 0
    q = []
    pre_error = []
    fresh_unavailable = 0
    speed = float(config.get("controller", {}).get("max_object_speed", 0.6))
    with gzip.open(folder / "registration.jsonl.gz", "rt", encoding="utf-8") as stream:
        for k, line in enumerate(stream):
            frame = json.loads(line)
            truth = frame["truth"][cid]["progress"]
            value = frame["team_progress"][cid]
            assert (
                frame["control_frame"] == k
                and abs(frame["time"] - k * config["dt"]) < 1e-8
            )
            assert value == steps[k]["estimated_progress"]
            if k:
                assert abs(truth - steps[k - 1]["true_progress"]) < 1e-12
            aligned_count += 1
            errors.append(value - truth)
            if brake is None and steps[k]["phase"] == "BRAKE":
                brake = {
                    "control_frame": k,
                    "time": frame["time"],
                    "true_progress": truth,
                    "estimated_progress": value,
                    "bias": value - truth,
                }
            for agent in frame["agents"]:
                reg = agent["registration"]
                if reg is None:
                    reasons["NO_FRESH_REGISTRATION"] += 1
                    continue
                registrations += 1
                for key in ("A", "b", "m_pre", "translation"):
                    reg[key] = np.asarray(reg[key])
                assert reg["frame"] == k and reg["A"].shape == (reg["matches"], 2)
                normals += reg["matches"]
                trimmed += reg["matched_before_trim"] - reg["matches"]
                post = np.asarray(agent["m_post"])
                fusion = agent["fusion"]
                f = np.zeros(2) if fusion is None else np.asarray(fusion["translation"])
                accounting_max = max(
                    accounting_max,
                    float(np.max(np.abs(post - reg["m_pre"] - reg["translation"] - f))),
                )
                decision = anchored_candidate(
                    agent["original"], reg, d, agent["gain"], agent["quality"], speed
                )
                reasons[decision["reason"]] += 1
                if "rank" in decision:
                    ranks[str(decision["rank"])] += 1
                if decision.get("identifiable"):
                    identifiable += 1
                    q.append(decision["amplification"])
                elif agent["quality"] != "unavailable":
                    coarse_false_accepts += 1
                if agent["quality"] == "unavailable":
                    fresh_unavailable += 1
                if agent["quality"] != "unavailable":
                    own_errors.append(agent["original"] - truth)
                    pre_error.append(float(d @ reg["m_pre"]) - truth)
                if decision["admitted"]:
                    admitted_original_errors.append(agent["original"] - truth)
                    candidate_errors.append(decision["candidate"] - truth)
                    delta = decision["candidate"] - agent["original"]
                    deltas.append(delta)
                    expected = agent["gain"] * (
                        decision["relative_map_scalar"] - d @ (reg["translation"] + f)
                    )
                    identity_max = max(identity_max, abs(delta - expected))
                if receipt["arm"] != "baseline":
                    assert agent["decision"]["reason"] == decision["reason"]
                    assert agent["decision"]["candidate"] == decision["candidate"]
    assert aligned_count == len(steps)
    return {
        "case_id": receipt["case_id"],
        "arm": receipt["arm"],
        "frames": len(steps),
        "endpoint_error": r["euclidean_endpoint_error"],
        "completed": r["task_completed"],
        "g500": r["g500_pass"],
        "settled": r["settled"],
        "watchdog": r["watchdog"],
        "mean_abs_aligned_team_error": float(np.mean(np.abs(errors))),
        "final_aligned_bias": errors[-1],
        "brake": brake,
        "registrations": registrations,
        "matched_normal_rows": normals,
        "removed_by_trim": trimmed,
        "identifiable": identifiable,
        "coarse_accepted_unidentifiable": coarse_false_accepts,
        "ranks": dict(ranks),
        "fresh_unavailable": fresh_unavailable,
        "admission_reasons": dict(reasons),
        "q_median": float(np.median(q)) if q else None,
        "q_p95": float(np.percentile(q, 95)) if q else None,
        "accounting_max_error": accounting_max,
        "candidate_identity_max_error": identity_max,
        "mean_abs_candidate_change_admitted": float(np.mean(np.abs(deltas)))
        if deltas
        else None,
        "own_mae_available": float(np.mean(np.abs(own_errors))),
        "candidate_mae_admitted": float(np.mean(np.abs(candidate_errors)))
        if candidate_errors
        else None,
        "original_mae_same_admitted": float(np.mean(np.abs(admitted_original_errors)))
        if admitted_original_errors
        else None,
        "pre_anchor_mae_available": float(np.mean(np.abs(pre_error))),
        "raw_trace_sha256": sha(folder / "registration.jsonl.gz"),
    }


def equivalence(base, cert):
    with (
        np.load(base / "replay.npz", allow_pickle=False) as a,
        np.load(cert / "replay.npz", allow_pickle=False) as b,
    ):
        assert a.files == b.files
        array_count = len(a.files)
        failures = [
            key
            for key in a.files
            if not np.array_equal(a[key], b[key], equal_nan=a[key].dtype.kind in "fc")
        ]
    a = json.loads((base / "paired_metrics.json").read_text())["steps"]
    b = json.loads((cert / "paired_metrics.json").read_text())["steps"]
    # This schema contains no wall-clock timing fields: compare every field,
    # including contact forces and solver fallback/barrier statistics.
    fields = list(a[0])
    # Agent diagnostics contain declared NaN placeholders. A stable JSON encoding
    # compares those explicitly as identical missing values, not as observations.
    same = len(a) == len(b) and all(
        json.dumps(x, sort_keys=True) == json.dumps(y, sort_keys=True)
        for x, y in zip(a, b)
    )
    assert not failures and same, (failures, same)
    return {
        "replay_arrays": array_count,
        "exact_array_equality": True,
        "frames": len(a),
        "scientific_fields": fields,
        "exact_field_equality": True,
    }


def analyze(phase):
    folder = STUDY / phase
    receipts = json.loads((folder / "results.json").read_text())
    manifest = json.loads((folder / "manifest.json").read_text())
    cases = {c["case_id"]: c for c in manifest["plan"]["cases"]}
    rows = []
    for receipt in receipts:
        assert receipt["status"] == "EXECUTED", receipt
        case = cases[receipt["case_id"]]
        rows.append(
            analyze_case(
                folder / receipt["case_id"] / receipt["arm"], receipt, case["config"]
            )
        )
    checks = {}
    if phase == "comparison":
        for cid in cases:
            checks[cid] = equivalence(
                folder / cid / "baseline", folder / cid / "certificate_only"
            )
            group = [r for r in receipts if r["case_id"] == cid]
            assert len({r["result"]["config_sha256"] for r in group}) == 1
            assert len({r["result"]["initial_state_sha256"] for r in group}) == 1
    (STUDY / f"{phase}_analysis.json").write_text(
        json.dumps({"cases": rows, "equivalence": checks}, indent=2)
    )
    flat = [
        {
            k: r[k]
            for k in [
                "case_id",
                "arm",
                "frames",
                "endpoint_error",
                "completed",
                "g500",
                "mean_abs_aligned_team_error",
                "final_aligned_bias",
                "registrations",
                "identifiable",
                "coarse_accepted_unidentifiable",
                "mean_abs_candidate_change_admitted",
            ]
        }
        for r in rows
    ]
    with (STUDY / f"{phase}_results.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows(flat)
    if phase == "comparison":
        publication(rows)
    print(json.dumps(flat, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--phase", choices=["diagnostic", "comparison"], required=True)
    analyze(p.parse_args().phase)
