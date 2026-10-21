#!/usr/bin/env python3
"""Aggregate saved case results into tables, intervals, and paper figures.

Reads case_result.json and baseline pair.json files. Does not resimulate.
Statistics use episodes or matched conditions. Missing events stay censored.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

Z95 = 1.959963984540054
BOOTSTRAP_SEED = 20261007
BOOTSTRAP_DRAWS = 10000


def wilson_interval(successes: int, total: int, z: float = Z95) -> dict:
    """95% Wilson interval for a binomial rate. Empty samples stay null."""
    if total <= 0:
        return {"k": successes, "n": total, "rate": None, "low": None, "high": None}
    p = successes / total
    denom = 1.0 + z * z / total
    centre = (p + z * z / (2.0 * total)) / denom
    half = z * math.sqrt(p * (1.0 - p) / total + z * z / (4.0 * total * total)) / denom
    return {
        "k": int(successes),
        "n": int(total),
        "rate": p,
        "low": max(0.0, centre - half),
        "high": min(1.0, centre + half),
    }


def paired_bootstrap(differences: list[float], seed: int = BOOTSTRAP_SEED) -> dict:
    """Mean paired difference with a percentile interval. No imputation."""
    values = np.asarray(differences, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {"n": 0, "mean": None, "median": None, "low": None, "high": None}
    rng = np.random.default_rng(seed)
    draws = values[rng.integers(0, values.size, size=(BOOTSTRAP_DRAWS, values.size))].mean(axis=1)
    return {
        "n": int(values.size),
        "mean": float(values.mean()),
        "median": float(np.median(values)),
        "low": float(np.quantile(draws, 0.025)),
        "high": float(np.quantile(draws, 0.975)),
    }


def seed_cluster_bootstrap(pairs: list[dict], field: str = "ready_difference_seconds") -> dict:
    """Keep all repeated-shape/perturbation observations of one seed together.

    Five seed clusters is a conservative reporting policy for this study, not a
    theorem that bootstrap coverage becomes reliable at that sample size.
    """
    groups = defaultdict(list)
    for row in pairs:
        value = row.get(field)
        if isinstance(value, (int, float)) and np.isfinite(value):
            groups[row.get("seed")].append(float(value))
    values = [value for group in groups.values() for value in group]
    result = {"n_pairs": len(values), "n_seed_clusters": len(groups), "resampling_unit": "seed",
              "mean": float(np.mean(values)) if values else None,
              "median": float(np.median(values)) if values else None,
              "low": None, "high": None, "status": "DESCRIPTIVE_ONLY_FEW_SEEDS"}
    if len(groups) >= 5:
        sums = np.array([sum(group) for group in groups.values()])
        counts = np.array([len(group) for group in groups.values()])
        rng = np.random.default_rng(BOOTSTRAP_SEED)
        picks = rng.integers(0, len(groups), size=(BOOTSTRAP_DRAWS, len(groups)))
        draws = sums[picks].sum(axis=1) / counts[picks].sum(axis=1)
        result.update(low=float(np.quantile(draws, 0.025)), high=float(np.quantile(draws, 0.975)),
                      status="EXPLORATORY_SEED_CLUSTER_INTERVAL")
    return result


def _load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def iter_results(root: Path):
    for path in sorted(root.rglob("case_result.json")):
        payload = _load_json(path)
        result = dict(payload.get("result", payload))
        result["_path"] = str(path)
        result["_dir"] = str(path.parent)
        yield result
    for path in sorted(root.rglob("pair.json")):
        payload = _load_json(path)
        rows = payload if isinstance(payload, list) else [payload]
        for result in rows:
            if not isinstance(result, dict) or "variant" not in result:
                continue
            copied = dict(result)
            copied.setdefault("shape", path.parents[1].name)
            copied.setdefault("seed", int(path.parent.name.replace("seed_", "")))
            copied.setdefault("case_id", f"{copied['shape']}_seed{copied['seed']}_{copied['variant']}")
            copied.setdefault("execution_status", "COMPLETED")
            copied.setdefault("stratum", "archived_gate")
            copied["_path"] = str(path)
            copied["_dir"] = str(path.parent / copied["variant"])
            yield copied


def _event_seconds(result: dict, name: str):
    events = result.get("events") or {}
    event = events.get(name) or {}
    if event:
        return event.get("seconds"), bool(event.get("right_censored", event.get("seconds") is None))
    if name == "CONTACT_READY":
        seconds = result.get("contact_ready_seconds")
        return seconds, seconds is None
    return None, True


def episode_row(result: dict, batch: str) -> dict:
    ready_s, ready_censored = _event_seconds(result, "CONTACT_READY")
    physical = result.get("physical_contact_ready") or {}
    physical_s = physical.get("seconds")
    physical_censored = bool(physical.get("right_censored", physical_s is None)) if physical else True
    status = result.get("execution_status", "COMPLETED")
    return {
        "batch": batch,
        "case_id": result.get("case_id"),
        "condition_id": result.get("condition_id"),
        "gate": result.get("gate"),
        "arm": result.get("arm", result.get("variant")),
        "split": result.get("split"),
        "shape": result.get("shape"),
        "seed": result.get("seed"),
        "variant": result.get("variant"),
        "stratum": result.get("stratum") or "unspecified",
        "execution_status": status,
        "task_completed": result.get("task_completed"),
        "settled": result.get("settled"),
        "g500_pass": result.get("g500_pass", result.get("g500_success")),
        "watchdog": result.get("watchdog"),
        "controller_ready_seconds": ready_s,
        "controller_ready_censored": ready_censored,
        "physical_ready_seconds": physical_s,
        "physical_ready_censored": physical_censored,
        "absolute_progress_error": result.get("absolute_progress_error"),
        "signed_progress_error": result.get("signed_progress_error"),
        "euclidean_endpoint_error": result.get("euclidean_endpoint_error"),
        "max_cross_track": result.get("max_cross_track"),
        "direction_error_deg": result.get("direction_error_deg"),
        "rotation_deg": result.get("rotation_deg"),
        "final_speed": result.get("final_speed"),
        "mean_contacts": result.get("mean_contacts"),
        "min_inter_agent_distance": result.get("min_inter_agent_distance"),
        "min_signed_clearance": result.get("min_signed_clearance"),
        "clearance_path_min": result.get("clearance_path_min"),
        "clearance_lower_bound_min": result.get("clearance_lower_bound_min"),
        "clearance_sampled_min": result.get("clearance_sampled_min"),
        "safety_evaluation_status": result.get("safety_evaluation_status"),
        "actual_violation_observed": result.get("actual_violation_observed"),
        "mean_robot_displacement": result.get("mean_robot_displacement"),
        "H_initial": result.get("H_initial"),
        "H_final": result.get("H_final"),
        "r_safe": result.get("r_safe"),
        "initial_state_sha256": result.get("initial_state_sha256"),
        "config_sha256": result.get("config_sha256"),
        "qp_intervention_fraction": result.get("qp_intervention_fraction"),
        "failure_reasons": " | ".join(result.get("failure_reasons") or []),
        "error": result.get("error"),
        "wall_seconds": result.get("wall_seconds"),
        "directory": result.get("_dir"),
    }


def condition_key(row: dict) -> tuple:
    return (row["batch"], row["shape"], row["seed"], row["stratum"], row.get("condition_id") or "")


def pair_rows(episodes: list[dict], left: str, right: str) -> list[dict]:
    grouped = defaultdict(dict)
    for row in episodes:
        if row["variant"] in (left, right):
            key = condition_key(row)
            if row["variant"] in grouped[key]:
                raise ValueError(f"Duplicate episode for condition {key}, method {row['variant']}")
            grouped[key][row["variant"]] = row
    pairs = []
    for key, methods in sorted(grouped.items()):
        if left not in methods or right not in methods:
            continue
        a, b = methods[left], methods[right]
        both_ready = not a["controller_ready_censored"] and not b["controller_ready_censored"]
        both_physical = not a["physical_ready_censored"] and not b["physical_ready_censored"]
        pairs.append(
            {
                "batch": key[0],
                "shape": key[1],
                "seed": key[2],
                "stratum": key[3],
                "condition_id": key[4],
                "left": left,
                "right": right,
                "ready_difference_seconds": (
                    a["controller_ready_seconds"] - b["controller_ready_seconds"] if both_ready else None
                ),
                "ready_pair_status": "both_observed" if both_ready else "censored",
                "physical_difference_seconds": (
                    a["physical_ready_seconds"] - b["physical_ready_seconds"] if both_physical else None
                ),
                "physical_pair_status": "both_observed" if both_physical else "censored",
                "absolute_progress_difference": _subtract(a["absolute_progress_error"], b["absolute_progress_error"]),
                "endpoint_difference": _subtract(a["euclidean_endpoint_error"], b["euclidean_endpoint_error"]),
                "cross_track_difference": _subtract(a["max_cross_track"], b["max_cross_track"]),
                "left_task_completed": a["task_completed"],
                "right_task_completed": b["task_completed"],
                "left_status": a["execution_status"],
                "right_status": b["execution_status"],
            }
        )
    return pairs


def _subtract(left, right):
    if left is None or right is None:
        return None
    return float(left) - float(right)


def _bool_count(rows: list[dict], field: str) -> dict:
    usable = [row for row in rows if row.get(field) is not None]
    interval = wilson_interval(sum(bool(row[field]) for row in usable), len(usable))
    interval["missing"] = len(rows) - len(usable)
    interval["n_seed_clusters"] = len({row.get("seed") for row in usable})
    interval["interval_status"] = "WILSON_DISTINCT_SEEDS"
    if interval["n_seed_clusters"] < len(usable):
        interval["low"] = interval["high"] = None
        interval["interval_status"] = "DESCRIPTIVE_ONLY_REUSED_SEEDS"
    return interval


def _batch_summary(episodes: list[dict], pairs: list[dict]) -> dict:
    by_shape = defaultdict(list)
    for row in episodes:
        by_shape[row["shape"]].append(row)
    arrival = [row for row in episodes if row["variant"] == "boundary_cvt"]
    spacing_pairs = pair_rows(episodes, "boundary_cvt", "local_spacing")
    return {
        "episodes": len(episodes),
        "execution_errors": sum(row["execution_status"] != "COMPLETED" for row in episodes),
        "task_completed": _bool_count(episodes, "task_completed"),
        "settled": _bool_count(episodes, "settled"),
        "g500_pass": _bool_count(episodes, "g500_pass"),
        "by_method": {
            method: {"n": len(rows), "task_completed": _bool_count(rows, "task_completed"),
                     "settled": _bool_count(rows, "settled"), "g500_pass": _bool_count(rows, "g500_pass"),
                     "controller_readiness": _arrival(rows, "controller_ready_seconds", "controller_ready_censored")}
            for method in sorted({row["variant"] for row in episodes})
            for rows in [[row for row in episodes if row["variant"] == method]]
        },
        "static_by_arm": {
            arm: {"n": len(rows),
                  "safe": sum(row["safety_evaluation_status"] == "CERTIFIED_SAFE" for row in rows),
                  "observed_violations": sum(row["safety_evaluation_status"] == "OBSERVED_VIOLATION" for row in rows),
                  "minimum_path_clearance": min((row["clearance_path_min"] for row in rows if row["clearance_path_min"] is not None), default=None),
                  "mean_cost_ratio": float(np.mean([row["H_final"] / row["H_initial"] for row in rows]))}
            for arm in sorted({row["arm"] for row in episodes if row.get("H_initial")})
            for rows in [[row for row in episodes if row["arm"] == arm and row.get("H_initial")]]
        },
        "by_shape": {
            shape: {
                "n": len(rows),
                "task_completed": _bool_count(rows, "task_completed"),
                "settled": _bool_count(rows, "settled"),
                "g500_pass": _bool_count(rows, "g500_pass"),
            }
            for shape, rows in sorted(by_shape.items())
        },
        "controller_readiness_boundary_cvt": _arrival(arrival, "controller_ready_seconds", "controller_ready_censored"),
        "paired_controller_ready_cvt_minus_nearest": paired_bootstrap(
            [row["ready_difference_seconds"] for row in pairs if row["ready_difference_seconds"] is not None]
        ),
        "paired_controller_ready_cvt_minus_spacing": paired_bootstrap(
            [row["ready_difference_seconds"] for row in spacing_pairs if row["ready_difference_seconds"] is not None]
        ),
        "seed_cluster_ready_cvt_minus_nearest": seed_cluster_bootstrap(pairs),
        "seed_cluster_ready_cvt_minus_spacing": seed_cluster_bootstrap(spacing_pairs),
        "paired_ready_censored_pairs": sum(row["ready_pair_status"] != "both_observed" for row in pairs),
        "paired_physical_ready_cvt_minus_nearest": paired_bootstrap(
            [row["physical_difference_seconds"] for row in pairs if row["physical_difference_seconds"] is not None]
        ),
        "paired_absolute_progress_cvt_minus_nearest": paired_bootstrap(
            [row["absolute_progress_difference"] for row in pairs if row["absolute_progress_difference"] is not None]
        ),
        "analysis_unit": "episode for rates; matched shape/seed/stratum pair for differences",
        "bootstrap_seed": BOOTSTRAP_SEED,
        "censoring": "unreached events excluded from means; their count is reported separately",
    }


def summarize(episodes: list[dict], pairs: list[dict]) -> dict:
    batches = sorted({row["batch"] for row in episodes})
    return {
        "note": "Rates are computed inside each batch. Batches are not pooled, because they repeat conditions and mix static safety with transport.",
        "batches": {
            batch: _batch_summary(
                [row for row in episodes if row["batch"] == batch],
                [row for row in pairs if row["batch"] == batch],
            )
            for batch in batches
        },
    }


def _arrival(rows: list[dict], value_field: str, censor_field: str) -> dict:
    observed = [row[value_field] for row in rows if not row[censor_field] and row[value_field] is not None]
    censored = sum(bool(row[censor_field]) for row in rows)
    return {
        "n": len(rows),
        "observed": len(observed),
        "censored": censored,
        "unreached_proportion": None if not rows else censored / len(rows),
        "conditional_mean_seconds": None if not observed else float(np.mean(observed)),
        "conditional_median_seconds": None if not observed else float(np.median(observed)),
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys()) if rows else ["empty"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _finite(rows, field):
    return [(row, float(row[field])) for row in rows if isinstance(row.get(field), (int, float))]


TRANSPORT_VARIANTS = {"boundary_cvt", "nearest_observed_target", "local_spacing"}


def _focus(rows: list[dict], field: str = "batch") -> list[dict]:
    if any(row.get(field) == "development" for row in rows):
        return [row for row in rows if row.get(field) == "development"]
    if any(row.get(field) == "test" for row in rows):
        return [row for row in rows if row.get(field) == "test"]
    return rows


def plot_readiness(pairs: list[dict], path: Path, focus: str | None = None) -> None:
    pairs = [row for row in pairs if row["batch"] == focus] if focus else _focus(pairs)
    observed = [row["ready_difference_seconds"] for row in pairs if row["ready_difference_seconds"] is not None]
    censored = sum(row["ready_pair_status"] != "both_observed" for row in pairs)
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    if observed:
        ax.axvline(0.0, color="#666666", lw=1)
        ax.hist(observed, bins=min(12, max(4, len(observed))), color="#2c6eab", edgecolor="white")
    ax.set_xlabel("Controller CONTACT_READY difference (CVT − nearest), seconds")
    ax.set_ylabel("Matched pairs")
    stats = paired_bootstrap(observed)
    ax.set_title(
        f"n pairs={len(pairs)}, both arrived={len(observed)}, censored={censored}\n"
        f"conditional mean {None if stats['mean'] is None else round(stats['mean'], 3)} s, "
        f"median {None if stats['median'] is None else round(stats['median'], 3)} s"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_rates(episodes: list[dict], path: Path, focus: str | None = None) -> None:
    chosen = [row for row in episodes if row["batch"] == focus] if focus else _focus(episodes)
    episodes = [row for row in chosen if row["variant"] in TRANSPORT_VARIANTS]
    groups = sorted({(row["shape"], row["variant"]) for row in episodes if row["shape"]})
    metrics = ("task_completed", "g500_pass", "settled")
    fig, ax = plt.subplots(figsize=(12, 4.8))
    width = 0.25
    x = np.arange(len(groups))
    for index, metric in enumerate(metrics):
        rates, lows, highs = [], [], []
        for shape, method in groups:
            rows = [row for row in episodes if row["shape"] == shape and row["variant"] == method]
            interval = _bool_count(rows, metric)
            rate = 0.0 if interval["rate"] is None else interval["rate"]
            rates.append(rate)
            low = rate if interval["low"] is None else interval["low"]
            high = rate if interval["high"] is None else interval["high"]
            lows.append(max(0.0, rate - low))
            highs.append(max(0.0, high - rate))
        ax.bar(x + (index - 1) * width, rates, width, yerr=np.vstack([lows, highs]), capsize=3, label=f"{metric}")
    labels = []
    for shape, method in groups:
        n = sum(row["shape"] == shape and row["variant"] == method for row in episodes)
        labels.append(f"{shape}\n{method}\nn={n}")
    ax.set_xticks(x, labels, rotation=35, ha="right", fontsize=7)
    ax.set_ylim(0, 1)
    ax.set_ylabel("Task rate; Wilson intervals only for distinct-seed groups")
    ax.legend()
    ax.set_title("Methods reported separately; completion, G500, and settling are distinct")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_errors(episodes: list[dict], path: Path, focus: str | None = None) -> None:
    chosen = [row for row in episodes if row["batch"] == focus] if focus else _focus(episodes)
    episodes = [row for row in chosen if row["variant"] in TRANSPORT_VARIANTS]
    fields = (
        ("absolute_progress_error", "Absolute progress error (m)"),
        ("euclidean_endpoint_error", "Euclidean endpoint error (m)"),
        ("max_cross_track", "Max cross-track (m)"),
    )
    variants = [name for name in ("boundary_cvt", "nearest_observed_target", "local_spacing") if any(row["variant"] == name for row in episodes)]
    fig, axes = plt.subplots(1, len(fields), figsize=(11.5, 4.2), sharey=False)
    for ax, (field, title) in zip(axes, fields):
        for index, variant in enumerate(variants):
            rows = [row for row in episodes if row["variant"] == variant]
            values = _finite(rows, field)
            ys = [value for _, value in values]
            watchdogs = [bool(row.get("watchdog")) for row, _ in values]
            x = np.full(len(ys), index, dtype=float)
            x = x + np.linspace(-0.12, 0.12, len(ys) or 1)[: len(ys)]
            colors = ["#b33a3a" if flag else "#24527a" for flag in watchdogs]
            ax.scatter(x, ys, c=colors, s=28, zorder=3)
        ax.set_xticks(range(len(variants)), [name.replace("_", "\n") for name in variants], fontsize=8)
        ax.set_title(title, fontsize=10)
    fig.suptitle("Red points reached the watchdog. Outliers are retained.")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _goal(directory: Path):
    result_path = directory / "case_result.json"
    if not result_path.exists():
        return None
    result = _load_json(result_path).get("result", {})
    goal = result.get("goal_direction")
    return None if goal is None else np.asarray(goal, dtype=float)


def plot_contacts(episodes: list[dict], path: Path, focus: str | None = None) -> None:
    pool = [row for row in episodes if row["batch"] == focus] if focus else _focus(episodes)
    completed = [
        row
        for row in pool
        if row["variant"] == "boundary_cvt" and row["task_completed"] is True and row["directory"]
    ]
    failures = [
        row
        for row in pool
        if row["variant"] == "boundary_cvt" and row["task_completed"] is False and row["directory"]
    ]
    success = sorted(completed, key=lambda row: row["case_id"])[:1]
    chosen = [("failure", sorted(failures, key=lambda row: row["case_id"])[:1]), ("success", success)]
    fig, axes = plt.subplots(2, 1, figsize=(8, 6.2), sharex=False)
    plotted = 0
    for label, rows in chosen:
        if not rows:
            continue
        row = rows[0]
        directory = Path(row["directory"])
        series = directory / "safety_timeseries.csv"
        if not series.exists():
            continue
        data = np.genfromtxt(series, delimiter=",", names=True)
        goal = _goal(directory)
        forward = data["net_force_x"]
        if goal is not None and goal.size == 2:
            forward = data["net_force_x"] * goal[0] + data["net_force_y"] * goal[1]
        axes[0].plot(data["time"], data["contacts"], label=f"{label}: {row['case_id']}")
        axes[1].plot(data["time"], forward, label=f"forward force {label}")
        axes[1].plot(data["time"], data["net_torque"], linestyle="--", label=f"torque {label}")
        plotted += 1
    axes[0].set_ylabel("Contact count")
    axes[1].set_ylabel("Force (N) / torque")
    axes[1].set_xlabel("Time (s)")
    if plotted:
        axes[0].legend(fontsize=7)
        axes[1].legend(fontsize=7)
    axes[0].set_title("First lexicographic CVT task failure and success in this batch" if plotted else "No selected timeseries")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_safety(episodes: list[dict], path: Path) -> None:
    rows = [row for row in episodes if row.get("clearance_path_min") is not None]
    fig, ax = plt.subplots(figsize=(8.4, 4.4))
    if rows:
        labels = [f"{row['arm']}\n{row['shape'][:3]}:{row['seed']}" for row in rows]
        x = np.arange(len(rows))
        path_mm = [1000 * float(row["clearance_path_min"]) for row in rows]
        bound_x, bound_y = [], []
        for index, row in enumerate(rows):
            if row["clearance_lower_bound_min"] is not None:
                bound_x.append(index)
                bound_y.append(1000 * float(row["clearance_lower_bound_min"]))
        ax.scatter(x, path_mm, label="path clearance", color="#24527a")
        ax.scatter(bound_x, bound_y, label="lower bound", color="#c47b2b", marker="x")
        ax.axhline(65.0, color="#b33a3a", lw=1, label="static 65 mm threshold")
        ax.set_xticks(x, labels, fontsize=6)
    ax.set_ylabel("Clearance (mm)")
    ax.set_title(f"Static path clearance versus lower bound, n={len(rows)}")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_static_quality(episodes: list[dict], path: Path) -> None:
    rows = [row for row in episodes if row.get("H_initial") and row.get("H_final") is not None]
    arms = sorted({row["arm"] for row in rows})
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for index, arm in enumerate(arms):
        group = [row for row in rows if row["arm"] == arm]
        x = index + np.linspace(-0.1, 0.1, len(group))
        axes[0].scatter(x, [r["H_final"] / r["H_initial"] for r in group], label=arm)
        axes[1].scatter(x, [r["mean_robot_displacement"] for r in group], label=arm)
    axes[0].axhline(1, color="gray", linestyle="--")
    axes[0].set_ylabel("Final / initial independent deployment cost")
    axes[1].set_ylabel("Mean robot displacement (m)")
    for ax in axes:
        ax.set_xticks(range(len(arms)), arms, rotation=25, ha="right", fontsize=8)
    fig.suptitle("Static safety must be read alongside deployment quality and motion")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_ablation(episodes: list[dict], path: Path) -> None:
    reference = {
        (row["shape"], row["seed"]): row
        for row in episodes
        if row["batch"] == "development" and row["variant"] == "boundary_cvt" and row["seed"] == 2
    }
    ablations = [row for row in episodes if row["batch"] == "ablation"]
    labels, ready, progress = [], [], []
    for row in ablations:
        base = reference.get((row["shape"], row["seed"]))
        if base is None or row["controller_ready_censored"] or base["controller_ready_censored"]:
            continue
        labels.append(row["case_id"].replace("ablation_", ""))
        ready.append(row["controller_ready_seconds"] - base["controller_ready_seconds"])
        progress.append(_subtract(row["absolute_progress_error"], base["absolute_progress_error"]))
    fig, ax = plt.subplots(figsize=(9.5, 4.6))
    y = np.arange(len(labels))
    ax.barh(y - 0.15, ready, height=0.3, label="readiness difference (s)")
    finite_progress = [0.0 if value is None else value for value in progress]
    ax.barh(y + 0.15, finite_progress, height=0.3, label="absolute progress difference (m)")
    ax.axvline(0.0, color="#666666", lw=1)
    ax.set_yticks(y, labels, fontsize=7)
    ax.legend(fontsize=8)
    ax.set_title(f"Ablation minus full boundary CVT, seed 2, n={len(labels)}")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def failure_taxonomy(episodes: list[dict]) -> dict:
    counts = defaultdict(int)
    for row in episodes:
        if row["execution_status"] != "COMPLETED":
            counts["execution_or_contract_error"] += 1
            continue
        if row["watchdog"]:
            counts["watchdog"] += 1
        if row["g500_pass"] is False:
            counts["g500_not_passed"] += 1
        if row["task_completed"] is False:
            counts["task_not_completed"] += 1
        if row["safety_evaluation_status"] == "OBSERVED_VIOLATION":
            counts["observed_static_violation"] += 1
        if row["controller_ready_censored"]:
            counts["controller_ready_censored"] += 1
    return {"episodes": len(episodes), "counts": dict(counts)}


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--batch", action="append", required=True, help="label=directory")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--by-split", action="store_true", help="Report each recorded protocol split separately")
    args = parser.parse_args(argv)
    episodes = []
    for item in args.batch:
        label, raw = item.split("=", 1)
        for result in iter_results(Path(raw)):
            episodes.append(episode_row(result, result.get("split") if args.by_split else label))
    pairs = pair_rows(episodes, "boundary_cvt", "nearest_observed_target")
    summary = summarize(episodes, pairs)
    summary["failure_taxonomy"] = {
        batch: failure_taxonomy([row for row in episodes if row["batch"] == batch])
        for batch in sorted({row["batch"] for row in episodes})
    }
    args.out.mkdir(parents=True, exist_ok=True)
    write_csv(args.out / "episodes.csv", episodes)
    write_csv(args.out / "pairs.csv", pairs)
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    tags = [batch for batch in ("development", "test") if any(row["batch"] == batch for row in episodes)]
    if args.by_split:
        tags = sorted({row["batch"] for row in episodes})
    if not tags:
        tags = [None]
    for tag in tags:
        suffix = f"_{tag}" if tag and len(tags) > 1 else ""
        plot_readiness(pairs, args.out / f"fig_readiness_pairs{suffix}.png", focus=tag)
        plot_rates(episodes, args.out / f"fig_completion_rates{suffix}.png", focus=tag)
        plot_errors(
            [row for row in episodes if row["execution_status"] == "COMPLETED"],
            args.out / f"fig_error_distributions{suffix}.png",
            focus=tag,
        )
        plot_contacts(episodes, args.out / f"fig_contact_force{suffix}.png", focus=tag)
    plot_safety(episodes, args.out / "fig_safety_clearance.png")
    plot_static_quality(episodes, args.out / "fig_static_quality.png")
    plot_ablation(episodes, args.out / "fig_ablation.png")
    print(json.dumps({"episodes": len(episodes), "pairs": len(pairs), "out": str(args.out)}, indent=2))


if __name__ == "__main__":
    main()
