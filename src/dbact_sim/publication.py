"""Deterministic case selection and provenance for publication artefacts.

The selection code reads the frozen Claude v2 result table.  It does not run,
rank, or tune the controller.  Its rules are intentionally narrow and recorded
in the output manifest so the showcase cannot silently become a highlight reel.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import platform
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import scipy


FLOAT_COLUMNS = {
    "alpha",
    "concavity_ratio",
    "diameter_m",
    "target_distance_m",
    "J",
    "J_over_target",
    "efficiency",
    "direction_error_deg",
    "max_cross_track",
    "max_cross_track_over_diameter",
    "max_strict_coverage",
    "final_strict_coverage",
    "min_inter_agent_distance",
    "d_min",
    "max_penetration",
    "penetration_budget",
    "frames_run",
    "hold_frame",
}
BOOLEAN_COLUMNS = {
    "success",
    "domain_eligible",
    "runtime_domain_eligible",
    "map_complete",
    "settled",
}

SUCCESS_MEDOID_FEATURES = (
    "concavity_ratio",
    "J_over_target",
    "efficiency",
    "max_cross_track_over_diameter",
    "frames_run",
)


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def environment_fingerprint() -> dict[str, str]:
    """Versions that can change long-horizon floating-point trajectories."""
    try:
        import matplotlib
    except ImportError:  # pragma: no cover - a renderer environment has it
        matplotlib_version = "unavailable"
    else:
        matplotlib_version = str(matplotlib.__version__)
    return {
        "python": sys.version.split()[0],
        "implementation": platform.python_implementation(),
        "platform": platform.platform(),
        "numpy": str(np.__version__),
        "scipy": str(scipy.__version__),
        "matplotlib": matplotlib_version,
    }


def load_episode_rows(path: str | Path) -> list[dict[str, Any]]:
    """Load a matrix CSV with explicit numeric and Boolean types."""
    with Path(path).open(newline="", encoding="utf-8") as handle:
        rows = [dict(row) for row in csv.DictReader(handle)]
    for row in rows:
        for key in FLOAT_COLUMNS:
            value = row.get(key, "")
            row[key] = float(value) if value not in {None, ""} else None
        for key in BOOLEAN_COLUMNS:
            value = str(row.get(key, "")).strip().lower()
            row[key] = value in {"1", "true", "yes"}
        row["seed"] = int(row["seed"])
        for key in ("solver_fallbacks", "solver_infeasible"):
            row[key] = int(row.get(key) or 0)
    return rows


def _median_alpha(rows: Iterable[dict[str, Any]]) -> float:
    values = sorted({float(row["alpha"]) for row in rows})
    if not values:
        raise ValueError("the episode table is empty")
    return float(np.median(values))


def _robust_medoid(
    rows: list[dict[str, Any]],
    features: tuple[str, ...],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    matrix = np.asarray([[float(row[key]) for key in features] for row in rows], dtype=float)
    if not np.isfinite(matrix).all():
        raise ValueError("representative-success features must all be finite")
    centre = np.median(matrix, axis=0)
    scale = np.median(np.abs(matrix - centre), axis=0)
    scale = np.where(scale > 1e-12, scale, 1.0)
    distances = np.sqrt(np.sum(((matrix - centre) / scale) ** 2, axis=1))
    ranking = sorted(
        (
            {"case_id": row["case_id"], "robust_distance": float(distance)}
            for row, distance in zip(rows, distances)
        ),
        key=lambda item: (item["robust_distance"], item["case_id"]),
    )
    selected_id = ranking[0]["case_id"]
    selected = next(row for row in rows if row["case_id"] == selected_id)
    return selected, ranking


def select_publication_cases(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Select one auditable success and one structural high-concavity failure.

    The success is the robust medoid of eligible non-convex successes at the
    matrix's median task-distance factor.  The failure is the median-progress
    member of the maximum-concavity eligible TRANSPORT_STALL group at that same
    factor.  Case identifiers break any exact tie.
    """
    alpha = _median_alpha(rows)
    success_pool = [
        row
        for row in rows
        if row["success"]
        and row.get("failure_class") == "SUCCESS"
        and row["runtime_domain_eligible"]
        and math.isclose(float(row["alpha"]), alpha, abs_tol=1e-12)
        and float(row["concavity_ratio"] or 0.0) > 0.05
    ]
    if not success_pool:
        raise ValueError("no eligible non-convex success exists at the median alpha")
    success, success_ranking = _robust_medoid(success_pool, SUCCESS_MEDOID_FEATURES)

    stall_pool = [
        row
        for row in rows
        if not row["success"]
        and row.get("failure_class") == "TRANSPORT_STALL"
        and row["runtime_domain_eligible"]
        and math.isclose(float(row["alpha"]), alpha, abs_tol=1e-12)
    ]
    if not stall_pool:
        raise ValueError("no eligible transport stall exists at the median alpha")
    maximum_concavity = max(float(row["concavity_ratio"]) for row in stall_pool)
    failure_pool = [
        row
        for row in stall_pool
        if math.isclose(
            float(row["concavity_ratio"]), maximum_concavity, rel_tol=0.0, abs_tol=1e-12
        )
    ]
    progress_median = float(np.median([float(row["J_over_target"]) for row in failure_pool]))
    failure_ranking = sorted(
        (
            {
                "case_id": row["case_id"],
                "distance_to_group_median_J_over_target": abs(
                    float(row["J_over_target"]) - progress_median
                ),
            }
            for row in failure_pool
        ),
        key=lambda item: (
            item["distance_to_group_median_J_over_target"],
            item["case_id"],
        ),
    )
    failure_id = failure_ranking[0]["case_id"]
    failure = next(row for row in failure_pool if row["case_id"] == failure_id)

    return {
        "selection_schema_version": 1,
        "median_alpha": alpha,
        "success_rule": (
            "robust medoid of runtime-eligible, failure_class=SUCCESS, non-convex "
            "(concavity_ratio>0.05) cases at median alpha; features="
            + ",".join(SUCCESS_MEDOID_FEATURES)
        ),
        "failure_rule": (
            "median J/L member of the maximum-concavity runtime-eligible "
            "TRANSPORT_STALL group at median alpha"
        ),
        "success_pool_size": len(success_pool),
        "failure_stall_pool_size": len(stall_pool),
        "failure_max_concavity_pool_size": len(failure_pool),
        "success_ranking": success_ranking,
        "failure_ranking": failure_ranking,
        "cases": {
            "representative_success": _case_record(success),
            "high_concavity_failure": _case_record(failure),
        },
    }


def _case_record(row: dict[str, Any]) -> dict[str, Any]:
    keys = (
        "case_id",
        "shape",
        "seed",
        "alpha",
        "concavity_ratio",
        "target_distance_m",
        "J",
        "J_over_target",
        "efficiency",
        "direction_error_deg",
        "max_cross_track_over_diameter",
        "max_strict_coverage",
        "final_strict_coverage",
        "min_inter_agent_distance",
        "d_min",
        "max_penetration",
        "penetration_budget",
        "solver_fallbacks",
        "solver_infeasible",
        "frames_run",
        "final_phase",
        "success",
        "runtime_domain_eligible",
        "failure_class",
    )
    return {key: row.get(key) for key in keys}


def build_selection_manifest(
    episodes_path: str | Path,
    matrix_manifest_path: str | Path,
    rows: list[dict[str, Any]],
    selection: dict[str, Any],
) -> dict[str, Any]:
    """Bind the selected cases to the complete frozen experiment denominator."""
    matrix_manifest = json.loads(Path(matrix_manifest_path).read_text(encoding="utf-8"))
    successes = sum(bool(row["success"]) for row in rows)
    eligible = [row for row in rows if row["runtime_domain_eligible"]]
    eligible_successes = sum(bool(row["success"]) for row in eligible)
    return {
        "schema_version": 1,
        "purpose": "traceable success/failure publication showcase; not a new experiment",
        "source": {
            "episodes_path": str(Path(episodes_path)),
            "episodes_sha256": sha256_file(episodes_path),
            "matrix_manifest_path": str(Path(matrix_manifest_path)),
            "matrix_manifest_sha256": sha256_file(matrix_manifest_path),
            "research_git_sha": matrix_manifest.get("git_sha"),
            "config_path": matrix_manifest.get("config_path"),
            "config_sha256": matrix_manifest.get("config_sha256"),
            "screening": matrix_manifest.get("screening"),
            "environment_fingerprint": matrix_manifest.get("environment_fingerprint"),
            "environment_fingerprint_available": bool(
                matrix_manifest.get("environment_fingerprint")
            ),
        },
        "current_environment": environment_fingerprint(),
        "denominator": {
            "episodes": len(rows),
            "successes": successes,
            "P_success": successes / len(rows) if rows else None,
            "runtime_eligible": len(eligible),
            "runtime_eligible_successes": eligible_successes,
            "P_success_given_runtime_eligible": (
                eligible_successes / len(eligible) if eligible else None
            ),
            "failure_composition": dict(Counter(row.get("failure_class") for row in rows)),
        },
        "selection": selection,
        "reruns": {},
        "artifacts": [],
        "publication_eligible": None,
        "blocking_reasons": [],
    }


def compare_rerun(source: dict[str, Any], observed: dict[str, Any]) -> dict[str, Any]:
    """Compare a rendered rerun with its frozen record without hiding drift."""
    exact_keys = (
        "case_id",
        "success",
        "frames_run",
        "solver_fallbacks",
        "solver_infeasible",
        "final_phase",
    )
    numeric_keys = (
        "J",
        "J_over_target",
        "efficiency",
        "max_cross_track",
        "max_strict_coverage",
        "final_strict_coverage",
        "min_inter_agent_distance",
        "max_penetration",
    )
    mismatches: list[dict[str, Any]] = []
    for key in exact_keys:
        if source.get(key) != observed.get(key):
            mismatches.append(
                {"field": key, "source": source.get(key), "observed": observed.get(key)}
            )
    for key in numeric_keys:
        expected = source.get(key)
        actual = observed.get(key)
        if expected is None or actual is None or not math.isclose(
            float(expected), float(actual), rel_tol=1e-10, abs_tol=1e-10
        ):
            mismatches.append({"field": key, "source": expected, "observed": actual})
    return {
        "passed": not mismatches,
        "exact_fields": list(exact_keys),
        "numeric_fields": list(numeric_keys),
        "relative_tolerance": 1e-10,
        "absolute_tolerance": 1e-10,
        "mismatches": mismatches,
    }


def verify_rerun(source: dict[str, Any], observed: dict[str, Any]) -> dict[str, Any]:
    """Fail closed if a rendered rerun drifts from its frozen CSV record."""
    comparison = compare_rerun(source, observed)
    if not comparison["passed"]:
        detail = "; ".join(
            f"{item['field']}: source={item['source']!r}, observed={item['observed']!r}"
            for item in comparison["mismatches"]
        )
        raise RuntimeError("publication rerun drifted from the frozen matrix: " + detail)
    return comparison


__all__ = [
    "build_selection_manifest",
    "compare_rerun",
    "environment_fingerprint",
    "load_episode_rows",
    "select_publication_cases",
    "sha256_file",
    "verify_rerun",
]
