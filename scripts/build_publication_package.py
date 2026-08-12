#!/usr/bin/env python3
"""Build the predeclared DBACT success/failure publication package.

The script first selects cases from the complete frozen matrix.  Unless
``--select-only`` is used, it then reruns exactly those two cases, fails closed
on metric drift, and writes immutable traces, MP4s, Figures A--H, and a
hash-indexed publication manifest.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.publication import (  # noqa: E402
    build_selection_manifest,
    compare_rerun,
    load_episode_rows,
    select_publication_cases,
    sha256_file,
    verify_rerun,
)
from dbact_sim.scenarios import load_yaml  # noqa: E402
from dbact_sim.trace import SimulationTrace, VisualizationRecorder  # noqa: E402
from dbact_sim.visualization import (  # noqa: E402
    render_animation,
    write_outcome_comparison,
    write_research_paper_figures,
)

# The matrix generator is a script rather than an importable package.  Importing
# it here reuses the exact frozen shape/yaw/scale construction without moving or
# duplicating research logic into the visualization layer.
from run_arbitrary_shape_monte_carlo import build_case_config  # noqa: E402


ROLE_NAMES = ("representative_success", "high_concavity_failure")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", default="docs/results/v2_shape_matrix/episodes.csv")
    parser.add_argument("--matrix-manifest", default="docs/results/v2_shape_matrix/manifest.json")
    parser.add_argument("--config", default="configs/sim/v2/shape_matrix.yaml")
    parser.add_argument("--output", default="runs/publication_showcase")
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--recorder-stride", type=int, default=5)
    parser.add_argument("--frame-stride", type=int, default=10)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--dpi", type=int, default=95)
    parser.add_argument("--paper-dpi", type=int, default=220)
    parser.add_argument("--paper-formats", default="png,pdf,svg")
    parser.add_argument("--select-only", action="store_true")
    parser.add_argument(
        "--reuse-existing",
        action="store_true",
        help="Reuse trace/video files already under --output; never rerun the controller.",
    )
    parser.add_argument(
        "--allow-numeric-drift-preview",
        action="store_true",
        help="Write a preview_only manifest instead of failing on frozen-metric drift.",
    )
    args = parser.parse_args()

    episodes_path = Path(args.episodes)
    matrix_manifest_path = Path(args.matrix_manifest)
    rows = load_episode_rows(episodes_path)
    by_id = {row["case_id"]: row for row in rows}
    selection = select_publication_cases(rows)
    manifest = build_selection_manifest(
        episodes_path, matrix_manifest_path, rows, selection
    )

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    if args.select_only:
        _write_manifest(output / "publication_manifest.json", manifest)
        _print_selection(selection)
        return 0

    formats = tuple(
        item.strip().lower() for item in args.paper_formats.split(",") if item.strip()
    )
    base = load_yaml(args.config)
    traces: dict[str, SimulationTrace] = {}
    comparisons: dict[str, dict[str, Any]] = {}
    displayed_records: dict[str, dict[str, Any]] = {}
    for role in ROLE_NAMES:
        selected = selection["cases"][role]
        source = by_id[selected["case_id"]]
        if args.reuse_existing:
            trace, observed, render_report = _reuse_case(
                role=role,
                source=source,
                output=output / role,
                frame_stride=args.frame_stride,
                fps=args.fps,
            )
        else:
            trace, observed, render_report = _run_and_render_case(
                role=role,
                source=source,
                base_config=base,
                output=output / role,
                max_frames=args.max_frames,
                recorder_stride=args.recorder_stride,
                frame_stride=args.frame_stride,
                fps=args.fps,
                dpi=args.dpi,
                paper_dpi=args.paper_dpi,
                formats=formats,
            )
        traces[role] = trace
        comparison = compare_rerun(source, observed)
        comparisons[role] = comparison
        displayed_records[role] = {**selected, **observed}
        if not comparison["passed"] and not args.allow_numeric_drift_preview:
            verify_rerun(source, observed)
        manifest["reruns"][role] = {
            "case_id": source["case_id"],
            "observed": observed,
            "source_match": comparison,
            "render_report": render_report,
        }

    comparison_paths = write_outcome_comparison(
        traces["representative_success"],
        traces["high_concavity_failure"],
        output / "comparison",
        success_record=displayed_records["representative_success"],
        failure_record=displayed_records["high_concavity_failure"],
        frozen_success_record=selection["cases"]["representative_success"],
        frozen_failure_record=selection["cases"]["high_concavity_failure"],
        preview_only=(
            not manifest["source"]["environment_fingerprint_available"]
            or any(not item["passed"] for item in comparisons.values())
        ),
        formats=formats,
        dpi=args.paper_dpi,
    )
    manifest["comparison"] = [str(path.relative_to(output)) for path in comparison_paths]
    if not manifest["source"]["environment_fingerprint_available"]:
        manifest["blocking_reasons"].append(
            "the frozen 180-case matrix did not record Python, OS, NumPy, or SciPy versions"
        )
    for role, comparison in comparisons.items():
        if not comparison["passed"]:
            manifest["blocking_reasons"].append(
                f"{role} continuous metrics drift from the frozen CSV in the current environment"
            )
    manifest["publication_eligible"] = not manifest["blocking_reasons"]
    manifest["status"] = (
        "publication_ready" if manifest["publication_eligible"] else "preview_only"
    )
    manifest["artifacts"] = _artifact_index(output)
    _write_manifest(output / "publication_manifest.json", manifest)
    _print_selection(selection)
    print(f"Publication package: {output.resolve()}")
    return 0


def _run_and_render_case(
    *,
    role: str,
    source: dict[str, Any],
    base_config: dict[str, Any],
    output: Path,
    max_frames: int,
    recorder_stride: int,
    frame_stride: int,
    fps: float,
    dpi: int,
    paper_dpi: int,
    formats: tuple[str, ...],
) -> tuple[SimulationTrace, dict[str, Any], dict[str, Any]]:
    config, metadata = build_case_config(
        base_config,
        str(source["shape"]),
        int(source["seed"]),
        float(source["alpha"]),
    )
    expected_id = f"{metadata['shape']}__a{metadata['alpha']:.2f}__seed{source['seed']:03d}"
    if expected_id != source["case_id"]:
        raise RuntimeError(f"case reconstruction mismatch: {expected_id} != {source['case_id']}")

    env = SimulationEnvironment(config, seed=int(source["seed"]))
    recorder = VisualizationRecorder(stride=recorder_stride, sensor_ray_stride=5)
    started = time.perf_counter()
    termination = env.run_until_settled(
        max_frames=max_frames,
        on_frame=recorder.capture,
    )
    wall_seconds = time.perf_counter() - started
    simulation_fps = int(termination["frames_run"]) / max(wall_seconds, 1e-12)
    trace = SimulationTrace.from_environment(env, recorder, simulation_fps=simulation_fps)

    output.mkdir(parents=True, exist_ok=True)
    trace.save(output / "trace")
    env.save_replay(output / "replay.npz")
    written = write_research_paper_figures(
        trace,
        output / "paper_figures",
        formats=formats,
        dpi=paper_dpi,
    )
    video_path = output / f"{role}.mp4"
    report = render_animation(
        trace,
        video_path,
        view_mode="demo",
        frame_stride=frame_stride,
        fps=fps,
        dpi=dpi,
    )
    observed = _observed_record(env, termination, source, wall_seconds=wall_seconds)
    render_report = report.as_dict()
    render_report["video"] = str(video_path)
    render_report["paper_figure_files"] = sum(len(paths) for paths in written.values())
    return trace, observed, render_report


def _reuse_case(
    *,
    role: str,
    source: dict[str, Any],
    output: Path,
    frame_stride: int,
    fps: float,
) -> tuple[SimulationTrace, dict[str, Any], dict[str, Any]]:
    trace = SimulationTrace.load(output / "trace")
    observed = _observed_from_trace(trace, source)
    video = output / f"{role}.mp4"
    if not video.exists():
        raise FileNotFoundError(f"existing preview video is missing: {video}")
    rendered_frames = list(range(0, trace.frame_count, max(1, frame_stride)))
    if rendered_frames[-1] != trace.frame_count - 1:
        rendered_frames.append(trace.frame_count - 1)
    report = {
        "video": str(video),
        "source_frames": trace.frame_count,
        "rendered_frames": len(rendered_frames),
        "frame_stride": int(frame_stride),
        "playback_fps": float(fps),
        "simulation_fps": trace.settings.get("simulation_fps"),
        "rendering_fps": None,
        "wall_seconds": None,
        "reused_existing": True,
    }
    return trace, observed, report


def _observed_record(
    env: SimulationEnvironment,
    termination: dict[str, Any],
    source: dict[str, Any],
    *,
    wall_seconds: float,
) -> dict[str, Any]:
    summary = env.summary()
    cargo_id = env.cargoes[0].object_id
    metrics = summary["cargoes"][cargo_id]["g500"]["metrics"]
    verdict = summary["cargoes"][cargo_id]["g500"]
    target = float(env.tasks[cargo_id].distance)
    diameter = float(source["diameter_m"])
    return {
        "case_id": source["case_id"],
        "success": bool(verdict["success"]),
        "frames_run": float(termination["frames_run"]),
        "solver_fallbacks": int(summary["solver"]["fallbacks"]),
        "solver_infeasible": int(summary["solver"]["infeasible"]),
        "final_phase": metrics["final_phase"],
        "J": float(metrics["J"]),
        "J_over_target": float(metrics["J"]) / target,
        "J_over_diameter": float(metrics["J"]) / diameter,
        "efficiency": float(metrics["efficiency"]),
        "max_cross_track": float(metrics["max_cross_track"]),
        "max_strict_coverage": float(metrics["max_strict_coverage"]),
        "final_strict_coverage": float(metrics["final_strict_coverage"]),
        "min_inter_agent_distance": float(metrics["min_inter_agent_distance"]),
        "max_penetration": float(metrics["max_penetration"]),
        "wall_seconds": float(wall_seconds),
    }


def _observed_from_trace(
    trace: SimulationTrace,
    source: dict[str, Any],
) -> dict[str, Any]:
    cargo_id = trace.cargo_ids[0]
    progress = trace.directional_progress[cargo_id]
    return {
        "case_id": source["case_id"],
        "success": trace.phase_labels[-1] == "HOLD",
        "frames_run": float(trace.frame_count - 1),
        "solver_fallbacks": int(max(trace.solver_fallbacks[-1], 0)),
        "solver_infeasible": int(max(trace.solver_infeasible[-1], 0)),
        "final_phase": trace.phase_labels[-1],
        "J": float(progress[-1]),
        "J_over_target": float(trace.progress_ratio[cargo_id][-1]),
        "J_over_diameter": float(progress[-1]) / float(source["diameter_m"]),
        "efficiency": float(trace.direction_efficiency[cargo_id][-1]),
        "max_cross_track": float(np.max(trace.cross_track_error[cargo_id])),
        "max_strict_coverage": float(np.max(trace.strict_coverage[cargo_id])),
        "final_strict_coverage": float(trace.strict_coverage[cargo_id][-1]),
        "min_inter_agent_distance": float(np.min(trace.min_distances)),
        "max_penetration": float(np.max(trace.max_penetration[cargo_id])),
        "wall_seconds": None,
    }


def _artifact_index(output: Path) -> list[dict[str, Any]]:
    manifest_path = output / "publication_manifest.json"
    return [
        {
            "path": str(path.relative_to(output)),
            "bytes": path.stat().st_size,
            "sha256": sha256_file(path),
        }
        for path in sorted(output.rglob("*"))
        if path.is_file() and path != manifest_path
    ]


def _write_manifest(path: Path, manifest: dict[str, Any]) -> None:
    path.write_text(json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _print_selection(selection: dict[str, Any]) -> None:
    for role in ROLE_NAMES:
        case = selection["cases"][role]
        print(f"{role}: {case['case_id']} ({case['failure_class']})")


if __name__ == "__main__":
    raise SystemExit(main())
