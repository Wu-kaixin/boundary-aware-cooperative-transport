#!/usr/bin/env python
"""D10-DIAG - post-discovery redeployment diagnosis.

    PYTHONPATH=src python scripts/diagnose_redeployment.py --seeds 0..7 --out runs/d10_diag

Measure first, explain second, modify third. Three approach heuristics -- ring
bearings, extent-corrected ring bearings, and wall-following at two scout
densities -- were each built and measured against the far-field pipeline and none
beat a go-to-point recall. What they have in common is that they all change where
the robots go *on the way in*, and the 513 frames between detection and
contact-ready are spent by a team that has already arrived. This script does not
change the controller. It records what every robot is doing on every one of those
frames and reports where they go.

Outputs, under ``--out``:

    diagnosis.json      every number below, per seed and aggregated
    diagnosis.md        the tables, ready to paste into the branch document
    figA_segments.png   post-detection stage durations, stacked, per seed
    figB_coverage.png   union map coverage, strict coverage, unobserved arc
    figC_modes.png      per-robot mode raster for the slowest seed
    figD_counts.png     redeploy candidates, visibility and contact vs time
    figE_tracks.png     the slowest seed's tracks, first sight and first far-side sight
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from dbact.diagnosis import (
    SEGMENTS,
    SEGMENT_LABELS,
    FrameRecord,
    SegmentRules,
    arc_of_run,
    backside_samples,
    longest_false_run,
    observed_boundary_mask,
    occupied_boundary_mask,
    segment,
)
from dbact.geometry import polygon_perimeter
from dbact.metrics import min_inter_agent_distance
from dbact.phase import Phase
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml

BOUNDARY_SAMPLES = 160


def parse_seeds(text: str) -> list[int]:
    if ".." in text:
        low, high = text.split("..")
        return list(range(int(low), int(high) + 1))
    return [int(part) for part in text.split(",") if part.strip()]


class SeedTrace:
    """One episode, recorded frame by frame while it runs."""

    def __init__(self, env: SimulationEnvironment):
        self.env = env
        self.controller = env.controller
        self.params = env.controller.params
        self.cargo = env.cargoes[0]
        self.object_id = self.cargo.object_id
        self.perimeter = polygon_perimeter(self.cargo.vertices)
        # A voxel map cannot place a point closer to the surface than half a
        # voxel, so the observation tolerance is sized against the map rather than
        # chosen: anything tighter measures the discretisation.
        self.map_tolerance = 1.5 * self.params.voxel_size
        self.contact_radius = float(env.evaluation_contact_radius)

        self.records: list[FrameRecord] = []
        self.agent_ids = [a.agent_id for a in env.agents]
        self.first_detection: int | None = None
        self.first_detector: str | None = None
        self.backside_mask: np.ndarray | None = None
        self.backside_first: int | None = None
        self.backside_first_agent: str | None = None
        self.detection_position: np.ndarray | None = None

    # ---------------------------------------------------------------- #

    def on_frame(self, frame: int, env: SimulationEnvironment) -> None:
        diagnostics = {d.agent_id: d for d in self.controller.diagnostics}
        if not diagnostics:
            return  # frame 0, before the first control step

        agents = env.agents
        positions = np.vstack([a.position for a in agents])
        scans = self.controller.last_scans
        order = [d for d in (diagnostics.get(a.agent_id) for a in agents) if d is not None]

        detectors = [a.agent_id for a in agents if len(scans.get(a.agent_id, ())) > 0]
        if self.first_detection is None and detectors:
            self.first_detection = frame
            self.first_detector = detectors[0]
            index = self.agent_ids.index(detectors[0])
            self.detection_position = positions[index].copy()
            self.backside_mask = backside_samples(
                self.cargo, self.detection_position, BOUNDARY_SAMPLES
            )

        union = [v.points for v in self.controller._views.values() if len(v)]
        union_points = np.vstack(union) if union else np.empty((0, 2))
        observed = observed_boundary_mask(
            self.cargo, union_points, self.map_tolerance, BOUNDARY_SAMPLES
        )
        occupied = occupied_boundary_mask(
            self.cargo, positions, self.contact_radius, BOUNDARY_SAMPLES
        )

        backside_seen = False
        if self.backside_mask is not None:
            backside_seen = bool(np.any(observed & self.backside_mask))
            if backside_seen and self.backside_first is None:
                self.backside_first = frame
                self.backside_first_agent = self._attribute_backside(agents, scans)

        saturation = 1.0 - self.params.redeploy_gap_ratio
        empty_mass = self.controller.empty_cell_mass
        waiting = [
            d for d in order
            if d.boundary_distance <= self.params.local_radius and not d.contact_ready
        ]
        mode_counts: dict[str, int] = {}
        reason_counts: dict[str, int] = {}
        for d in order:
            mode_counts[d.mode] = mode_counts.get(d.mode, 0) + 1
            key = d.redeploy_reason or "not_reached"
            reason_counts[key] = reason_counts.get(key, 0) + 1
        record = FrameRecord(
            frame=frame,
            phase=int(self.controller.phase),
            agents=len(agents),
            informed=sum(1 for d in order if d.map_points > 0),
            direct_visible=len(detectors),
            arrived=sum(1 for d in order if d.boundary_distance <= self.params.local_radius),
            contact_ready=sum(1 for d in order if d.contact_ready),
            redeploy_active=sum(1 for d in order if d.redeploy_active),
            redeploy_requested=sum(1 for d in order if d.redeploy_requested),
            redeploy_no_candidate=sum(1 for d in order if d.redeploy_reason == "no_candidate"),
            candidates_total=sum(d.redeploy_candidates for d in order),
            candidates_max=max((d.redeploy_candidates for d in order), default=0),
            agents_with_candidates=sum(1 for d in order if d.redeploy_candidates > 0),
            cells_saturated=sum(
                1 for d in order if d.cell_mass > empty_mass and d.cell_held_fraction >= saturation
            ),
            empty_cells=sum(1 for d in order if d.cell_mass <= empty_mass),
            mode_counts=mode_counts,
            reason_counts=reason_counts,
            map_angular_coverage_max=max((d.map_angular_coverage for d in order), default=0.0),
            mean_speed_waiting=float(np.mean([d.speed_command for d in waiting])) if waiting else 0.0,
            mean_centroid_distance_waiting=float(
                np.mean([d.centroid_distance for d in waiting])
            ) if waiting else 0.0,
            union_map_coverage=float(np.mean(observed)),
            largest_unobserved_arc=arc_of_run(
                longest_false_run(observed), self.perimeter, BOUNDARY_SAMPLES
            ),
            largest_unheld_arc=arc_of_run(
                longest_false_run(occupied | ~observed), self.perimeter, BOUNDARY_SAMPLES
            ),
            strict_coverage=float(env.log.strict_coverage[self.object_id][-1]),
            contact_count=int(env.log.contact_counts[self.object_id][-1]),
            min_inter_agent=min_inter_agent_distance(agents),
            backside_observed=backside_seen,
            modes=[d.mode for d in order],
            boundary_distance=np.array([d.boundary_distance for d in order]),
            redeploy_reasons=[d.redeploy_reason for d in order],
            positions=positions.copy(),
        )
        self.records.append(record)

    def _attribute_backside(self, agents, scans) -> str | None:
        """Which robot's own sensor produced the first far-side return.

        The union map is what the *team* holds; attribution is a question about
        who pointed a sensor at it, so it is answered from the own scans, and a
        robot whose map contains far-side points it heard about from a neighbour
        is not the answer.
        """
        assert self.backside_mask is not None
        boundary, _ = self.cargo.boundary_samples(BOUNDARY_SAMPLES)
        far = boundary[self.backside_mask]
        for agent in agents:
            scan = scans.get(agent.agent_id)
            if scan is None or len(scan) == 0:
                continue
            distance = np.min(
                np.linalg.norm(far[:, None, :] - scan.points[None, :, :], axis=2), axis=1
            )
            if np.any(distance <= self.map_tolerance):
                return agent.agent_id
        return None


# --------------------------------------------------------------------------- #


def run_seed(config_path: str, seed: int, max_frames: int, settle_frames: int) -> dict:
    env = SimulationEnvironment(load_yaml(config_path), seed=seed)
    env.controller.trace_enabled = True
    trace = SeedTrace(env)
    started = time.perf_counter()
    termination = env.run_until_settled(
        max_frames=max_frames, settle_frames=settle_frames, on_frame=trace.on_frame
    )
    wall = time.perf_counter() - started
    phases = env.controller.phase_monitor.as_dict()
    summary = env.summary()
    entry = next(iter(summary["cargoes"].values()))
    return {
        "trace": trace,
        "termination": termination,
        "phases": phases,
        "summary": summary,
        "entry": entry,
        "wall": wall,
    }


def summarise(seed: int, result: dict, rules: SegmentRules) -> dict:
    trace: SeedTrace = result["trace"]
    phases = result["phases"]
    detect = trace.first_detection
    contact = phases.get("contact_ready_frame")
    end = contact if contact is not None else result["termination"]["frames_run"]
    post = max(0, end - (detect or 0))

    counts = segment(trace.records, rules, trace.perimeter, detect or 0, end)
    window = [r for r in trace.records if (detect or 0) <= r.frame < end]

    # The hypothesis under test, stated as three counters over the stalled
    # frames: how often the rule asked for a target, how often it asked and the
    # candidate set was empty, and how much boundary was missing from the union
    # map while that was happening.
    stalled = [r for r in window if r.arrived >= rules.quorum and r.contact_ready < rules.quorum]
    asked = sum(r.redeploy_requested for r in stalled)
    empty = sum(r.redeploy_no_candidate for r in stalled)
    covered_when_empty = [
        r.union_map_coverage for r in stalled if r.redeploy_no_candidate > 0
    ]
    unobserved_when_empty = [
        r.largest_unobserved_arc for r in stalled if r.redeploy_no_candidate > 0
    ]

    backside = trace.backside_first
    return {
        "seed": seed,
        "frames_run": result["termination"]["frames_run"],
        "terminated_by": result["termination"]["terminated_by"],
        "first_detection_frame": detect,
        "first_detector": trace.first_detector,
        "contact_ready_frame": contact,
        "transport_frame": phases.get("transport_frame"),
        "hold_frame": phases.get("hold_frame"),
        "post_detection_frames": post,
        "segments": counts,
        "segment_fractions": {k: (v / post if post else 0.0) for k, v in counts.items()},
        "backside_first_frame": backside,
        "backside_first_agent": trace.backside_first_agent,
        "backside_delay": (backside - detect) if (backside is not None and detect is not None) else None,
        "backside_to_contact": (contact - backside) if (backside is not None and contact is not None) else None,
        "peak_strict_coverage": float(entry_max(trace, "strict_coverage")),
        "final_strict_coverage": float(trace.records[-1].strict_coverage) if trace.records else 0.0,
        "peak_union_map_coverage": float(entry_max(trace, "union_map_coverage")),
        "min_inter_agent": float(min(r.min_inter_agent for r in trace.records)) if trace.records else float("nan"),
        "redeploy": {
            "stalled_frames": len(stalled),
            "agent_frames_requested": int(asked),
            "agent_frames_no_candidate": int(empty),
            "empty_fraction_of_requests": float(empty / asked) if asked else None,
            "agent_frames_active": int(sum(r.redeploy_active for r in stalled)),
            "frames_with_any_active": int(sum(1 for r in stalled if r.redeploy_active > 0)),
            "mean_union_coverage_when_empty": float(np.mean(covered_when_empty)) if covered_when_empty else None,
            "mean_unobserved_arc_when_empty": float(np.mean(unobserved_when_empty)) if unobserved_when_empty else None,
            # H2 vs H3. If the rule is candidate-starved, ``agents_with_candidates``
            # is near zero through the stall. If robots are simply not moving,
            # ``speed_waiting`` is near zero. The two are different failures and
            # only measuring both tells them apart.
            "mean_agents_with_candidates": float(np.mean([r.agents_with_candidates for r in stalled])) if stalled else None,
            "mean_cells_saturated": float(np.mean([r.cells_saturated for r in stalled])) if stalled else None,
            "mean_speed_waiting": float(np.mean([r.mean_speed_waiting for r in stalled])) if stalled else None,
            "mean_centroid_distance_waiting": float(np.mean([r.mean_centroid_distance_waiting for r in stalled])) if stalled else None,
            "mean_unobserved_arc": float(np.mean([r.largest_unobserved_arc for r in stalled])) if stalled else None,
            "mean_union_coverage": float(np.mean([r.union_map_coverage for r in stalled])) if stalled else None,
            # Does redeploying reduce the unobserved arc? Compare the mean
            # per-frame change in the arc on frames where somebody is redeploying
            # against frames where nobody is. A rule that cannot address unseen
            # boundary shows no difference.
            "mean_empty_cells": float(np.mean([r.empty_cells for r in stalled])) if stalled else None,
            **_arc_rate(stalled),
        },
        "stalled_modes": _tally(stalled, "mode_counts"),
        "stalled_reasons": _tally(stalled, "reason_counts"),
        "perimeter": trace.perimeter,
        "solver": result["summary"]["solver"],
        "wall": result["wall"],
        "fps": result["termination"]["frames_run"] / result["wall"] if result["wall"] else 0.0,
    }


def _tally(records: list[FrameRecord], field: str) -> dict[str, int]:
    """Agent-frames per key, summed over a set of frames."""
    total: dict[str, int] = {}
    for record in records:
        for key, count in getattr(record, field).items():
            total[key] = total.get(key, 0) + count
    return dict(sorted(total.items(), key=lambda kv: -kv[1]))


def _arc_rate(stalled: list[FrameRecord]) -> dict:
    """Rate of change of the unobserved arc, split by whether anyone is redeploying."""
    if len(stalled) < 2:
        return {"arc_rate_redeploying": None, "arc_rate_idle": None}
    delta = np.diff([r.largest_unobserved_arc for r in stalled])
    active = np.asarray([r.redeploy_active > 0 for r in stalled[:-1]])
    return {
        "arc_rate_redeploying": float(np.mean(delta[active])) if np.any(active) else None,
        "arc_rate_idle": float(np.mean(delta[~active])) if np.any(~active) else None,
    }


def entry_max(trace: SeedTrace, field: str) -> float:
    return max((getattr(r, field) for r in trace.records), default=0.0)


SERIES_FIELDS = (
    "frame", "phase", "informed", "direct_visible", "arrived", "contact_ready",
    "redeploy_active", "redeploy_requested", "redeploy_no_candidate",
    "candidates_total", "candidates_max", "agents_with_candidates", "cells_saturated",
    "map_angular_coverage_max", "union_map_coverage", "largest_unobserved_arc",
    "largest_unheld_arc", "strict_coverage", "contact_count", "min_inter_agent",
    "mean_speed_waiting", "mean_centroid_distance_waiting", "empty_cells",
)


def save_series(out: Path, seed: int, trace: SeedTrace) -> None:
    """One NPZ of per-frame series per seed, so the analysis can be rerun without
    re-running the simulation. A diagnosis whose numbers can only be reproduced by
    a twenty-minute sweep does not get re-examined."""
    arrays = {
        name: np.asarray([getattr(r, name) for r in trace.records], dtype=float)
        for name in SERIES_FIELDS
    }
    arrays["positions"] = np.stack([r.positions for r in trace.records])
    arrays["boundary_distance"] = np.stack([r.boundary_distance for r in trace.records])
    modes = sorted({m for r in trace.records for m in r.modes})
    reasons = sorted({m for r in trace.records for m in r.redeploy_reasons})
    arrays["mode_index"] = np.asarray(
        [[modes.index(m) for m in r.modes] for r in trace.records], dtype=np.int16
    )
    arrays["reason_index"] = np.asarray(
        [[reasons.index(m) for m in r.redeploy_reasons] for r in trace.records], dtype=np.int16
    )
    arrays["mode_names"] = np.asarray(modes)
    arrays["reason_names"] = np.asarray(reasons)
    arrays["agent_ids"] = np.asarray(trace.agent_ids)
    arrays["cargo_vertices"] = trace.cargo.vertices
    np.savez_compressed(out / f"series_seed{seed}.npz", **arrays)


# --------------------------------------------------------------------------- #
# figures
# --------------------------------------------------------------------------- #


def make_figures(out: Path, results: dict[int, dict], reports: list[dict], rules: SegmentRules) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colours = {
        "A": "#5B8FF9", "B": "#61DDAA", "C": "#F6BD16", "D": "#7262FD",
        "E": "#E8684A", "F": "#78D3F8", "G": "#9661BC",
    }

    # Figure A -- where the post-detection frames go, per seed.
    fig, ax = plt.subplots(figsize=(9, 4.5))
    seeds = [r["seed"] for r in reports]
    bottom = np.zeros(len(seeds))
    for key, name in SEGMENTS:
        values = np.array([r["segments"][key] for r in reports], dtype=float)
        ax.bar(seeds, values, bottom=bottom, color=colours[key], label=f"{key} {name}")
        bottom += values
    ax.set_xlabel("seed")
    ax.set_ylabel("frames, first detection to contact-ready")
    ax.set_title("A. Post-detection stage durations")
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(out / "figA_segments.png", dpi=150)
    plt.close(fig)

    slowest = max(reports, key=lambda r: r["post_detection_frames"])
    trace: SeedTrace = results[slowest["seed"]]["trace"]

    # Figure B -- what the team knows, over time, on every seed.
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for report in reports:
        t: SeedTrace = results[report["seed"]]["trace"]
        frames = [r.frame for r in t.records]
        axes[0].plot(frames, [r.union_map_coverage for r in t.records], lw=1, label=f"seed {report['seed']}")
        axes[1].plot(frames, [r.strict_coverage for r in t.records], lw=1)
        axes[2].plot(frames, [r.largest_unobserved_arc for r in t.records], lw=1)
    axes[0].set_ylabel("union map coverage")
    axes[1].set_ylabel("strict coverage (truth)")
    axes[2].set_ylabel("largest unobserved arc [m]")
    axes[2].axhline(rules.unobserved_arc_fraction * trace.perimeter, color="k", ls="--", lw=0.8,
                    label="segmentation threshold")
    axes[2].set_xlabel("frame")
    axes[0].legend(fontsize=6, ncol=4)
    axes[2].legend(fontsize=7)
    axes[0].set_title("B. Observed boundary, held boundary, and the gap")
    fig.tight_layout()
    fig.savefig(out / "figB_coverage.png", dpi=150)
    plt.close(fig)

    # Figure C -- per-robot mode raster, slowest seed.
    modes = sorted({m for r in trace.records for m in r.modes})
    index = {m: k for k, m in enumerate(modes)}
    grid = np.full((len(trace.agent_ids), len(trace.records)), np.nan)
    for column, record in enumerate(trace.records):
        for row, mode in enumerate(record.modes):
            grid[row, column] = index[mode]
    fig, ax = plt.subplots(figsize=(10, 4.5))
    image = ax.imshow(grid, aspect="auto", interpolation="nearest", cmap="tab10",
                      vmin=-0.5, vmax=len(modes) - 0.5,
                      extent=[trace.records[0].frame, trace.records[-1].frame, len(trace.agent_ids), 0])
    bar = fig.colorbar(image, ax=ax, ticks=range(len(modes)))
    bar.ax.set_yticklabels(modes)
    for frame, colour, label in (
        (trace.first_detection, "w", "first sight"),
        (trace.backside_first, "k", "first far-side sight"),
        (slowest["contact_ready_frame"], "r", "contact-ready"),
    ):
        if frame is not None:
            ax.axvline(frame, color=colour, ls="--", lw=1.2, label=label)
    ax.set_xlabel("frame")
    ax.set_ylabel("robot")
    ax.set_title(f"C. Per-robot mode, seed {slowest['seed']} (slowest post-detection)")
    ax.legend(fontsize=7, loc="upper right")
    fig.tight_layout()
    fig.savefig(out / "figC_modes.png", dpi=150)
    plt.close(fig)

    # Figure D -- the counters the hypothesis is stated over.
    fig, ax = plt.subplots(figsize=(9, 4.5))
    frames = [r.frame for r in trace.records]
    ax.plot(frames, [r.candidates_total for r in trace.records], lw=1, label="redeploy candidates (team total)")
    ax.plot(frames, [r.direct_visible for r in trace.records], lw=1, label="robots with direct visibility")
    ax.plot(frames, [r.arrived for r in trace.records], lw=1, label="robots at the object")
    ax.plot(frames, [r.contact_ready for r in trace.records], lw=1, label="robots contact-ready")
    ax.plot(frames, [r.redeploy_active for r in trace.records], lw=1, label="robots redeploying")
    ax.axhline(rules.quorum, color="k", ls=":", lw=0.8, label="quorum")
    ax.set_xlabel("frame")
    ax.set_ylabel("count")
    ax.set_yscale("symlog", linthresh=20)
    ax.set_title(f"D. Candidates, visibility and contact, seed {slowest['seed']}")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "figD_counts.png", dpi=150)
    plt.close(fig)

    # Figure E -- the slowest seed on the plane.
    fig, ax = plt.subplots(figsize=(7, 7))
    cargo = trace.cargo
    end = slowest["contact_ready_frame"] or trace.records[-1].frame
    window = [r for r in trace.records if r.frame <= end]
    tracks = np.stack([r.positions for r in window])  # (T, N, 2)
    for k in range(tracks.shape[1]):
        ax.plot(tracks[:, k, 0], tracks[:, k, 1], lw=0.8, alpha=0.7)
    ax.plot(tracks[0, :, 0], tracks[0, :, 1], "o", ms=3, color="0.4", label="start")
    ax.plot(tracks[-1, :, 0], tracks[-1, :, 1], "o", ms=4, color="C3", label="at contact-ready")
    outline = np.vstack([cargo.vertices, cargo.vertices[:1]])
    ax.plot(outline[:, 0], outline[:, 1], "k-", lw=1.5, label="cargo (final pose)")
    if trace.detection_position is not None:
        ax.plot(*trace.detection_position, "*", ms=16, color="C1", label="first sight")
    if trace.backside_first is not None:
        frame_index = next(k for k, r in enumerate(window) if r.frame == trace.backside_first)
        who = trace.agent_ids.index(trace.backside_first_agent) if trace.backside_first_agent else 0
        ax.plot(*tracks[frame_index, who], "P", ms=12, color="C4", label="first far-side sight")
    ax.set_aspect("equal")
    ax.set_title(f"E. Tracks to contact-ready, seed {slowest['seed']}")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(out / "figE_tracks.png", dpi=150)
    plt.close(fig)


# --------------------------------------------------------------------------- #


def stats(values: list[float]) -> dict:
    live = [v for v in values if v is not None and not (isinstance(v, float) and np.isnan(v))]
    if not live:
        return {"mean": None, "sd": None, "min": None, "max": None, "n": 0}
    arr = np.asarray(live, dtype=float)
    return {
        "mean": float(arr.mean()),
        "sd": float(arr.std(ddof=1)) if len(arr) > 1 else 0.0,
        "min": float(arr.min()),
        "max": float(arr.max()),
        "n": len(arr),
    }


def _merge_tallies(reports: list[dict], key: str) -> dict[str, int]:
    total: dict[str, int] = {}
    for report in reports:
        for name, count in report[key].items():
            total[name] = total.get(name, 0) + count
    return dict(sorted(total.items(), key=lambda kv: -kv[1]))


def write_markdown(out: Path, reports: list[dict], rules: SegmentRules, sensitivity: dict) -> None:
    lines: list[str] = []
    add = lines.append
    add("# D10-DIAG - post-discovery redeployment diagnosis\n")
    add(f"`configs/sim/d/l_shape_search.yaml`, seeds "
        f"{', '.join(str(r['seed']) for r in reports)}, run to completion, controller unchanged.\n")
    add(f"Segmentation rules: quorum {rules.quorum}, arrival radius {rules.arrival_radius} m, "
        f"unobserved-arc threshold {rules.unobserved_arc_fraction:.2f} of the perimeter.\n")

    add("\n## A. Where the post-detection frames go\n")
    header = "| seed | detect | contact-ready | post | " + " | ".join(k for k, _ in SEGMENTS) + " |"
    add(header)
    add("| " + " --- |" * (4 + len(SEGMENTS)))
    for r in reports:
        cells = " | ".join(str(r["segments"][k]) for k, _ in SEGMENTS)
        add(f"| {r['seed']} | {r['first_detection_frame']} | {r['contact_ready_frame']} | "
            f"{r['post_detection_frames']} | {cells} |")
    total = {k: sum(r["segments"][k] for r in reports) for k, _ in SEGMENTS}
    grand = sum(total.values())
    add(f"| **all** | | | **{grand}** | " +
        " | ".join(str(total[k]) for k, _ in SEGMENTS) + " |")
    add(f"| **share** | | | | " +
        " | ".join(f"{100.0 * total[k] / grand:.1f}%" if grand else "-" for k, _ in SEGMENTS) + " |")
    add("")
    for key, name in SEGMENTS:
        add(f"* `{key}` {name}")
    add("")

    add("\n## B. The redeploy rule over the stalled frames\n")
    add("| seed | stalled frames | agent-frames requesting | of those, no candidate | "
        "agent-frames redeploying | union map coverage when empty | unobserved arc when empty |")
    add("| --- | --- | --- | --- | --- | --- | --- |")
    for r in reports:
        d = r["redeploy"]
        share = d["empty_fraction_of_requests"]
        empty = str(d["agent_frames_no_candidate"])
        if share is not None:
            empty += f" ({100.0 * share:.0f}%)"
        coverage = "-" if d["mean_union_coverage_when_empty"] is None \
            else f"{d['mean_union_coverage_when_empty']:.3f}"
        arc = "-" if d["mean_unobserved_arc_when_empty"] is None \
            else f"{d['mean_unobserved_arc_when_empty']:.2f} m"
        add(f"| {r['seed']} | {d['stalled_frames']} | {d['agent_frames_requested']} | {empty} | "
            f"{d['agent_frames_active']} | {coverage} | {arc} |")
    add("")

    add("\n## B2. What the robots are doing over the stalled frames\n")
    add("Agent-frames, summed over the frames on which a quorum had arrived and the "
        "contact quorum had not yet formed.\n")
    modes = _merge_tallies(reports, "stalled_modes")
    reasons = _merge_tallies(reports, "stalled_reasons")
    add("| control mode | agent-frames | share |")
    add("| --- | --- | --- |")
    mode_total = sum(modes.values()) or 1
    for key, value in modes.items():
        add(f"| `{key}` | {value} | {100.0 * value / mode_total:.1f}% |")
    add("")
    add("| redeploy branch | agent-frames | share |")
    add("| --- | --- | --- |")
    reason_total = sum(reasons.values()) or 1
    for key, value in reasons.items():
        add(f"| `{key}` | {value} | {100.0 * value / reason_total:.1f}% |")
    add("")

    add("\n## C. Far-side discovery\n")
    add("| seed | first sight | first far-side sight | delay | contact-ready | far-side to contact-ready | "
        "who saw it | peak strict coverage |")
    add("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for r in reports:
        add(f"| {r['seed']} | {r['first_detection_frame']} | {r['backside_first_frame']} | "
            f"{r['backside_delay']} | {r['contact_ready_frame']} | {r['backside_to_contact']} | "
            f"{r['backside_first_agent']} | {r['peak_strict_coverage']:.3f} |")
    add("")

    add("\n## D. Segmentation sensitivity to the unobserved-arc threshold\n")
    add("| threshold (fraction of perimeter) | " + " | ".join(k for k, _ in SEGMENTS) + " |")
    add("| " + " --- |" * (1 + len(SEGMENTS)))
    for threshold, counts in sensitivity.items():
        add(f"| {threshold} | " + " | ".join(str(counts[k]) for k, _ in SEGMENTS) + " |")
    add("")

    (out / "diagnosis.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/sim/d/l_shape_search.yaml")
    parser.add_argument("--seeds", default="0..7")
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--settle-frames", type=int, default=40)
    parser.add_argument("--unobserved-arc-fraction", type=float, default=0.20)
    parser.add_argument("--out", default="runs/d10_diag")
    parser.add_argument("--no-figures", action="store_true")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    seeds = parse_seeds(args.seeds)

    config = load_yaml(args.config)
    quorum = int(config["controller"].get("min_push_agents", 4))
    local_radius = float(config["controller"].get("local_radius", 0.80))
    rules = SegmentRules(
        quorum=quorum,
        arrival_radius=local_radius,
        unobserved_arc_fraction=args.unobserved_arc_fraction,
    )

    results: dict[int, dict] = {}
    reports: list[dict] = []
    for seed in seeds:
        result = run_seed(args.config, seed, args.max_frames, args.settle_frames)
        results[seed] = result
        report = summarise(seed, result, rules)
        reports.append(report)
        save_series(out, seed, result["trace"])
        seg = report["segments"]
        print(f"seed {seed:2d}  detect@{report['first_detection_frame']}  "
              f"far-side@{report['backside_first_frame']}  "
              f"contact-ready@{report['contact_ready_frame']}  "
              f"post={report['post_detection_frames']:4d}  "
              f"[" + " ".join(f"{k}={seg[k]}" for k, _ in SEGMENTS) + "]  "
              f"{report['fps']:.1f} fps ({report['terminated_by']})")

    # Sensitivity: the one threshold in the cascade, at three values.
    sensitivity: dict[str, dict[str, int]] = {}
    for fraction in (0.10, 0.20, 0.30):
        alt = SegmentRules(quorum=quorum, arrival_radius=local_radius,
                           unobserved_arc_fraction=fraction)
        totals = {k: 0 for k, _ in SEGMENTS}
        for report in reports:
            trace: SeedTrace = results[report["seed"]]["trace"]
            end = report["contact_ready_frame"] or report["frames_run"]
            counts = segment(trace.records, alt, trace.perimeter,
                             report["first_detection_frame"] or 0, end)
            for key in totals:
                totals[key] += counts[key]
        sensitivity[f"{fraction:.2f}"] = totals

    aggregate = {
        "T_detect": stats([r["first_detection_frame"] for r in reports]),
        "T_contact_ready": stats([r["contact_ready_frame"] for r in reports]),
        "T_post_detection": stats([r["post_detection_frames"] for r in reports]),
        "T_backside_discovery": stats([r["backside_delay"] for r in reports]),
        "T_backside_to_contact": stats([r["backside_to_contact"] for r in reports]),
        "peak_strict_coverage": stats([r["peak_strict_coverage"] for r in reports]),
        "peak_union_map_coverage": stats([r["peak_union_map_coverage"] for r in reports]),
        "min_inter_agent": stats([r["min_inter_agent"] for r in reports]),
        "segments_total": {k: sum(r["segments"][k] for r in reports) for k, _ in SEGMENTS},
        "segment_labels": SEGMENT_LABELS,
        "sensitivity": sensitivity,
    }

    payload = {
        "config": args.config,
        "seeds": seeds,
        "rules": {
            "quorum": rules.quorum,
            "arrival_radius": rules.arrival_radius,
            "informed_fraction": rules.informed_fraction,
            "unobserved_arc_fraction": rules.unobserved_arc_fraction,
        },
        "seed_reports": reports,
        "aggregate": aggregate,
    }
    (out / "diagnosis.json").write_text(json.dumps(payload, indent=2, default=float), encoding="utf-8")
    write_markdown(out, reports, rules, sensitivity)
    if not args.no_figures:
        make_figures(out, results, reports, rules)

    print()
    for name in ("T_detect", "T_backside_discovery", "T_backside_to_contact",
                 "T_contact_ready", "T_post_detection", "peak_strict_coverage"):
        s = aggregate[name]
        if s["mean"] is None:
            print(f"{name:24s}  -")
        else:
            print(f"{name:24s}  {s['mean']:8.2f} +/- {s['sd']:7.2f}   "
                  f"[{s['min']:.2f}, {s['max']:.2f}]  n={s['n']}")
    total = sum(aggregate["segments_total"].values())
    print()
    for key, name in SEGMENTS:
        count = aggregate["segments_total"][key]
        print(f"{key} {name:24s} {count:6d} frames  {100.0 * count / total if total else 0:5.1f}%")
    print(f"\nwritten to {out}")


if __name__ == "__main__":
    main()
