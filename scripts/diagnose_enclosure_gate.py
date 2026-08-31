#!/usr/bin/env python
"""D10-ENC - is the DISCOVER -> ENCLOSE guard still measuring the wrong thing?

    PYTHONPATH=src python scripts/diagnose_enclosure_gate.py --seeds 0..7 --out runs/d10_enc

The controller is not modified. Every candidate gate is evaluated *offline* on
the recorded trace, so this answers "when would it have fired" and not "what would
have happened" -- the second needs a run, and that is what the A/B in
`scripts/ab_enclosure_gate.py` is for. Saying which of the two a number is
matters here, because a counterfactual gate changes the trajectory it is being
scored against.

The separation that keeps this honest: candidate gates read `GateInputs`, which
contains only on-board quantities, and the truth quantities live in a different
record that only the scoring touches.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from diagnose_redeployment import parse_seeds, stats  # noqa: E402

from dbact.diagnosis import (  # noqa: E402
    arc_of_run,
    longest_false_run,
    observed_boundary_mask,
    occupied_boundary_mask,
)
from dbact.enclosure_gate import (  # noqa: E402
    BINS,
    BitmapConsensus,
    GateInputs,
    bearing_bins,
    direction_bins,
    g0_current,
    g1_known,
    g2_operational,
    g3_hybrid,
    gap_degrees,
    occupancy,
)
from dbact.geometry import polygon_perimeter  # noqa: E402
from dbact.metrics import min_inter_agent_distance  # noqa: E402
from dbact.phase import Phase  # noqa: E402
from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402

BOUNDARY_SAMPLES = 160
D_MIN_TOLERANCE = 1e-6


@dataclass
class GateFrame:
    """One frame. On-board fields feed the gates; truth fields only ever score."""

    frame: int
    phase: int
    # --- on board ---
    informed: int = 0
    agents: int = 0
    best_own_coverage: float = 0.0
    best_own_agent: str = ""
    best_own_gap_deg: float = 360.0
    mean_own_coverage: float = 0.0
    # Normal-direction certificates: reference free, by consensus.
    known_normal_gap_deg: float = 360.0
    known_normal_bins: int = 0
    known_normal_gap_ideal_deg: float = 360.0
    held_normal_gap_deg: float = 360.0
    held_normal_bins: int = 0
    # The bearing family, kept to show it failing rather than to gate on.
    union_bearing_coverage: float = 0.0
    union_bearing_gap_deg: float = 360.0
    contact_ready: int = 0
    own_coverage: list[float] = field(default_factory=list)
    # --- truth, evaluation only ---
    strict_coverage: float = 0.0
    truth_map_coverage: float = 0.0
    truth_unobserved_arc: float = 0.0
    truth_unheld_arc: float = 0.0
    contact_count: int = 0
    min_inter_agent: float = float("inf")

    def inputs(self) -> GateInputs:
        return GateInputs(
            informed=self.informed,
            agents=self.agents,
            best_own_coverage=self.best_own_coverage,
            known_normal_gap_deg=self.known_normal_gap_deg,
            held_normal_gap_deg=self.held_normal_gap_deg,
            held_normal_bins=self.held_normal_bins,
            contact_ready=self.contact_ready,
            union_bearing_coverage=self.union_bearing_coverage,
            union_bearing_gap_deg=self.union_bearing_gap_deg,
        )


class GateTrace:
    def __init__(self, env: SimulationEnvironment):
        self.env = env
        self.controller = env.controller
        self.params = env.controller.params
        self.cargo = env.cargoes[0]
        self.object_id = self.cargo.object_id
        self.perimeter = polygon_perimeter(self.cargo.vertices)
        self.map_tolerance = 1.5 * self.params.voxel_size
        self.contact_radius = float(env.evaluation_contact_radius)
        self.agent_ids = [a.agent_id for a in env.agents]
        self.records: list[GateFrame] = []
        self.first_detection: int | None = None
        # Which boundary *directions* the team has seen. Monotone, like the phase
        # machine it would feed: "has anybody ever seen a face pointing this way".
        self.known = BitmapConsensus(bins=BINS, ttl=None)
        # Which directions currently have somebody standing on them. Not monotone
        # -- a robot that leaves the band has to stop counting -- so it expires on
        # the same TTL the object token already uses.
        self.held = BitmapConsensus(bins=BINS, ttl=self.params.token_ttl)

    # ---------------------------------------------------------------- #

    def _neighbours(self, positions: np.ndarray) -> dict[str, list[str]]:
        d = np.linalg.norm(positions[:, None, :] - positions[None, :, :], axis=2)
        np.fill_diagonal(d, np.inf)
        return {
            self.agent_ids[i]: [self.agent_ids[j] for j in np.flatnonzero(row <= self.params.comm_range)]
            for i, row in enumerate(d)
        }

    def on_frame(self, frame: int, env: SimulationEnvironment) -> None:
        diagnostics = {d.agent_id: d for d in self.controller.diagnostics}
        if not diagnostics:
            return
        agents = env.agents
        positions = np.vstack([a.position for a in agents])
        views = self.controller._views
        scans = self.controller.last_scans
        if self.first_detection is None and any(len(scans.get(a.agent_id, ())) for a in agents):
            self.first_detection = frame

        # Every robot's own reference for the bins is its own token -- the same
        # relayed estimate the recall already uses. No robot needs the true pose.
        tokens = {
            a.agent_id: self.controller._token_target(a.agent_id) for a in agents
        }
        own_known: dict[str, np.ndarray] = {}
        own_held: dict[str, np.ndarray] = {}
        own_coverage: list[float] = []
        for agent in agents:
            view = views.get(agent.agent_id)
            if view is None or len(view) == 0:
                own_known[agent.agent_id] = np.zeros(BINS, dtype=bool)
                own_held[agent.agent_id] = np.zeros(BINS, dtype=bool)
                own_coverage.append(0.0)
                continue
            own_known[agent.agent_id] = direction_bins(view.normals)
            mask = np.zeros(BINS, dtype=bool)
            if diagnostics[agent.agent_id].contact_ready:
                # The face this robot is actually standing on: its own nearest
                # observation's outward normal. One direction per robot, which is
                # what makes the held set a statement about robots rather than
                # about map size.
                delta = view.points - agent.position[None, :]
                nearest = int(np.argmin(np.einsum("ij,ij->i", delta, delta)))
                mask = direction_bins(view.normals[nearest].reshape(1, 2))
            own_held[agent.agent_id] = mask
            own_coverage.append(_own_map_coverage(view.points))

        neighbours = self._neighbours(positions)
        self.known.step(own_known, neighbours, env.t)
        self.held.step(own_held, neighbours, env.t)

        # The idealised union: what the team collectively holds with no
        # communication delay at all. Reported beside the consensus value so the
        # cost of the relay is a number rather than an assumption.
        union_points = [v.points for v in views.values() if len(v)]
        union_normals = [v.normals for v in views.values() if len(v)]
        ideal = direction_bins(np.vstack(union_normals)) if union_normals else np.zeros(BINS, dtype=bool)

        consensus = [self.known.view(a.agent_id, env.t) for a in agents]
        held_views = [self.held.view(a.agent_id, env.t) for a in agents]
        # A gate is evaluated by each robot on its own view, and the transition it
        # would produce is the first frame *any* robot's own view satisfies it --
        # the same "any robot" the committed guard already uses.
        best_known = min(consensus, key=gap_degrees) if consensus else np.zeros(BINS, dtype=bool)
        best_held = min(held_views, key=gap_degrees) if held_views else np.zeros(BINS, dtype=bool)

        # The bearing family, recorded to be refuted rather than used.
        reference = next((t for t in tokens.values() if t is not None), None)
        if union_points and reference is not None:
            bearing = bearing_bins(np.vstack(union_points), reference)
        else:
            bearing = np.zeros(BINS, dtype=bool)

        observed_truth = observed_boundary_mask(
            self.cargo, np.vstack(union_points) if union_points else np.empty((0, 2)),
            self.map_tolerance, BOUNDARY_SAMPLES,
        )
        occupied_truth = occupied_boundary_mask(
            self.cargo, positions, self.contact_radius, BOUNDARY_SAMPLES
        )
        best_index = int(np.argmax(own_coverage)) if own_coverage else 0

        self.records.append(
            GateFrame(
                frame=frame,
                phase=int(self.controller.phase),
                informed=sum(1 for v in views.values() if len(v) >= 8),
                agents=len(agents),
                best_own_coverage=float(max(own_coverage) if own_coverage else 0.0),
                best_own_agent=self.agent_ids[best_index],
                best_own_gap_deg=gap_degrees(own_known[self.agent_ids[best_index]]),
                mean_own_coverage=float(np.mean(own_coverage)) if own_coverage else 0.0,
                known_normal_gap_deg=gap_degrees(best_known),
                known_normal_bins=int(np.count_nonzero(best_known)),
                known_normal_gap_ideal_deg=gap_degrees(ideal),
                held_normal_gap_deg=gap_degrees(best_held),
                held_normal_bins=int(np.count_nonzero(best_held)),
                union_bearing_coverage=occupancy(bearing),
                union_bearing_gap_deg=gap_degrees(bearing),
                contact_ready=sum(1 for d in diagnostics.values() if d.contact_ready),
                own_coverage=[float(c) for c in own_coverage],
                strict_coverage=float(env.log.strict_coverage[self.object_id][-1]),
                truth_map_coverage=float(np.mean(observed_truth)),
                truth_unobserved_arc=arc_of_run(
                    longest_false_run(observed_truth), self.perimeter, BOUNDARY_SAMPLES
                ),
                truth_unheld_arc=arc_of_run(
                    longest_false_run(occupied_truth | ~observed_truth),
                    self.perimeter, BOUNDARY_SAMPLES,
                ),
                contact_count=int(env.log.contact_counts[self.object_id][-1]),
                min_inter_agent=min_inter_agent_distance(agents),
            )
        )


def _own_map_coverage(points: np.ndarray, bins: int = BINS) -> float:
    """The committed guard's own quantity: bearing bins about the map's centroid."""
    if len(points) < 3:
        return 0.0
    return occupancy(bearing_bins(points, points.mean(axis=0), bins))


# --------------------------------------------------------------------------- #


def first_frame(records: list[GateFrame], predicate) -> int | None:
    for record in records:
        if predicate(record):
            return record.frame
    return None


def run_seed(config: dict, seed: int, max_frames: int, settle_frames: int) -> dict:
    env = SimulationEnvironment(json.loads(json.dumps(config)), seed=seed)
    env.controller.trace_enabled = True
    trace = GateTrace(env)
    started = time.perf_counter()
    termination = env.run_until_settled(
        max_frames=max_frames, settle_frames=settle_frames, on_frame=trace.on_frame
    )
    wall = time.perf_counter() - started
    summary = env.summary()
    entry = next(iter(summary["cargoes"].values()))
    return {
        "trace": trace,
        "termination": termination,
        "phases": env.controller.phase_monitor.as_dict(),
        "entry": entry,
        "solver": summary["solver"],
        "min_inter_agent": min(env.log.min_distances),
        "d_min": float(env.controller.params.d_min),
        "wall": wall,
    }


def milestones(result: dict, quorum: int) -> dict:
    trace: GateTrace = result["trace"]
    records = trace.records
    phases = result["phases"]
    return {
        "T_first_detection": trace.first_detection,
        "T_backside_discovery": None,  # filled by the caller from the arc series
        "T_known_normals_120": first_frame(records, lambda r: r.known_normal_gap_deg <= 120.0),
        "T_known_normals_ideal_120": first_frame(records, lambda r: r.known_normal_gap_ideal_deg <= 120.0),
        "T_held_normals_120": first_frame(records, lambda r: r.held_normal_gap_deg <= 120.0),
        "T_union_bearing_70": first_frame(records, lambda r: r.union_bearing_coverage >= 0.70),
        "T_any_local_map_70": first_frame(records, lambda r: r.best_own_coverage >= 0.70),
        "T_strict_coverage_70": first_frame(records, lambda r: r.strict_coverage >= 0.70),
        "T_contact_quorum": first_frame(records, lambda r: r.contact_ready >= quorum),
        "T_current_gate": first_frame(records, lambda r: g0_current(r.inputs())),
        "T_enclose_phase": phases.get("enclosure_frame"),
        "T_contact_ready": phases.get("contact_ready_frame"),
        "T_transport": phases.get("transport_frame"),
        "T_hold": phases.get("hold_frame"),
    }


CANDIDATES: dict[str, object] = {}


def build_candidates(quorum: int) -> dict:
    """Candidate gates over a threshold grid. The grid is swept, not chosen."""
    out: dict[str, object] = {"G0 current": lambda r: g0_current(r.inputs())}
    for gap in (60.0, 90.0, 120.0, 150.0, 170.0):
        out[f"G1 known<={gap:.0f}"] = (lambda g: lambda r: g1_known(r.inputs(), g))(gap)
    for gap in (60.0, 90.0, 120.0, 150.0, 170.0):
        out[f"G2 held<={gap:.0f}"] = (
            lambda g: lambda r: g2_operational(r.inputs(), quorum, g)
        )(gap)
    for known in (90.0, 120.0, 150.0):
        for held in (90.0, 120.0, 150.0, 170.0):
            out[f"G3 known<={known:.0f} & held<={held:.0f}"] = (
                lambda k, h: lambda r: g3_hybrid(r.inputs(), k, h, quorum)
            )(known, held)
    return out


def downstream(records: list[GateFrame], frame: int | None, window: int, d_min: float) -> dict:
    """What the baseline world looks like at and after a hypothetical transition.

    This is a screen, not a prediction: the baseline trajectory is the one the
    current gate produced, so it says what state the team was *in* at that moment,
    not what would have happened had the phase actually advanced there. A
    candidate that fires while a third of the boundary is unheld is rejected on
    this evidence; a candidate that survives it still has to be run.
    """
    if frame is None:
        return {"fired": False}
    at = next((r for r in records if r.frame >= frame), None)
    if at is None:
        return {"fired": False}
    after = [r for r in records if frame <= r.frame <= frame + window]
    return {
        "fired": True,
        "frame": frame,
        "strict_coverage_at": at.strict_coverage,
        "strict_coverage_min_after": min((r.strict_coverage for r in after), default=at.strict_coverage),
        "truth_map_coverage_at": at.truth_map_coverage,
        "unobserved_arc_at": at.truth_unobserved_arc,
        "unheld_arc_at": at.truth_unheld_arc,
        "contact_count_at": at.contact_count,
        "contact_ready_at": at.contact_ready,
        "min_inter_agent_after": min((r.min_inter_agent for r in after), default=float("inf")),
        "d_min_breach_after": bool(
            min((r.min_inter_agent for r in after), default=float("inf")) < d_min - D_MIN_TOLERANCE
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/sim/d/l_shape_search.yaml")
    parser.add_argument("--seeds", default="0..7")
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--settle-frames", type=int, default=40)
    parser.add_argument("--window", type=int, default=120,
                        help="Frames after a hypothetical transition to screen for collapse.")
    parser.add_argument("--out", default="runs/d10_enc")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    seeds = parse_seeds(args.seeds)
    config = load_yaml(args.config)
    quorum = int(config["controller"].get("min_push_agents", 4))
    candidates = build_candidates(quorum)

    reports: list[dict] = []
    results: dict[int, dict] = {}
    for seed in seeds:
        result = run_seed(config, seed, args.max_frames, args.settle_frames)
        results[seed] = result
        trace: GateTrace = result["trace"]
        marks = milestones(result, quorum)
        fires = {
            name: first_frame(trace.records, predicate)
            for name, predicate in candidates.items()
        }
        screens = {
            name: downstream(trace.records, frame, args.window, result["d_min"])
            for name, frame in fires.items()
        }
        reports.append({
            "seed": seed,
            "milestones": marks,
            "gate_frames": fires,
            "screens": screens,
            "frames_run": result["termination"]["frames_run"],
            "terminated_by": result["termination"]["terminated_by"],
            "peak_strict_coverage": result["entry"]["max_strict_coverage"],
            "min_inter_agent": result["min_inter_agent"],
            "fallbacks": result["solver"]["fallbacks"],
            "fps": result["termination"]["frames_run"] / result["wall"] if result["wall"] else 0.0,
        })
        np.savez_compressed(
            out / f"gate_seed{seed}.npz",
            **{
                key: np.asarray([getattr(r, key) for r in trace.records], dtype=float)
                for key in (
                    "frame", "phase", "informed", "best_own_coverage", "mean_own_coverage",
                    "known_normal_gap_deg", "known_normal_bins", "known_normal_gap_ideal_deg",
                    "held_normal_gap_deg", "held_normal_bins", "union_bearing_coverage",
                    "union_bearing_gap_deg", "contact_ready", "strict_coverage",
                    "truth_map_coverage", "truth_unobserved_arc", "truth_unheld_arc",
                    "contact_count", "min_inter_agent",
                )
            },
        )
        print(f"seed {seed:2d}  gate@{marks['T_current_gate']}  CR@{marks['T_contact_ready']}  "
              f"known120@{marks['T_known_normals_120']}  held120@{marks['T_held_normals_120']}  "
              f"quorum@{marks['T_contact_quorum']}  strict70@{marks['T_strict_coverage_70']}  "
              f"bearing70@{marks['T_union_bearing_70']}  {reports[-1]['fps']:.1f} fps")

    payload = {
        "config": args.config,
        "seeds": seeds,
        "window": args.window,
        "reports": reports,
        "correlations": correlations(reports),
        "tradeoff": tradeoff(reports, candidates),
    }
    (out / "gate.json").write_text(json.dumps(payload, indent=2, default=float), encoding="utf-8")
    print_summary(payload)
    print(f"\nwritten to {out}")


def correlations(reports: list[dict]) -> dict:
    """Correlation of each milestone with the contact-ready frame, across seeds."""
    target = np.asarray([r["milestones"]["T_contact_ready"] for r in reports], dtype=float)
    out: dict[str, dict] = {}
    for key in reports[0]["milestones"]:
        if key == "T_contact_ready":
            continue
        values = [r["milestones"][key] for r in reports]
        if any(v is None for v in values):
            out[key] = {"correlation": None, "lag_mean": None, "n": sum(v is not None for v in values)}
            continue
        arr = np.asarray(values, dtype=float)
        out[key] = {
            "correlation": float(np.corrcoef(arr, target)[0, 1]) if arr.std() > 0 else None,
            "lag_mean": float(np.mean(target - arr)),
            "lag_sd": float(np.std(target - arr, ddof=1)) if len(arr) > 1 else 0.0,
            "n": len(arr),
        }
    return out


def tradeoff(reports: list[dict], candidates: dict) -> dict:
    """Delay against the state of the world at that delay, per candidate gate."""
    out: dict[str, dict] = {}
    for name in candidates:
        frames = [r["gate_frames"][name] for r in reports]
        fired = [f for f in frames if f is not None]
        screens = [r["screens"][name] for r in reports if r["screens"][name]["fired"]]
        current = [r["milestones"]["T_current_gate"] for r in reports]
        delta = [
            f - c for f, c in zip(frames, current) if f is not None and c is not None
        ]
        out[name] = {
            "fires_on": len(fired),
            "of": len(frames),
            "frame_mean": float(np.mean(fired)) if fired else None,
            "delta_vs_current_mean": float(np.mean(delta)) if delta else None,
            "strict_coverage_at": float(np.mean([s["strict_coverage_at"] for s in screens])) if screens else None,
            "unheld_arc_at": float(np.mean([s["unheld_arc_at"] for s in screens])) if screens else None,
            "contact_ready_at": float(np.mean([s["contact_ready_at"] for s in screens])) if screens else None,
            "coverage_collapse": sum(
                1 for s in screens if s["strict_coverage_min_after"] < 0.75 * s["strict_coverage_at"]
            ),
            "d_min_breach_after": sum(1 for s in screens if s["d_min_breach_after"]),
        }
    return out


def print_summary(payload: dict) -> None:
    print("\ncorrelation with T_contact_ready (8 seeds)")
    rows = sorted(
        payload["correlations"].items(),
        key=lambda kv: -(kv[1]["correlation"] or -2),
    )
    for name, entry in rows:
        if entry["correlation"] is None:
            print(f"  {name:26s}  n/a (fires on {entry['n']}/8)")
        else:
            print(f"  {name:26s}  r={entry['correlation']:+.3f}  lag={entry['lag_mean']:+7.1f}"
                  f" ± {entry['lag_sd']:.1f}")
    print("\ncandidate gates: when they fire, and the state of the world there")
    print(f"  {'gate':34s} {'fires':>6} {'frame':>7} {'vs G0':>7} {'strict':>7} "
          f"{'unheld':>7} {'ready':>6} {'collapse':>9} {'breach':>7}")
    for name, entry in payload["tradeoff"].items():
        fmt = lambda v, spec: ("-" if v is None else format(v, spec))  # noqa: E731
        print(f"  {name:34s} {entry['fires_on']}/{entry['of']:<4} "
              f"{fmt(entry['frame_mean'], '7.0f')} {fmt(entry['delta_vs_current_mean'], '+7.0f')} "
              f"{fmt(entry['strict_coverage_at'], '7.3f')} {fmt(entry['unheld_arc_at'], '7.2f')} "
              f"{fmt(entry['contact_ready_at'], '6.1f')} {entry['coverage_collapse']:>9} "
              f"{entry['d_min_breach_after']:>7}")


if __name__ == "__main__":
    main()
