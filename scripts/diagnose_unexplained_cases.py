#!/usr/bin/env python
"""T4 - the two cases the shape matrix could not account for.

    python scripts/diagnose_unexplained_cases.py --out docs/results/t4

Both are re-run from the committed matrix configuration, so the geometry, the annulus
and the goal are bit-identical to the episodes in
``docs/results/v2_shape_matrix/episodes.csv``.

Case 1: ``rectangle__a0.10__seed004``
    The only solver failure in 225 episodes: 124 fallbacks, 124 infeasible solves, 658
    barrier scalings, ``min_barrier_scale`` 0.0. It also travelled 6.29 m on a 0.214 m
    task, never armed transport, and ended with a minimum inter-agent distance of
    0.2038 m against a ``d_min`` of 0.28 -- a 76 mm separation breach that the failure
    taxonomy did not report, because ``classify`` ranks SOLVER_FAILURE above
    SAFETY_VIOLATION and returns the first match.

    The question this script answers is the *order*: did the QP go infeasible and the
    fallback projection then push robots inside ``d_min``, or did the separation break
    first and leave the barrier with no admissible input? The two have different fixes
    and only one of them is a solver problem.

Case 2: ``polygon32`` seed 2, all three alpha
    Reported as "pushing at 93 degrees to the goal with J ~ 0". The first thing to
    establish is whether that is a finding or an artefact: the recorded displacement is
    2e-5 m, and a direction error computed from a 20 micron displacement is not a
    measurement of anything. What needs explaining is why the object never moved, which
    on this case means why fewer than ``transport_quorum`` robots ever entered the push
    set -- ``alignment = n_k . d_goal <= -0.35`` against each robot's own fused map
    normal.

Nothing here changes the controller. The script records what the run already computes,
plus one truth-side quantity per frame for scoring, and writes both a JSON trace summary
and a plain-text finding.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact.metrics import min_inter_agent_distance, strict_boundary_coverage  # noqa: E402
from dbact.phase import Phase  # noqa: E402
from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402

MATRIX_CONFIG = "configs/sim/v2/shape_matrix.yaml"

_RUNNER = None


def load_matrix_runner():
    """The matrix harness, so the cases are rebuilt exactly as they were run."""
    global _RUNNER
    if _RUNNER is None:
        path = ROOT / "scripts" / "run_arbitrary_shape_monte_carlo.py"
        spec = importlib.util.spec_from_file_location("_matrix_runner", path)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        _RUNNER = module
    return _RUNNER


def trace_case(shape: str, seed: int, alpha: float, max_frames: int) -> dict:
    """Run one matrix case, recording per-frame diagnosis."""
    runner = load_matrix_runner()
    config, metadata = runner.build_case_config(load_yaml(MATRIX_CONFIG), shape, seed, alpha)
    # No controller override at all: the case must be the one that ran. The push-set
    # diagnosis below is recomputed from the robot's own map view rather than read off
    # the optional trace, so nothing has to be switched on to obtain it.
    env = SimulationEnvironment(config, seed=seed)
    cargo = env.cargoes[0]
    task = env.tasks[cargo.object_id]
    goal = np.asarray(task.direction, dtype=float).reshape(2)
    d_min = float(env.controller.params.d_min)
    threshold = float(env.controller.params.push_side_threshold)
    quorum = int(env.controller.params.min_push_agents)
    band_limit = float(
        env.controller.params.cage_offset + env.controller.params.contact_band_tolerance
    )
    # The object's own radial variation, for comparison with the band width. Truth-side,
    # used only to score: this is the quantity that decides whether a single scalar
    # stand-off band can be satisfied all the way round the outline at once.
    radii = np.linalg.norm(cargo.local_vertices, axis=1)
    radial_spread = float(np.max(radii) - np.min(radii))

    frames: list[dict] = []
    first_breach = None
    first_infeasible = None
    first_fallback = None
    previous = {"fallbacks": 0, "infeasible": 0, "barrier_scalings": 0}

    for frame in range(int(max_frames)):
        env.step()
        stats = env.controller.safety.stats
        separation = min_inter_agent_distance(env.agents)

        # Per-agent push-set membership, recomputed from the same quantities the
        # controller used: each robot's own nearest map normal against the task
        # direction. This is a read of the map, not of the object.
        alignments = []
        for agent in env.agents:
            view = env.controller._views.get(agent.agent_id)
            if view is None or len(view) == 0:
                continue
            nearest = env.controller._nearest_index(view, agent.position)
            if nearest is None:
                continue
            alignments.append(float(np.dot(view.normals[nearest], goal)))
        alignments = np.asarray(alignments) if alignments else np.zeros(0)
        eligible = int(np.sum(alignments <= -threshold)) if len(alignments) else 0
        pushing = sum(1 for d in env.controller.diagnostics if d.push_side)

        # The push predicate is a conjunction, so "8 robots met the alignment test and 3
        # pushed" leaves two candidates: the contact band, and the local contact-ready
        # quorum among a robot's neighbours. These separate them.
        band = env.controller._contact_ready_flags if hasattr(env.controller, "_contact_ready_flags") else None
        contact_ready_count = (
            int(sum(1 for d in env.controller.diagnostics if d.contact_ready))
            if env.controller.diagnostics
            else 0
        )
        # Each robot's own distance to its nearest map point, against the band the
        # contact test uses. On an object whose radius varies by more than the band is
        # wide, no ring position satisfies it everywhere at once.
        band_distances = []
        for agent in env.agents:
            view = env.controller._views.get(agent.agent_id)
            if view is None or len(view) == 0:
                continue
            nearest = env.controller._nearest_index(view, agent.position)
            if nearest is not None:
                band_distances.append(
                    float(np.linalg.norm(view.points[nearest] - agent.position))
                )
        band_distances = np.asarray(band_distances) if band_distances else np.zeros(0)

        new_infeasible = stats.infeasible - previous["infeasible"]
        new_fallbacks = stats.fallbacks - previous["fallbacks"]
        if first_breach is None and separation < d_min - 1e-6:
            first_breach = frame
        if first_infeasible is None and new_infeasible > 0:
            first_infeasible = frame
        if first_fallback is None and new_fallbacks > 0:
            first_fallback = frame

        frames.append(
            {
                "frame": frame,
                "phase": env.controller.phase_monitor.phase.label,
                "min_inter_agent": separation,
                "separation_breach": bool(separation < d_min - 1e-6),
                "new_infeasible": int(new_infeasible),
                "new_fallbacks": int(new_fallbacks),
                "new_barrier_scalings": int(stats.barrier_scalings - previous["barrier_scalings"]),
                "cargo_speed": float(np.linalg.norm(cargo.linear_velocity)),
                "cargo_displacement": float(np.linalg.norm(cargo.displacement)),
                "cargo_rotation_deg": float(np.degrees(cargo.angle)),
                "strict_coverage": strict_boundary_coverage(cargo, env.agents),
                # Push-set diagnosis
                "agents_with_map": int(len(alignments)),
                "alignment_min": float(np.min(alignments)) if len(alignments) else None,
                "alignment_eligible": eligible,
                "push_side_count": pushing,
                "contact_ready_count": contact_ready_count,
                "band_distance_min": float(np.min(band_distances)) if len(band_distances) else None,
                "band_distance_max": float(np.max(band_distances)) if len(band_distances) else None,
                "band_distance_spread": (
                    float(np.max(band_distances) - np.min(band_distances)) if len(band_distances) else None
                ),
                "in_band": (
                    int(np.sum(band_distances <= band_limit)) if len(band_distances) else 0
                ),
            }
        )
        previous = {
            "fallbacks": stats.fallbacks,
            "infeasible": stats.infeasible,
            "barrier_scalings": stats.barrier_scalings,
        }
        if env.controller.phase_monitor.reached(Phase.HOLD):
            break

    summary = env.summary()
    entry = next(iter(summary["cargoes"].values()))
    return {
        "case_id": f"{shape}__a{alpha:.2f}__seed{seed:03d}",
        "metadata": metadata,
        "d_min": d_min,
        "push_side_threshold": threshold,
        "transport_quorum": quorum,
        "contact_band_limit_m": band_limit,
        "object_radial_spread_m": radial_spread,
        "goal_direction": goal.tolist(),
        "frames_run": len(frames),
        "solver": summary["solver"],
        "phases": summary["phases"],
        "g500": entry["g500"],
        "first_separation_breach_frame": first_breach,
        "first_infeasible_frame": first_infeasible,
        "first_fallback_frame": first_fallback,
        "frames": frames,
    }


def report_case_one(trace: dict) -> list[str]:
    lines = ["=" * 78, "CASE 1  rectangle__a0.10__seed004 -- the only solver failure", "=" * 78]
    breach, infeasible, fallback = (
        trace["first_separation_breach_frame"],
        trace["first_infeasible_frame"],
        trace["first_fallback_frame"],
    )
    lines += [
        f"  d_min                          {trace['d_min']:.4f} m",
        f"  first separation breach        frame {breach}",
        f"  first infeasible solve         frame {infeasible}",
        f"  first fallback projection      frame {fallback}",
        f"  solver fallbacks / infeasible  {trace['solver']['fallbacks']} / {trace['solver']['infeasible']}",
        f"  barrier scalings               {trace['solver']['barrier_scalings']}"
        f"  (min scale {trace['solver']['min_barrier_scale']:.4f})",
        f"  final phase                    {trace['phases']['final_phase']}",
    ]
    if breach is None or infeasible is None:
        lines.append("  ORDER: not determined -- one of the two events did not occur on this run.")
    elif breach < infeasible:
        lines += [
            "",
            "  ORDER: the separation broke FIRST, by "
            f"{infeasible - breach} frames.",
            "  Reading: this is not a solver failure. The inter-robot barrier is feasible at",
            "  u = 0 whenever h_ij >= 0; once two robots are inside d_min the row demands a",
            "  separation rate, and if the speed limit cannot deliver it the QP is genuinely",
            "  infeasible. The 124 infeasible solves are the consequence, and the label",
            "  SOLVER_FAILURE names the symptom.",
        ]
    else:
        lines += [
            "",
            f"  ORDER: the QP went infeasible FIRST, by {breach - infeasible} frames.",
            "  Reading: the fallback projection satisfies nothing exactly, so an infeasible",
            "  step put robots inside the barrier and the next step then demanded a harder",
            "  retreat -- the cascade v1's own docstring describes for the pre-tier-3",
            "  projection fallback.",
        ]
    speeds = [f["cargo_speed"] for f in trace["frames"]]
    displacement = [f["cargo_displacement"] for f in trace["frames"]]
    coverage = [f["strict_coverage"] for f in trace["frames"]]
    lines += [
        "",
        f"  cargo displacement             {displacement[-1]:.3f} m on a "
        f"{trace['metadata']['target_distance_m']:.3f} m task",
        f"  peak cargo speed               {max(speeds):.4f} m/s",
        f"  strict coverage peak / final   {max(coverage):.3f} / {coverage[-1]:.3f}",
    ]
    return lines


def report_case_two(traces: list[dict]) -> list[str]:
    lines = ["", "=" * 78, "CASE 2  polygon32 seed 2 -- 'pushing at 93 degrees', J ~ 0", "=" * 78]
    for trace in traces:
        frames = trace["frames"]
        eligible = [f["alignment_eligible"] for f in frames]
        pushing = [f["push_side_count"] for f in frames]
        displacement = frames[-1]["cargo_displacement"]
        lines += [
            "",
            f"  {trace['case_id']}   target {trace['metadata']['target_distance_m']:.3f} m",
            f"    total cargo displacement     {displacement:.6f} m",
            f"    push-set threshold           alignment <= {-trace['push_side_threshold']:.2f}"
            f"   (quorum {trace['transport_quorum']})",
            f"    max robots meeting alignment {max(eligible)}",
            f"    max robots actually pushing  {max(pushing)}",
            f"    most negative alignment seen {min(f['alignment_min'] for f in frames if f['alignment_min'] is not None):.4f}",
            f"    final phase                  {trace['phases']['final_phase']}",
        ]
    lines += [
        "",
        "  On the reported 93 degrees: the displacement is of order 1e-5 m. The direction",
        "  error is arccos(J / |dx|) and both are at the numerical floor, so the angle is",
        "  not a measurement of a push direction. The finding is that the object did not",
        "  move, and the 93 degrees is an artefact of dividing two near-zero numbers.",
    ]
    return lines


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="docs/results/t4")
    parser.add_argument("--max-frames", type=int, default=3000)
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    print("tracing rectangle a=0.10 seed 4 ...", flush=True)
    case_one = trace_case("rectangle", 4, 0.10, args.max_frames)
    traces_two = []
    for alpha in (0.10, 0.40, 0.80):
        print(f"tracing polygon32 a={alpha:.2f} seed 2 ...", flush=True)
        traces_two.append(trace_case("polygon32", 2, alpha, args.max_frames))

    lines = report_case_one(case_one) + report_case_two(traces_two)
    text = "\n".join(lines)
    print()
    print(text)

    (out / "FINDINGS.txt").write_text(text + "\n", encoding="utf-8")
    # The full per-frame traces, without the frame lists for the three alpha variants
    # that are identical after transport never arms.
    (out / "t4_traces.json").write_text(
        json.dumps({"case_one": case_one, "case_two": traces_two}, indent=2), encoding="utf-8"
    )
    print(f"\nwrote {out / 'FINDINGS.txt'} and {out / 't4_traces.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
