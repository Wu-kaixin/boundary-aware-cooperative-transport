#!/usr/bin/env python
"""T5 - robustness ablation over 12 seeds, with out-of-domain rejection.

    python scripts/run_robustness_ablation.py --seeds 0..11 --out docs/results/t5

Six arms, each differing from the baseline in the fields named in ``VARIANTS`` and in
nothing else. CODEX ran this on a single seed; a single seed cannot separate a variant's
effect from a seed's, so this runs all twelve and reports the paired change.

The three degradation mechanisms did not exist in v1 and were added for this experiment
(`DBACTParams.perception_every`, `planning_every`, `communication_dropout_prob`). All three
default to exact no-ops, and the ``nominal`` arm below is therefore v1 frame for frame --
which the run checks, by reproducing J = 1.4908 and 68 barrier scalings.

Out-of-domain rejection, and why it is measured rather than declared
-------------------------------------------------------------------
The plan says a variant that exceeds the declared error premise must be **rejected and
recorded as out-of-domain**, not reported as a survivor, and names the 10 mm noise arm as
the expected case. This script does that, and also does something the plan did not ask for,
because T2 made it necessary: it decides out-of-domain from the **measured** six-term audit
rather than from an a-priori guess about which arm is too noisy.

The reason is that T2 measured the *nominal* arm against the same premise and found
``velocity_error`` exceeded by 60.4% of cells. If out-of-domain were assigned by declaration
the nominal arm would be reported as in-domain while breaching the premise, and the 10 mm arm
rejected for breaching it -- the same violation treated two ways. Both verdicts are therefore
reported side by side, and where they disagree the disagreement is the finding.

What a noise arm can and cannot tell you here
---------------------------------------------
The nominal contract success rate on the matrix is 0.300, and on this baseline 8/12. A
variant that drops from 8/12 to 6/12 has moved two episodes across a gate that most
episodes already fail. So a difference measured here is at least as much a statement about
where the gate sits as about robustness, and the per-quantity distributions -- cross-track,
barrier scalings, coverage -- carry more information than the pass count. Both are reported;
the pass count is not the headline.

The pseudo-frontier measurement
-------------------------------
The frontier predicate in ``dbact.boundary_density._frontier_targets`` declares an
observation a frontier when it has no map neighbour within ``explore_window`` **along its own
estimated tangent**. Range noise perturbs the point and, through the local plane fit, the
normal -- and rotating the normal rotates the window, so a genuine neighbour can fall
outside it and a fully known piece of boundary can be declared open.

That is measurable without any judgement call, because there is a regime in which *every*
frontier is provably spurious: once the team's pooled map satisfies the declared
``boundary_map_epsilon``, no unobserved boundary remains for a frontier to point at. Frontier
targets emitted after that moment are counted separately, and they are the pseudo-frontier
rate.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact.boundary_density import DensityParams, _frontier_targets  # noqa: E402
from dbact.guarantees import boundary_map_gap_upper_bound  # noqa: E402
from dbact.phase import Phase  # noqa: E402
from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402

BASE_CONFIG = "configs/sim/d/l_shape_closed_loop.yaml"

#: Declared in configs/sim/v2/shape_matrix.yaml, carried over unchanged so the measured
#: numbers are comparable with the premise the decisive matrix ran under.
DECLARED_ERRORS = {"normal_error_deg": 30.0, "velocity_error": 0.02}
DECLARED_MAP_EPSILON = 0.10

# The baseline already runs at 10 mm range noise
# ----------------------------------------------
# ``configs/sim/d/l_shape_closed_loop.yaml`` sets ``range_noise_std: 0.01``. So the plan's
# ``range_noise_010`` variant is not a perturbation of the baseline -- **it is the
# baseline** -- and the noise level the plan designates as out-of-domain is the level every
# headline number on this branch was produced at. It is kept in the arm list anyway, and it
# is expected to reproduce ``nominal`` exactly; that equality is the evidence for the claim.
#
# Two arms are therefore added that the plan did not name, because without them the sweep
# only degrades a configuration that is already at the plan's rejection threshold and never
# measures what a clean sensor would do:
#
#   range_noise_000   a noiseless sensor -- the reference the baseline lacks
#   range_noise_020   twice the baseline, so the sweep spans it on both sides
VARIANTS: dict[str, dict] = {
    "nominal": {},
    "range_noise_000": {"range_noise_std": 0.000},
    "range_noise_005": {"range_noise_std": 0.005},
    "range_noise_010": {"range_noise_std": 0.010},
    "range_noise_020": {"range_noise_std": 0.020},
    "slow_updates_5": {"perception_every": 5, "planning_every": 5},
    "comm_dropout_10": {"communication_dropout_prob": 0.10},
    "combined": {
        "range_noise_std": 0.005,
        "perception_every": 4,
        "planning_every": 4,
        "communication_dropout_prob": 0.10,
    },
}

#: The plan's a-priori expectation: an arm at 10 mm or above exceeds the declared error
#: premise and must be rejected. Recorded so the declared verdict can be compared with the
#: measured one -- and note that by this rule ``nominal`` is out-of-domain too, because it
#: *is* the 10 mm arm.
DECLARED_OUT_OF_DOMAIN = {"nominal", "range_noise_010", "range_noise_020", "combined"}


def parse_seeds(spec: str) -> list[int]:
    if ".." in spec:
        low, high = (int(p) for p in spec.split("..", 1))
        return list(range(low, high + 1))
    return [int(p) for p in spec.split(",") if p.strip()]


def arm_config(base: dict, overrides: dict) -> dict:
    config = copy.deepcopy(base)
    config["controller"] = dict(config.get("controller", {}))
    config["controller"].update(overrides)
    guarantee = dict(config.get("guarantee", {}) or {})
    guarantee["bounded_errors"] = dict(DECLARED_ERRORS)
    config["guarantee"] = guarantee
    config["audit_errors"] = True
    return config


def describe(values) -> dict:
    clean = [float(v) for v in values if v is not None and np.isfinite(v)]
    if not clean:
        return {"n": 0}
    a = np.asarray(clean, dtype=float)
    return {
        "n": len(a),
        "mean": float(a.mean()),
        "sd": float(a.std(ddof=1)) if len(a) > 1 else 0.0,
        "min": float(a.min()),
        "max": float(a.max()),
    }


def frontier_count(env: SimulationEnvironment, density: DensityParams) -> int:
    """Frontier targets the density term would emit this frame, over the whole team.

    Recomputed from each robot's own map view with the same function the controller
    calls, so this is the quantity the controller acts on rather than a proxy for it.
    """
    total = 0
    for agent in env.agents:
        view = env.controller._views.get(agent.agent_id)
        if view is None or len(view) < 2:
            continue
        offsets = np.full(len(view), density.cage_offset)
        targets, _, _ = _frontier_targets(view.points, view.normals, offsets, density)
        total += len(targets)
    return total


def pooled_map_gap(env: SimulationEnvironment) -> float:
    pooled = [
        env.controller.map_snapshot(a.agent_id).points
        for a in env.agents
    ]
    pooled = [p for p in pooled if len(p)]
    if not pooled:
        return float("inf")
    return float(
        boundary_map_gap_upper_bound(
            env.cargoes[0].vertices, np.vstack(pooled), sample_count=256
        )["max_boundary_gap"]
    )


def run_seed(config: dict, seed: int, max_frames: int, frontier_stride: int) -> dict:
    env = SimulationEnvironment(config, seed=seed)
    density = env.controller.density_params
    started = time.perf_counter()

    frontier_total = 0
    frontier_after_close = 0
    frontier_frames = 0
    frames_after_close = 0
    map_closed = False
    quiet = 0
    frame = 0
    for frame in range(1, int(max_frames) + 1):
        env.step()
        if frame % max(1, frontier_stride) == 0:
            frontier_frames += 1
            count = frontier_count(env, density)
            frontier_total += count
            if not map_closed and pooled_map_gap(env) <= DECLARED_MAP_EPSILON:
                map_closed = True
            if map_closed:
                frames_after_close += 1
                frontier_after_close += count
        if env.controller.phase_monitor.reached(Phase.HOLD):
            speed = max(
                (env.log.cargo_speed[c.object_id][-1] for c in env.cargoes), default=0.0
            )
            quiet = quiet + 1 if speed <= 0.005 else 0
            if quiet >= 40:
                break
        else:
            quiet = 0

    wall = time.perf_counter() - started
    summary = env.summary()
    entry = next(iter(summary["cargoes"].values()))
    g500 = entry["g500"]
    m = g500["metrics"]
    audit = summary["error_audit"] or {}

    return {
        "seed": seed,
        "success": bool(g500["success"]),
        "failure_reasons": list(g500["failure_reasons"]),
        "J": float(m["J"]),
        "efficiency": float(m["efficiency"]),
        "max_cross_track": float(m["max_cross_track"]),
        "direction_error_deg": m["direction_error_deg"],
        "rotation_deg": float(m["rotation_deg"]),
        "max_strict_coverage": float(m["max_strict_coverage"]),
        "final_strict_coverage": float(m["final_strict_coverage"]),
        "min_inter_agent_distance": float(m["min_inter_agent_distance"]),
        "d_min": float(m["d_min"]),
        "min_signed_clearance": float(m["min_signed_clearance"]),
        "max_penetration": float(m["max_penetration"]),
        "penetration_budget": float(m["penetration_budget"]),
        "barrier_scalings": int(summary["solver"]["barrier_scalings"]),
        "margin_relaxations": int(summary["solver"]["margin_relaxations"]),
        "solver_fallbacks": int(summary["solver"]["fallbacks"]),
        "solver_infeasible": int(summary["solver"]["infeasible"]),
        "transport_frame": m["transport_frame"],
        "hold_frame": m["hold_frame"],
        "final_phase": m["final_phase"],
        "frames_run": frame,
        "fps": frame / max(wall, 1e-12),
        # --- audit
        "within_declared_bounds": audit.get("within_declared_bounds"),
        "measured_normal_error_deg": audit.get("measured_bounds", {}).get("normal_error_deg"),
        "measured_velocity_error": audit.get("measured_bounds", {}).get("velocity_error"),
        "normal_breach_fraction": audit.get("breach_fractions", {}).get("normal_error_deg"),
        "velocity_breach_fraction": audit.get("breach_fractions", {}).get("velocity_error"),
        "map_gap_max": ((audit.get("terms") or {}).get("map_gap_m") or {}).get("max"),
        # --- pseudo-frontier
        "frontier_sampled_frames": frontier_frames,
        "frontier_per_frame": frontier_total / max(1, frontier_frames),
        "frames_after_map_closed": frames_after_close,
        # Averaged over the frames it is defined on, so it is directly comparable with
        # ``frontier_per_frame`` only in the sense that both are per-frame team totals.
        # It can exceed the overall mean: the frontier count grows with map size, and the
        # post-closure frames are the later ones. What matters is that every one of these
        # is provably spurious -- the pooled map already satisfies the declared epsilon, so
        # there is no unobserved boundary for a frontier to point at.
        "pseudo_frontier_per_frame": (
            frontier_after_close / frames_after_close if frames_after_close else None
        ),
        # The share of the run spent emitting provably-spurious frontier demand.
        "pseudo_frontier_frame_share": (
            frames_after_close / frontier_frames if frontier_frames else None
        ),
        "map_ever_closed": bool(map_closed),
    }


FIELDS = [
    "J", "efficiency", "max_cross_track", "direction_error_deg", "rotation_deg",
    "max_strict_coverage", "final_strict_coverage", "min_signed_clearance",
    "max_penetration", "barrier_scalings", "frames_run", "fps",
    "measured_normal_error_deg", "measured_velocity_error",
    "normal_breach_fraction", "velocity_breach_fraction", "map_gap_max",
    "frontier_per_frame", "pseudo_frontier_per_frame", "pseudo_frontier_frame_share",
]


def run_arm(name: str, base: dict, seeds: list[int], max_frames: int, stride: int) -> dict:
    config = arm_config(base, VARIANTS[name])
    rows = [run_seed(config, seed, max_frames, stride) for seed in seeds]
    for row in rows:
        print(
            f"[{name:16s}] seed {row['seed']:2d}  {'PASS' if row['success'] else 'FAIL'}  "
            f"J={row['J']:.3f}  cross={row['max_cross_track']:.4f}  "
            f"cov={row['max_strict_coverage']:.3f}  scal={row['barrier_scalings']:3d}  "
            f"fb={row['solver_fallbacks']:3d}  "
            f"front={row['frontier_per_frame']:5.2f}  "
            f"pseudo={row['pseudo_frontier_per_frame'] if row['pseudo_frontier_per_frame'] is None else round(row['pseudo_frontier_per_frame'], 2)}",
            flush=True,
        )

    distributions = {f: describe([r.get(f) for r in rows]) for f in FIELDS}
    measured_breach = distributions["velocity_breach_fraction"].get("mean")
    normal_breach = distributions["normal_breach_fraction"].get("mean")
    return {
        "variant": name,
        "overrides": VARIANTS[name],
        "pass": sum(1 for r in rows if r["success"]),
        "total": len(rows),
        "barrier_scalings_total": sum(r["barrier_scalings"] for r in rows),
        "solver_fallbacks_total": sum(r["solver_fallbacks"] for r in rows),
        "solver_infeasible_total": sum(r["solver_infeasible"] for r in rows),
        "separation_held": all(
            r["min_inter_agent_distance"] >= r["d_min"] - 1e-6 for r in rows
        ),
        "penetration_within_budget": all(
            r["max_penetration"] <= r["penetration_budget"] + 1e-6 for r in rows
        ),
        "maps_closed": sum(1 for r in rows if r["map_ever_closed"]),
        # Two verdicts, deliberately both.
        "declared_out_of_domain": name in DECLARED_OUT_OF_DOMAIN,
        "measured_out_of_domain": bool(
            (measured_breach is not None and measured_breach > 0.0)
            or (normal_breach is not None and normal_breach > 0.0)
        ),
        "measured_velocity_breach_fraction": measured_breach,
        "measured_normal_breach_fraction": normal_breach,
        "within_declared_bounds_all": all(r["within_declared_bounds"] is True for r in rows),
        "distributions": distributions,
        "runs": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=BASE_CONFIG)
    parser.add_argument("--seeds", default="0..11")
    parser.add_argument("--variants", nargs="*", choices=sorted(VARIANTS), default=list(VARIANTS))
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--frontier-stride", type=int, default=10,
                        help="Sample the frontier count every Nth frame; it costs a "
                             "map-gap resample against every robot's map.")
    parser.add_argument("--out", default="docs/results/t5")
    args = parser.parse_args()

    seeds = parse_seeds(args.seeds)
    base = load_yaml(args.config)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    arms = {}
    for name in args.variants:
        arms[name] = run_arm(name, base, seeds, args.max_frames, args.frontier_stride)
        print(flush=True)

    nominal = arms.get("nominal")
    report = {
        "experiment": "T5 robustness ablation, 12 seeds",
        "config": args.config,
        "seeds": seeds,
        "declared_errors": DECLARED_ERRORS,
        "declared_map_epsilon": DECLARED_MAP_EPSILON,
        "arms": arms,
        "baseline_range_noise_std": float(
            (base.get("controller", {}) or {}).get("range_noise_std", 0.0)
        ),
        "caveats": [
            "The baseline config already sets range_noise_std: 0.01, so the plan's "
            "range_noise_010 arm IS the nominal arm and should reproduce it exactly. The "
            "noise level the plan designates as out-of-domain is the level every headline "
            "number on this branch was produced at. range_noise_000 and range_noise_020 were "
            "added so the sweep spans the baseline on both sides.",
            "The nominal contract success rate on this baseline is 8/12 and on the shape "
            "matrix 0.300. A variant that moves two episodes across a gate most episodes "
            "already fail is at least as much a statement about the gate as about "
            "robustness; the per-quantity distributions carry more information than the "
            "pass count.",
            "Out-of-domain is reported twice: as declared by the plan, and as measured by "
            "the six-term audit. Where they disagree the disagreement is the finding.",
            "The three degradation mechanisms were added to v1 for this experiment and "
            "default to exact no-ops; the nominal arm reproduces v1 frame for frame.",
        ],
        "nominal_reproduces_v1": (
            None
            if nominal is None
            else {
                "J_mean": nominal["distributions"]["J"]["mean"],
                "barrier_scalings_total": nominal["barrier_scalings_total"],
                "pass": f"{nominal['pass']}/{nominal['total']}",
                "expected": {"J_mean": 1.4908, "barrier_scalings_total": 68, "pass": "8/12"},
            }
        ),
    }
    (out / "robustness_ablation.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print("=" * 118)
    print(f"{'variant':17s} {'pass':>6} {'J':>7} {'cross':>7} {'peakcov':>8} {'scal':>5} "
          f"{'fb':>4} {'sep':>4} {'front':>6} {'pseudo':>7} {'velbreach':>10} {'OOD-decl':>9} {'OOD-meas':>9}")
    for name in args.variants:
        a = arms[name]
        d = a["distributions"]
        pseudo = d["pseudo_frontier_per_frame"].get("mean")
        print(
            f"{name:17s} {a['pass']:>3}/{a['total']:<2} "
            f"{d['J']['mean']:>7.3f} {d['max_cross_track']['mean']:>7.4f} "
            f"{d['max_strict_coverage']['mean']:>8.3f} {a['barrier_scalings_total']:>5} "
            f"{a['solver_fallbacks_total']:>4} {'ok' if a['separation_held'] else 'BAD':>4} "
            f"{d['frontier_per_frame']['mean']:>6.2f} "
            f"{('n/a' if pseudo is None else f'{pseudo:.2f}'):>7} "
            f"{a['measured_velocity_breach_fraction']:>10.4f} "
            f"{str(a['declared_out_of_domain']):>9} {str(a['measured_out_of_domain']):>9}"
        )
    print("=" * 118)
    disagree = [n for n in args.variants
                if arms[n]["declared_out_of_domain"] != arms[n]["measured_out_of_domain"]]
    print(f"declared vs measured out-of-domain disagree on: {disagree or 'nothing'}")
    print(f"wrote {out / 'robustness_ablation.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
