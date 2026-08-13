#!/usr/bin/env python
"""T6 - one representative episode, saved with everything a figure or a frame needs.

    python scripts/run_publication_representative.py --seed 5 --out docs/results/representative

Writes the four products v1's own pipeline expects, and nothing else:

* ``summary.json``          the run's contracts, solver provenance and G500 verdict
* ``safety_timeseries.csv`` per frame per cargo: coverage, clearance, penetration,
                            contact count, net force and net torque
* ``trajectories.csv``      agent and cargo positions per frame
* ``replay.npz``            everything ``dbact_sim.replay`` needs, and nothing it does not

The animation is **not** produced here. Rendering during or after the run inside this
script would put matplotlib on the same clock as the physics, which is the thing
``env.save_replay``'s docstring exists to prevent. Render it afterwards:

    python scripts/render_closed_loop.py docs/results/representative

That path goes through v1's ``dbact_sim.replay``, which keeps the v1 phase palette and
draws **one robot's own map** beside the true outline rather than reconstructing a surface
from ground truth.

Which seed is representative, and why saying so matters
------------------------------------------------------
The default is seed 5, and it is chosen for being *ordinary* rather than for being good:
on the 12-seed baseline it passes the contract, its cross-track is 0.070 m against a 0.186 m
mean, and it needs no barrier scalings. A figure set built on the best seed is a highlight
reel; one built on a failure is a different kind of misdirection. ``--seed`` is exposed so
that a reader can produce the same products for any of the twelve, and the seed is recorded
in ``summary.json``, so no figure can claim to be the typical case without naming which case
it is.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402

BASE_CONFIG = "configs/sim/d/l_shape_closed_loop.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=BASE_CONFIG)
    parser.add_argument("--seed", type=int, default=5)
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--out", default="docs/results/representative")
    args = parser.parse_args()

    config = load_yaml(args.config)
    # The six-term error audit, so the representative run carries the same measurement the
    # ablations do and a reader can check one episode's perception error by hand.
    guarantee = dict(config.get("guarantee", {}) or {})
    guarantee["bounded_errors"] = {"normal_error_deg": 30.0, "velocity_error": 0.02}
    config["guarantee"] = guarantee
    config["audit_errors"] = True

    env = SimulationEnvironment(config, seed=args.seed)
    started = time.perf_counter()
    termination = env.run_until_settled(max_frames=args.max_frames)
    wall = time.perf_counter() - started

    out = Path(args.out)
    summary = env.save_outputs(out)
    entry = next(iter(summary["cargoes"].values()))
    g500 = entry["g500"]
    m = g500["metrics"]

    print(f"seed {args.seed}  {'PASS' if g500['success'] else 'FAIL'}")
    print(f"  J                {m['J']:.4f} m   efficiency {m['efficiency']:.4f}")
    print(f"  max cross-track  {m['max_cross_track']:.4f} m")
    print(f"  direction error  {m['direction_error_deg']:.2f} deg")
    print(f"  rotation         {m['rotation_deg']:+.3f} deg")
    print(f"  barrier scalings {summary['solver']['barrier_scalings']}   "
          f"fallbacks {summary['solver']['fallbacks']}   "
          f"infeasible {summary['solver']['infeasible']}")
    audit = summary.get("error_audit") or {}
    if audit:
        print(f"  within declared error bounds: {audit.get('within_declared_bounds')}   "
              f"reasons {audit.get('fail_closed_reasons')}")
    print(f"  frames {termination['frames_run']} ({termination['terminated_by']})  "
          f"{termination['frames_run'] / max(wall, 1e-12):.1f} fps")
    print()
    for name in ("summary.json", "safety_timeseries.csv", "trajectories.csv", "replay.npz"):
        path = out / name
        print(f"  {'ok ' if path.exists() else 'MISSING'} {path}")
    print()
    print(f"render the frames with:  python scripts/render_closed_loop.py {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
