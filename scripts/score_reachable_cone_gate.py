#!/usr/bin/env python
"""Score the 12-seed baseline under the reachable-cone gate instead of an absolute distance.

    python scripts/score_reachable_cone_gate.py --out docs/results/t3

Reads the committed `docs/results/t3/lateral_authority.json`; runs no episodes.

The case for changing the gate
------------------------------
`CLOSED_LOOP_V2.md` §3.2 established the identity ``max cross-track = J sin(direction
error)`` at correlation 0.981, so ``cross_track_max <= 0.15 m`` is a direction-error
requirement whose threshold depends on how far the object travelled. §7a then measured the
consequence directly: across the distance sweep the implied requirement falls from 13.52
degrees at alpha = 0.2 to **2.65 degrees** at alpha = 1.0, purely as arithmetic, while the
measured direction error stays in a 5-8 degree band and coverage, efficiency, separation and
solver behaviour are all flat. At alpha = 1.0 every episode fails a gate that has become
strict for reasons unconnected to the team's ability to aim.

The replacement
---------------
The net force is a nonnegative combination of the press directions ``{-n_k}``, so the
achievable direction set is their convex hull: an angular interval ``Phi``. Write
``s = dist(d_goal, Phi)`` for the angular shortfall, zero when the goal is reachable. The
part of the direction error the team could not have avoided is ``s``; the rest is control
error. So the gate becomes

    direction error  <=  s  +  eps_control

and, through the identity, equivalently ``cross-track <= J sin(s + eps_control)``.

``eps_control`` is a **declared** control-accuracy budget, exactly as ``cross_track_max``
was a declared distance budget. It is not fitted here: the script sweeps it and reports the
whole curve, so the choice stays a decision someone makes rather than a number this script
picked to make a result come out.

Every term is measurable on board. Each robot knows its own observed normal; the interval
bounds max-consensus across the team exactly as the enclosure bitmap does. An absolute 0.15 m
cross-track cannot be evaluated by the team at all, because no robot knows ``J``.

What this does not do
---------------------
It does not change `g500`. The committed results stay scored by the gate the earlier results
were scored by; this reports what a different gate would have said, which is the evidence
someone would need before changing one.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

SOURCE = ROOT / "docs" / "results" / "t3" / "lateral_authority.json"
CROSS_TRACK_GATE = 0.15


def score(runs: list[dict], eps_control_deg: float, shortfall_key: str) -> dict:
    """Both gates over the same episodes, so the difference is the gate and nothing else."""
    absolute, cone, both, neither = [], [], [], []
    for r in runs:
        s = float(r[shortfall_key])
        allowed = s + eps_control_deg
        passes_absolute = r["max_cross_track"] <= CROSS_TRACK_GATE
        passes_cone = r["direction_error_deg"] <= allowed
        (absolute if passes_absolute else neither).append(r["seed"])
        if passes_cone:
            cone.append(r["seed"])
        if passes_absolute and passes_cone:
            both.append(r["seed"])
    return {
        "eps_control_deg": eps_control_deg,
        "shortfall_key": shortfall_key,
        "absolute_pass": len(absolute),
        "cone_pass": len(cone),
        "agree": len(runs) - len(set(absolute) ^ set(cone)),
        "absolute_only": sorted(set(absolute) - set(cone)),
        "cone_only": sorted(set(cone) - set(absolute)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--source", default=str(SOURCE))
    parser.add_argument("--eps", default="2,4,6,8,10",
                        help="Declared control-accuracy budgets in degrees, swept.")
    parser.add_argument("--out", default="docs/results/t3")
    args = parser.parse_args()

    payload = json.loads(Path(args.source).read_text(encoding="utf-8"))
    runs = payload["runs"]
    epsilons = [float(e) for e in args.eps.split(",") if e.strip()]

    print(f"{'seed':>5} {'J':>7} {'dir':>7} {'cross':>8} {'shortfall':>10} "
          f"{'implied abs gate':>17} {'absolute':>9}")
    for r in runs:
        implied = (
            math.degrees(math.asin(min(1.0, CROSS_TRACK_GATE / r["J"])))
            if r["J"] > CROSS_TRACK_GATE else 90.0
        )
        print(f"{r['seed']:>5} {r['J']:>7.3f} {r['direction_error_deg']:>7.2f} "
              f"{r['max_cross_track']:>8.4f} {r['shortfall_mean_deg']:>10.3f} "
              f"{implied:>17.2f} {'pass' if r['max_cross_track'] <= CROSS_TRACK_GATE else 'FAIL':>9}")

    tables = {}
    for key in ("shortfall_mean_deg", "shortfall_max_deg"):
        tables[key] = [score(runs, e, key) for e in epsilons]

    report = {
        "experiment": "reachable-cone gate, scored over the committed 12-seed lateral-authority run",
        "source": str(Path(args.source).name),
        "absolute_gate_m": CROSS_TRACK_GATE,
        "identity": payload["identity"],
        "note": (
            "Reported, not adopted. g500 still scores cross_track_max <= 0.15 so the "
            "committed results stay comparable; this is what a cone gate would have said."
        ),
        "sweeps": tables,
    }
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "cone_gate_scoring.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print()
    for key, rows in tables.items():
        print(f"--- shortfall taken as {key} ---")
        print(f"{'eps_control':>12} {'absolute':>9} {'cone':>6} {'agree':>7} "
              f"{'abs only':>20} {'cone only':>20}")
        for row in rows:
            print(f"{row['eps_control_deg']:>12.1f} {row['absolute_pass']:>6}/12 "
                  f"{row['cone_pass']:>3}/12 {row['agree']:>5}/12 "
                  f"{str(row['absolute_only']):>20} {str(row['cone_only']):>20}")
        print()
    print("The absolute gate's verdict is fixed at 5/12 here because it does not depend on")
    print("eps_control. The cone column is what a declared control budget would give, and the")
    print("two 'only' columns name the episodes on which the choice of gate actually matters.")
    print(f"wrote {out / 'cone_gate_scoring.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
