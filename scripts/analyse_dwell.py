#!/usr/bin/env python
"""D10-DWELL - screening candidate contact-quorum predicates without re-running.

    PYTHONPATH=src python scripts/analyse_dwell.py --run runs/d10_dwell

Reads the per-frame distance and standoff-setpoint arrays that
`scripts/diagnose_dwell.py` wrote and re-evaluates the dwell under alternative
membership predicates.

**These are screens, not predictions, and the difference is not a formality.**
Unlike the enclosure gate -- where `ENCLOSE` turned out to be inert in the control
path, which made the counterfactual exact -- `contact_ready` is read by the
transport gate, by the standoff loop and by the redeploy rule as well as by the
phase machine. Changing it changes the trajectory these arrays were recorded on.
What the numbers below establish is whether a candidate is worth an A/B, and
nothing more.

Candidates:

    baseline    d <= band
    hysteresis  enter at d <= band, leave at d <= band + h
    arc-aware   d <= band AND ring <= band

The third is the one the diagnosis points at. A robot whose own standoff floor
sits above the band is being actively driven out of it by design -- the leading
arc has to stand clear so it does not resist the push -- so counting it toward a
*contact* quorum admits members that are guaranteed to leave. Excluding them is
not a threshold change: it is the statement that the quorum should count robots
that are supposed to be in contact.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def streak(inside: np.ndarray, frames: np.ndarray, quorum: int, dwell: int) -> int | None:
    """First frame the quorum has been held for ``dwell`` consecutive steps."""
    counts = np.count_nonzero(inside, axis=1)
    run = 0
    for index, count in enumerate(counts):
        run = run + 1 if count >= quorum else 0
        if run >= dwell:
            return int(frames[index])
    return None


def hysteresis_membership(distance: np.ndarray, band: float, extra: float) -> np.ndarray:
    inside = np.zeros(distance.shape, dtype=bool)
    state = np.zeros(distance.shape[1], dtype=bool)
    for step in range(distance.shape[0]):
        state = np.where(state, distance[step] <= band + extra, distance[step] <= band)
        inside[step] = state
    return inside


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", default="runs/d10_dwell")
    args = parser.parse_args()

    run = Path(args.run)
    payload = json.loads((run / "dwell.json").read_text(encoding="utf-8"))
    quorum, dwell = payload["quorum"], payload["dwell"]

    for gain_key, rows in payload["arms"].items():
        gain = float(gain_key)
        band = rows[0]["threshold"]
        print(f"\n=== explore_gain = {gain:g}, band = {band:.3f} m " + "=" * 34)
        print(f"{'seed':>4} {'baseline':>9} {'hyst+.01':>9} {'hyst+.02':>9} {'arc-aware':>10} "
              f"{'pool min':>9} {'pool med':>9} {'actual CR':>10}")
        totals: dict[str, list[float]] = {}
        for row in rows:
            seed = row["seed"]
            data = np.load(run / f"dwell_gain{gain:g}_seed{seed}.npz")
            frames, distance, ring = data["frame"], data["distance"], data["ring"]
            eligible = np.isfinite(ring) & (ring <= band)
            base = distance <= band

            results = {
                "baseline": streak(base, frames, quorum, dwell),
                "hyst0.01": streak(hysteresis_membership(distance, band, 0.01), frames, quorum, dwell),
                "hyst0.02": streak(hysteresis_membership(distance, band, 0.02), frames, quorum, dwell),
                "arc": streak(base & eligible, frames, quorum, dwell),
            }
            # Does the arc-aware pool ever fall below the quorum? If it does, the
            # predicate cannot certify a quorum however long the run continues,
            # and that is a deadlock rather than a delay.
            pool = np.count_nonzero(eligible, axis=1)
            for key, value in results.items():
                totals.setdefault(key, []).append(value)
            fmt = lambda v: "never" if v is None else str(v)  # noqa: E731
            print(f"{seed:>4} {fmt(results['baseline']):>9} {fmt(results['hyst0.01']):>9} "
                  f"{fmt(results['hyst0.02']):>9} {fmt(results['arc']):>10} "
                  f"{int(pool.min()):>9} {int(np.median(pool)):>9} {row['contact_ready']:>10}")

        print()
        for key, values in totals.items():
            live = [v for v in values if v is not None]
            missing = len(values) - len(live)
            note = f"   ({missing} never reach it -> deadlock)" if missing else ""
            mean = float(np.mean(live)) if live else float("nan")
            print(f"  {key:12s} mean streak {mean:8.1f}{note}")


if __name__ == "__main__":
    main()
