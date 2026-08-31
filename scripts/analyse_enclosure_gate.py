#!/usr/bin/env python
"""D10-ENC - what any change to the enclosure gate could possibly buy.

    PYTHONPATH=src python scripts/analyse_enclosure_gate.py --run runs/d10_enc

Reads the trace `scripts/diagnose_enclosure_gate.py` wrote. No simulation, so the
argument can be re-checked in a second instead of in twenty minutes.

The whole thing rests on one structural fact, which is checked here rather than
assumed: **nothing in the control path reads `ENCLOSE`.** Grep the controller and
the only phases it branches on are `CONTACT_READY`, `TRANSPORT` and `HOLD`; the
`ENCLOSE` label exists solely as the precondition the monotone machine needs
before it will arm the contact dwell. So moving the `DISCOVER -> ENCLOSE` gate
cannot change a single command issued before contact-ready, and therefore cannot
change `T_streak20`, the frame at which the contact quorum has been held for the
dwell. That gives an exact identity,

    T_contact_ready  =  max( T_gate, T_streak20 )  -  1

which is verified against the measured runs on every seed. It turns "when would
this candidate gate have fired" into "what would contact-ready have been" with no
counterfactual hand-waving, and it puts a hard ceiling on the whole exercise: the
best conceivable gate -- one that fires at frame 0 -- lands at `T_streak20`.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

DWELL_DEFAULT = 20
QUORUM_DEFAULT = 4


def streak_frame(frames: np.ndarray, contact_ready: np.ndarray, quorum: int, dwell: int) -> int | None:
    """First frame at which the contact quorum has been held for ``dwell`` steps."""
    run = 0
    for index, count in enumerate(contact_ready):
        run = run + 1 if count >= quorum else 0
        if run >= dwell:
            return int(frames[index])
    return None


def load(run: Path) -> tuple[dict, dict[int, dict]]:
    payload = json.loads((run / "gate.json").read_text(encoding="utf-8"))
    series = {
        report["seed"]: dict(np.load(run / f"gate_seed{report['seed']}.npz"))
        for report in payload["reports"]
    }
    return payload, series


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", default="runs/d10_enc")
    parser.add_argument("--quorum", type=int, default=QUORUM_DEFAULT)
    parser.add_argument("--dwell", type=int, default=DWELL_DEFAULT)
    parser.add_argument("--figure", action="store_true", help="Write the tradeoff figure.")
    args = parser.parse_args()

    run = Path(args.run)
    payload, series = load(run)
    reports = payload["reports"]
    seeds = [r["seed"] for r in reports]

    gate = {r["seed"]: r["milestones"]["T_current_gate"] for r in reports}
    actual = {r["seed"]: r["milestones"]["T_contact_ready"] for r in reports}
    streak = {
        seed: streak_frame(series[seed]["frame"].astype(int), series[seed]["contact_ready"],
                           args.quorum, args.dwell)
        for seed in seeds
    }

    residual = [actual[s] - (max(gate[s], streak[s]) - 1) for s in seeds]
    identity_holds = set(residual) == {0}
    print(f"identity  T_contact_ready == max(T_gate, T_streak{args.dwell}) - 1 : "
          f"{'holds on all seeds' if identity_holds else f'FAILS, residuals {residual}'}")
    if not identity_holds:
        raise SystemExit(
            "The identity does not hold, so the exact counterfactual below is not valid. "
            "Either the control path now reads ENCLOSE, or the dwell/quorum differ from "
            "the arguments given."
        )

    print(f"\n{'seed':>4} {'gate':>6} {'streak':>7} {'binds':>6} {'CR':>6} "
          f"{'gate cost':>10} {'quorum':>7} {'chatter':>8}")
    quorum_first = {
        seed: next((int(f) for f, c in zip(series[seed]["frame"], series[seed]["contact_ready"])
                    if c >= args.quorum), None)
        for seed in seeds
    }
    for seed in seeds:
        binds = "gate" if gate[seed] >= streak[seed] else "dwell"
        chatter = streak[seed] - (quorum_first[seed] or streak[seed]) - (args.dwell - 1)
        print(f"{seed:>4} {gate[seed]:>6} {streak[seed]:>7} {binds:>6} {actual[seed]:>6} "
              f"{max(0, gate[seed] - streak[seed]):>10} {quorum_first[seed]:>7} {chatter:>8}")

    now = float(np.mean([actual[s] for s in seeds]))
    floor = float(np.mean([streak[s] - 1 for s in seeds]))
    print(f"\ncurrent contact-ready      {now:7.1f}")
    print(f"dwell floor (oracle gate)  {floor:7.1f}")
    print(f"headroom for ANY gate      {now - floor:7.1f} frames ({100 * (now - floor) / now:.1f}%)")
    chatter = float(np.mean([
        streak[s] - (quorum_first[s] or streak[s]) - (args.dwell - 1) for s in seeds
    ]))
    print(f"quorum chatter beyond the dwell {chatter:6.1f} frames -- larger than the gate headroom")

    print(f"\n{'candidate gate':34s} {'fires':>6} {'mean CR':>9} {'vs now':>8} {'strict@fire':>12}")
    rows = []
    for name in reports[0]["gate_frames"]:
        frames = {r["seed"]: r["gate_frames"][name] for r in reports}
        misses = [s for s in seeds if frames[s] is None]
        screens = [r["screens"][name] for r in reports if r["screens"][name]["fired"]]
        quality = float(np.mean([s["strict_coverage_at"] for s in screens])) if screens else float("nan")
        if misses:
            print(f"{name:34s} {8 - len(misses)}/8    never fires on seeds {misses} -> deadlock")
            rows.append((name, np.nan, quality, len(misses)))
            continue
        counterfactual = float(np.mean([max(frames[s], streak[s]) - 1 for s in seeds]))
        print(f"{name:34s} 8/8    {counterfactual:9.1f} {counterfactual - now:+8.1f} {quality:12.3f}")
        rows.append((name, counterfactual, quality, 0))

    print(f"{'ORACLE (fires at frame 0)':34s} 8/8    {floor:9.1f} {floor - now:+8.1f}")

    if args.figure:
        write_figure(run, rows, now, floor)
        print(f"\nfigure written to {run / 'figF_gate_tradeoff.png'}")


def write_figure(run: Path, rows: list[tuple], now: float, floor: float) -> None:
    """Delay against the enclosure it certifies -- the tradeoff, not a ranking.

    A gate is only interesting in the lower-right: earlier *and* certifying a
    real enclosure. The region is empty here, and that is the result.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 6))
    for name, cf, quality, misses in rows:
        if misses:
            continue
        colour = "C3" if name.startswith("G0") else "C0"
        ax.scatter(cf, quality, s=70 if name.startswith("G0") else 40, color=colour, zorder=3)
        ax.annotate(name, (cf, quality), fontsize=6, xytext=(4, 4), textcoords="offset points")
    dead = [(n, q) for n, _, q, m in rows if m]
    if dead:
        ax.scatter([now] * len(dead), [q for _, q in dead], marker="x", color="0.6", s=40,
                   label=f"never fires on >=1 seed ({len(dead)} candidates)")
    ax.axvline(now, color="C3", ls="--", lw=1, label="current gate")
    ax.axvline(floor, color="k", ls=":", lw=1, label="dwell floor: no gate can beat this")
    ax.set_xlabel("mean contact-ready frame (exact counterfactual)")
    ax.set_ylabel("true strict coverage when the gate fires")
    ax.set_title("F. Enclosure gate: delay against the enclosure it certifies")
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(run / "figF_gate_tradeoff.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
