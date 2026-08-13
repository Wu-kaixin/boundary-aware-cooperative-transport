#!/usr/bin/env python
"""T6 - report the conditional finite-time bound for a scenario, and why it is unavailable.

    python scripts/derive_finite_time_bound.py configs/sim/v2/shape_matrix.yaml

Ported from CODEX's ``derive_finite_time_bound.py``. It is a thin front end over
:func:`dbact.guarantees.derive_conditional_finite_time_bound`, and the reason it exists as a
separate command is that the bound's *unavailability* is a result somebody should be able to
reproduce in one line rather than infer from an absent number.

What this prints and what it does not
-------------------------------------
``available`` is the only field a caller should gate on, and it is ``False`` for every
configuration in this repository. It is ``False`` even when the arithmetic is complete and
self-consistent, because the bound is stated over three contraction rates --
``enclosure_contraction_rate_hz``, ``transport_progress_rate_mps`` and
``brake_contraction_rate_hz`` -- and none of them holds an independent certificate. Writing
a number for a rate in a YAML file does not certify it.

So the phase totals below are **what the bound would be** if those rates were certified.
They are not a bound, they are not an estimate of the completion time, and they must not be
compared with measured episode durations to argue either way: the measured durations are
right-censored (42 of 180 matrix episodes ended on the watchdog), and the analytic figure is
a sufficient bound conditional on premises that do not hold.

``--assume-certified`` exists so a reader can see the arithmetic reached, and it prints a
banner saying the resulting number carries no certificate. Nothing in the repository passes
that flag, and nothing should.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact.guarantees import (  # noqa: E402
    UNCERTIFIED_CONTRACTION_RATES,
    GuaranteeSpecError,
    _required,
    derive_conditional_finite_time_bound,
)
from dbact_sim.scenarios import controller_params_from_config, load_yaml  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("config")
    parser.add_argument("--distance", type=float, default=None,
                        help="Task distance in metres. Defaults to the config's task.distance_max, "
                             "because the bound is monotone in it and the longest admissible task "
                             "is the one worth bounding.")
    parser.add_argument("--search-bound-s", type=float, default=None,
                        help="Search release bound. Defaults to the sweep bound implied by the "
                             "lane partition, or 0 when the config declares no search.")
    parser.add_argument("--assume-certified", action="store_true",
                        help="Reach the arithmetic by asserting a certificate this repository "
                             "does not hold. The output is labelled accordingly.")
    parser.add_argument("--json", action="store_true", help="Emit the payload as JSON only.")
    args = parser.parse_args()

    config = load_yaml(args.config)
    controller = controller_params_from_config(config)
    spec = (config.get("guarantee", {}) or {})
    finite_time = spec.get("finite_time")
    if not isinstance(finite_time, dict):
        print(
            f"{args.config} declares no guarantee.finite_time block. The bound has no premises "
            "here, and this command refuses to invent them: see dbact.guarantees._required.",
            file=sys.stderr,
        )
        return 2

    dt = float(config.get("dt", 0.05))
    distance = args.distance
    if distance is None:
        distance = float((config.get("task", {}) or {}).get("distance_max", 0.0))
    if distance <= 0.0:
        print("no positive task distance available; pass --distance", file=sys.stderr)
        return 2

    try:
        bound = derive_conditional_finite_time_bound(
            dt=dt,
            search_bound_s=float(args.search_bound_s or 0.0),
            map_bound_s=_required(finite_time, "map_bound_s"),
            enclosure_initial_error_m=_required(finite_time, "enclosure_initial_error_m"),
            enclosure_terminal_error_m=_required(finite_time, "enclosure_terminal_error_m"),
            enclosure_contraction_rate_hz=_required(finite_time, "enclosure_contraction_rate_hz"),
            transport_distance_m=distance,
            brake_activation_distance_m=(1.0 - controller.brake_fraction) * distance,
            transport_progress_rate_mps=_required(finite_time, "transport_progress_rate_mps"),
            brake_initial_error_m=_required(finite_time, "brake_initial_error_m"),
            brake_terminal_error_m=_required(finite_time, "brake_terminal_error_m"),
            brake_contraction_rate_hz=_required(finite_time, "brake_contraction_rate_hz"),
            hold_dwell_s=controller.contact_dwell * dt,
            contraction_rates_certified=bool(args.assume_certified),
        )
    except GuaranteeSpecError as exc:
        print(f"refusing to build the bound: {exc}", file=sys.stderr)
        return 2

    payload = {
        "config": args.config,
        "task_distance_m": distance,
        "assume_certified": bool(args.assume_certified),
        "bound": bound,
    }
    if args.json:
        print(json.dumps(payload, indent=2))
        return 0

    print(f"config              {args.config}")
    print(f"task distance       {distance:.4f} m")
    print(f"bound id            {bound['bound_id']}")
    print(f"classification      {bound['classification']}")
    print()
    print(f"available           {bound['available']}")
    print(f"arithmetic ok       {bound['arithmetic_consistent']}")
    print(f"rates certified     {bound['contraction_rates_certified']}")
    if bound["uncertified_rates"]:
        print("uncertified rates:")
        for name in bound["uncertified_rates"]:
            print(f"    {name} = {bound['premises'][name]}")
    if bound["failure_reasons"]:
        print("arithmetic failures:")
        for name in bound["failure_reasons"]:
            print(f"    {name}")
    print()
    if bound["phase_bounds_s"] is None:
        print("no phase bounds: the premises are not arithmetically consistent.")
    else:
        print("phase bounds (WHAT THE BOUND WOULD BE, not a bound):")
        for name, seconds in bound["phase_bounds_s"].items():
            print(f"    {name:10s} {seconds:9.3f} s   {bound['phase_bounds_frames'][name]:6d} frames")
        print(f"    {'TOTAL':10s} {bound['total_bound_s']:9.3f} s   {bound['total_bound_frames']:6d} frames")
    print()
    if not bound["available"]:
        print("NOT A BOUND. `available` is False, so the totals above are arithmetic over")
        print("premises that hold no independent certificate. Do not compare them with")
        print("measured episode durations: those are right-censored, and this is a")
        print("sufficient bound conditional on assumptions that are not established.")
    else:
        print("!! `available` is True only because --assume-certified was passed. No")
        print(f"!! certificate exists for {', '.join(UNCERTIFIED_CONTRACTION_RATES)}.")
        print("!! This output carries no guarantee.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
