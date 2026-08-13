#!/usr/bin/env python3
"""Compute a conditional analytic finite-time bound from declared premises."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact.guarantees import derive_conditional_finite_time_bound  # noqa: E402
from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="configs/sim/research/adaptive_progress_closed_loop.yaml",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--enclosure-initial-error", type=float, required=True)
    parser.add_argument("--enclosure-terminal-error", type=float, required=True)
    parser.add_argument("--enclosure-rate", type=float, required=True)
    parser.add_argument("--progress-rate", type=float, required=True)
    parser.add_argument("--brake-initial-error", type=float, required=True)
    parser.add_argument("--brake-terminal-error", type=float, required=True)
    parser.add_argument("--brake-rate", type=float, required=True)
    parser.add_argument("--hold-dwell", type=float)
    parser.add_argument("--output")
    args = parser.parse_args()

    config = load_yaml(args.config)
    env = SimulationEnvironment(config, seed=args.seed)
    params = env.controller.params
    certificate = next(iter(env.guarantee_certificates.values()))
    search = certificate["search"]
    search_bound_s = search["sweep_bound_frames"] * env.dt
    map_bound_s = (
        (search["rendezvous_bound_frames"] + search["gossip_bound_frames"]) * env.dt
        + params.boundary_mapping_time
    )
    bound = derive_conditional_finite_time_bound(
        dt=env.dt,
        search_bound_s=search_bound_s,
        map_bound_s=map_bound_s,
        enclosure_initial_error_m=args.enclosure_initial_error,
        enclosure_terminal_error_m=args.enclosure_terminal_error,
        enclosure_contraction_rate_hz=args.enclosure_rate,
        transport_distance_m=params.transport_distance,
        brake_activation_distance_m=params.brake_activation_distance,
        transport_progress_rate_mps=args.progress_rate,
        brake_initial_error_m=args.brake_initial_error,
        brake_terminal_error_m=args.brake_terminal_error,
        brake_contraction_rate_hz=args.brake_rate,
        hold_dwell_s=(
            params.brake_dwell_steps * env.dt if args.hold_dwell is None else args.hold_dwell
        ),
    )
    payload = {
        "config": args.config,
        "seed": args.seed,
        "source": "declared analytic premises; no episode timings consumed",
        "bound": bound,
    }
    rendered = json.dumps(payload, indent=2)
    if args.output:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(rendered, encoding="utf-8")
    print(rendered)
    return 0 if bound["eligible"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
