#!/usr/bin/env python
"""D0 - baseline reproduction and open-loop force-ceiling probe.

Two questions have to be answered before a transport controller is worth
writing, and neither of them is answered by tuning a gain.

**Does the stall reproduce?** Run the A-branch configuration for the full frame
budget and record ``J(t)``. If ``J`` is flat after some frame, the system is at an
equilibrium, and an equilibrium that does not depend on whether the object is
moving cannot be escaped by making the same feedback larger.

**Can this configuration move the object at all?** Bypass the coverage law and the
transport bias, command every robot on the trailing arc straight at the object,
and measure the steady object speed that results. The safety filter stays on, so
the answer is the ceiling of what a *safe* team can do, not of what an
unconstrained one could. If that ceiling is near zero the geometry is wrong and no
outer loop will fix it.

    python scripts/probe_transport_ceiling.py --config configs/sim/v2/l_shape_v2.yaml \
        --steps 500 --out runs/d0_probe
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from dbact.geometry import normalize
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.scenarios import load_yaml


def reproduce_stall(config: str, steps: int, seed: int) -> dict:
    env = SimulationEnvironment(load_yaml(config), seed=seed)
    started = time.perf_counter()
    env.run(steps)
    wall = time.perf_counter() - started

    summary = env.summary()
    cargo_id = env.cargoes[0].object_id
    entry = summary["cargoes"][cargo_id]
    goal = env.goal_directions[cargo_id]
    centers = np.vstack(env.log.cargo_centers[cargo_id])
    progress = (centers - centers[0]) @ goal

    # A plateau is the last frame at which J still changed by more than a
    # millimetre; after it the run is producing frames but not transport.
    moving = np.where(np.abs(np.diff(progress)) > 1e-3)[0]
    plateau_frame = int(moving[-1]) + 1 if len(moving) else 0

    return {
        "config": config,
        "seed": seed,
        "steps": steps,
        "wall_seconds": wall,
        "frames_per_second": steps / wall if wall > 0 else float("inf"),
        "J_final": float(progress[-1]),
        "J_plateau_frame": plateau_frame,
        "J_at_plateau": float(progress[plateau_frame]),
        "displacement": entry["displacement"],
        "final_strict_coverage": entry["final_strict_coverage"],
        "mean_contacts": entry["mean_contacts"],
        "peak_net_force": entry["peak_net_force"],
        "solver": summary["solver"]["statuses"],
        "fallbacks": summary["solver"]["fallbacks"],
        "success": entry["success"],
        "failure_reasons": entry["failure_reasons"],
        "J_trace": [float(progress[k]) for k in range(0, steps + 1, max(1, steps // 20))],
    }


def force_ceiling(config: str, steps: int, seed: int) -> dict:
    """Open-loop probe: press the trailing arc in, measure what the cargo does.

    The command is ``kp * (target - p)`` with the target one penetration budget
    inside the *true* boundary. That reads cargo geometry deliberately -- this is
    an instrument, not a controller, and it is never part of a scored run.
    """
    env = SimulationEnvironment(load_yaml(config), seed=seed)
    cargo = env.cargoes[0]
    params = env.controller.params
    contact = env.contact_params
    goal = normalize(env.goal_directions[cargo.object_id])
    target_depth = params.robot_radius - params.r_safe  # delta_max

    speeds: list[float] = []
    forces: list[float] = []
    contacts: list[int] = []
    for _ in range(steps):
        positions = np.vstack([a.position for a in env.agents])
        signed, normals, foot = cargo.signed_distance(positions)
        # Trailing arc only: outward normal opposing the goal.
        pushing = (normals @ goal) < -params.push_side_threshold
        commands = []
        for i, agent in enumerate(env.agents):
            if pushing[i]:
                aim = foot[i] + (params.robot_radius - target_depth) * normals[i]
                u_nom = 1.5 * (aim - agent.position)
            else:
                # Everyone else holds station outside contact so the probe measures
                # the trailing arc rather than a collapse of the whole ring.
                aim = foot[i] + (params.lead_offset or 0.22) * normals[i]
                u_nom = 0.9 * (aim - agent.position)
            neighbours = [b.position for b in env.agents if b is not agent]
            result = env.controller.safety.filter_velocity(agent.position, u_nom, neighbours)
            commands.append(result.velocity)
        for agent, u in zip(env.agents, commands):
            agent.velocity = u
            agent.position = agent.position + u * env.dt
        report = env.engine.step(env.cargoes, env.agents, env.dt)[0]
        speeds.append(float(np.dot(cargo.linear_velocity, goal)))
        forces.append(float(np.dot(report.net_force, goal)))
        contacts.append(report.contact_count)

    tail = slice(len(speeds) // 2, None)
    return {
        "breakaway_force": contact.breakaway_force(cargo.mass),
        "per_robot_force_at_cage_ring": contact.stiffness * max(params.robot_radius - params.cage_offset, 0.0),
        "per_robot_force_at_delta_max": contact.stiffness * target_depth,
        "min_cooperating_robots": contact.min_cooperating_robots(cargo.mass, params.cage_offset),
        "steady_speed_along_goal": float(np.mean(speeds[tail])),
        "peak_speed_along_goal": float(np.max(speeds)),
        "steady_force_along_goal": float(np.mean(forces[tail])),
        "peak_force_along_goal": float(np.max(forces)),
        "mean_contacts": float(np.mean(contacts)),
        "displacement_along_goal": float(np.dot(cargo.position - cargo.initial_position, goal)),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/sim/v2/l_shape_v2.yaml")
    parser.add_argument("--steps", type=int, default=500)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", default="runs/d0_probe")
    parser.add_argument("--skip-ceiling", action="store_true")
    args = parser.parse_args()

    report = {"baseline": reproduce_stall(args.config, args.steps, args.seed)}
    if not args.skip_ceiling:
        report["ceiling"] = force_ceiling(args.config, args.steps, args.seed)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    (out / "probe.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    base = report["baseline"]
    print(f"baseline: J={base['J_final']:.4f} m, flat from frame {base['J_plateau_frame']}, "
          f"{base['frames_per_second']:.2f} frame/s, success={base['success']}")
    if "ceiling" in report:
        c = report["ceiling"]
        print(f"ceiling : breakaway {c['breakaway_force']:.2f} N, open-loop force along goal "
              f"{c['steady_force_along_goal']:.2f} N, steady speed {c['steady_speed_along_goal']:.4f} m/s, "
              f"displacement {c['displacement_along_goal']:.4f} m")
    print(f"wrote {out / 'probe.json'}")


if __name__ == "__main__":
    main()
