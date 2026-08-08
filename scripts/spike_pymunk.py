#!/usr/bin/env python
"""Gate 1 mechanical spike: can disc robots push a concave rigid body by contact?

This is the question that decides whether the task is physically possible at all,
so it is answered before any control design. Robots are driven kinematically
along a fixed direction; nothing here is closed-loop and nothing here is a
result about the controller. Its output is an upper bound on what the mechanism
allows.

    python scripts/spike_pymunk.py
    python scripts/spike_pymunk.py --engine penalty --steps 900
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact.cargo import Cargo  # noqa: E402
from dbact.contact_dynamics import ContactParams  # noqa: E402
from dbact.transport_dynamics import build_engine  # noqa: E402
from dbact.types import AgentState  # noqa: E402


def build_pushers(cargo: Cargo, direction: np.ndarray, count: int, robot_radius: float, speed: float) -> list[AgentState]:
    """Place robots on the trailing face and drive them along ``direction``.

    Trailing means the outward normal opposes the push direction, which is the
    only side from which a contact force can do positive work along it.
    """
    points, normals = cargo.boundary_samples(360)
    trailing = np.where(normals @ direction < -0.5)[0]
    if len(trailing) == 0:
        trailing = np.argsort(normals @ direction)[:count]
    picks = trailing[np.linspace(0, len(trailing) - 1, count).astype(int)]
    return [
        AgentState(
            agent_id=f"pusher_{i:02d}",
            position=points[k] + (robot_radius + 0.01) * normals[k],
            velocity=direction * speed,
        )
        for i, k in enumerate(picks)
    ]


def run_spike(engine_name: str, steps: int, dt: float, robots: int, speed: float) -> dict:
    direction = np.array([1.0, 0.0])
    cargo = Cargo.l_shape("cargo_0", [0.0, 0.0], scale=1.5, surface_density=2.0)
    params = ContactParams(robot_radius=0.16, stiffness=500.0, damping=12.0, friction=0.6,
                           ground_friction=0.45, gravity=9.81, substeps=4)
    engine = build_engine(engine_name, params)
    agents = build_pushers(cargo, direction, robots, params.robot_radius, speed)

    contacts = []
    for _ in range(steps):
        for agent in agents:
            agent.position = agent.position + agent.velocity * dt
        statuses = engine.step([cargo], agents, dt)
        contacts.append(statuses[0].contact_count)

    displacement = cargo.displacement
    norm = float(np.linalg.norm(displacement))
    return {
        "engine": engine_name,
        "steps": steps,
        "dt": dt,
        "robots": robots,
        "pusher_speed": speed,
        "displacement": displacement.tolist(),
        "displacement_norm": norm,
        "progress_along_push_direction": float(np.dot(displacement, direction)),
        "rotation_deg": math.degrees(cargo.angle),
        "mean_contacts": float(np.mean(contacts)) if contacts else 0.0,
        "pass": norm > 0.5,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Gate 1 mechanical spike.")
    parser.add_argument("--engine", nargs="+", default=["penalty", "pymunk"], help="Engines to run.")
    parser.add_argument("--steps", type=int, default=900)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--robots", type=int, default=5)
    parser.add_argument("--speed", type=float, default=0.30)
    parser.add_argument("--json", default="")
    args = parser.parse_args()

    results = []
    for engine in args.engine:
        try:
            result = run_spike(engine, args.steps, args.dt, args.robots, args.speed)
        except ImportError as exc:
            print(f"[SKIP] {engine}: {exc}")
            continue
        results.append(result)
        print(json.dumps(result, indent=2))
        print(
            f"--> {engine}: {'PASS' if result['pass'] else 'FAIL'}  "
            f"|dx|={result['displacement_norm']:.4f} m along the push direction\n"
        )

    if args.json and results:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(results, indent=2), encoding="utf-8")
    ok = bool(results) and all(r["pass"] for r in results)
    print("GATE 1 PASS" if ok else "GATE 1 FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
