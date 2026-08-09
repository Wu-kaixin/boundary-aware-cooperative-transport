"""Two-day decision spike: can 12 local robots physically push an L-shaped body?

This is intentionally independent of the enclosure controller. It tests the
paper-identity prerequisite only: contact geometry, concave-body decomposition,
and rigid-body motion in PyMunk. A passing result authorizes the transport
framing; it does not validate the full DBACT controller.

Example (conda dbact):
  conda run -n dbact python scripts/spike_pymunk_l_shape.py \
    --output runs/spikes/pymunk_l_shape
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact.cargo import Cargo
from dbact.transport_dynamics import PymunkTransportDynamics, TransportParams
from dbact.types import AgentState


def run_spike(
    *,
    steps: int = 300,
    dt: float = 0.02,
    robot_count: int = 12,
    push_speed: float = 0.35,
) -> tuple[dict, list[dict], Cargo]:
    cargo = Cargo.l_shape("L", center=[0.0, 0.0], scale=1.0)
    initial_center = cargo.center.copy()
    robot_radius = 0.06
    start_x = float(np.min(cargo.vertices[:, 0]) - robot_radius - 0.18)
    # Center the contact line on the cargo COM to avoid injecting artificial
    # torque. Agent-agent collision is intentionally disabled by the model.
    ys = np.linspace(initial_center[1] - 0.35, initial_center[1] + 0.35, robot_count)
    agents = [
        AgentState(f"agent_{index:02d}", np.array([start_x, y], dtype=float))
        for index, y in enumerate(ys)
    ]
    params = TransportParams(
        backend="pymunk",
        robot_radius=robot_radius,
        cargo_mass=5.0,
        cargo_friction=0.9,
        cargo_elasticity=0.0,
        agent_friction=0.95,
        linear_damping=0.25,
        angular_damping=0.8,
        substeps=4,
    )
    transport = PymunkTransportDynamics(params, [cargo], agents)
    records: list[dict] = []
    for step in range(steps + 1):
        center, angle = transport.world.cargo_pose(cargo.object_id)
        records.append(
            {
                "step": step,
                "time": step * dt,
                "cargo_x": float(center[0]),
                "cargo_y": float(center[1]),
                "cargo_yaw": float(angle),
            }
        )
        if step == steps:
            break
        for agent in agents:
            agent.velocity = np.array([push_speed, 0.0], dtype=float)
        transport.step([cargo], agents, dt)

    final_center = cargo.center.copy()
    _, final_yaw = transport.world.cargo_pose(cargo.object_id)
    displacement = final_center - initial_center
    finite = bool(np.all(np.isfinite(cargo.vertices)))
    thresholds = {
        "min_forward_displacement": 0.50,
        "max_lateral_drift": 0.25,
        "max_abs_yaw_rad": 0.60,
    }
    checks = {
        "finite_state": finite,
        "forward_displacement": float(displacement[0]) >= thresholds["min_forward_displacement"],
        "lateral_drift": abs(float(displacement[1])) <= thresholds["max_lateral_drift"],
        "yaw": abs(float(final_yaw)) <= thresholds["max_abs_yaw_rad"],
    }
    result = {
        "spike": "12-point-robot L-shape PyMunk push",
        "status": "PASS" if all(checks.values()) else "FAIL",
        "robot_count": robot_count,
        "steps": steps,
        "dt": dt,
        "push_speed": push_speed,
        "initial_center": initial_center.tolist(),
        "final_center": final_center.tolist(),
        "displacement": displacement.tolist(),
        "final_yaw_rad": float(final_yaw),
        "thresholds": thresholds,
        "checks": checks,
        "interpretation": (
            "Contact-based transport framing is mechanically viable for the codebase."
            if all(checks.values())
            else "Use enclosure framing until the failed mechanical checks are resolved."
        ),
    }
    return result, records, cargo


def _write_outputs(output: Path, result: dict, records: list[dict]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    (output / "result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    with (output / "trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)

    import matplotlib.pyplot as plt

    times = [row["time"] for row in records]
    xs = [row["cargo_x"] for row in records]
    ys = [row["cargo_y"] for row in records]
    yaws = [row["cargo_yaw"] for row in records]
    fig, axes = plt.subplots(1, 2, figsize=(9.0, 3.6))
    axes[0].plot(xs, ys, color="#1769aa", linewidth=2.0)
    axes[0].scatter([xs[0], xs[-1]], [ys[0], ys[-1]], c=["#6b7280", "#d97706"], zorder=3)
    axes[0].set_title("L-shape center trajectory")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].axis("equal")
    axes[0].grid(alpha=0.25)
    axes[1].plot(times, yaws, color="#7c3aed", linewidth=2.0)
    axes[1].axhline(0.60, color="#9ca3af", linestyle="--", linewidth=1.0)
    axes[1].axhline(-0.60, color="#9ca3af", linestyle="--", linewidth=1.0)
    axes[1].set_title("Cargo yaw")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("yaw [rad]")
    axes[1].grid(alpha=0.25)
    fig.suptitle(f"PyMunk decision spike: {result['status']}", fontweight="bold")
    fig.tight_layout()
    fig.savefig(output / "spike_summary.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="runs/spikes/pymunk_l_shape")
    parser.add_argument("--steps", type=int, default=300)
    parser.add_argument("--dt", type=float, default=0.02)
    args = parser.parse_args()
    result, records, _ = run_spike(steps=args.steps, dt=args.dt)
    _write_outputs(Path(args.output), result, records)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["status"] == "PASS" else 2)


if __name__ == "__main__":
    main()
