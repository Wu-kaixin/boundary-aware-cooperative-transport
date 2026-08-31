#!/usr/bin/env python
"""D10-DWELL - why the contact quorum forms and breaks before it holds.

    PYTHONPATH=src python scripts/diagnose_dwell.py --seeds 0..7 --out runs/d10_dwell

D10-ENC established that after the exploration term landed, the enclosure gate
binds on two seeds of eight and the *dwell* binds on six -- and that the quorum
first forms 67 to 115 frames before it finally holds, costing 58 frames a seed
against the 39 an oracle gate could ever recover. So the question is what makes
band membership unstable.

Membership is one predicate:

    contact_ready_i  <=>  || p_i - b_i ||  <=  cage_offset + contact_band_tolerance

with ``b_i`` the nearest point of the robot's *own* map. It is one-sided, so a
robot only ever leaves by its distance growing, and that distance can grow for
two quite different reasons:

    the robot moved away from the boundary          -- a control problem
    the boundary moved away from the robot          -- an estimation problem

Those are separable exactly, because the two effects compose:

    d_t - d_{t-1}  =  ( |p_t - b_{t-1}| - d_{t-1} )  +  ( d_t - |p_t - b_{t-1}| )
                   =        robot term              +        map term

The first holds the map fixed and moves the robot; the second holds the robot
fixed and lets the map update. They sum to the total with no residual, and the
split says which layer to look at. The controller is not modified.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from diagnose_redeployment import parse_seeds, stats  # noqa: E402

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402


@dataclass
class Exit:
    """One robot leaving the contact band."""

    seed: int
    frame: int
    agent: str
    mode: str
    distance_before: float
    distance_after: float
    robot_term: float
    map_term: float
    robot_step: float
    residence: int
    broke_quorum: bool
    # The standoff setpoint this robot was being regulated to when it left. If it
    # is above the band threshold, leaving is not a disturbance -- it is the loop
    # doing what it was asked.
    ring: float = float("nan")
    alignment: float = float("nan")

    @property
    def cause(self) -> str:
        """Which layer moved the distance across the threshold.

        Attributed by the larger *positive* contribution: both terms can be
        non-zero on the same step, and what matters is which one carried the
        distance over the line.
        """
        if self.robot_term <= 0.0 and self.map_term <= 0.0:
            return "neither"          # left by an earlier drift, crossing now
        return "robot" if self.robot_term >= self.map_term else "map"


class DwellTrace:
    def __init__(self, env: SimulationEnvironment, seed: int, quorum: int, dwell: int):
        self.env = env
        self.seed = seed
        self.controller = env.controller
        self.params = env.controller.params
        self.threshold = self.params.cage_offset + self.params.contact_band_tolerance
        self.quorum = quorum
        self.dwell = dwell
        self.agent_ids = [a.agent_id for a in env.agents]

        self.frames: list[int] = []
        self.counts: list[int] = []
        self.distance: list[np.ndarray] = []
        self.ready: list[np.ndarray] = []
        self.ring: list[np.ndarray] = []
        self.modes: list[list[str]] = []
        self.exits: list[Exit] = []

        self._previous: dict[str, tuple[np.ndarray, np.ndarray]] = {}   # position, nearest point
        self._entered: dict[str, int] = {}

    def on_frame(self, frame: int, env: SimulationEnvironment) -> None:
        diagnostics = {d.agent_id: d for d in self.controller.diagnostics}
        if not diagnostics:
            return
        views = self.controller._views
        distances = np.full(len(env.agents), np.inf)
        rings = np.full(len(env.agents), np.nan)
        alignments = np.full(len(env.agents), np.nan)
        ready = np.zeros(len(env.agents), dtype=bool)
        modes: list[str] = []
        current: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        previous_count = self.counts[-1] if self.counts else 0

        exits_here: list[Exit] = []
        for index, agent in enumerate(env.agents):
            view = views.get(agent.agent_id)
            modes.append(diagnostics[agent.agent_id].mode)
            if view is None or len(view) == 0:
                self._previous.pop(agent.agent_id, None)
                self._entered.pop(agent.agent_id, None)
                continue
            delta = view.points - agent.position[None, :]
            best = int(np.argmin(np.einsum("ij,ij->i", delta, delta)))
            nearest = view.points[best]
            distance = float(np.linalg.norm(nearest - agent.position))
            distances[index] = distance
            ready[index] = distance <= self.threshold
            current[agent.agent_id] = (agent.position.copy(), nearest.copy())
            rings[index], alignments[index] = self._setpoint(agent, view, best)

            was = self.ready[-1][index] if self.ready else False
            if ready[index] and not was:
                self._entered[agent.agent_id] = frame
            elif was and not ready[index]:
                exits_here.append(
                    self._exit(agent, index, frame, distance, diagnostics,
                               rings[index], alignments[index])
                )

        self._previous = current
        self.frames.append(frame)
        self.distance.append(distances)
        self.ready.append(ready)
        self.ring.append(rings)
        self.modes.append(modes)
        count = int(np.count_nonzero(ready))
        self.counts.append(count)
        broke = previous_count >= self.quorum and count < self.quorum
        for record in exits_here:
            record.broke_quorum = broke
        self.exits.extend(exits_here)

    def _setpoint(self, agent, view, nearest_index: int) -> tuple[float, float]:
        """The standoff offset ``_transport_command`` would regulate this robot to.

        Reproduced from the controller's own pieces rather than guessed: the same
        enclosure geometry, the same steered direction, the same ``offsets_for``.
        The number matters because the offset is graded continuously by how much
        the robot's face opposes the goal, while band membership is a hard
        threshold -- so somewhere on that ramp the setpoint crosses the threshold,
        and a robot there is asked to stand outside the band it has to be inside.
        """
        controller = self.controller
        normal = view.normals[nearest_index]
        object_id = controller._object_for(view, agent.position)
        goal = controller.goal_directions.get(object_id) if object_id else None
        if goal is None:
            return float("nan"), float("nan")
        shape = controller._enclosure_geometry(agent.agent_id, view)
        command = controller._steered_direction(agent.agent_id, object_id, goal)
        ring = float(shape.offsets_for(normal[None, :], command)[0])
        return ring, float(np.dot(normal, goal))

    def _exit(self, agent, index: int, frame: int, distance: float, diagnostics,
              ring: float, alignment: float) -> Exit:
        """Split this step's growth in distance into a robot term and a map term.

        ``self._previous`` is only overwritten after the agent loop, so it still
        holds the previous frame here. The intermediate quantity is the distance
        the robot would be at had it moved while the map stood still; subtracting
        it from either end gives the two terms, and they sum to the total exactly.
        """
        agent_id = agent.agent_id
        before = self.distance[-1][index] if self.distance else float("nan")
        robot_term = map_term = step = float("nan")
        history = self._previous.get(agent_id)
        if history is not None:
            old_position, old_nearest = history
            middle = float(np.linalg.norm(agent.position - old_nearest))
            robot_term = middle - before
            map_term = distance - middle
            step = float(np.linalg.norm(agent.position - old_position))
        entered = self._entered.pop(agent_id, frame)
        return Exit(
            seed=self.seed,
            frame=frame,
            agent=agent_id,
            mode=diagnostics[agent_id].mode,
            distance_before=before,
            distance_after=distance,
            robot_term=robot_term,
            map_term=map_term,
            robot_step=step,
            residence=frame - entered,
            broke_quorum=False,
            ring=ring,
            alignment=alignment,
        )

    # ---------------------------------------------------------------- #

    def streak_frame(self, hysteresis: float = 0.0) -> int | None:
        """First frame the quorum has been held for the dwell, under a band whose
        exit threshold is ``threshold + hysteresis``.

        Approximate for ``hysteresis > 0``: ``contact_ready`` feeds the transport
        gate and the redeploy rule as well as the phase machine, so widening the
        band changes the trajectory this series was recorded on. It is a screen
        for whether the idea is worth an A/B, not a prediction.
        """
        if not self.frames:
            return None
        run = 0
        inside = np.zeros(len(self.agent_ids), dtype=bool)
        for step, frame in enumerate(self.frames):
            distance = self.distance[step]
            inside = np.where(
                inside,
                distance <= self.threshold + hysteresis,
                distance <= self.threshold,
            )
            run = run + 1 if int(np.count_nonzero(inside)) >= self.quorum else 0
            if run >= self.dwell:
                return int(frame)
        return None


def run_seed(config: dict, seed: int, gain: float, max_frames: int, settle: int,
             quorum: int, dwell: int) -> DwellTrace:
    cfg = json.loads(json.dumps(config))
    cfg["controller"]["explore_gain"] = float(gain)
    env = SimulationEnvironment(cfg, seed=seed)
    env.controller.trace_enabled = True
    trace = DwellTrace(env, seed, quorum, dwell)
    env.run_until_settled(max_frames=max_frames, settle_frames=settle, on_frame=trace.on_frame)
    trace.phases = env.controller.phase_monitor.as_dict()
    trace.entry = next(iter(env.summary()["cargoes"].values()))
    return trace


def summarise(trace: DwellTrace) -> dict:
    contact_ready = trace.phases.get("contact_ready_frame")
    window = [e for e in trace.exits if contact_ready is None or e.frame <= contact_ready]
    first_quorum = next(
        (f for f, c in zip(trace.frames, trace.counts) if c >= trace.quorum), None
    )
    streak = trace.streak_frame()
    causes: dict[str, int] = {}
    for record in window:
        causes[record.cause] = causes.get(record.cause, 0) + 1
    breaking = [e for e in window if e.broke_quorum]
    residences = [e.residence for e in window if e.residence > 0]
    near = np.concatenate([d[np.isfinite(d) & (d < 0.60)] for d in trace.distance]) \
        if trace.distance else np.empty(0)
    return {
        "seed": trace.seed,
        "threshold": trace.threshold,
        "first_quorum": first_quorum,
        "streak": streak,
        "contact_ready": contact_ready,
        "chatter": (streak - first_quorum - (trace.dwell - 1)) if (streak and first_quorum) else None,
        "exits": len(window),
        "exits_breaking_quorum": len(breaking),
        "causes": causes,
        "cause_of_breaks": _tally(breaking),
        "modes_at_exit": _tally_by(window, lambda e: e.mode),
        "residence_mean": float(np.mean(residences)) if residences else None,
        "residence_median": float(np.median(residences)) if residences else None,
        "robot_term_mean": float(np.nanmean([e.robot_term for e in window])) if window else None,
        "map_term_mean": float(np.nanmean([e.map_term for e in window])) if window else None,
        "robot_step_mean": float(np.nanmean([e.robot_step for e in window])) if window else None,
        "distance_near_band_median": float(np.median(near)) if len(near) else None,
        # The structural question: was the robot being regulated to a standoff
        # outside the band it had to stay inside?
        "ring_at_exit_mean": float(np.nanmean([e.ring for e in window])) if window else None,
        "exits_with_ring_above_band": sum(
            1 for e in window if np.isfinite(e.ring) and e.ring > trace.threshold
        ),
        "ring_above_band_agent_frames": int(
            np.sum([np.count_nonzero(r[np.isfinite(r)] > trace.threshold) for r in trace.ring])
        ) if trace.ring else 0,
        "in_band_agent_frames": int(np.sum([np.count_nonzero(r) for r in trace.ready]))
        if trace.ready else 0,
        "fraction_within_1cm_of_threshold": float(
            np.mean(np.abs(near - trace.threshold) <= 0.01)
        ) if len(near) else None,
        "peak_strict_coverage": trace.entry["max_strict_coverage"],
        "hysteresis_screen": {
            f"{h:.3f}": trace.streak_frame(h) for h in (0.0, 0.01, 0.02, 0.04, 0.08)
        },
    }


def _tally(records: list[Exit]) -> dict[str, int]:
    return _tally_by(records, lambda e: e.cause)


def _tally_by(records: list[Exit], key) -> dict[str, int]:
    out: dict[str, int] = {}
    for record in records:
        name = key(record)
        out[name] = out.get(name, 0) + 1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="configs/sim/d/l_shape_search.yaml")
    parser.add_argument("--seeds", default="0..7")
    parser.add_argument("--gains", default="6,0",
                        help="Compared so that 'did the exploration term cause this' is measured.")
    parser.add_argument("--max-frames", type=int, default=3000)
    parser.add_argument("--settle-frames", type=int, default=40)
    parser.add_argument("--out", default="runs/d10_dwell")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    config = load_yaml(args.config)
    quorum = int(config["controller"].get("min_push_agents", 4))
    dwell = int(config["controller"].get("contact_dwell", 20))
    seeds = parse_seeds(args.seeds)
    gains = [float(g) for g in args.gains.split(",")]

    arms: dict[float, list[dict]] = {}
    for gain in gains:
        rows = []
        for seed in seeds:
            started = time.perf_counter()
            trace = run_seed(config, seed, gain, args.max_frames, args.settle_frames, quorum, dwell)
            row = summarise(trace)
            row["wall"] = time.perf_counter() - started
            rows.append(row)
            print(f"gain {gain:g}  seed {seed}  quorum@{row['first_quorum']}  "
                  f"streak@{row['streak']}  CR@{row['contact_ready']}  "
                  f"chatter={row['chatter']}  exits={row['exits']} "
                  f"(breaking {row['exits_breaking_quorum']})  {row['causes']}")
            np.savez_compressed(
                out / f"dwell_gain{gain:g}_seed{seed}.npz",
                frame=np.asarray(trace.frames),
                counts=np.asarray(trace.counts),
                distance=np.vstack([d.reshape(1, -1) for d in trace.distance]),
                ready=np.vstack([r.reshape(1, -1) for r in trace.ready]),
                ring=np.vstack([r.reshape(1, -1) for r in trace.ring]),
            )
        arms[gain] = rows

    payload = {"config": args.config, "seeds": seeds, "gains": gains,
               "quorum": quorum, "dwell": dwell, "arms": {str(g): arms[g] for g in gains}}
    (out / "dwell.json").write_text(json.dumps(payload, indent=2, default=float), encoding="utf-8")
    report(arms, dwell)
    print(f"\nwritten to {out}")


def report(arms: dict[float, list[dict]], dwell: int) -> None:
    for gain, rows in arms.items():
        print(f"\n--- explore_gain = {gain:g} " + "-" * 50)
        for name in ("chatter", "exits", "exits_breaking_quorum", "residence_mean",
                     "robot_term_mean", "map_term_mean", "robot_step_mean",
                     "fraction_within_1cm_of_threshold", "ring_at_exit_mean",
                     "exits_with_ring_above_band"):
            s = stats([r[name] for r in rows])
            if s["mean"] is None:
                print(f"  {name:34s} -")
            else:
                print(f"  {name:34s} {s['mean']:9.4f} ± {s['sd']:.4f}   "
                      f"[{s['min']:.4f}, {s['max']:.4f}]")
        causes: dict[str, int] = {}
        for row in rows:
            for key, value in row["causes"].items():
                causes[key] = causes.get(key, 0) + value
        total = sum(causes.values()) or 1
        print("  exit cause: " + ", ".join(
            f"{k} {v} ({100 * v / total:.0f}%)" for k, v in sorted(causes.items(), key=lambda kv: -kv[1])
        ))
        breaks: dict[str, int] = {}
        for row in rows:
            for key, value in row["cause_of_breaks"].items():
                breaks[key] = breaks.get(key, 0) + value
        broken = sum(breaks.values()) or 1
        print("  cause of quorum breaks: " + ", ".join(
            f"{k} {v} ({100 * v / broken:.0f}%)" for k, v in sorted(breaks.items(), key=lambda kv: -kv[1])
        ))
        modes: dict[str, int] = {}
        for row in rows:
            for key, value in row["modes_at_exit"].items():
                modes[key] = modes.get(key, 0) + value
        every = sum(modes.values()) or 1
        print("  mode at exit: " + ", ".join(
            f"{k} {v} ({100 * v / every:.0f}%)" for k, v in sorted(modes.items(), key=lambda kv: -kv[1])
        ))
        print("  hysteresis screen (mean streak frame, APPROXIMATE -- see docstring):")
        for h in ("0.000", "0.010", "0.020", "0.040", "0.080"):
            values = [r["hysteresis_screen"].get(h) for r in rows]
            s = stats(values)
            missing = sum(1 for v in values if v is None)
            note = f"  ({missing} seeds never reach it)" if missing else ""
            print(f"    +{h} m : {s['mean']:8.1f}{note}" if s["mean"] is not None else f"    +{h} m : -")


if __name__ == "__main__":
    main()
