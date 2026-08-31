#!/usr/bin/env python
"""T7 - what ``explore_gain = 6`` costs in frame rate.

    python scripts/profile_explore_gain.py --seeds 0..5 --out docs/results/t7

``configs/sim/v2/shape_matrix.yaml`` runs with ``explore_gain: 6.0`` and its control
``shape_matrix_eg0.yaml`` with ``0.0``. The control experiment already established what
the term buys in *success* -- l_shape 2/15 -> 5/15, star10 and concave_random15 unmoved
at 0/15. This measures what it costs in wall-clock time.

Read as machine-dependent empirical evidence and nothing else
------------------------------------------------------------
This is a stopwatch reading on one machine, in one Python, against one BLAS, with
whatever else that machine was doing. It is not a runtime bound, it is not a complexity
result, and it does not transfer. The reason it is worth recording anyway is that
"explore_gain is free" and "explore_gain halves the frame rate" are different facts about
the same configuration, and the branch reports 22.0 fps as a headline number without
saying which arm produced it.

Both arms are timed on the same seeds in the same process, alternating arm by seed, so
that a thermal drift or a background task during the run perturbs both arms rather than
one. The per-seed pairing is what makes the ratio worth more than either absolute
number.
"""

from __future__ import annotations

import argparse
import copy
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from dbact_sim.environment import SimulationEnvironment  # noqa: E402
from dbact_sim.scenarios import load_yaml  # noqa: E402

BASE_CONFIG = "configs/sim/d/l_shape_closed_loop.yaml"


def parse_seeds(spec: str) -> list[int]:
    if ".." in spec:
        low, high = (int(p) for p in spec.split("..", 1))
        return list(range(low, high + 1))
    return [int(p) for p in spec.split(",") if p.strip()]


def timed(config: dict, seed: int, frames: int) -> dict:
    """A fixed frame count, not run-until-settled.

    Timing an until-settled run measures episode *length* as much as per-frame cost, and
    the two arms do not settle at the same frame. A fixed budget makes the comparison a
    per-frame one.
    """
    env = SimulationEnvironment(config, seed=seed)
    started = time.perf_counter()
    env.run(frames)
    elapsed = time.perf_counter() - started
    return {
        "seed": seed,
        "frames": frames,
        "seconds": elapsed,
        "fps": frames / max(elapsed, 1e-12),
        "ms_per_frame": 1000.0 * elapsed / frames,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=BASE_CONFIG)
    parser.add_argument("--seeds", default="0..5")
    parser.add_argument("--frames", type=int, default=400)
    parser.add_argument("--out", default="docs/results/t7")
    args = parser.parse_args()

    base = load_yaml(args.config)
    seeds = parse_seeds(args.seeds)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    arms: dict[str, list[dict]] = {"eg0": [], "eg6": []}
    configs = {}
    for name, gain in (("eg0", 0.0), ("eg6", 6.0)):
        config = copy.deepcopy(base)
        config["controller"] = dict(config.get("controller", {}))
        config["controller"]["explore_gain"] = gain
        configs[name] = config

    # Alternate arm by seed so a thermal drift hits both.
    for seed in seeds:
        for name in ("eg0", "eg6"):
            row = timed(configs[name], seed, args.frames)
            arms[name].append(row)
            print(
                f"[{name}] seed {seed:2d}  {row['fps']:6.2f} fps  "
                f"{row['ms_per_frame']:6.2f} ms/frame",
                flush=True,
            )

    def stats(rows: list[dict]) -> dict:
        fps = np.array([r["fps"] for r in rows])
        ms = np.array([r["ms_per_frame"] for r in rows])
        return {
            "n": len(rows),
            "fps_mean": float(fps.mean()),
            "fps_sd": float(fps.std(ddof=1)) if len(fps) > 1 else 0.0,
            "fps_min": float(fps.min()),
            "fps_max": float(fps.max()),
            "ms_per_frame_mean": float(ms.mean()),
        }

    eg0, eg6 = stats(arms["eg0"]), stats(arms["eg6"])
    # Paired per-seed ratio, which is the quantity the alternation buys.
    ratios = np.array(
        [b["ms_per_frame"] / a["ms_per_frame"] for a, b in zip(arms["eg0"], arms["eg6"])]
    )

    report = {
        "experiment": "T7 explore_gain frame-rate cost",
        "status": "MACHINE-DEPENDENT EMPIRICAL EVIDENCE, NOT A RUNTIME BOUND",
        "config": args.config,
        "frames_per_episode": args.frames,
        "seeds": seeds,
        "machine": {
            "platform": platform.platform(),
            "processor": platform.processor(),
            "python": platform.python_version(),
            "numpy": np.__version__,
        },
        "explore_gain_0": eg0,
        "explore_gain_6": eg6,
        "paired_cost_ratio": {
            "mean": float(ratios.mean()),
            "sd": float(ratios.std(ddof=1)) if len(ratios) > 1 else 0.0,
            "min": float(ratios.min()),
            "max": float(ratios.max()),
        },
        "runs": arms,
    }
    (out / "explore_gain_profile.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print()
    print("=" * 72)
    print(f"explore_gain 0.0   {eg0['fps_mean']:6.2f} +- {eg0['fps_sd']:.2f} fps   "
          f"{eg0['ms_per_frame_mean']:6.2f} ms/frame")
    print(f"explore_gain 6.0   {eg6['fps_mean']:6.2f} +- {eg6['fps_sd']:.2f} fps   "
          f"{eg6['ms_per_frame_mean']:6.2f} ms/frame")
    print(f"paired cost ratio  {ratios.mean():.4f} +- "
          f"{(ratios.std(ddof=1) if len(ratios) > 1 else 0.0):.4f}  "
          f"(per-frame time, eg6 / eg0)")
    print("=" * 72)
    print("Machine-dependent empirical evidence. Not a runtime bound.")
    print(f"wrote {out / 'explore_gain_profile.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
