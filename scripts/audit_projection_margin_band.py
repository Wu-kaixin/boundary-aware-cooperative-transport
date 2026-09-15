"""Targeted re-run: collect ISSf margin-band / zero-with-rho stats (no observer).

Reuses the same N=16 static theorem config and seeds 2/5/8. Writes into the
existing n16_discrete_dissipation artifact folder without deleting prior H*
offline checks.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

OUT = Path(
    r"E:\boundary-aware-cooperative-transport\artifacts"
    r"\theorem_audit_theorem_mode_2026-09-11\n16_discrete_dissipation"
)


def main() -> None:
    from dbact_sim.environment import SimulationEnvironment

    config_path = ROOT / "configs" / "sim" / "theorem" / "static_l_shape_n16_oracle.yaml"
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    frames = 600
    seeds = [2, 5, 8]
    aggregate = {
        "agent_steps": 0,
        "optimal": 0,
        "relaxed_margin": 0,
        "scaled_barrier": 0,
        "zero_barrier_false": 0,
        "zero_with_rho_false": 0,
        "inside_margin_band": 0,
        "object_rows_active_sum": 0,
        "modification_gt_1e-9": 0,
    }
    per_seed = []
    rho = None
    for seed in seeds:
        env = SimulationEnvironment(cfg, seed=seed)
        rho = float(env.controller.params.rho)
        counts = {
            "seed": seed,
            "optimal": 0,
            "relaxed_margin": 0,
            "scaled_barrier": 0,
            "zero_barrier_false": 0,
            "zero_with_rho_false": 0,
            "inside_margin_band": 0,
            "object_rows_active_sum": 0,
            "modification_gt_1e-9": 0,
            "agent_steps": 0,
        }
        for _ in range(frames):
            cmds = env.controller.step(env.agents, env.cargoes, env.t, env.dt)
            for r in env.controller._last_filter_results:
                counts["agent_steps"] += 1
                counts[r.status] = counts.get(r.status, 0) + 1
                if not r.zero_input_feasible:
                    counts["zero_barrier_false"] += 1
                if not getattr(r, "zero_input_feasible_with_rho", True):
                    counts["zero_with_rho_false"] += 1
                if getattr(r, "inside_margin_band", False):
                    counts["inside_margin_band"] += 1
                counts["object_rows_active_sum"] += int(r.object_rows_active)
                if r.modification > 1e-9:
                    counts["modification_gt_1e-9"] += 1
            env.controller.apply_commands(env.agents, cmds, env.dt)
            env.engine.step(env.cargoes, env.agents, env.dt)
            env.t += env.dt
        per_seed.append(counts)
        for k in (
            "agent_steps",
            "optimal",
            "relaxed_margin",
            "scaled_barrier",
            "zero_barrier_false",
            "zero_with_rho_false",
            "inside_margin_band",
            "object_rows_active_sum",
            "modification_gt_1e-9",
        ):
            aggregate[k] += counts[k]
        print(json.dumps(counts, ensure_ascii=False))

    obstacle = None
    if aggregate["zero_with_rho_false"] > 0:
        obstacle = (
            "u=0 is not feasible for the final QP (object rows with rho) on "
            f"{aggregate['zero_with_rho_false']} / {aggregate['agent_steps']} agent-steps; "
            "projection comparison against 0 is unavailable on those steps. "
            "rho rows were not deleted."
        )
    payload = {
        "rho": float(rho) if rho is not None else None,
        "per_seed": per_seed,
        "aggregate": aggregate,
        "fraction_inside_margin_band": float(
            aggregate["inside_margin_band"] / aggregate["agent_steps"]
        )
        if aggregate["agent_steps"]
        else None,
        "projection_optimality_obstacle": obstacle,
        "note": (
            "zero_barrier = margin-free object RHS; "
            "zero_with_rho = full final constraint set including rho"
        ),
    }
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / "projection_margin_band.json"
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    # Merge into projection_conditions.json without wiping prior fields.
    proj_path = OUT / "projection_conditions.json"
    prior = {}
    if proj_path.exists():
        prior = json.loads(proj_path.read_text(encoding="utf-8"))
    prior.update(
        {
            "zero_input_infeasible_with_rho_agent_steps": aggregate["zero_with_rho_false"],
            "inside_margin_band_agent_steps": aggregate["inside_margin_band"],
            "fraction_inside_margin_band": payload["fraction_inside_margin_band"],
            "projection_optimality_obstacle": obstacle,
            "zero_input_vs_rho": payload["note"],
            "verdict": (
                "U = Pi_F(u_nom) on full intended set (all optimal); "
                "u=0 fails object rho rows on margin-band steps — "
                "0-feasible dissipation path blocked there"
                if aggregate["zero_with_rho_false"] > 0 and aggregate["relaxed_margin"] == 0
                else prior.get("verdict")
            ),
            "margin_band_audit": payload,
        }
    )
    proj_path.write_text(json.dumps(prior, indent=2, ensure_ascii=False), encoding="utf-8")
    print("wrote", path)


if __name__ == "__main__":
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    main()
