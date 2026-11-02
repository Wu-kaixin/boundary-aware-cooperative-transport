"""Preflight and freeze the nine-condition development census, without stepping physics."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from dbact.cargo import Cargo
from dbact.geometry import polygon_diameter, rotate
from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import config_for, dump, encode
from dbact_sim.experiments.shapes import smooth_outline
from dbact_sim.experiments.study import source_hashes

ROOT = Path(__file__).resolve().parents[1]
METHODS = ("boundary_cvt", "nearest_observed_target", "local_spacing")
CONDITIONS = (
    ("circle", "circle", 0),
    ("ellipse_parallel", "ellipse24", 0),
    ("ellipse_perpendicular", "ellipse24", 90),
    ("rectangle_parallel", "rectangle", 0),
    ("rectangle_perpendicular", "rectangle", 90),
    ("l_reference", "l_shape", 0),
    ("l_rotated", "l_shape", 180),
    ("c_open_toward", "c_shape", -90),
    ("c_open_away", "c_shape", 90),
)


def build_plan():
    base = config_for(6, "l_shape")
    baseline = SimulationEnvironment(copy.deepcopy(base), seed=2)
    target_area = baseline.cargoes[0].area
    density = baseline.cargoes[0].surface_density
    outlines = {}
    sources = {}
    for shape in ("circle", "ellipse24", "rectangle", "l_shape", "c_shape"):
        if shape in ("circle", "ellipse24"):
            probe = Cargo("cargo_0", smooth_outline(shape))
            source = "scripts/run_arbitrary_shape_monte_carlo.py"
        else:
            probe = SimulationEnvironment(config_for(6, shape), seed=2).cargoes[0]
            source = f"configs/sim/theorem/static_{shape}_n16_oracle.yaml"
        outlines[shape] = probe.local_vertices * np.sqrt(target_area / probe.area)
        sources[shape] = {"path": source, "sha256": hashlib.sha256((ROOT / source).read_bytes()).hexdigest(),
                          "normalization": "centroid recentered, isotropic equal polygon area"}
    # One common legal annulus: no shape-dependent positions or post-outcome resampling.
    reach = max(float(np.linalg.norm(v, axis=1).max()) for v in outlines.values())
    base["agents"].update(center=[4.0, 4.0], radius_min=reach + 0.35, radius_max=reach + 1.15)
    base["task"].update(mode="random_constrained", angle_min_deg=0.0, angle_max_deg=0.0,
                         distance_min=1.2, distance_max=1.2)
    cases, metadata = [], []
    for name, shape, angle in CONDITIONS:
        cfg = copy.deepcopy(base)
        vertices = rotate(outlines[shape], np.deg2rad(angle)) + [4.0, 4.0]
        cfg["cargoes"] = [{"id": "cargo_0", "shape": "polygon", "vertices": vertices.tolist(),
                           "movable": True, "surface_density": density}]
        env = SimulationEnvironment(copy.deepcopy(cfg), seed=2)  # validates robot/object/wall clearance
        cargo = env.cargoes[0]
        initial = [a.position.tolist() for a in env.agents]
        task = env.tasks[cargo.object_id].as_dict()
        meta = {"condition": name, "shape": shape, "angle_deg": angle, "geometry_source": sources[shape],
                "area_m2": cargo.area, "mass_kg": cargo.mass, "inertia_kg_m2": cargo.inertia,
                "perimeter_m": cargo.perimeter, "diameter_m": float(polygon_diameter(cargo.vertices)),
                "reach_m": float(np.linalg.norm(cargo.local_vertices, axis=1).max()),
                "vertices": len(cargo.vertices), "initial_centroid": cargo.position.tolist(),
                "task": task, "agent_initial_sha256": encode(initial),
                "complete_initial_sha256": encode({"robots": initial, "vertices": cargo.vertices.tolist(), "task": task})}
        metadata.append(meta)
        for method in METHODS:
            cases.append({"case_id": f"{name}_{method}", "condition_id": name, "shape": shape,
                          "split": "development", "gate": 6, "seed": 2, "frames": 3000,
                          "variant": method, "config": cfg,
                          "expected_initial_state_sha256": meta["agent_initial_sha256"], "factors": meta})
    return {"version": "multishape-census-v1", "scope": "Development census, one seed; no generalization claim",
            "run_dir": "runs/research/2026-10-31-multishape-census-v1/run", "conditions": metadata,
            "fixed_budget": {"episodes": 27, "seed": 2, "max_frames": 3000, "dt": .05, "workers": 2,
                             "stopping": "original HOLD + speed<=0.005 m/s for 40 frames, else watchdog"},
            "normalization": {"area_m2": target_area, "surface_density": density, "distance_m": 1.2,
                              "agent_annulus": base["agents"], "controller": "unchanged l_shape_closed_loop",
                              "physics": "unchanged penalty contact, four substeps", "feedback": "original; no anchored readout"},
            "visual_selection": {"overview_method": "boundary_cvt", "overview_conditions": [CONDITIONS[i][0] for i in (0,1,3,5,7)],
                                 "comparison": "all nine conditions; three methods share physical time and view",
                                 "stages": "first CONTACT_READY; midpoint of TRANSPORT-to-BRAKE if present; actual final frame",
                                 "missing_stage": "show explicit not reached, never substitute success",
                                 "video": "every fifth control frame plus final, 10 fps (2.5x physical time); ended runs held and labelled"},
            "cases": cases}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    plan = build_plan()
    if args.dry_run:
        print(json.dumps({"conditions": plan["conditions"], "episodes": len(plan["cases"])}, indent=2))
        return
    target = args.out.resolve()
    if not target.is_relative_to(ROOT / "experiments/ccta2027") or target.exists():
        raise ValueError("Use a new protocol path under experiments/ccta2027; never replace a freeze")
    plan["frozen_source_sha256"] = encode(source_hashes())
    dump(target, plan)  # JSON is also accepted by the unified YAML protocol loader.
    print(f"Frozen {len(plan['cases'])} cases: {target}")


if __name__ == "__main__":
    main()
