"""Static geometry/units audit only: no environment, controller or physics calls."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
import time
from datetime import datetime, timezone
from decimal import Decimal
from pathlib import Path

import numpy as np
import yaml

from dbact.geometry import make_l_shape, make_rectangle
from dbact_sim.experiments.shapes import cargo_outline, smooth_outline

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = "experiments/ccta2027/post_acquisition_finite_support_v1.json"
DESIGN = "experiments/ccta2027/phase2_local_geometry_contract_v1_design.md"
GEOMETRIES = {
    name: f"configs/sim/theorem/static_{name}_n16_oracle.yaml"
    for name in ("l_shape", "rectangle", "c_shape")
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def area(points):
    x, y = np.asarray(points).T
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2)


def describe(points):
    p = np.asarray(points, dtype=float)
    edges = np.roll(p, -1, axis=0) - p
    lengths = np.linalg.norm(edges, axis=1)
    tangents = edges / lengths[:, None]
    previous = np.roll(tangents, 1, axis=0)
    cross = previous[:, 0] * tangents[:, 1] - previous[:, 1] * tangents[:, 0]
    turns = np.arctan2(cross, np.einsum("ij,ij->i", previous, tangents))
    # Every owning outline is counterclockwise; fail rather than mislabel concavity.
    assert abs(float(turns.sum()) - 2 * math.pi) < 1e-10
    jumps = np.linalg.norm(tangents - previous, axis=1)
    assert np.allclose(jumps, 2 * np.abs(np.sin(turns / 2)), atol=1e-14)
    return {
        "vertices_m": p.tolist(), "vertex_count": len(p), "area_m2": area(p),
        "perimeter_m": float(lengths.sum()), "reentrant_corners": int((turns < -1e-10).sum()),
        "minimum_edge_m": float(lengths.min()), "maximum_edge_m": float(lengths.max()),
        "total_tangent_jump_variation": float(jumps.sum()),
        "maximum_single_jump": float(jumps.max()),
        "identity": "actual finite polygon, not a globally C2 boundary",
    }


def main():
    started = time.perf_counter()
    if Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        raise RuntimeError("Use the root .venv")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    output = parser.parse_args().output.resolve()
    if not output.is_relative_to(ROOT / "runs") or output.exists():
        raise ValueError("Use a new output file under runs/")
    protocol = json.loads((ROOT / PROTOCOL).read_text(encoding="utf-8"))
    c = protocol["cases"][0]["config"]["controller"]
    names = ("robot_radius", "delta_max", "gamma_obj", "rho", "range_noise_std", "ray_count")
    constants = {k: c[k] for k in names}
    assert all(all(case["config"]["controller"][k] == c[k] for k in names)
               for case in protocol["cases"])
    eps = Decimal(3) * Decimal(str(c["range_noise_std"]))
    delta, rho, gamma = (Decimal(str(c[k])) for k in ("delta_max", "rho", "gamma_obj"))
    structure = (delta - rho / gamma) / 2 - eps
    velocity = gamma * (delta - 2 * eps) - rho
    assert structure == Decimal("0.00125") and velocity == Decimal("0.02")
    angle = 2 * math.pi / c["ray_count"]
    budget = {
        "epsilon_m": str(eps), "strict_structure_limit_at_D_plus_a_zero_m": str(structure),
        "strict_D_plus_a_limit_at_E_epsilon_m_per_s": str(velocity),
        "strict_M_theta_limit_m_per_rad2": 8 * float(structure) / angle**2,
        "strict_one_right_corner_Lbar_limit_m": 4 * float(structure) / math.sqrt(2),
        "scope": "Necessary nonempty-band condition for the stated robust certificate only",
    }
    configs = {k: yaml.safe_load((ROOT / v).read_text())["cargoes"][0]
               for k, v in GEOMETRIES.items()}
    lc, rc = configs["l_shape"], configs["rectangle"]
    outlines = {
        "l_shape": make_l_shape(lc["center"], lc["scale"]),
        "rectangle": make_rectangle(rc["center"], rc["width"], rc["height"]),
        "c_shape": np.asarray(configs["c_shape"]["vertices"], dtype=float),
    }
    reference_area = area(outlines["l_shape"])
    scales = {}
    for name in ("circle", "ellipse24", "skew_box"):
        p = cargo_outline(name) if name == "skew_box" else smooth_outline(name)
        scales[name] = math.sqrt(reference_area / area(p))
        outlines[name] = p * scales[name] + [4, 4]
    shapes = {k: describe(v) for k, v in outlines.items()}
    assert {k: v["vertex_count"] for k, v in shapes.items()} == {
        "l_shape": 6, "rectangle": 4, "c_shape": 8, "circle": 64, "ellipse24": 24, "skew_box": 4}
    frozen = {}
    for case in protocol["cases"]:
        name = case["condition_id"]
        p = case["config"]["cargoes"][0]["vertices"]
        if name in frozen:
            assert frozen[name]["vertices_m"] == p
        else:
            frozen[name] = describe(p) | {"shape": case["shape"]}
    radius = 0.58 * scales["circle"]
    a, b = 0.72 * scales["ellipse24"], 0.42 * scales["ellipse24"]
    step = 2 * math.pi / 24
    p0, p1 = np.array([a, 0]), np.array([a * math.cos(step), b * math.sin(step)])
    mid = np.array([a * math.cos(step / 2), b * math.sin(step / 2)])
    t = float(np.dot(mid - p0, p1 - p0) / np.dot(p1 - p0, p1 - p0))
    assert 0 <= t <= 1
    mesh = {
        "scope": "Only needed if replacing actual polygons by ideal smooth parent curves",
        "circle_radius_m": radius, "circle_Hausdorff_exact_m": radius * (1 - math.cos(math.pi / 64)),
        "ellipse_semiaxes_m": [a, b], "ellipse_Hausdorff_upper_m": a * step**2 / 8,
        "ellipse_first_edge_midpoint_witness_m": float(np.linalg.norm(mid - (p0 + t * (p1 - p0)))),
    }
    alpha, leg = 0.1, 0.04
    chord = 2 * math.sin(alpha / 2)
    toy = {
        "major_unit_circle_arc": {
            "endpoint_angle_rad": alpha, "true_arc_length_m": 2 * math.pi - alpha,
            "true_chord_m": chord, "invalid_chord_substitution_m": chord**2 / 8,
            "far_point_distance_to_chord_m": 1 + math.cos(alpha / 2),
            "valid_arc_length_bound_m": (2 * math.pi - alpha)**2 / 8,
        },
        "two_right_angle_edges": {
            "each_leg_m": leg, "true_chord_m": math.sqrt(2) * leg,
            "exact_Hausdorff_m": leg / math.sqrt(2),
            "BV_bound_m": 2 * leg * math.sqrt(2) / 4,
        },
        "scope": "Fixed analytic examples, not physical episodes or observed trajectories",
    }
    assert math.isclose(toy["two_right_angle_edges"]["exact_Hausdorff_m"],
                        toy["two_right_angle_edges"]["BV_bound_m"], abs_tol=1e-15)
    sources = [PROTOCOL, DESIGN, "configs/research/best_known_transport_v1.yaml",
               "src/dbact/geometry.py", "src/dbact/sensing/finite_support.py",
               "src/dbact_sim/experiments/shapes.py", "src/dbact_sim/experiments/deployment.py",
               *GEOMETRIES.values(), str(Path(__file__).relative_to(ROOT)).replace("\\", "/")]
    result = {
        "status": "STATIC_AUDIT_COMPLETE_NOT_A_CONTROLLER_VALIDATION",
        "completed_at_utc": datetime.now(timezone.utc).isoformat(), "executable": sys.executable,
        "new_physical_episodes": 0, "controller_replay_frames": 0,
        "frozen_constants": constants, "conditional_contact_budget": budget,
        "library_geometry": shapes, "frozen_physical_conditions_geometry_only": frozen,
        "smooth_parent_discrepancy": mesh, "fixed_analytic_examples": toy,
        "source_sha256": {name: digest(ROOT / name) for name in sources},
        "wall_seconds": time.perf_counter() - started,
    }
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
    print(json.dumps({"output": str(output), "budget": budget, "mesh": mesh,
                      "seconds": result["wall_seconds"]}, indent=2))


if __name__ == "__main__":
    main()
