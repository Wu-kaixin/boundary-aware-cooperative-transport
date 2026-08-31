#!/usr/bin/env python
"""Rewrite pre-refactor scenario files into the contract-satisfying schema.

Kept in the repository rather than run once and deleted, because it records what
happened to the old scenarios. Three classes of change:

*Retired names.* ``map_ttl`` (the map has age decay, not a TTL), ``cbf_gamma``
(there are now two class-K gains, inter-robot and object), ``cbf_use_qp`` and
``cbf_slack_weight`` (the filter is a hard QP with no slack). ``DBACTParams``
rejects unknown keys, so these had to go rather than be ignored.

*Required fields.* ``transport.engine`` and ``controller.backend`` have no
defaults any more.

*Geometry that the contracts make binding.* The old files put 12-14 robots of
radius 0.16 around objects roughly 1 m across. With ``d_min >= 2 r_robot`` those
robots need more cage ring than such an object has, and they end up pinned against
each other instead of reaching contact. The cargo is therefore scaled up until the
ring holds the team at a workable spacing, and the block start is replaced by a
scattered one so the robots do not all arrive from the same side.

    python scripts/migrate_configs.py --dry-run
    python scripts/migrate_configs.py
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact.cargo import Cargo  # noqa: E402

ROBOT_RADIUS = 0.16
CAGE_OFFSET = 0.135
D_MIN = 0.34
RING_SPACING = 0.50  # ring length budgeted per robot, with headroom over d_min

CONTROLLER = {
    "sensor_range": 1.20,
    "ray_count": 96,
    "range_noise_std": 0.010,
    "pca_neighbors": 5,
    "residual_tolerance": 0.030,
    "min_confidence": 0.15,
    "comm_range": 1.60,
    "voxel_size": 0.06,
    "age_decay": 0.30,
    "max_voxels_per_object": 600,
    "density_mode": "offset",
    "cage_offset": CAGE_OFFSET,
    "sigma": 0.20,
    "base_density": 0.001,
    "gap_gain": 0.6,
    "gap_radius": 0.35,
    "local_radius": 0.80,
    "grid_resolution": 24,
    "approach_mass_ratio": 3.0,
    "redeploy_gap_ratio": 0.15,
    "robot_radius": ROBOT_RADIUS,
    "delta_max": 0.05,
    "d_min": D_MIN,
    "gamma_agent": 6.0,
    "gamma_obj": 8.0,
    "rho": 0.05,
    "max_speed": 0.30,
    "backend": "qp",
    "object_row_range": 0.60,
    "object_row_window": 0.28,
    "kp_explore": 0.25,
    "kp_cage": 0.90,
    "kp_transport": 0.60,
    "push_side_threshold": 0.35,
    "min_push_agents": 4,
    "contact_band_tolerance": 0.08,
}

TRANSPORT = {
    "engine": "penalty",
    "stiffness": 500.0,
    "damping": 12.0,
    "friction": 0.6,
    "tangential_stiffness": 60.0,
    "ground_friction": 0.45,
    "gravity": 9.81,
    "substeps": 4,
}

EVALUATION = {"contact_radius": 0.42, "j_min": 0.15, "efficiency_min": 0.7, "displacement_gate": 0.1}

RETIRED = ("map_ttl", "cbf_gamma", "cbf_use_qp", "cbf_slack_weight")


def scale_for_team(cfg: dict, cargo_cfg: dict, count: int) -> tuple[dict, float]:
    """Grow the cargo until its cage ring holds ``count`` robots."""
    cargo = Cargo.from_config(cargo_cfg)
    ring = cargo.perimeter + 2.0 * math.pi * CAGE_OFFSET
    needed = count * RING_SPACING
    factor = max(1.0, needed / ring)
    if factor <= 1.0 + 1e-9:
        return cargo_cfg, ring

    updated = dict(cargo_cfg)
    if "scale" in cargo_cfg or cargo_cfg.get("shape") in ("l_shape", "nonconvex"):
        updated["scale"] = round(float(cargo_cfg.get("scale", 1.0)) * factor, 3)
    elif cargo_cfg.get("shape") == "circle":
        updated["radius"] = round(float(cargo_cfg.get("radius", 0.5)) * factor, 3)
    elif cargo_cfg.get("shape") == "rectangle":
        updated["width"] = round(float(cargo_cfg.get("width", 1.0)) * factor, 3)
        updated["height"] = round(float(cargo_cfg.get("height", 0.5)) * factor, 3)
    elif cargo_cfg.get("shape") == "polygon":
        vertices = np.asarray(cargo_cfg["vertices"], dtype=float)
        centroid = vertices.mean(axis=0)
        updated["vertices"] = [[round(v, 4) for v in p] for p in (centroid + factor * (vertices - centroid))]
    return updated, Cargo.from_config(updated).perimeter + 2.0 * math.pi * CAGE_OFFSET


def migrate(path: Path) -> dict | None:
    cfg = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    cargoes = cfg.get("cargoes") or []
    agents = dict(cfg.get("agents", {}))
    count = int(agents.get("count", 12))

    new_cargoes = []
    extents = []
    for cargo_cfg in cargoes:
        scaled, _ = scale_for_team(cfg, cargo_cfg, count if len(cargoes) == 1 else max(4, count // len(cargoes)))
        new_cargoes.append(scaled)
        cargo = Cargo.from_config(scaled)
        extents.append((cargo.position, float(np.max(np.linalg.norm(cargo.vertices - cargo.position, axis=1)))))

    controller = dict(CONTROLLER)
    old = cfg.get("controller", {})
    controller["task_mode"] = old.get("task_mode", "transport" if new_cargoes else "coverage")
    if controller["task_mode"] == "transport":
        controller["lead_offset"] = 0.22
        controller["lead_threshold"] = 0.35
    for key in ("target_center", "target_radius", "target_sensor_range", "target_samples"):
        if key in old:
            controller[key] = old[key]
    # Preserve a deliberately conservative inter-robot gain if the file had one.
    if "cbf_gamma" in old:
        controller["gamma_agent"] = float(old["cbf_gamma"])

    if new_cargoes:
        center, extent = extents[0] if len(extents) == 1 else (
            np.mean([e[0] for e in extents], axis=0),
            max(np.linalg.norm(e[0] - np.mean([x[0] for x in extents], axis=0)) + e[1] for e in extents),
        )
        agents = {
            "count": count,
            "layout": "scatter",
            "center": [round(float(center[0]), 3), round(float(center[1]), 3)],
            "radius_min": round(float(extent) + 0.45, 3),
            "radius_max": round(float(extent) + 0.45 + 0.85, 3),
            "min_separation": 0.40,
        }

    out = {"dt": float(cfg.get("dt", 0.05)), "domain": cfg.get("domain", {}), "agents": agents}
    if new_cargoes:
        out["cargoes"] = new_cargoes
    out["controller"] = controller
    out["transport"] = dict(TRANSPORT)
    out["evaluation"] = dict(EVALUATION)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Migrate pre-refactor scenario files.")
    parser.add_argument("--dir", default="configs/sim")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    root = Path(args.dir)
    changed = 0
    for path in sorted(root.glob("*.yaml")):
        original = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        retired = [k for k in RETIRED if k in original.get("controller", {})]
        needs = retired or "engine" not in original.get("transport", {}) or "backend" not in original.get("controller", {})
        if not needs:
            print(f"[ok]      {path.name}")
            continue
        migrated = migrate(path)
        print(f"[migrate] {path.name}  retired={retired or '-'}")
        changed += 1
        if not args.dry_run:
            header = (
                "# Migrated from the pre-refactor schema by scripts/migrate_configs.py.\n"
                "# Not a paper configuration: the main experiment lives in configs/sim/v2/.\n"
                "# Coverage numbers produced before the object-boundary CBF existed are void,\n"
                "# because robots standing inside the cargo counted as covering its boundary.\n\n"
            )
            path.write_text(header + yaml.safe_dump(migrated, sort_keys=False, default_flow_style=False), encoding="utf-8")
    print(f"\n{changed} file(s) {'would be ' if args.dry_run else ''}migrated")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
