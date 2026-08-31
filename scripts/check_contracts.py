#!/usr/bin/env python
"""C1-C3 contract self-check for a scenario configuration.

Runs in well under a second and is meant to be run before anything else: it
answers "can this configuration possibly produce the experiment it describes?"
without simulating a single step.

    python scripts/check_contracts.py --config configs/sim/v2/l_shape_v2.yaml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dbact.contracts import ContractViolation  # noqa: E402
from dbact_sim.scenarios import (  # noqa: E402
    contact_params_from_config,
    controller_params_from_config,
    goal_directions_from_config,
    is_paper_config,
    load_yaml,
    validate_config,
)

OK = "PASS"
BAD = "FAIL"


def check(config_path: Path) -> tuple[bool, list[str]]:
    lines: list[str] = []
    failed = False

    cfg = load_yaml(config_path)
    lines.append(f"config      : {config_path}  (paper={is_paper_config(cfg)})")

    try:
        validate_config(cfg)
        lines.append(f"  [{OK}] C2 provenance: transport.engine={cfg['transport']['engine']!r}, "
                     f"controller.backend={cfg['controller']['backend']!r} both explicit")
    except ContractViolation as exc:
        failed = True
        lines.append(f"  [{BAD}] C2 provenance:\n      {exc}")
        return not failed, lines

    params = controller_params_from_config(cfg)

    contract = params.contact_contract()
    lo, hi = contract.contact_band
    problems = contract.violations()
    if problems:
        failed = True
        lines.append(f"  [{BAD}] C1 contact/safety:")
        for problem in problems:
            lines.append(f"      {problem}")
    else:
        lines.append(
            f"  [{OK}] C1 contact/safety: r_safe={lo:.4f} < d_c={params.cage_offset:.4f} < "
            f"r_robot={hi:.4f}; gamma_obj*(d_c-r_safe)-rho={contract.barrier_margin:+.4f} > 0; "
            f"d_min={params.d_min:.4f} >= 2*r_robot={2 * params.robot_radius:.4f}"
        )
        lines.append(
            f"         => penetration budget delta_max={params.delta_max:.4f} m, "
            f"peak normal force at the cage ring ~ k_p*(r_robot-d_c) = "
            f"{contact_params_from_config(cfg).stiffness * (params.robot_radius - params.cage_offset):.2f} N"
        )

    coverage = params.coverage_contract()
    coverage_problems = coverage.violations()
    if coverage_problems:
        failed = True
        lines.append(f"  [{BAD}] coverage/neighbour completeness:")
        for problem in coverage_problems:
            lines.append(f"      {problem}")
    else:
        lines.append(
            f"  [{OK}] coverage/neighbour completeness: R_l={params.local_radius:.4f} <= "
            f"R_comm/2={0.5 * params.comm_range:.4f}, so the local Voronoi cell is exact"
        )

    goals = goal_directions_from_config(cfg)
    if params.task_mode == "transport":
        missing = [c.get("id", "cargo") for c in cfg.get("cargoes", []) if str(c.get("id", "cargo")) not in goals]
        if missing:
            failed = True
            lines.append(f"  [{BAD}] C3 success criterion: no goal direction for {missing}")
        else:
            evaluation = cfg.get("evaluation", {})
            lines.append(
                f"  [{OK}] C3 success criterion: J_min={evaluation.get('j_min', 0.15)} m, "
                f"efficiency_min={evaluation.get('efficiency_min', 0.7)}, "
                f"goal directions for {sorted(goals)}"
            )
    else:
        lines.append(f"  [--]  C3 success criterion: task_mode={params.task_mode!r}, transport success not evaluated")

    contact = contact_params_from_config(cfg)
    if abs(contact.robot_radius - params.robot_radius) > 1e-9:
        failed = True
        lines.append(
            f"  [{BAD}] radius consistency: contact model r={contact.robot_radius:.4f} != "
            f"controller r={params.robot_radius:.4f}"
        )
    else:
        lines.append(f"  [{OK}] radius consistency: contact model and safety filter share r={contact.robot_radius:.4f}")

    return not failed, lines


def main() -> int:
    parser = argparse.ArgumentParser(description="Check the C1-C3 contracts for one or more configs.")
    parser.add_argument("--config", nargs="+", required=True, help="Scenario YAML file(s) or directory.")
    args = parser.parse_args()

    paths: list[Path] = []
    for item in args.config:
        p = Path(item)
        paths.extend(sorted(p.glob("*.yaml")) if p.is_dir() else [p])

    all_ok = True
    for path in paths:
        ok, lines = check(path)
        all_ok &= ok
        print("\n".join(lines))
        print()
    print("ALL CONTRACTS PASS" if all_ok else "CONTRACT CHECK FAILED")
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
