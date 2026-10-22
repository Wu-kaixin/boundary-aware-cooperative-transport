"""Freeze and audit the complete, explicitly bounded CCTA experiment matrix."""
from __future__ import annotations

import argparse
import copy
import json
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import yaml

from dbact_sim.environment import SimulationEnvironment
from dbact_sim.experiments.deployment import config_for, dump, encode
from dbact_sim.experiments.paths import repository_root
from dbact_sim.experiments.study import cached_result, digest, source_hashes


def build_cases(plan: dict, root: Path) -> list[dict]:
    cases = []

    def add(split, condition, shape, seed, gate, cfg, variant, arm=None, factors=None):
        cases.append({"case_id": f"{split}_{condition}_{arm or variant}", "condition_id": condition,
                      "split": split, "gate": gate, "shape": shape, "seed": seed,
                      "variant": variant, "arm": arm or variant, "stratum": condition,
                      "frames": plan["static_frames" if gate == 5 else "transport_frames"],
                      "factors": factors or {}, "config": copy.deepcopy(cfg)})

    for shape in plan["shapes"]:
        for seed in plan["development_seeds"]:
            condition = f"{shape}_seed{seed}"
            for arm in plan["static_arms"]:
                cfg = config_for(5, shape)
                variant = "oracle" if arm == "oracle" else "local"
                cfg["controller"]["theorem_map_source"] = variant
                cfg["controller"]["theorem_local_segments"] = arm == "local_segments"
                add("static", condition, shape, seed, 5, cfg, variant, arm)
            cfg = config_for(6, shape)
            for method in plan["methods"]:
                add("development", condition, shape, seed, 6, cfg, method)
        # Old plans retain their original one-seed/null-gain diagnostic exactly.
        # Prospective plans can request independent seeds and an active gain.
        for seed in plan.get("ablation_seeds", [plan["development_seeds"][0]]):
            cfg = config_for(6, shape)
            factors = {}
            if "ablation_exploration_gain" in plan:
                gain = float(plan["ablation_exploration_gain"])
                if not np.isfinite(gain) or gain <= 0:
                    raise ValueError("ablation_exploration_gain must be positive and finite")
                cfg["controller"]["explore_gain"] = gain
                factors = {"explore_gain": gain, "diagnostic": "active_gain_ablation"}
            for method in plan["ablations"]:
                add("ablation", f"{shape}_seed{seed}", shape, seed, 6, cfg, method, factors=factors)
            cfg = config_for(6, shape)
            cfg["controller"]["explore_gain"] = plan["active_exploration_gain"]
            for method in ("boundary_cvt", "no_exploration"):
                add("active_exploration", f"{shape}_seed{seed}", shape, seed, 6, cfg, method,
                    factors={"explore_gain": plan["active_exploration_gain"]})
        for seed in plan["robustness_seeds"]:
            for perturbation in plan["robustness"]:
                cfg = config_for(6, shape)
                cfg["controller"].update(perturbation["controller"])
                condition = f"{shape}_seed{seed}_{perturbation['name']}"
                for method in plan["methods"]:
                    add("robustness", condition, shape, seed, 6, cfg, method, factors=perturbation)

    # The old geometry/layout factors are retained, but the random seeds are new.
    # These cases are not used for choosing repairs or thresholds.
    template = yaml.safe_load((root / plan["generalization_template"]).read_text(encoding="utf-8"))
    conditions = {}
    for original in template["cases"]:
        if original["split"] != "test" or original["variant"] != "boundary_cvt":
            continue
        key = (original["shape"], original["stratum"])
        if key in conditions:
            raise ValueError(f"Duplicate generalization template condition: {key}")
        conditions[key] = original
    for index, ((shape, stratum), original) in enumerate(sorted(conditions.items())):
        seed = plan["generalization_seed_start"] + index
        condition = f"{shape}_{stratum.replace('.', 'p')}_seed{seed}"
        for method in plan["methods"]:
            add("generalization", condition, shape, seed, 6, original["config"], method,
                factors={"template_stratum": stratum, "new_seed": True})

    for backend in ("qp", "cvxpy"):
        cfg = config_for(5, "rectangle")
        cfg["controller"]["backend"] = backend
        add("numerical", f"rectangle_qp_{backend}", "rectangle", 2, 5, cfg, "oracle", f"oracle_{backend}")
    for engine in ("penalty", "pymunk"):
        cfg = config_for(6, "l_shape")
        cfg["transport"]["engine"] = engine
        add("engine", f"l_shape_{engine}", "l_shape", 2, 6, cfg, "boundary_cvt")
    if len({case["case_id"] for case in cases}) != len(cases):
        raise ValueError("Duplicate case IDs in the experiment design")
    return cases


def freeze(plan_path: Path, output: Path) -> dict:
    root = repository_root()
    if output.exists():
        raise RuntimeError("A frozen protocol must not be overwritten")
    snapshot = output.parent / "execution_source"
    if snapshot.exists():
        raise RuntimeError("Execution snapshot already exists; choose an independent freeze directory")
    plan = yaml.safe_load(plan_path.read_text(encoding="utf-8"))
    cases = build_cases(plan, root)
    # Preflight all cases without executing a frame. No invalid initialization is
    # replaced or resampled selectively after looking at its measured outcome.
    for case in cases:
        env = SimulationEnvironment(copy.deepcopy(case["config"]), seed=case["seed"])
        case["expected_initial_state_sha256"] = encode(np.vstack([a.position for a in env.agents]).tolist())
        case["layout_notes"] = env.layout_notes
    source = source_hashes(root)
    protocol = {"version": plan["version"], "design_sha256": digest(plan_path),
                "template_sha256": digest(root / plan["generalization_template"]),
                "frozen_source_sha256": encode(source), "run_dir": "runs/ccta2027/draft_v1",
                "acceptance": plan["acceptance"], "planned_splits": dict(Counter(c["split"] for c in cases)),
                "cases": cases}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(protocol, sort_keys=False), encoding="utf-8")
    for name in source:
        target = snapshot / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(root / name, target)
    dump(output.parent / "freeze_manifest.json", {"protocol_sha256": digest(output), "source_files": source,
         "source_sha256": encode(source), "planned_splits": protocol["planned_splits"],
         "execution_source": str(snapshot), "preflight": "PASS", "cases": len(cases)})
    return protocol


def audit(protocol_path: Path, run: Path) -> dict:
    protocol = yaml.safe_load(protocol_path.read_text(encoding="utf-8"))
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    cases = {c["case_id"]: c for c in protocol["cases"]}
    results = summary["results"]
    failures, groups = [], defaultdict(list)
    if {r["case_id"] for r in results} != set(cases) or len(results) != len(cases):
        failures.append("Missing, unexpected or duplicated episodes")
    if summary.get("source_integrity") != "PASS":
        failures.append("Source integrity not established")
    if summary["manifest"]["identity"]["protocol_sha256"] != digest(protocol_path):
        failures.append("Protocol identity mismatch")
    if summary["manifest"]["identity"]["source_sha256"] != protocol["frozen_source_sha256"]:
        failures.append("Frozen source mismatch")
    for result in results:
        case = cases[result["case_id"]]
        try:
            identity = {**summary["manifest"]["identity"], "case_sha256": encode(case)}
            cached = cached_result(run / case["case_id"], identity)
            if cached != result:
                failures.append(f"Summary differs from case receipt: {case['case_id']}")
        except RuntimeError as exc:
            failures.append(str(exc))
        if result["execution_status"] != "COMPLETED":
            failures.append(f"Execution error: {case['case_id']}")
            continue
        if result.get("initial_state_sha256") != case["expected_initial_state_sha256"]:
            failures.append(f"Initial state mismatch: {case['case_id']}")
        groups[(case["split"], case["condition_id"])].append(result)
        if case["gate"] == 5 and case["arm"] in ("oracle", "local_segments", "oracle_qp", "oracle_cvxpy"):
            if result.get("safety_evaluation_status") != "CERTIFIED_SAFE":
                failures.append(f"Static path safety not established: {case['case_id']}")
        if case["gate"] == 5 and result.get("frames_completed") != case["frames"]:
            failures.append(f"Incomplete static horizon: {case['case_id']}")
    for key, rows in groups.items():
        if len({r["initial_state_sha256"] for r in rows}) != 1:
            failures.append(f"Unmatched initial states: {key}")
        dynamic = [r for r in rows if r.get("gate") == 6]
        if len({r["config_sha256"] for r in dynamic}) > 1 or len({encode(r["task"]) for r in dynamic}) > 1:
            failures.append(f"Unmatched downstream config or task: {key}")
    return {"status": "PASS" if not failures else "FAIL", "failures": failures,
            "planned": len(cases), "recorded": len(results),
            "splits": dict(Counter(r.get("split") for r in results)),
            "scientific_note": "Experimental completeness and implementation checks; no superiority or dynamic-safety theorem implied"}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    freeze_parser = sub.add_parser("freeze")
    freeze_parser.add_argument("--plan", type=Path, required=True)
    freeze_parser.add_argument("--out", type=Path, required=True)
    audit_parser = sub.add_parser("audit")
    audit_parser.add_argument("--protocol", type=Path, required=True)
    audit_parser.add_argument("--run", type=Path, required=True)
    audit_parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command == "freeze":
        protocol = freeze(args.plan, args.out)
        print(json.dumps({"cases": len(protocol["cases"]), "splits": protocol["planned_splits"]}, indent=2))
        return 0
    result = audit(args.protocol, args.run)
    dump(args.out, result)
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 1
