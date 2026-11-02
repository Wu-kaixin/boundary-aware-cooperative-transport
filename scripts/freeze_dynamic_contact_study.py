"""Freeze one 27-episode push-only contact study; never overwrite an identity."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.study import cached_result, source_hashes, environment_versions
from dbact_sim.experiments.protocol import load_protocol

ROOT = Path(__file__).resolve().parents[1]
MISSION = "2026-11-01-dynamic-contact-v1"
HISTORY = ROOT / "runs/research/2026-10-31-multishape-census-v1/run"
PROTOCOL = ROOT / "experiments/ccta2027/dynamic_contact_v1.json"
ARMS = ("legacy", "feedback_fix", "dynamic_contact")


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def freeze(dry_run=False):
    if Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        raise RuntimeError("Use root .venv")
    if PROTOCOL.exists():
        raise RuntimeError("Freeze already exists; use its study identity/resume, do not refreeze")
    previous = read(HISTORY / "resolved_protocol.json")
    originals = [c for c in previous["cases"] if c["variant"] == "boundary_cvt"]
    if len(originals) != 9:
        raise RuntimeError("Expected the nine original geometry/orientation conditions")
    inputs, cases = {}, []
    for original in originals:
        directory = HISTORY / original["case_id"]
        receipt = read(directory / "case_result.json")
        cached_result(directory, receipt["identity"])  # verify every historical product
        inputs[(directory / "case_result.json").relative_to(ROOT).as_posix()] = sha(directory / "case_result.json")
        for name, digest in receipt["products"].items():
            inputs[(directory / name).relative_to(ROOT).as_posix()] = digest
        for arm in ARMS:
            case = copy.deepcopy(original)
            case["case_id"] = f"{original['condition_id']}_{arm}"
            case["arm"] = arm
            case["split"] = "development"
            case["config"]["controller"].update(
                transport_safety_feedback=arm != "legacy",
                dynamic_contact_roles=arm == "dynamic_contact")
            case["factors"]["source_case"] = original["case_id"]
            cases.append(case)
    # Identical initial states and every config field except the two declared
    # algorithm switches. This check precedes any new physics execution.
    for condition in {c['condition_id'] for c in cases}:
        group = [c for c in cases if c['condition_id'] == condition]
        normalized = []
        for case in group:
            cfg = copy.deepcopy(case['config'])
            for key in ('transport_safety_feedback', 'dynamic_contact_roles'):
                cfg['controller'].pop(key)
            normalized.append(encode(cfg))
        if len(set(normalized)) != 1 or len({c['expected_initial_state_sha256'] for c in group}) != 1:
            raise RuntimeError("Pairing failure")
    plan = {
        "version": "dynamic-contact-v1", "status": "FROZEN_BEFORE_EXECUTION",
        "scope": "One development block; no reliability/generalization or hardware claim",
        "question": "Can local contact-direction assessment and bounded lateral redeployment improve full-task completion with push-only contact?",
        "run_dir": f"runs/research/{MISSION}/run", "cases": cases,
        "conditions": previous["conditions"], "arms": list(ARMS),
        "fixed_budget": {"episodes":27, "seed":2, "frames_per_episode":3000, "dt":.05,
                         "workers":8, "internal_threads":1, "parameter_variants":1,
                         "stopping":"unchanged HOLD and linear speed<=0.005 for 40 frames, else watchdog",
                         "no_retry_until_success":True},
        "primary": {"completion":"unchanged settled + 2D endpoint <= 0.12 m",
                    "unit":"paired condition within one seed block; not nine independent random seeds",
                    "development_continue_gate":"candidate nets >=2 additional completions vs both legacy and feedback_fix, loses <=1 prior success vs either; no new execution error or hard recorded safety violation",
                    "interpretation":"development screening only; neither significance nor proof of safety"},
        "candidate": {"information":"own fused local map, received neighbour positions, own displacement/velocity estimate, existing task and common phase signals",
                      "fixed":"perception, progress estimator/gain, steering gains, contact engine, safety constraints, phase/stop/success thresholds, initial state, random seed",
                      "normal_allocation":"nonnegative least squares with 0.01 identity augmentation, normalized max coefficient=1; pressure role >0.05",
                      "cone_limit":"direction-only residual; no force/torque/traction/reachability certificate",
                      "relocation":"deficit persists contact_dwell*dt; observed patch within local_radius; normal improvement > push_share_floor; normal dot current >0; no occupied or closer-neighbour target",
                      "relocation_budget":"max3 attempts per robot, each <=2*local_radius/max_speed; one contact_dwell cooldown; total task clock unchanged",
                      "clearance":"max(hold_offset, robot_radius+voxel_size); tangent only after within 0.01m; unchanged QP always runs",
                      "known_limit":"No new lateral stop gate or relaunch after HOLD; estimates may be biased; local map does not establish full wrench feasibility"},
        "historical_inputs": inputs, "frozen_source_sha256": encode(source_hashes()),
        "freeze_script_sha256": sha(Path(__file__)), "environment": environment_versions(),
        "analysis_rules": {"denominator":"all 27 launched/planned episodes; errors and watchdogs retained",
                           "endpoints":"all nine pairs with signed differences, no post-hoc exclusions",
                           "timing":"failed-task termination time is not completion time",
                           "safety":"report raw minimum clearances, penetration, solver fallbacks/scalings and original G500; no new tolerance",
                           "default_compatibility":"all replay arrays and all old per-frame scientific fields must exactly match frozen legacy cases; new diagnostic-only keys excluded explicitly",
                           "stopping":"one candidate version only; failure records remain, no tuning/reruns to achieve success"}}
    if dry_run:
        print(json.dumps({"episodes":len(cases), "conditions":len(originals), "source":plan['frozen_source_sha256']}))
        return
    dump(PROTOCOL, plan)
    load_protocol(PROTOCOL, "development", ROOT)
    audit = ROOT / "artifacts/migrations" / MISSION
    dump(audit / "freeze_receipt.json", {"protocol":str(PROTOCOL.relative_to(ROOT)), "sha256":sha(PROTOCOL)})
    print(f"Frozen: {PROTOCOL}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    freeze(args.dry_run)
