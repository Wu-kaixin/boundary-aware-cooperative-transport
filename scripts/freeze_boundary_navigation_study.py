"""Freeze one six-method, nine-condition development batch. No parameter search."""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
import sys
from pathlib import Path

from dbact_sim.experiments.deployment import dump, encode
from dbact_sim.experiments.protocol import load_protocol
from dbact_sim.experiments.study import cached_result, source_hashes, environment_versions

ROOT = Path(__file__).resolve().parents[1]
MISSION = "2026-11-01-boundary-navigation-v1"
ARMS = {"cvt": ("boundary_cvt", "legacy"), "spacing": ("local_spacing", "legacy"),
        "push_cone": ("push_cone", "legacy"), "adaptive_arc": ("boundary_cvt", "adaptive_arc"),
        "vector_cvt": ("boundary_cvt", "vector_goal"), "vector_spacing": ("local_spacing", "vector_goal")}


def read(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def sha(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def freeze(dry_run=False):
    if Path(sys.prefix).resolve() != (ROOT/".venv").resolve():
        raise RuntimeError("Use the project root .venv")
    target = ROOT/"experiments/ccta2027/boundary_navigation_v1.json"
    if target.exists(): raise RuntimeError("Already frozen; do not refreeze")
    history = ROOT/"runs/research/2026-10-31-multishape-census-v1/run"
    old = read(history/"resolved_protocol.json")
    originals = [c for c in old["cases"] if c["variant"] == "boundary_cvt"]
    originals.sort(key=lambda c: c["condition_id"] != "c_open_away")  # known longest job first
    inputs, cases = {}, []
    for original in originals:
        # Verify every previously frozen control we will compare, not just a summary.
        for variant in ("boundary_cvt", "local_spacing"):
            name = original["case_id"].replace("boundary_cvt", variant)
            folder = history/name
            receipt = read(folder/"case_result.json")
            cached_result(folder, receipt["identity"])
            inputs[(folder/"case_result.json").relative_to(ROOT).as_posix()] = sha(folder/"case_result.json")
            for name, digest in receipt["products"].items():
                inputs[(folder/name).relative_to(ROOT).as_posix()] = digest
        for arm, (variant, navigation) in ARMS.items():
            case = copy.deepcopy(original)
            case.update(case_id=f"{original['condition_id']}_{arm}", arm=arm,
                        split="development", frames=12000, variant=variant)
            case["config"]["controller"].update(boundary_navigation=navigation,
                transport_safety_feedback=False, dynamic_contact_roles=False, confirmed_contact_handoff=False)
            case["factors"]["source_case"] = original["case_id"]
            cases.append(case)
    assert len(cases) == 54
    for name in {c["condition_id"] for c in cases}:
        group = [c for c in cases if c["condition_id"] == name]
        normalized = []
        for c in group:
            cfg = copy.deepcopy(c["config"])
            cfg["controller"].pop("boundary_navigation")
            normalized.append(encode(cfg))
        assert len(set(normalized)) == 1
        assert len({c["expected_initial_state_sha256"] for c in group}) == 1
    audit = ROOT/"artifacts/migrations"/MISSION
    design = read(audit/"design_before_implementation.json")
    plan = dict(version="boundary-navigation-v1", status="FROZEN_BEFORE_EXECUTION",
        cases=cases, arms=list(ARMS), conditions=old["conditions"],
        run_dir=f"runs/research/{MISSION}/run", design=design,
        budget=dict(episodes=54, seed=2, physical_seconds=600, report_horizons=[150,600],
                    workers=8, internal_threads=1, wall_limit_seconds=3600, versions=1),
        evaluation=dict(completion="Original HOLD + speed <= .005 for 40 frames AND final Euclidean error <= .12 m",
            horizon="Use saved trajectory at 150 s for unended runs; completion time only for tasks already terminated successfully",
            censoring="Unfinished at 600 s is right-censored; early off-target HOLD is separately reported",
            safety="Original separation, clearance, penetration and QP failure checks; scaling events separately reported",
            comparisons="All six methods paired within the same nine geometry/direction conditions; no independent reliability estimate",
            gate="No default promotion on this reused development seed; a promising candidate gains >=2 completions vs both controls, loses <=1, and introduces no hard safety violations",
            error_policy="Every planned/launched case remains in denominator; errors, timeouts, regressions and budget cutoffs retained"),
        information=dict(local="Own boundary map and one-hop fresh credible displacement/velocity messages",
            shared="All-team estimate used for common phase only; not fully decentralized",
            truth="Only unchanged sensor/physics/independent evaluation paths; never in new navigation functions",
            communication="Vector arms expose two displacement floats per live directed neighbor link each frame; baseline scalar progress is one"),
        historical_inputs=inputs, frozen_source_sha256=encode(source_hashes()),
        environment=environment_versions(), freeze_script_sha256=sha(Path(__file__)))
    if dry_run:
        print(json.dumps({"episodes":len(cases),"source":plan["frozen_source_sha256"]})); return
    dump(target, plan)
    load_protocol(target, "development", ROOT)
    for source in (Path(__file__), ROOT/"tests/test_boundary_navigation.py"):
        dest = audit/"execution_helpers"/source.name
        dest.parent.mkdir(parents=True, exist_ok=True); dest.write_bytes(source.read_bytes())
    dump(audit/"freeze_receipt.json", {"protocol":target.relative_to(ROOT).as_posix(),"sha256":sha(target)})
    print(f"Frozen {len(cases)} episodes: {target}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    freeze(parser.parse_args().dry_run)
