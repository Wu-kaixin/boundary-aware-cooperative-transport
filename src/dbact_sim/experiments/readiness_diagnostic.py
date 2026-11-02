"""Frozen small oracle diagnostic, never a deployable control method."""

import copy
import hashlib
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from dbact_sim.experiments import deployment

ROOT = Path(__file__).resolve().parents[3]


def install_progress_oracle(env):
    controller = env.controller
    original = controller._update_progress
    env.oracle_events = []

    def update(agents, neighbors):
        original(agents, neighbors)
        for cargo in env.cargoes:
            cid = cargo.object_id
            truth = env.tasks[cid].progress(cargo.position)
            env.oracle_events.append(
                {
                    "time": env.t,
                    "object": cid,
                    "estimated_before": controller.team_progress[cid],
                    "oracle": truth,
                }
            )
            controller.team_progress[cid] = truth
            for agent in agents:
                controller._progress[agent.agent_id][cid] = truth

    controller._update_progress = update


def run_case(payload):
    case, mode, out_string = payload
    from dbact.cpu_budget import limit_blas_threads

    limit_blas_threads(1)
    out = Path(out_string)
    base = deployment.SimulationEnvironment
    created = []

    def factory(*args, **kwargs):
        env = base(*args, **kwargs)
        if mode == "oracle_progress_only":
            install_progress_oracle(env)
        created.append(env)
        return env

    start = time.monotonic()
    try:
        deployment.SimulationEnvironment = factory  # isolated worker; always restored
        result = deployment.transport_arm(
            copy.deepcopy(case["config"]),
            case["seed"],
            "boundary_cvt",
            case["frames"],
            out,
        )
        # transport_arm describes a different deployment ablation. The outer
        # diagnostic identity is authoritative; never inherit that old label.
        result["changed_mechanism"] = (
            "Original boundary-CVT feedback; no diagnostic intervention"
            if mode == "baseline"
            else "Only per-agent and team scalar progress feedback replaced by truth, from episode start"
        )
        if mode == "oracle_progress_only":
            deployment.dump(out / "oracle_events.json", created[0].oracle_events)
        assert result["initial_state_sha256"] == case["expected_initial_state_sha256"]
        receipt = {
            "status": "EXECUTED",
            "case_id": case["case_id"],
            "mode": mode,
            "result": result,
            "seconds": time.monotonic() - start,
            "information": "TRUE_PROGRESS_ONLY"
            if mode.startswith("oracle")
            else "ORIGINAL_OBSERVATIONS",
        }
    except Exception as exc:  # noqa: BLE001 -- preserve a failed episode; aggregate exits nonzero
        receipt = {
            "status": "EXECUTION_ERROR",
            "case_id": case["case_id"],
            "mode": mode,
            "error": repr(exc),
            "seconds": time.monotonic() - start,
        }
    finally:
        deployment.SimulationEnvironment = base
    deployment.dump(out / "diagnostic_receipt.json", receipt)
    return receipt


def run(protocol_path, output):
    protocol = json.loads(Path(protocol_path).read_text(encoding="utf-8"))
    output = Path(output).resolve()
    if not output.is_relative_to(ROOT / "runs/research"):
        raise ValueError("New research output must stay inside runs/research")
    output.mkdir(parents=True, exist_ok=False)
    files = [
        *sorted((ROOT / "src").rglob("*.py")),
        ROOT / "requirements.lock.txt",
        ROOT / "pyproject.toml",
        Path(protocol_path).resolve(),
    ]
    hashes = {}
    for path in files:
        name = path.relative_to(ROOT)
        data = path.read_bytes()
        target = output / "source_snapshot" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        hashes[str(name)] = hashlib.sha256(data).hexdigest()
    deployment.dump(
        output / "manifest.json",
        {
            "protocol": protocol,
            "source_hashes": hashes,
            "git_head": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
            ).strip(),
            "git_diff": subprocess.check_output(
                ["git", "diff", "--binary"], cwd=ROOT, text=True
            ),
            "python": sys.version,
            "executable": sys.executable,
            "workers": 2,
            "purpose": "Post-hoc selected development diagnostic, no confirmatory inference",
        },
    )
    payloads = [
        (case, mode, str(output / case["case_id"] / mode))
        for case in protocol["cases"]
        for mode in protocol["modes"]
    ]
    with ProcessPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run_case, payloads))
    deployment.dump(output / "results.json", results)
    return int(any(r["status"] != "EXECUTED" for r in results))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    raise SystemExit(run(args.protocol, args.out))
