"""Fixed-budget v2 progress study with aligned, evaluation-only truth logging."""

from __future__ import annotations

import argparse
import copy
import gzip
import hashlib
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from dbact.control.task_progress_readout import ProgressReadout
from dbact_sim.experiments import deployment

ROOT = Path(__file__).resolve().parents[3]


def json_array(value):
    if isinstance(value, (np.ndarray, np.generic)):
        return value.tolist()
    raise TypeError(type(value).__name__)


def install_aligned_evaluator(env, arm, stream):
    """Truth is read only here AFTER the local readout has returned."""
    controller = env.controller
    observer = ProgressReadout(controller, env.agents, arm)
    original = controller._update_progress

    def update(agents, neighbors):
        original(agents, neighbors)
        row = {
            "control_frame": controller._frame,
            "time": env.t,
            "phase_before_update": controller.phase.name,
            "team_progress": dict(controller.team_progress),
            "truth": {
                c.object_id: {
                    "progress": env.tasks[c.object_id].progress(c.position),
                    "position": c.position.copy(),
                    "angle": float(c.angle),
                }
                for c in env.cargoes
            },
            "agents": [],
        }
        for (aid, cid), readout in observer.readouts.items():
            trace = observer.captures[aid]
            row["agents"].append(
                {
                    "agent": aid,
                    "object": cid,
                    **readout,
                    "registration": trace.current(cid),
                    "fusion": trace.current_fusion(cid),
                    "local_feedback": controller._progress[aid][cid],
                    "transport_velocity": controller._transport_feedback[aid].get(cid),
                }
            )
        stream.write(json.dumps(row, default=json_array, allow_nan=False) + "\n")

    controller._update_progress = update


def run_case(payload):
    from dbact.cpu_budget import limit_blas_threads

    limit_blas_threads(1)
    case, arm, out_name = payload
    out = Path(out_name)
    out.mkdir(parents=True, exist_ok=False)
    original_env = deployment.SimulationEnvironment
    started = time.monotonic()
    with gzip.open(out / "registration.jsonl.gz", "wt", encoding="utf-8") as stream:

        def factory(*args, **kwargs):
            env = original_env(*args, **kwargs)
            install_aligned_evaluator(env, arm, stream)
            return env

        try:
            deployment.SimulationEnvironment = factory
            result = deployment.transport_arm(
                copy.deepcopy(case["config"]),
                case["seed"],
                "boundary_cvt",
                case["frames"],
                out,
            )
            assert (
                result["initial_state_sha256"] == case["expected_initial_state_sha256"]
            )
            result["changed_mechanism"] = {
                "baseline": "Passive matched-registration capture; original feedback",
                "certificate_only": "Passive capture and certificate calculation; original feedback",
                "anchored_readout": "Local map-anchored scalar and its hold cache only; original gain retained",
            }[arm]
            receipt = {
                "status": "EXECUTED",
                "case_id": case["case_id"],
                "arm": arm,
                "result": result,
                "seconds": time.monotonic() - started,
                "truth_permission": "evaluation_only_at_control_time_before_physics",
            }
        except Exception as exc:  # noqa: BLE001 -- persist failure, return nonzero aggregate
            receipt = {
                "status": "EXECUTION_ERROR",
                "case_id": case["case_id"],
                "arm": arm,
                "error": repr(exc),
                "seconds": time.monotonic() - started,
            }
        finally:
            deployment.SimulationEnvironment = original_env
    deployment.dump(out / "receipt.json", receipt)
    return receipt


def run(protocol_path, output, phase):
    protocol_path = Path(protocol_path).resolve()
    plan = json.loads(protocol_path.read_text())
    output = Path(output).resolve()
    if not output.is_relative_to(ROOT / "runs/research"):
        raise ValueError("Output must be inside repository runs/research")
    output.mkdir(parents=True, exist_ok=False)
    arms = ["baseline"] if phase == "diagnostic" else plan["arms"]
    hashes = {}
    for path in [
        *sorted((ROOT / "src").rglob("*.py")),
        ROOT / "requirements.lock.txt",
        ROOT / "pyproject.toml",
        protocol_path,
    ]:
        name = path.relative_to(ROOT)
        data = path.read_bytes()
        target = output / "source_snapshot" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        hashes[name.as_posix()] = hashlib.sha256(data).hexdigest()
    deployment.dump(
        output / "manifest.json",
        {
            "phase": phase,
            "plan": plan,
            "source_hashes": hashes,
            "python": sys.version,
            "numpy": np.__version__,
            "git_head": subprocess.check_output(
                ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
            ).strip(),
            "git_diff": subprocess.check_output(
                ["git", "diff", "--no-textconv", "--no-ext-diff", "--binary"],
                cwd=ROOT,
                text=True,
            ),
            "workers": 2,
            "execution_identity": "uncommitted source snapshot authoritative",
        },
    )
    jobs = [
        (c, arm, str(output / c["case_id"] / arm))
        for c in plan["cases"]
        for arm in arms
    ]
    with ProcessPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run_case, jobs))
    deployment.dump(output / "results.json", results)
    return int(any(r["status"] != "EXECUTED" for r in results))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--phase", choices=["diagnostic", "comparison"], required=True)
    args = parser.parse_args()
    os.environ["OPENBLAS_NUM_THREADS"] = "1"
    os.environ["OMP_NUM_THREADS"] = "1"
    raise SystemExit(run(args.protocol, args.out, args.phase))
