"""Regression tests for stale evidence and incomplete acceptance reports."""
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest


def load(name):
    path = Path(__file__).parents[1] / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def prior():
    return load("run_apriori_centroid_bound")


@pytest.fixture
def gates():
    return load("run_consolidation_gates")


@pytest.fixture
def report():
    return load("build_consolidation_report")


@pytest.mark.parametrize("field", ["ntheta", "nradial", "density_mesh", "restrict_local_mesh"])
def test_observer_and_prior_resolution_invalidate_resume(prior, field):
    args = SimpleNamespace(ntheta=128, nradial=12, density_mesh=160, restrict_local_mesh=24)
    before = prior.prior_rule_fingerprint(args)
    setattr(args, field, getattr(args, field) * 2)
    assert prior.prior_rule_fingerprint(args) != before


def make_sources(root):
    for name in ["src/dbact/static_deployment_diagnostics.py", "configs/sim/theorem/sample.yaml",
                 "scripts/run_apriori_centroid_bound.py", "pyproject.toml", "requirements.txt"]:
        p = root / name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text("original\n")


@pytest.mark.parametrize("name", ["src/dbact/static_deployment_diagnostics.py", "configs/sim/theorem/sample.yaml"])
def test_transitive_source_and_config_invalidate_resume(prior, tmp_path, monkeypatch, name):
    make_sources(tmp_path)
    monkeypatch.setattr(prior, "ROOT", tmp_path)
    before = prior.source_fingerprint()
    (tmp_path / name).write_text("changed\n")
    assert prior.source_fingerprint() != before


def test_resume_requires_config_and_intact_outputs(prior, tmp_path):
    cfg = {"dt": 0.05}
    for name in prior._CASE_OUTPUTS:
        (tmp_path / name).write_text("{}")
    meta = {"schema": 2, "source_fp": "source", "prior_fp": "prior",
            "config_sha256": prior.config_fingerprint(cfg), "outputs": prior.case_output_hashes(tmp_path)}
    (tmp_path / "COMPLETE.json").write_text(json.dumps(meta))
    assert prior._reuse_ok(tmp_path, "source", "prior", cfg)
    assert not prior._reuse_ok(tmp_path, "source", "prior", {"dt": 0.1})
    (tmp_path / "trajectory.npz").write_bytes(b"corrupt")
    assert not prior._reuse_ok(tmp_path, "source", "prior", cfg)


def test_old_completion_record_is_not_trusted(prior, tmp_path):
    (tmp_path / "COMPLETE.json").write_text(json.dumps({"source_fp": "s", "prior_fp": "p"}))
    (tmp_path / "summary.json").write_text("{}")
    assert not prior._reuse_ok(tmp_path, "s", "p", {})


def test_gate_cache_requires_matching_receipt_and_hashes(gates, tmp_path, monkeypatch):
    monkeypatch.setattr(gates, "ROOT", tmp_path)
    artifact = tmp_path / "diagnosis.json"
    artifact.write_text("{}")
    receipt = tmp_path / "receipt.json"
    signature = {"source": "a", "config": "b", "command": ["run", "--frames", "600"]}
    assert not gates.artifacts_current([artifact.name], receipt, signature)
    receipt.write_text(json.dumps({"signature": signature, "outputs": gates.output_hashes([artifact.name])}))
    assert gates.artifacts_current([artifact.name], receipt, signature)
    assert not gates.artifacts_current([artifact.name], receipt, {**signature, "config": "changed"})
    assert not gates.artifacts_current([artifact.name], receipt, {**signature, "command": ["run", "--frames", "80"]})
    artifact.write_text('{"changed": true}')
    assert not gates.artifacts_current([artifact.name], receipt, signature)


def test_render_receipt_tracks_replay_content(gates, tmp_path, monkeypatch):
    make_sources(tmp_path)
    monkeypatch.setattr(gates, "ROOT", tmp_path)
    folder = tmp_path / "run"
    folder.mkdir()
    (folder / "summary.json").write_text("{}")
    (folder / "replay.npz").write_bytes(b"old")
    args = ["scripts/render_closed_loop.py", "run", "--fps", "25"]
    before = gates.input_signature(args, [2])
    (folder / "replay.npz").write_bytes(b"new")
    assert before != gates.input_signature(args, [2])


def test_nonzero_exit_needs_fresh_complete_scored_failure(gates, tmp_path, monkeypatch):
    monkeypatch.setattr(gates, "ROOT", tmp_path)
    summary = tmp_path / "summary.json"
    summary.write_text(json.dumps({"steps": 10, "timing": {"frames": 10, "terminated_by": "settled"}, "cargoes": {"cargo": {"g500": {"success": False}}}}))
    log = tmp_path / "run.log"
    log.write_text("G500: FAIL")
    args = ["scripts/run_closed_loop.py"]
    assert gates.scientific_failure(args, [summary.name], 1, 0, log)
    assert not gates.scientific_failure(args, [summary.name], 2, 0, log)
    assert not gates.scientific_failure(args, [summary.name], 1, summary.stat().st_mtime_ns + 1, log)
    log.write_text("Traceback (most recent call last):\nRuntimeError")
    assert not gates.scientific_failure(args, [summary.name], 1, 0, log)


def trajectory(frames=600):
    return {"J": np.zeros(frames), "E": np.zeros(frames), "H_star": np.zeros(frames + 1),
            "positions": np.zeros((frames + 1, 16, 2)), "vertices": np.zeros((4, 2))}


def test_partial_shape_set_cannot_pass(report):
    check = report.compare_trajectories(trajectory(), trajectory())
    assert report.serial_verdict({}) == "INCOMPLETE"
    assert report.serial_verdict({"l_shape": check}) == "INCOMPLETE"
    assert report.serial_verdict({"l_shape": check, "rectangle": check}) == "PASS"


@pytest.mark.parametrize("kind", ["short", "nan", "different", "geometry"])
def test_bad_trajectory_cannot_pass(report, kind):
    a, b = trajectory(), trajectory()
    if kind == "short":
        a = b = trajectory(80)
    elif kind == "nan":
        a["J"][0] = np.nan
    elif kind == "different":
        a["positions"][0, 0, 0] = 0.1
    else:
        a["vertices"][0, 0] = 0.1
    check = report.compare_trajectories(a, b)
    assert not check["pass"]
    assert report.serial_verdict({"l_shape": check, "rectangle": check}) == "FAIL"


def test_d10_caption_and_table_are_derived_from_json(report):
    diagnosis = {"aggregate": {"segments_total": {"A": 12, "F": 3}}}
    ab = {"arms": {"0.0": [{"T_contact_ready": 4, "barrier_scalings": 7},
                            {"T_contact_ready": None, "barrier_scalings": 2}]}}
    text = "\n".join(report.d10_block(diagnosis, ab))
    assert "共 15 帧" in text and "为 3 帧" in text
    assert "| 0.0 | 2 | 4; n=1 | 9 |" in text
