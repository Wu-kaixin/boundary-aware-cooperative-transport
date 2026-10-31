from pathlib import Path
import json

import pytest

from dbact_sim.experiments import study
from dbact_sim.experiments.deployment import dump


def test_changed_or_missing_products_cannot_be_reused(tmp_path):
    product = tmp_path / "trace.json"
    product.write_text("{}")
    dump(tmp_path / "case_result.json", {"identity": {"source": "a"},
         "products": {"trace.json": study.digest(product)}, "result": {"execution_status": "COMPLETED"}})
    assert study.cached_result(tmp_path, {"source": "a"})["execution_status"] == "COMPLETED"
    with pytest.raises(RuntimeError, match="identity mismatch"):
        study.cached_result(tmp_path, {"source": "b"})
    product.write_text("changed")
    with pytest.raises(RuntimeError, match="integrity failure"):
        study.cached_result(tmp_path, {"source": "a"})
    product.unlink()
    with pytest.raises(RuntimeError, match="integrity failure"):
        study.cached_result(tmp_path, {"source": "a"})


def test_empty_product_receipt_is_not_evidence(tmp_path):
    dump(tmp_path / "case_result.json", {"identity": {}, "products": {}, "result": {}})
    with pytest.raises(RuntimeError, match="Missing product receipt"):
        study.cached_result(tmp_path, {})


def test_fingerprint_covers_moved_implementation(tmp_path):
    path = tmp_path / "src/dbact/deployment/cvt.py"
    path.parent.mkdir(parents=True)
    path.write_text("first")
    before = study.source_hashes(tmp_path)
    path.write_text("second")
    assert study.source_hashes(tmp_path) != before


@pytest.mark.parametrize("specification", ["requirements.lock.txt", ".python-version"])
def test_environment_spec_change_rejects_cached_run(tmp_path, specification):
    spec = tmp_path / specification
    spec.write_text("original")
    identity = {"source": study.source_hashes(tmp_path)}
    product = tmp_path / "trace.json"
    product.write_text("{}")
    dump(tmp_path / "case_result.json", {"identity": identity,
         "products": {"trace.json": study.digest(product)}, "result": {"execution_status": "COMPLETED"}})
    assert study.cached_result(tmp_path, identity)["execution_status"] == "COMPLETED"
    spec.write_text("updated")
    with pytest.raises(RuntimeError, match="identity mismatch"):
        study.cached_result(tmp_path, {"source": study.source_hashes(tmp_path)})


def test_failed_episode_stays_in_receipt_and_denominator(tmp_path, monkeypatch):
    def fail(*args):
        raise ValueError("invalid initial geometry")
    monkeypatch.setattr(study, "transport_arm", fail)
    case = {"case_id": "failed", "gate": 6, "config": {}, "seed": 2,
            "shape": "rectangle", "variant": "boundary_cvt", "frames": 1}
    result = study.worker((case, str(tmp_path), {"source": "a"}))
    assert result["execution_status"] == "ERROR"
    assert result["task_completed"] is False
    assert study.cached_result(tmp_path, {"source": "a"}) == result


@pytest.mark.parametrize("damage", ["changed", "missing", "inventory"])
def test_resume_preserves_damaged_execution_snapshot(tmp_path, monkeypatch, damage):
    root = tmp_path / "repository"
    source_file = root / "src/experiment.py"
    source_file.parent.mkdir(parents=True)
    source_file.write_text("original implementation")
    protocol = root / "protocol.yaml"
    protocol.write_text("cases: []")
    monkeypatch.setattr(study, "ROOT", root)
    # No simulation is needed to exercise the resume transaction boundary.
    monkeypatch.setattr(study, "load_protocol", lambda *_: ({}, []))
    output = tmp_path / "run"
    study.run_study(protocol, "smoke", output, 1, False)
    snapshot = output / "source_snapshot/src/experiment.py"
    if damage == "changed":
        snapshot.write_text("changed saved evidence")
    elif damage == "missing":
        snapshot.unlink()
    else:
        manifest = json.loads((output / "manifest.json").read_text())
        manifest["source_files"] = {}
        dump(output / "manifest.json", manifest)
    before = {p.relative_to(output): p.read_bytes() for p in output.rglob("*") if p.is_file()}
    with pytest.raises(RuntimeError, match="[Ss]ource (snapshot|inventory)"):
        study.run_study(protocol, "smoke", output, 1, True)
    after = {p.relative_to(output): p.read_bytes() for p in output.rglob("*") if p.is_file()}
    assert after == before
