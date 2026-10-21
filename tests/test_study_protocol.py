import copy
from pathlib import Path

import pytest
import yaml

from dbact_sim.experiments.protocol import load_protocol

ROOT = Path(__file__).parents[1]
CASE = {"case_id": "case_one", "split": "development", "gate": 6,
        "shape": "rectangle", "seed": 2, "variant": "boundary_cvt", "frames": 2, "config": {"dt": 0.05}}


def protocol_file(tmp_path, cases):
    path = tmp_path / "protocol.yaml"
    path.write_text(yaml.safe_dump({"cases": cases}), encoding="utf-8")
    return path


@pytest.mark.parametrize("field,value", [
    ("case_id", "../escape"), ("case_id", "a/b"), ("case_id", "NUL"), ("gate", 7), ("frames", 0),
    ("frames", True), ("seed", -1), ("variant", "unknown"), ("shape", "unknown"),
])
def test_invalid_case_rejected_before_execution(tmp_path, field, value):
    case = {**CASE, field: value}
    with pytest.raises(ValueError):
        load_protocol(protocol_file(tmp_path, [case]), "development", ROOT)


def test_duplicate_ids_and_empty_split_rejected(tmp_path):
    with pytest.raises(ValueError, match="Duplicate"):
        load_protocol(protocol_file(tmp_path, [CASE, CASE]), "development", ROOT)
    with pytest.raises(ValueError, match="Empty split"):
        load_protocol(protocol_file(tmp_path, [CASE]), "test", ROOT)


def test_configs_are_copied_and_reference_is_confined(tmp_path):
    case = copy.deepcopy(CASE)
    _, resolved = load_protocol(protocol_file(tmp_path, [case]), "development", ROOT)
    resolved[0]["config"]["dt"] = 2
    assert case["config"]["dt"] == 0.05
    case.pop("config")
    case["config_ref"] = "../outside.yaml"
    with pytest.raises(ValueError, match="inside the repository"):
        load_protocol(protocol_file(tmp_path, [case]), "development", ROOT)


def test_historical_frozen_test_still_resolves_without_rewriting_it():
    path = ROOT / "docs/paper_simulation_protocol.test_frozen_v1.yaml"
    before = path.read_bytes()
    _, cases = load_protocol(path, "test", ROOT)
    assert len(cases) == 36
    assert path.read_bytes() == before
