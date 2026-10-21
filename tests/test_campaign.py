from collections import Counter, defaultdict
from pathlib import Path

import pytest
import yaml

from dbact_sim.experiments.campaign import build_cases, freeze
from dbact_sim.experiments.deployment import encode
from dbact_sim.experiments.protocol import load_protocol

ROOT = Path(__file__).parents[1]


def test_campaign_covers_every_declared_experiment_without_confounded_pairs():
    plan = yaml.safe_load((ROOT / "experiments/ccta2027/revision.yaml").read_text())
    cases = build_cases(plan, ROOT)
    assert Counter(c["split"] for c in cases) == {
        "static": 27, "development": 27, "ablation": 15, "active_exploration": 6,
        "robustness": 54, "generalization": 36, "numerical": 2, "engine": 2,
    }
    assert len({c["case_id"] for c in cases}) == len(cases)
    groups = defaultdict(list)
    for case in cases:
        groups[(case["split"], case["condition_id"])].append(case)
    for (split, _), group in groups.items():
        if split != "static":
            assert len({encode(c["config"]) for c in group}) == 1
    new_seeds = {c["seed"] for c in cases if c["split"] == "generalization"}
    assert len(new_seeds) == 12
    assert new_seeds.isdisjoint({2, 5, 8, *range(12, 32), *range(910071, 910083)})


def test_freeze_does_not_overwrite_an_existing_snapshot(tmp_path):
    (tmp_path / "execution_source").mkdir()
    with pytest.raises(RuntimeError, match="snapshot already exists"):
        freeze(ROOT / "experiments/ccta2027/revision.yaml", tmp_path / "protocol.yaml")
    assert not (tmp_path / "protocol.yaml").exists()


def test_all_split_preserves_individual_case_splits(tmp_path):
    cases = [{"case_id": f"{split}_case", "split": split, "gate": 6, "shape": "rectangle",
              "seed": 2, "variant": "boundary_cvt", "frames": 1, "config": {}}
             for split in ("development", "generalization")]
    path = tmp_path / "protocol.yaml"
    path.write_text(yaml.safe_dump({"cases": cases}))
    _, resolved = load_protocol(path, "all", ROOT)
    assert {c["split"] for c in resolved} == {"development", "generalization"}
