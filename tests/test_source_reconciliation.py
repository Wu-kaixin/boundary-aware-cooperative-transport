"""Preservation failures must stop publication or deletion of migration sources."""
import hashlib
import importlib.util
import json
from pathlib import Path
import stat

import pytest


@pytest.fixture
def migration(tmp_path, monkeypatch):
    path = Path(__file__).parents[1] / "scripts/maintenance/reconcile_sources.py"
    spec = importlib.util.spec_from_file_location("source_reconciliation", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "ROOT", tmp_path)
    mission = tmp_path / "artifacts/migrations/test"
    (mission / "backup").mkdir(parents=True)
    return module, mission


def item_for(module, content=b"original data"):
    name = f"PC2/{module.ROOT.name}/runs/batch/result.json"
    source = module.ROOT / name
    source.parent.mkdir(parents=True)
    source.write_bytes(content)
    return {"source": "PC2", "original_path": name, "relative_path": "runs/batch/result.json",
            "size": str(len(content)), "sha256": hashlib.sha256(content).hexdigest(),
            "action": "copy", "final_location": "runs/batch/result.json"}


def test_source_change_prevents_copy(migration):
    module, mission = migration
    item = item_for(module)
    (module.ROOT / item["original_path"]).write_bytes(b"later data")
    with pytest.raises(RuntimeError, match="Source changed"):
        module.apply(mission, [item])
    assert not (module.ROOT / item["final_location"]).exists()


def test_conflicting_target_is_never_overwritten(migration):
    module, mission = migration
    item = item_for(module)
    target = module.ROOT / item["final_location"]
    target.parent.mkdir(parents=True)
    target.write_bytes(b"unique existing")
    with pytest.raises(RuntimeError, match="Target conflict"):
        module.apply(mission, [item])
    assert target.read_bytes() == b"unique existing"
    assert (module.ROOT / item["original_path"]).exists()


def test_completed_partial_copy_can_resume_without_overwrite(migration):
    module, mission = migration
    item = item_for(module)
    target = module.ROOT / item["final_location"]
    target.parent.mkdir(parents=True)
    target.with_name(target.name + ".dbact-migration-partial").write_bytes(b"original data")
    module.apply(mission, [item])
    module.apply(mission, [item])
    assert target.read_bytes() == b"original data"
    assert (module.ROOT / item["original_path"]).exists()


@pytest.mark.parametrize("name", ["../outside", "/absolute", "C:/outside", "nested\\outside"])
def test_paths_must_remain_inside_repository(migration, name):
    module, _ = migration
    with pytest.raises(ValueError):
        module.confined(name)


def test_new_source_file_blocks_cleanup(migration):
    module, mission = migration
    item = item_for(module)
    module.apply(mission, [item])
    (mission / "migration_manifest.csv").write_text("test manifest")
    (mission / "cleanup_gate.json").write_text(json.dumps({"status": "PASS",
        "manifest_sha256": module.digest(mission / "migration_manifest.csv")}))
    source = module.ROOT / item["original_path"]
    source.with_name("new-unreviewed.json").write_bytes(b"new data")
    with pytest.raises(RuntimeError, match="membership changed"):
        module.cleanup(mission, [item])
    assert source.exists()
    assert source.with_name("new-unreviewed.json").exists()


def test_cleanup_preserves_explicitly_retained_original(migration):
    module, mission = migration
    item = item_for(module)
    module.apply(mission, [item])
    (mission / "migration_manifest.csv").write_text("test manifest")
    (mission / "cleanup_gate.json").write_text(json.dumps({"status": "PASS",
        "manifest_sha256": module.digest(mission / "migration_manifest.csv"),
        "retain_originals": [item["original_path"]]}))
    # With no PC3 entries, the other source should be treated as absent.
    module.cleanup(mission, [item])
    assert (module.ROOT / item["original_path"]).read_bytes() == b"original data"


def test_verified_readonly_source_can_be_cleaned_without_touching_canonical_copy(migration):
    module, mission = migration
    item = item_for(module)
    module.apply(mission, [item])
    (mission / "migration_manifest.csv").write_text("test manifest")
    (mission / "cleanup_gate.json").write_text(json.dumps({"status": "PASS",
        "manifest_sha256": module.digest(mission / "migration_manifest.csv")}))
    source = module.ROOT / item["original_path"]
    source.chmod(stat.S_IREAD)
    module.cleanup(mission, [item])
    assert not source.exists()
    assert (module.ROOT / item["final_location"]).read_bytes() == b"original data"
