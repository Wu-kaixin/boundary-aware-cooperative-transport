"""Keep migration preservation checks valid after the rebuilt files are committed."""
import hashlib
import importlib.util
from pathlib import Path
from zipfile import ZipFile


def load_audit():
    path = Path(__file__).parents[1] / "scripts/maintenance/reconcile_transfer.py"
    spec = importlib.util.spec_from_file_location("migration_audit", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_original_backup_membership_survives_later_commits(tmp_path):
    audit = load_audit()
    original = b"original bytes"
    rows = [{"path": name, "sha256": hashlib.sha256(original).hexdigest()}
            for name in ("previously_tracked.py", "later_tracked.py")]
    (tmp_path / "previously_tracked.py").write_bytes(b"rebuilt implementation")
    (tmp_path / "later_tracked.py").write_bytes(original)
    backup = tmp_path / "before.zip"
    with ZipFile(backup, "w") as archive:
        archive.writestr("previously_tracked.py", original)
    # A later commit can track both files; only the historical archive defines
    # which originals are backed up versus still required at their live paths.
    with ZipFile(backup) as archive:
        failures, local, tracked = audit.verify_local_originals(rows, archive, tmp_path)
    assert (failures, local, tracked) == ([], 1, 1)

    (tmp_path / "later_tracked.py").write_bytes(b"changed after migration")
    with ZipFile(backup) as archive:
        failures, local, tracked = audit.verify_local_originals(rows, archive, tmp_path)
    assert failures == [{"local_original_changed": "later_tracked.py"}]
    assert (local, tracked) == (1, 1)
