# Daily Repository Health Check - 2026-06-03

Generated: 2026-06-03, Asia/Tokyo

## Scope

- Repository: `E:\DBACT\boundary-aware-cooperative-transport`
- Environment: Conda `dbact`
- Python: `3.10.20`
- Pytest: `9.0.3`
- Branch refs checked from clean temporary archives:
  - `main` at `d9eb65a103ccd5ade03209187cab98efe6b79d8a`
  - `stage2-mas-virtual-object` at `a2635959454942570aaa072367e73ff473ace6e3`
  - `stage3-mas-dry-run` at `82dc0c3fd18519160410f9aca1f9dd156af1e0c3`
  - `stage4-optitrack-readonly` at `ca3522d3843f78fe55869564aa5089c282a24543`
  - `origin/cursor/cloud-dev-env-setup-b5ed` at `f3f244007ce366b69cd518cdc636a1c7bbaf7665`

## Commands

The final branch-ref checks can be run from `E:\DBACT\boundary-aware-cooperative-transport` with the active Python environment:

```powershell
cd /d E:\DBACT\boundary-aware-cooperative-transport
python -m pytest -q tests
python -m compileall -q src tests scripts platforms\mas_public\src platforms\mas_public\apps
python -c "<parse configs/**/*.yaml and platforms/mas_public/configs/**/*.yaml>"
python -m pytest -q platforms\mas_public\apps\pytest_tests
```

Temporary branch archives were extracted under `%TEMP%`. `PYTHONPATH` was set per archive to the archive `src` and `platforms\mas_public` directories so platform tests imported the branch under test. Platform pytest was run with an explicit writable `--basetemp`, for example `%TEMP%\pytest-dbact`, to avoid local Windows permission issues.

## Branch Results

| Branch ref | Root pytest | Compileall | YAML parse | Platform pytest |
| --- | --- | --- | --- | --- |
| `main` | PASS, 6 passed | PASS | PASS, 26 files | PASS, 106 passed |
| `stage2-mas-virtual-object` | PASS, 6 passed | PASS | PASS, 26 files | PASS, 106 passed |
| `stage3-mas-dry-run` | PASS, 6 passed | PASS | PASS, 26 files | PASS, 106 passed |
| `stage4-optitrack-readonly` | PASS, 6 passed | PASS | PASS, 26 files | PASS, 106 passed |
| `origin/cursor/cloud-dev-env-setup-b5ed` | PASS, 6 passed | PASS | PASS, 26 files | PASS, 106 passed |

## Current Working Tree Validation

The current `main` working tree also contains additional local changes for coverage mode, optional QP-backed CBF filtering, live/paper-style visualization, two new simulation configs, and one new root test file. That final working tree was checked separately after the branch-ref matrix.

| Check | Result |
| --- | --- |
| Root pytest | PASS, 10 passed |
| Compileall | PASS |
| YAML parse | PASS, 28 files |
| Platform pytest | PASS, 106 passed |

Pytest emitted cache write warnings for `.pytest_cache` in the root and platform directories because those cache directories are not writable in this local workspace. These warnings did not fail the test run.

## Notes

- A first platform-test attempt through `conda run -n dbact` hit a Windows console encoding problem while conda printed pytest output. The final checks used the same Conda environment's `python.exe` directly.
- A second platform-test attempt without `--basetemp` hit a local pytest temp directory permission problem. The final checks used explicit writable temporary directories and passed.
- No branch-ref code failures were observed.
- The repository inventory for this date is recorded in `docs/repository_inventory_2026-06-03.md`.
