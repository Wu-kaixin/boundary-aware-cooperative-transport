# AGENTS.md

## Cursor Cloud specific instructions

### Product overview

Python research monorepo: **DBACT** (decentralized boundary-aware cooperative transport) + vendored **MAS S1** lab stack (`platforms/mas_public`). No Node/npm or Makefile.

### Python environment (canonical: Conda `dbact`)

The project README and local workflow use:

```bash
conda activate dbact
export MPLBACKEND=Agg PYTHONUTF8=1
```

- Expected Python: **3.10.x** (README documents 3.10.20).
- Install (repo root): `pip install -r requirements.txt && pip install -e .`
- MAS extras: `pip install -r platforms/mas_public/requirements.txt` from repo root, or `cd platforms/mas_public` and install there.

**Cloud VM note:** Miniconda lives at `~/miniconda3` with env `dbact`. New shells need `source ~/miniconda3/etc/profile.d/conda.sh` before `conda activate dbact` (conda init is in `~/.bashrc`).

**MAS on Python 3.10:** `rm_libmedia_codec==0.0.1` (test PyPI) may not install on 3.10; if `pip install -r platforms/mas_public/requirements.txt` fails, install `pyzmq`, `ruff`, and the `robomaster` git dependency manually—tests and dry-runs still pass without `rm_libmedia_codec`.

A legacy `/workspace/.venv` may exist from an earlier cloud setup; prefer **`conda activate dbact`** to match the README.

### Services (what to run when)

| Goal | Command | Notes |
| --- | --- | --- |
| DBACT simulation | `python -m dbact_sim.run_sim --config configs/sim/<scenario>.yaml --output runs/<name>` | No ZMQ/hardware |
| Root mock MAS bridge | `python scripts/run_mock_mas_pipeline.py` | Single process |
| MAS dtransport dry-run | `cd platforms/mas_public && python apps/dbact/run_dtransport_dry_run.py` | No OptiTrack/Robot/ZMQ |
| MAS mock closed loop | `mock_optitrack.py` → `run_controller.py` → `mock_robot.py` (from `platforms/mas_public`) | ZMQ on localhost; start mock OptiTrack first |
| Real hardware | Motive + `run_optitrack.py` + `run_robot_comm.py` + `run_controller.py` | Not in cloud VM |

Mock OptiTrack poses can trigger `world_out_of_bounds` against default `system.yaml` bounds; that still validates ZMQ wiring.

### Lint / test (see `README.md`)

| Scope | Command |
| --- | --- |
| Root tests | `conda activate dbact` then `python -m pytest` (expect **6 passed**) |
| MAS tests | `cd platforms/mas_public && python -m pytest -q apps/pytest_tests` |
| Byte-compile | `python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps` |
| MAS lint | `cd platforms/mas_public && python -m ruff check .` |

Long-running MAS modules: use **tmux** (`tmux -f /exec-daemon/tmux.portal.conf`).

### Generated output

- `runs/` — simulation
- `platforms/mas_public/data/experiments/` — MAS runs
- `platforms/mas_public/data/dry_runs/` — dry-runs
