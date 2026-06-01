# AGENTS.md

## Cursor Cloud specific instructions

### Product overview

Python research monorepo: **DBACT** (decentralized boundary-aware cooperative transport) + vendored **MAS S1** lab stack (`platforms/mas_public`) for OptiTrack/RoboMaster integration. There is no Node/npm or Makefile.

### Python environment

- Use the repo virtualenv at `/workspace/.venv` (Python 3.12+).
- Activate before any command: `source /workspace/.venv/bin/activate`
- Set headless plotting: `export MPLBACKEND=Agg PYTHONUTF8=1`
- On a fresh Ubuntu VM, `python3 -m venv` requires the system package `python3.12-venv` (one-time `apt` install outside the update script).

Install layout (also what the VM update script refreshes):

1. `pip install -r requirements.txt && pip install -e .` from repo root
2. `pip install -r platforms/mas_public/requirements.txt` (adds `pyzmq`, `ruff`, RoboMaster SDK from git)

### Services (what to run when)

| Goal | Processes / command | Notes |
| --- | --- | --- |
| DBACT simulation only | `python -m dbact_sim.run_sim --config configs/sim/<scenario>.yaml --output runs/<name>` | No ZMQ or hardware |
| Root mock MAS bridge | `python scripts/run_mock_mas_pipeline.py` | Single process; no ZMQ |
| MAS controller dry-run | `cd platforms/mas_public && python apps/dbact/run_dtransport_dry_run.py` | No OptiTrack/Robot/ZMQ |
| MAS software closed loop | Three terminals from `platforms/mas_public`: `mock_optitrack.py` → `run_controller.py` → `mock_robot.py` | Start OptiTrack mock first; localhost ZMQ ports in `configs/system.yaml` |
| Real hardware | Motive/NatNet + `run_optitrack.py` + `run_robot_comm.py` + `run_controller.py` | Not available in cloud VM |

**Mock closed-loop caveat:** `mock_optitrack.py` publishes poses outside the default `system.yaml` world box (`x/y` roughly ±1 m). The controller may emit `world_out_of_bounds` stop commands until mock poses are retuned or world bounds widened—this still proves ZMQ wiring works.

### Lint / test / smoke (see `README.md` for full lists)

| Scope | Command (from repo root unless noted) |
| --- | --- |
| Root tests | `python -m pytest` |
| MAS tests | `cd platforms/mas_public && python -m pytest -q apps/pytest_tests` |
| Byte-compile smoke | `python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps` |
| MAS lint | `cd platforms/mas_public && python -m ruff check .` (pre-existing style findings possible) |
| Dependency sanity | `python -m pip check` |

Long-running MAS modules should use **tmux** (`tmux -f /exec-daemon/tmux.portal.conf`), not detached one-shot shells.

### Generated output

- Simulation: `runs/` (gitignored)
- MAS experiments: `platforms/mas_public/data/experiments/`
- MAS dry-runs: `platforms/mas_public/data/dry_runs/`
