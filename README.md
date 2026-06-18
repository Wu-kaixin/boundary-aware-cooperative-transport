# DBACT: Boundary-Aware Cooperative Transport

DBACT is a research software stack for decentralized cooperative transport with multiple mobile robots. The repository combines a hardware-independent controller, simulation scenarios for unknown object caging, a MAS-compatible controller adapter, OptiTrack read-only tooling, and conservative RoboMaster S1 command smoke tests.

The current `main` branch is the maintained branch. Historical stage branches were merged into `main`, checked, and removed after verification.

## Project Scope

The project focuses on cooperative manipulation of an object whose complete geometry is not assumed by the controller. Robots receive local boundary observations, build local target distributions, allocate cage targets through local CVT-style updates, apply safety filtering, and generate robot-level velocity commands.

High-level flow:

```text
local boundary observation
  -> boundary-aware density
  -> local target allocation
  -> safety filtering
  -> caging / transport command
  -> MAS ControlCommand or S1 command backend
  -> simulation, dry-run, or hardware validation path
```

The software is structured to separate algorithm validation from hardware execution. Simulation and dry-run checks should pass before any physical robot movement.

## Current Status

| Area | Status | Notes |
| --- | --- | --- |
| Core DBACT simulation | Working | Unknown polygon caging, local sensing, density, CVT target allocation, safety filtering, and metrics are implemented. |
| MAS controller adapter | Working | The `dtransport` controller can produce MAS-compatible `ControlCommand` messages from a `WorldState`. |
| MAS dry-runs | Working | Controller-level and ControllerModule-level dry-runs run without OptiTrack, RoboMaster hardware, or network runtime. |
| OptiTrack read-only bridge | Partially validated | Mock logging and NatNet client import are available. Real Motive streaming requires configured rigid bodies that publish usable robot poses. |
| Seven-S1 command smoke path | Working in mock mode | The command layer supports mock execution and optional low-speed real S1 command streaming. |
| Full physical experiment | Not complete | Real object observation, closed-loop OptiTrack-to-controller integration, and controlled low-speed transport trials still require hardware validation. |

Important limitation: the MAS path currently includes robot-pose ingestion, but object/cargo observation is still represented by a virtual-object observer unless a real perception source is added.

## Latest Health Check

Latest full local health check: **2026-06-18**, Conda environment `dbact`, Python `3.10.20`.

Before branch cleanup, all existing branch refs were checked from clean temporary archives after `git fetch --all --prune`.

| Branch ref | Root pytest | Compileall | YAML parse | Platform pytest |
| --- | --- | --- | --- | --- |
| `main` | PASS, 16 passed | PASS | PASS, 28 files | PASS |
| `stage2-mas-virtual-object` | PASS, 6 passed | PASS | PASS, 26 files | PASS |
| `stage3-mas-dry-run` | PASS, 6 passed | PASS | PASS, 26 files | PASS |
| `stage4-optitrack-readonly` | PASS, 6 passed | PASS | PASS, 26 files | PASS |

The three stage branches were ancestors of `main`, so they did not contain unique work that needed to remain as active branches. They have been removed locally and from the remote repository.

## Repository Layout

```text
.
|-- src/dbact/                 # Core controller, sensing, density, CVT, safety, metrics, command policies
|-- src/dbact_sim/             # Simulation environment, scenario loading, plotting, CLI
|-- src/mas_adapter/           # Root-level MAS-compatible controller adapter
|-- configs/sim/               # Simulation scenarios
|-- configs/mas/               # Root-level MAS adapter configs
|-- scripts/                   # Batch runs, mock MAS pipeline, S1 command smoke test
|-- tests/                     # Root unit and smoke tests
|-- docs/                      # Architecture, roadmap, stage notes, health reports, inventory
`-- platforms/mas_public/      # Vendored MAS, OptiTrack, RoboMaster, app, config, and test code
```

Generated outputs such as `runs/`, `outputs/`, `.pytest_cache/`, `__pycache__/`, platform data logs, CSV files, images, animations, and virtual environments should stay out of Git.

## Key Components

| Path | Purpose |
| --- | --- |
| `src/dbact/controller.py` | End-to-end DBACT caging and coverage controller pipeline. |
| `src/dbact/local_sensing.py` | Local boundary observation generation for simulated objects. |
| `src/dbact/boundary_density.py` | Boundary-aware density over cage or coverage target regions. |
| `src/dbact/local_cvt.py` | Local weighted CVT centroid approximation. |
| `src/dbact/local_cbf_qp.py` | Optional QP-backed safety filter with projection fallback. |
| `src/dbact/agent_control.py` | Command policies, state providers, safety limits, mock backend, and optional S1 backend. |
| `src/dbact_sim/run_sim.py` | Simulation CLI entry point. |
| `src/mas_adapter/decentralized_transport_controller.py` | MAS-compatible DBACT controller wrapper. |
| `src/mas_adapter/object_observer.py` | Virtual object observer placeholder for MAS integration. |
| `platforms/mas_public/apps/dbact/log_optitrack_world_state.py` | Safe OptiTrack read-only logging path. |
| `scripts/run_seven_s1_cvt_test.py` | Mock or real seven-S1 low-speed command smoke test. |

## Installation

The known working local environment is the Conda environment named `dbact`.

```powershell
conda activate dbact
python --version
```

If the environment needs to be created:

```powershell
conda create -n dbact python=3.10
conda activate dbact
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .[dev]
```

For the vendored MAS platform:

```powershell
conda activate dbact
cd platforms\mas_public
python -m pip install -r requirements.txt
```

## Verification

From the repository root:

```powershell
conda activate dbact
python -m pytest -q tests
python -m compileall -q src tests scripts platforms\mas_public\src platforms\mas_public\apps
```

YAML configuration parse check:

```powershell
python -c "from pathlib import Path; import yaml; paths=list(Path('configs').rglob('*.yaml')); p=Path('platforms/mas_public/configs'); paths += list(p.rglob('*.yaml')) if p.exists() else []; [yaml.safe_load(x.read_text(encoding='utf-8')) for x in paths]; print('YAML ok', len(paths), 'files')"
```

MAS platform tests:

```powershell
python -m pytest -q --rootdir platforms\mas_public platforms\mas_public\apps\pytest_tests
```

If Windows temporary cache permissions interfere with pytest, provide an explicit base temp directory:

```powershell
python -m pytest -q --basetemp %TEMP%\dbact_pytest_tmp tests
```

## Run Simulations

Single scenario:

```powershell
conda activate dbact
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape
```

Live paper-style viewer:

```powershell
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape --live
```

Batch run:

```powershell
python scripts/run_all_scenarios.py
```

Representative scenarios:

```powershell
python -m dbact_sim.run_sim --config configs/sim/baseline_unknown_polygon_caging_tight.yaml --steps 900 --output runs/baseline_unknown_polygon_caging_tight
python -m dbact_sim.run_sim --config configs/sim/one_rectangle_polygon_caging_tight.yaml --steps 900 --output runs/one_rectangle_polygon_caging_tight
python -m dbact_sim.run_sim --config configs/sim/one_nonconvex_polygon_caging_tight.yaml --steps 1000 --output runs/one_nonconvex_polygon_caging_tight
python -m dbact_sim.run_sim --config configs/sim/decentralized_cvt_coverage.yaml --steps 220 --output runs/decentralized_cvt_coverage
```

## Run Mock MAS Pipeline

From the repository root:

```powershell
conda activate dbact
python scripts/run_mock_mas_pipeline.py --steps 80 --dt 0.05 --print-every 20 --output runs/mock_mas_pipeline
```

Expected outputs:

```text
runs/mock_mas_pipeline/
|-- states.csv
|-- commands.csv
`-- mock_trajectory.png
```

## Run MAS Platform Dry-Runs

From `platforms/mas_public`:

```powershell
python apps\dbact\run_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data\dry_runs\dtransport_auto_init --clamp-to-world-bounds
```

ControllerModule-level dry-run:

```powershell
python apps\dbact\run_controller_module_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data\dry_runs\controller_module_dtransport
```

These dry-runs should produce nonzero `dbact_cage` commands while staying inside configured world and velocity limits.

## Run Seven-S1 Command Smoke Test

Mock mode:

```powershell
conda activate dbact
python scripts/run_seven_s1_cvt_test.py --duration 3
```

Real hardware connect-only check:

```powershell
python scripts/run_seven_s1_cvt_test.py --real --connect-only
```

Low-speed real CVT command stream:

```powershell
python scripts/run_seven_s1_cvt_test.py --real --duration 12 --max-speed 0.05 --max-vx 0.05 --max-vy 0.05
```

Low-speed virtual-box DBACT command stream:

```powershell
python scripts/run_seven_s1_cvt_test.py --real --controller dbact-box --duration 12 --max-speed 0.05
```

Use `--connect-only` before any real movement command. Keep physical emergency stop access available during hardware tests.

## OptiTrack Read-Only Path

Use this path before enabling robot command output.

Mock NatNet check:

```powershell
conda activate dbact
cd platforms\mas_public
python -m py_compile apps\dbact\log_optitrack_world_state.py
python apps\dbact\log_optitrack_world_state.py --mock --frames 50 --hz 100 --print-every 10 --output data\optitrack_readonly\mock_world_states.csv
```

Real Motive / NatNet read-only check:

```powershell
conda activate dbact
cd platforms\mas_public
python apps\dbact\log_optitrack_world_state.py --frames 500 --hz 100 --print-every 50 --output data\optitrack_readonly\real_world_states.csv
```

Read-only data flow:

```text
NatNetAdapter or MockNatNetAdapter
  -> NatNetRigidBody
  -> RigidBodyMapper
  -> TrackingValidator
  -> StateEstimator
  -> RobotState
  -> WorldState
  -> CSV log
```

This path does not publish `ControlCommand` messages.

## Runtime Integration Path

The intended full runtime path is:

```text
Motive / NatNet
  -> OptiTrackModule
  -> WORLD_STATE over ZeroMQ
  -> ControllerModule
  -> DecentralizedTransportController
  -> CONTROL_COMMAND over ZeroMQ
  -> RoboMaster S1 communication
```

Start runtime modules manually in separate terminals only after read-only logs and dry-run command outputs are verified:

```powershell
cd platforms\mas_public
python apps\run_optitrack.py
```

```powershell
cd platforms\mas_public
python apps\run_controller.py
```

```powershell
cd platforms\mas_public
python apps\run_robot_comm.py
```

Do not run the full process launcher until module configuration, robot mapping, OptiTrack frames, world bounds, and command limits have all been checked.

## Safety Rules

- Run read-only OptiTrack logging before enabling controller output.
- Verify robot ID to rigid-body mapping by moving one robot at a time.
- Verify coordinate axes and yaw direction before sending movement commands.
- Keep first physical tests at very low speed.
- Keep transport gain disabled during initial caging-only validation.
- Use a physical emergency stop; keyboard interruption is not a physical safety system.
- Inspect command logs after every hardware run.

## Development Roadmap

1. Keep `main` as the only active branch after merged stage branches are removed.
2. Validate Motive rigid bodies until `raw_bodies > 0` and robot poses are stable.
3. Confirm world-frame axes, yaw, rigid-body names, IDs, and bounds.
4. Run seven-S1 `--connect-only`, then very-low-speed command streaming only after safety checks.
5. Add object/cargo OptiTrack or perception support.
6. Run `OptiTrack -> ControllerModule -> ControlCommand` with robot output disabled.
7. Run a low-speed caging-only physical experiment.
8. Add real-experiment visualization and compare logs against simulation outputs.
9. Replace virtual object assumptions with a real boundary-observation pipeline.

## Documentation

| File | Content |
| --- | --- |
| `docs/ARCHITECTURE.md` | Package layout and data flow. |
| `docs/ALGORITHM.md` | Core algorithm notes. |
| `docs/MAS_INTEGRATION.md` | MAS integration guide. |
| `docs/ROADMAP.md` | Staged roadmap. |
| `docs/stage1_results.md` | Simulation result notes. |
| `docs/stage2_mas_virtual_object.md` | MAS virtual-object integration notes. |
| `docs/stage3_mas_dry_run.md` | MAS dry-run notes. |
| `docs/stage4_optitrack_readonly.md` | OptiTrack read-only bridge notes. |
| `docs/daily_health_2026-06-18.md` | Latest branch and working-tree health report. |
| `docs/repository_inventory_2026-06-08.md` | Current detailed repository inventory. |
| `platforms/mas_public/docs/*.md` | MAS platform setup, config, usage, and debug notes. |

## License

MIT License.
