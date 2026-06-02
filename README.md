# DBACT: Decentralized Boundary-Aware Cooperative Transportation

**Full title:** Decentralized Boundary-Aware Cooperative Transportation of Arbitrarily Shaped Objects Without Prior Object Knowledge.

**中文理解:** 面向未知任意形状物体的去中心化边界感知协同搬运算法。

This repository is a research and experiment stack for multi-robot cooperative transport. It starts from a hardware-independent DBACT controller, validates unknown arbitrary polygon caging in simulation, then connects the controller to a MAS / RoboMaster S1 platform with OptiTrack/Motive tracking.

Current local workspace: `E:\DBACT\boundary-aware-cooperative-transport`

Current answer: **yes, the project is on the correct path.** The completed work follows the right order for a real robot experiment: simulation first, MAS controller integration second, no-hardware dry-runs third, OptiTrack read-only validation fourth, and only then low-speed RoboMaster experiments.

```text
local boundary sensing
  -> boundary-aware density over cage targets
  -> local CVT allocation
  -> local CBF-style safety filtering
  -> caging / transport commands
  -> MAS ControlCommand
  -> OptiTrack / RoboMaster experiment path
```

## Current Status

The project has four staged layers.

| Stage | Status | Meaning |
| --- | --- | --- |
| Stage 1: DBACT simulation | Done | Arbitrary polygon cargo caging works without using cargo geometry as a controller prior. |
| Stage 2: MAS virtual-object integration | Done | `dtransport` controller is registered in the MAS platform and can produce MAS `ControlCommand`. |
| Stage 3: MAS dry-run | Done | Controller-level and ControllerModule-level dry-runs work without OptiTrack, RoboMaster, or network runtime. |
| Stage 4: OptiTrack read-only bridge | In progress | Mock logger works; NatNet/Motive connection works; current blocker is creating/enabling Motive rigid bodies so `raw_bodies > 0`. |
| Final experiment | Not yet complete | Real cargo observation, real OptiTrack-to-DBACT object bridge, low-speed RoboMaster trials, and visualization remain. |

Important current limitation: the MAS stack already reads robot poses from OptiTrack, but the DBACT `ObjectObserver` is still a virtual-object placeholder. For a physical unknown-object experiment, cargo/object boundary observations still need to be connected from OptiTrack markers, vision, tactile/contact sensing, or another object-perception source.

## What Has Already Been Built

### Stage 1: Unknown Polygon Caging

Completed:

- Arbitrary polygon cargo model.
- Local boundary sensing.
- Boundary-aware density field.
- Local CVT allocation.
- Agent-agent CBF-style safety filtering.
- Caging-only unknown-cargo baseline.
- `recruited_agents`, coverage, path length, and safety metrics.
- Tight baseline configs that improve recruitment while keeping inter-agent distance safe.

The controller intentionally does **not** use these as direct control priors in the caging baseline:

```text
cargo.center
cargo.radius
cargo.vertices
cargo.closest_boundary()
```

Simulator-side sensing and offline metrics may use cargo geometry. The controller receives local `BoundaryObservation` objects.

Key Stage 1 results:

| Scenario | Final coverage | Recruited agents | Min inter-agent distance |
| --- | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging` | 0.7625 | 6 / 12 | 0.3446 |
| `one_rectangle_polygon_caging` | 0.7000 | 6 / 12 | 0.3446 |
| `one_nonconvex_polygon_caging` | 0.90625 | 9 / 14 | 0.3393 |
| `baseline_unknown_polygon_caging_tight` | 0.95625 | 11 / 12 | 0.3450 |
| `one_rectangle_polygon_caging_tight` | 0.99375 | 9 / 12 | 0.3450 |
| `one_nonconvex_polygon_caging_tight` | 0.9750 | 13 / 14 | 0.3370 |

### Stage 2: MAS Integration

Completed:

- `src/mas_adapter/decentralized_transport_controller.py`
- `src/mas_adapter/object_observer.py`
- `configs/mas/controller.yaml`
- `configs/mas/dtransport.yaml`
- `configs/mas/dtransport_mock.yaml`
- vendored MAS platform under `platforms/mas_public`
- MAS `config_loader` supports `controller.type: dtransport`
- MAS `ControllerModule` can build `DecentralizedTransportController`
- MAS `dtransport.yaml` supports `virtual_object`
- `compute(WorldState)` returns MAS `ControlCommand`
- velocity and yaw-rate limits are clamped by both system and dtransport limits

Validated chain:

```text
mock WorldState
  -> DecentralizedTransportController
  -> ObjectObserver virtual object
  -> DBACTController
  -> planar velocity commands
  -> integrated mock WorldState
  -> states.csv / commands.csv / mock_trajectory.png
```

MAS message chain:

```text
MAS WorldState
  -> DecentralizedTransportController.compute()
  -> MAS ControlCommand
  -> RobotCommand(robot_1 / robot_2 / robot_3)
```

### Stage 3: MAS Dry-Run

Completed:

- `platforms/mas_public/apps/dbact/run_dtransport_dry_run.py`
- `platforms/mas_public/apps/dbact/run_controller_module_dtransport_dry_run.py`
- world-bound checks
- dry-run trajectory output
- dry-run command/state/event CSV outputs
- automatic dry-run robot initialization
- clamp-to-world-bounds mode
- ControllerModule-level dry-run through MAS helper methods

Controller-level dry-run:

```text
synthetic WorldState
  -> DecentralizedTransportController
  -> ControlCommand
  -> states.csv / commands.csv / events.csv / trajectory.png
```

ControllerModule-level dry-run:

```text
synthetic WorldState
  -> ControllerModule helper methods
  -> _world_state_for_control_frame()
  -> DecentralizedTransportController.compute()
  -> _apply_gimbal_control()
  -> _normalize_command_for_mode()
  -> ControlCommand
  -> states.csv / commands.csv / events.csv
```

This stage is important because it verifies the MAS controller side before introducing OptiTrack coordinates, ZMQ timing, RoboMaster networking, and physical safety risk.

### Stage 4: OptiTrack Read-Only Bridge

Current Stage 4 goal:

```text
Motive / NatNet or MockNatNet
  -> NatNetRigidBody
  -> RobotState
  -> WorldState
  -> CSV log
```

The repository now includes a safe read-only logger:

```text
platforms/mas_public/apps/dbact/log_optitrack_world_state.py
```

It does not start `ControllerModule`, does not start RoboMaster communication, and does not publish `ControlCommand`. Use it before any real robot experiment to confirm that OptiTrack rigid bodies, robot IDs, coordinate axes, yaw, velocities, and world bounds are sane.

Current Stage 4 hardware-side status:

- NatNet Python SDK files are present under `platforms/mas_public/third_party/natnet_client`.
- `NatNetClient` can be imported.
- Real `NatNetAdapter` can be selected instead of `MockNatNetAdapter`.
- Motive connection through `127.0.0.1` Unicast has been validated.
- Python receives continuous MoCap frames.
- The read-only logger writes CSV headers and supports `--print-raw-bodies`.
- Current blocker: Motive has not yet created or streamed robot rigid bodies, so the logger can receive frames while still reporting `raw_bodies=0` and `robots=0`.

## Repository Structure

```text
.
|-- src/dbact/                 # Core DBACT algorithm modules
|-- src/dbact_sim/             # Simulation environment, scenarios, metrics, plots
|-- src/mas_adapter/           # Hardware-independent MAS controller adapter
|-- configs/sim/               # Simulation YAML scenarios
|-- configs/mas/               # DBACT MAS adapter configs
|-- scripts/                   # Batch simulation and mock MAS utilities
|-- tests/                     # Root DBACT tests
|-- docs/                      # Architecture, algorithm, roadmap, stage notes
`-- platforms/mas_public/      # Vendored MAS / OptiTrack / RoboMaster platform
```

## Essential Files

These files are necessary for the actual project, GitHub release, or local reproducibility.

### Root Project

| File | Why it matters |
| --- | --- |
| `README.md` | Main project guide and current status. |
| `README_DBACT.md` | Longer historical DBACT notes and reference material. |
| `pyproject.toml` | Editable package metadata and pytest config. |
| `requirements.txt` | Root DBACT dependencies. |
| `LICENSE` | MIT license. |
| `.gitignore` | Keeps generated runs, CSVs, logs, images, caches, and venv files out of Git. |

### Core DBACT

| File | Role |
| --- | --- |
| `src/dbact/types.py` | `AgentState`, `BoundaryObservation`, and `ControlCommand`. |
| `src/dbact/cargo.py` | Polygon cargo representation and simulator geometry. |
| `src/dbact/local_sensing.py` | Local boundary observation generation. |
| `src/dbact/boundary_map.py` | Per-agent local memory and neighbor observation exchange. |
| `src/dbact/boundary_density.py` | Gaussian density around boundary-offset cage targets. |
| `src/dbact/local_cvt.py` | Local weighted CVT centroid approximation. |
| `src/dbact/local_cbf_qp.py` | Lightweight CBF-style velocity safety filter. |
| `src/dbact/controller.py` | End-to-end DBACT controller pipeline. |
| `src/dbact/transport_dynamics.py` | Simplified simulation caging / pushing dynamics. |
| `src/dbact/metrics.py` | Coverage, recruitment, safety, and path metrics. |

### Simulation

| File or directory | Role |
| --- | --- |
| `src/dbact_sim/run_sim.py` | CLI entry point for simulation. |
| `src/dbact_sim/environment.py` | Simulation loop, logging, output generation. |
| `src/dbact_sim/scenarios.py` | YAML scenario loading and object/agent construction. |
| `src/dbact_sim/visualization.py` | Snapshot and trajectory plots. |
| `configs/sim/*.yaml` | Circle, rectangle, L-shape, nonconvex, caging, tight caging, and multi-object scenarios. |
| `scripts/run_all_scenarios.py` | Batch simulation runner. |

### MAS / RoboMaster / OptiTrack

| File or directory | Role |
| --- | --- |
| `src/mas_adapter/decentralized_transport_controller.py` | Root package MAS-compatible DBACT controller wrapper. |
| `src/mas_adapter/object_observer.py` | Virtual object placeholder; replace for real cargo perception. |
| `configs/mas/*.yaml` | Root-level MAS adapter configs and mock pipeline configs. |
| `platforms/mas_public/src/common/messages.py` | MAS message dataclasses: `WorldState`, `RobotState`, `ControlCommand`, `RobotCommand`. |
| `platforms/mas_public/src/common/config_loader.py` | MAS config validation; includes `dtransport`. |
| `platforms/mas_public/src/controller/controller_module.py` | MAS controller runtime module; builds `dtransport`. |
| `platforms/mas_public/src/controller/decentralized_transport_controller.py` | Vendored MAS DBACT controller. |
| `platforms/mas_public/src/controller/object_observer.py` | Vendored MAS virtual-object observer. |
| `platforms/mas_public/src/optitrack/*` | NatNet adapter, rigid-body mapping, tracking validation, velocity estimation. |
| `platforms/mas_public/src/robot/*` | RoboMaster S1 communication and command limiting. |
| `platforms/mas_public/src/messaging/*` | ZeroMQ transport layer. |
| `platforms/mas_public/configs/*.yaml` | MAS runtime configs. |
| `platforms/mas_public/configs/controllers/dtransport.yaml` | Real MAS dtransport parameters. |
| `platforms/mas_public/apps/run_optitrack.py` | Starts OptiTrack module and publishes `WORLD_STATE`. |
| `platforms/mas_public/apps/run_controller.py` | Starts controller module. |
| `platforms/mas_public/apps/run_robot_comm.py` | Starts RoboMaster robot module. |
| `platforms/mas_public/apps/run_supervisor.py` | Starts enabled modules from supervisor config. Do not use first in read-only Stage 4. |
| `platforms/mas_public/apps/dbact/*.py` | DBACT-specific MAS dry-run and OptiTrack read-only tools. |

## Test, Mock, and Temporary Files

These files are for testing, validation, debugging, or generated output. They are important for development, but they are not the final robot runtime.

| Path | Type |
| --- | --- |
| `tests/*.py` | Root DBACT unit and smoke tests. |
| `platforms/mas_public/apps/pytest_tests/*.py` | MAS platform automated tests. |
| `platforms/mas_public/apps/manual_tests/*.py` | Manual mock/debug scripts for no-hardware checks. |
| `scripts/run_mock_mas_pipeline.py` | Root mock MAS pipeline validation, not final runtime. |
| `platforms/mas_public/apps/dbact/run_dtransport_dry_run.py` | Controller-level dry-run validation. |
| `platforms/mas_public/apps/dbact/run_controller_module_dtransport_dry_run.py` | ControllerModule-level dry-run validation. |
| `platforms/mas_public/apps/dbact/log_optitrack_world_state.py` | Read-only OptiTrack validation bridge before experiments. |
| `scripts/make_repo_tree.py` | Repository tree utility only. |
| `runs/` | Generated simulation outputs, ignored by Git. |
| `platforms/mas_public/data/dry_runs/` | Generated MAS dry-run outputs, ignored by Git through CSV/output rules. |
| `platforms/mas_public/data/optitrack_readonly/` | Generated OptiTrack read-only CSV outputs. |
| `.pytest_cache/`, `__pycache__/`, `.venv/` | Local caches/environments, ignored by Git. |

## Healthy File Policy

The GitHub repository should contain source code, configs, docs, tests, platform integration code, and the local Codex environment file. It should not contain generated experiment data, temporary test directories, bytecode, plots, CSV logs, caches, or local virtual environments.

Currently healthy tracked file groups:

| Group | Count | Purpose |
| --- | ---: | --- |
| `.codex` | 1 | Codex local environment setup for this project. |
| root files | 6 | README, license, packaging, dependency, and ignore metadata. |
| `configs` | 16 | Root simulation and MAS adapter configs. |
| `docs` | 9 | Algorithm, architecture, roadmap, and stage notes. |
| `platforms/mas_public` | 100 | MAS / OptiTrack / RoboMaster platform subtree. |
| `scripts` | 3 | Root utility and dry-run scripts. |
| `src` | 25 | DBACT core, simulator, MAS adapter, and currently tracked package metadata. |
| `tests` | 5 | Root DBACT tests. |

Generated or unhealthy files that should stay out of Git:

```text
runs/
outputs/
*.csv
*.png
*.log
__pycache__/
.pytest_cache/
.venv/
platforms/mas_public/data/
platforms/mas_public/logs/
platforms/mas_public/**/__pycache__/
```

Note: `src/dbact.egg-info/*` is currently tracked historical packaging metadata. It is not normally edited by hand; future cleanup can remove it from tracking if the repository policy is tightened.

## Complete Tracked File Inventory

This inventory is based on the current local Git tracked files. These are the files that should be considered part of the repository's healthy project state unless a future cleanup explicitly removes them.

```text
.codex/environments/environment.toml
.gitignore
LICENSE
README.md
README_DBACT.md
pyproject.toml
requirements.txt

configs/mas/controller.yaml
configs/mas/dtransport.yaml
configs/mas/dtransport_mock.yaml
configs/sim/baseline_unknown_polygon_caging.yaml
configs/sim/baseline_unknown_polygon_caging_tight.yaml
configs/sim/circle.yaml
configs/sim/l_shape.yaml
configs/sim/multi_object.yaml
configs/sim/nonconvex.yaml
configs/sim/one_circle_caging.yaml
configs/sim/one_nonconvex_polygon_caging.yaml
configs/sim/one_nonconvex_polygon_caging_tight.yaml
configs/sim/one_polygon_caging.yaml
configs/sim/one_rectangle_polygon_caging.yaml
configs/sim/one_rectangle_polygon_caging_tight.yaml
configs/sim/rectangle.yaml

docs/ALGORITHM.md
docs/ARCHITECTURE.md
docs/MAS_INTEGRATION.md
docs/ROADMAP.md
docs/daily_health_2026-05-30.md
docs/stage1_results.md
docs/stage2_mas_virtual_object.md
docs/stage3_mas_dry_run.md
docs/stage4_optitrack_readonly.md

scripts/make_repo_tree.py
scripts/run_all_scenarios.py
scripts/run_mock_mas_pipeline.py

src/dbact/__init__.py
src/dbact/boundary_density.py
src/dbact/boundary_map.py
src/dbact/cargo.py
src/dbact/controller.py
src/dbact/geometry.py
src/dbact/local_cbf_qp.py
src/dbact/local_cvt.py
src/dbact/local_sensing.py
src/dbact/metrics.py
src/dbact/transport_dynamics.py
src/dbact/types.py
src/dbact.egg-info/PKG-INFO
src/dbact.egg-info/SOURCES.txt
src/dbact.egg-info/dependency_links.txt
src/dbact.egg-info/requires.txt
src/dbact.egg-info/top_level.txt
src/dbact_sim/__init__.py
src/dbact_sim/environment.py
src/dbact_sim/run_sim.py
src/dbact_sim/scenarios.py
src/dbact_sim/visualization.py
src/mas_adapter/__init__.py
src/mas_adapter/decentralized_transport_controller.py
src/mas_adapter/object_observer.py

tests/test_cargo.py
tests/test_controller_smoke.py
tests/test_density.py
tests/test_mas_adapter_import.py
tests/test_mas_adapter_mock_pipeline.py

platforms/mas_public/.gitignore
platforms/mas_public/README.md
platforms/mas_public/pyproject.toml
platforms/mas_public/requirements.txt
platforms/mas_public/apps/check_experiment.py
platforms/mas_public/apps/plot_experiment.py
platforms/mas_public/apps/run_controller.py
platforms/mas_public/apps/run_optitrack.py
platforms/mas_public/apps/run_robot_comm.py
platforms/mas_public/apps/run_supervisor.py
platforms/mas_public/apps/dbact/log_optitrack_world_state.py
platforms/mas_public/apps/dbact/run_controller_module_dtransport_dry_run.py
platforms/mas_public/apps/dbact/run_dtransport_dry_run.py
platforms/mas_public/apps/manual_tests/mock_optitrack.py
platforms/mas_public/apps/manual_tests/mock_robot.py
platforms/mas_public/apps/manual_tests/test_closed_loop_io.py
platforms/mas_public/apps/manual_tests/test_optitrack_module.py
platforms/mas_public/apps/manual_tests/test_robot_module.py
platforms/mas_public/apps/pytest_tests/test_check_experiment.py
platforms/mas_public/apps/pytest_tests/test_command_limiter.py
platforms/mas_public/apps/pytest_tests/test_config_loader.py
platforms/mas_public/apps/pytest_tests/test_controller_autoplot.py
platforms/mas_public/apps/pytest_tests/test_controller_command_normalization.py
platforms/mas_public/apps/pytest_tests/test_cvt_controller.py
platforms/mas_public/apps/pytest_tests/test_data_recorder.py
platforms/mas_public/apps/pytest_tests/test_experiment_logger.py
platforms/mas_public/apps/pytest_tests/test_messages.py
platforms/mas_public/apps/pytest_tests/test_natnet_adapter_callbacks.py
platforms/mas_public/apps/pytest_tests/test_optitrack_diagnostics.py
platforms/mas_public/apps/pytest_tests/test_plotter_cvt.py
platforms/mas_public/apps/pytest_tests/test_point_controller.py
platforms/mas_public/apps/pytest_tests/test_robomaster_adapter.py
platforms/mas_public/apps/pytest_tests/test_robot_command_transform.py
platforms/mas_public/apps/pytest_tests/test_robot_module_startup.py
platforms/mas_public/apps/pytest_tests/test_supervisor_config.py
platforms/mas_public/apps/pytest_tests/test_tracking_validator.py
platforms/mas_public/apps/pytest_tests/test_world_bounds.py
platforms/mas_public/configs/controller.yaml
platforms/mas_public/configs/logging.yaml
platforms/mas_public/configs/optitrack.yaml
platforms/mas_public/configs/robots.yaml
platforms/mas_public/configs/supervisor.yaml
platforms/mas_public/configs/system.yaml
platforms/mas_public/configs/controllers/cvt.yaml
platforms/mas_public/configs/controllers/dtransport.yaml
platforms/mas_public/configs/controllers/manual.yaml
platforms/mas_public/configs/controllers/point.yaml
platforms/mas_public/docs/config_description.md
platforms/mas_public/docs/hardware_setup.md
platforms/mas_public/docs/usage_and_debug.md
platforms/mas_public/src/__init__.py
platforms/mas_public/src/common/__init__.py
platforms/mas_public/src/common/config_loader.py
platforms/mas_public/src/common/exceptions.py
platforms/mas_public/src/common/logger.py
platforms/mas_public/src/common/math_utils.py
platforms/mas_public/src/common/messages.py
platforms/mas_public/src/common/time_utils.py
platforms/mas_public/src/controller/__init__.py
platforms/mas_public/src/controller/base_controller.py
platforms/mas_public/src/controller/controller_module.py
platforms/mas_public/src/controller/coordinate_transform.py
platforms/mas_public/src/controller/cvt_controller.py
platforms/mas_public/src/controller/cvt_utils.py
platforms/mas_public/src/controller/data_recorder.py
platforms/mas_public/src/controller/decentralized_transport_controller.py
platforms/mas_public/src/controller/experiment_logger.py
platforms/mas_public/src/controller/manual_controller.py
platforms/mas_public/src/controller/object_observer.py
platforms/mas_public/src/controller/point_controller.py
platforms/mas_public/src/controller/world_bounds.py
platforms/mas_public/src/controller/plotting/__init__.py
platforms/mas_public/src/controller/plotting/common_plots.py
platforms/mas_public/src/controller/plotting/cvt_plots.py
platforms/mas_public/src/controller/plotting/experiment_plotter.py
platforms/mas_public/src/messaging/__init__.py
platforms/mas_public/src/messaging/base_transport.py
platforms/mas_public/src/messaging/factory.py
platforms/mas_public/src/messaging/topics.py
platforms/mas_public/src/messaging/zmq_transport.py
platforms/mas_public/src/optitrack/__init__.py
platforms/mas_public/src/optitrack/natnet_adapter.py
platforms/mas_public/src/optitrack/optitrack_module.py
platforms/mas_public/src/optitrack/rigid_body_mapper.py
platforms/mas_public/src/optitrack/state_estimator.py
platforms/mas_public/src/optitrack/tracking_validator.py
platforms/mas_public/src/robot/__init__.py
platforms/mas_public/src/robot/command_limiter.py
platforms/mas_public/src/robot/robomaster_adapter.py
platforms/mas_public/src/robot/robot_command_transform.py
platforms/mas_public/src/robot/robot_module.py
platforms/mas_public/src/robot/robot_registry.py
platforms/mas_public/src/robot/video_interface.py
platforms/mas_public/src/robot/watchdog.py
platforms/mas_public/src/supervisor/__init__.py
platforms/mas_public/src/supervisor/process_manager.py
platforms/mas_public/src/supervisor/supervisor.py
platforms/mas_public/third_party/natnet_client/DataDescriptions.py
platforms/mas_public/third_party/natnet_client/MoCapData.py
platforms/mas_public/third_party/natnet_client/NatNetClient.py
```

## Installation

The project is currently developed in the Conda environment named `dbact`.

```powershell
conda activate dbact
python --version
```

The known working environment is Python 3.10.x; the shared project history used Python 3.10.20.

If the environment does not exist yet:

```powershell
conda create -n dbact python=3.10
conda activate dbact

python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .
```

For development:

```powershell
conda activate dbact
pip install -e .[dev]
```

For the vendored MAS platform:

```powershell
conda activate dbact
cd platforms\mas_public
python -m pip install -r requirements.txt
```

If MAS tests fail with missing packages, install:

```powershell
python -m pip install pandas pyzmq
```

## Verification

From the repository root:

```powershell
conda activate dbact
python -m pytest
python -m compileall -q src tests scripts platforms\mas_public\src platforms\mas_public\apps
```

Expected root status:

```text
6 passed
```

MAS platform tests:

```powershell
conda activate dbact
cd platforms\mas_public
python -m pytest -q apps\pytest_tests
```

Historical verified status from Stage 2 / Stage 3:

```text
DBACT: 6 passed
MAS platform: 106 passed
```

## Run Simulations

Single scenario:

```powershell
conda activate dbact
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape
```

Expected outputs:

```text
runs/l_shape/
|-- trajectories.csv
|-- metrics.json
|-- final_snapshot.png
`-- trajectory.png
```

Useful scenarios:

```powershell
python -m dbact_sim.run_sim --config configs/sim/circle.yaml --steps 300 --output runs/circle
python -m dbact_sim.run_sim --config configs/sim/rectangle.yaml --steps 400 --output runs/rectangle
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 500 --output runs/l_shape
python -m dbact_sim.run_sim --config configs/sim/nonconvex.yaml --steps 500 --output runs/nonconvex
python -m dbact_sim.run_sim --config configs/sim/multi_object.yaml --steps 600 --output runs/multi_object
```

Unknown polygon caging baselines:

```powershell
python -m dbact_sim.run_sim --config configs/sim/baseline_unknown_polygon_caging_tight.yaml --steps 900 --output runs/baseline_unknown_polygon_caging_tight
python -m dbact_sim.run_sim --config configs/sim/one_rectangle_polygon_caging_tight.yaml --steps 900 --output runs/one_rectangle_polygon_caging_tight
python -m dbact_sim.run_sim --config configs/sim/one_nonconvex_polygon_caging_tight.yaml --steps 1000 --output runs/one_nonconvex_polygon_caging_tight
```

Run the default batch:

```powershell
python scripts/run_all_scenarios.py
```

## Run Root Mock MAS Pipeline

This is the first software bridge before the vendored MAS platform:

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

These dry-runs should produce nonzero `dbact_cage` commands while staying inside configured safety limits.

## Get OptiTrack Data Through MAS

There are two different OptiTrack paths. Keep them separate.

### A. Safe Read-Only Path

Use this before any robot moves.

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

This validates:

```text
NatNetAdapter / MockNatNetAdapter
  -> NatNetRigidBody
  -> RigidBodyMapper
  -> TrackingValidator
  -> StateEstimator
  -> RobotState
  -> WorldState
  -> CSV
```

No `ControlCommand` is published in this path.

### B. MAS Runtime Path

After read-only data is correct, the real MAS runtime path is:

```text
Motive GUI
  -> NatNet
  -> platforms/mas_public/apps/run_optitrack.py
  -> OptiTrackModule
  -> WORLD_STATE over ZeroMQ
  -> platforms/mas_public/apps/run_controller.py
  -> ControllerModule
  -> DecentralizedTransportController.compute()
  -> CONTROL_COMMAND over ZeroMQ
  -> platforms/mas_public/apps/run_robot_comm.py
  -> RoboMaster S1
```

Start manually in separate terminals only after safety checks:

```powershell
# Terminal 1
cd platforms\mas_public
python apps\run_optitrack.py
```

```powershell
# Terminal 2
cd platforms\mas_public
python apps\run_controller.py
```

```powershell
# Terminal 3
cd platforms\mas_public
python apps\run_robot_comm.py
```

Do **not** start `run_supervisor.py` during read-only Stage 4 unless `configs/supervisor.yaml` has been made safe. The current full runtime config may enable OptiTrack, controller, and robot together.

For read-only Stage 4, use:

```yaml
use_optitrack: true
use_controller: false
use_robot: false
```

## OptiTrack Config Checklist

Before real OptiTrack tests, check:

```text
platforms/mas_public/configs/optitrack.yaml
platforms/mas_public/configs/robots.yaml
platforms/mas_public/configs/system.yaml
platforms/mas_public/configs/supervisor.yaml
```

Critical values:

| Config | Check |
| --- | --- |
| `optitrack.yaml` | `server_ip`, `client_ip`, `connection_type`, `stream_type`, `data_port`, `command_port`, `python_client_path`. |
| `robots.yaml` | `robot_id`, `rigid_body_name`, `rigid_body_id`, RoboMaster `sn`. |
| `system.yaml` | `z_up_transform`, `world` bounds, control frequency, ZMQ ports, command transforms. |
| `supervisor.yaml` | Ensure robot/controller are disabled for read-only logging. |

NatNet Python client files should exist under:

```text
platforms/mas_public/third_party/natnet_client/
|-- NatNetClient.py
|-- DataDescriptions.py
`-- MoCapData.py
```

## How OptiTrack Should Enter DBACT

Current working path for robot poses:

```text
NatNet rigid bodies for robots
  -> RigidBodyMapper
  -> WorldState.robots
  -> ControllerModule
  -> DBACT agent states
```

Current object path:

```text
virtual_object in dtransport.yaml
  -> ObjectObserver.observe()
  -> Cargo polygon
  -> DBACT local boundary sensing
```

Final required object path for physical experiments:

```text
OptiTrack cargo markers / cargo rigid body / perception system
  -> object pose or boundary points in MAS world frame
  -> ObjectObserver or extended WorldState object channel
  -> Cargo polygon or BoundaryObservation list
  -> DBACTController.step()
  -> MAS ControlCommand
```

Recommended implementation path:

1. Keep robot OptiTrack read-only logging working first.
2. Add cargo rigid-body or marker config, for example `cargo_0` with Motive rigid-body ID or marker names.
3. Extend the object observer so `observe(world_state)` or an OptiTrack-backed observer can produce a `Cargo` polygon in MAS world coordinates.
4. For the first physical caging demo, allow a known local cargo polygon shape transformed by OptiTrack pose.
5. For the stronger unknown-object claim, replace the known polygon with local boundary observations from vision, contact, or marker-derived boundary samples.
6. Keep `max_speed`, `kp_cage`, and `kp_transport` conservative until read-only logs, dry-run commands, and robot stop behavior are verified.

Minimal first real experiment target:

```text
OptiTrack robot poses + virtual/known cargo polygon
  -> DBACT caging commands
  -> very low speed RoboMaster S1 motion
  -> data/experiments/<run>/
  -> offline plots
```

Paper-quality final target:

```text
OptiTrack robot poses + real object boundary observations
  -> DBACT boundary-aware caging / transport
  -> low-speed multi-robot experiment
  -> recorded WorldState, ControlCommand, robot status, plots, metrics
```

## Safety Rules for Real Robots

- Always run read-only OptiTrack logging before controller output.
- Verify `robot_id` to rigid-body mapping by moving one robot at a time.
- Verify coordinate axes and yaw direction before enabling robot commands.
- Keep `max_speed <= 0.08` for the first physical tests.
- Keep `kp_transport = 0.0` for caging-only validation.
- Use a physical emergency stop. `Ctrl+C` is not a physical safety system.
- Do not run `run_supervisor.py` until manual module startup is verified.
- Record every experiment and inspect `control_command.csv` for shutdown rows.

## Documentation Map

| File | Content |
| --- | --- |
| `docs/ARCHITECTURE.md` | Package layout and data flow. |
| `docs/ALGORITHM.md` | Core algorithm notes. |
| `docs/stage1_results.md` | Unknown polygon caging baseline and tight baseline results. |
| `docs/stage2_mas_virtual_object.md` | MAS virtual-object integration progress. |
| `docs/stage3_mas_dry_run.md` | MAS dry-run stage notes. |
| `docs/stage4_optitrack_readonly.md` | OptiTrack / NatNet read-only status and validation commands. |
| `docs/MAS_INTEGRATION.md` | MAS integration guide. |
| `docs/ROADMAP.md` | Staged roadmap. |
| `platforms/mas_public/docs/*.md` | MAS platform hardware, config, usage, and debug notes. |

## Roadmap

Next engineering steps:

1. Run `log_optitrack_world_state.py` against real Motive/NatNet with robots only.
2. Confirm axes, yaw, rigid-body names/IDs, and world bounds.
3. Add object/cargo OptiTrack observation support.
4. Run `OptiTrack -> ControllerModule -> ControlCommand` dry-run with robot output disabled.
5. Run low-speed caging-only RoboMaster experiment.
6. Add visualization for real experiments and compare simulation vs hardware logs.
7. Replace virtual/known object with a real boundary-observation pipeline.

Research work still open:

- Formal QP solver for CBF filtering.
- Adaptive recruitment and boundary gap detection.
- Object pose and boundary estimation from accumulated local observations.
- Nonholonomic/contact-rich dynamics.
- Real unknown-object transport beyond caging.

## License

MIT License.
