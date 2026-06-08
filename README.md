# DBACT: Decentralized Boundary-Aware Cooperative Transportation

**Full title:** Decentralized Boundary-Aware Cooperative Transportation of Arbitrarily Shaped Objects Without Prior Object Knowledge.

DBACT is a research and experiment stack for cooperative multi-robot transport. It starts from a hardware-independent controller, validates unknown arbitrary polygon caging in simulation, and connects the controller to a MAS / RoboMaster S1 platform with OptiTrack/Motive tracking.

Plain-language summary: robots discover and cage an unknown object from local boundary observations, then cooperate through local allocation and safety filtering.

Use the repository root as the working directory for all commands below.

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

The project is on the correct path for a real robot experiment: simulation first, MAS controller integration second, no-hardware dry-runs third, OptiTrack read-only validation fourth, and only then low-speed RoboMaster trials.

| Stage | Status | Meaning |
| --- | --- | --- |
| Stage 1: DBACT simulation | Done | Arbitrary polygon cargo caging works without using cargo geometry as a controller prior. |
| Stage 2: MAS virtual-object integration | Done | `dtransport` controller is registered in the MAS platform and can produce MAS `ControlCommand`. |
| Stage 3: MAS dry-run | Done | Controller-level and ControllerModule-level dry-runs work without OptiTrack, RoboMaster, or network runtime. |
| Stage 4: OptiTrack read-only bridge | In progress | Mock logger works; NatNet/Motive connection works; current blocker is creating/enabling Motive rigid bodies so `raw_bodies > 0`. |
| Final experiment | Not yet complete | Real cargo observation, real OptiTrack-to-DBACT object bridge, low-speed RoboMaster trials, and real-experiment visualization remain. |

Important current limitation: the MAS stack reads robot poses from OptiTrack, but the DBACT `ObjectObserver` is still a virtual-object placeholder. For a physical unknown-object experiment, cargo/object boundary observations still need to come from OptiTrack markers, vision, tactile/contact sensing, or another object-perception source.

## Latest Health

Last full health check: **2026-06-08** in Conda environment `dbact`.

| Scope | Root pytest | Compileall | YAML parse | Platform pytest |
| --- | --- | --- | --- | --- |
| All active branch refs | PASS, `main` 10 passed and stage branches 6 passed | PASS | PASS, `main` 28 files and stage branches 26 files | PASS, 106 passed each |
| Current `main` working tree | PASS, 16 passed | PASS | PASS, 28 files | PASS, 106 passed |

Branch refs checked:

```text
main
stage2-mas-virtual-object
stage3-mas-dry-run
stage4-optitrack-readonly
```

Detailed reports:

```text
docs/daily_health_2026-06-08.md
docs/repository_inventory_2026-06-08.md
```

## What Has Been Built

### Stage 1: Unknown Polygon Caging

Completed:

- Arbitrary polygon cargo model.
- Local boundary sensing.
- Boundary-aware density field.
- Local CVT allocation.
- Agent-agent CBF-style safety filtering.
- Optional QP-backed CBF solve with projection fallback.
- Caging-only unknown-cargo baseline.
- Coverage mode for decentralized target-region CVT.
- `recruited_agents`, coverage, path length, displacement, and safety metrics.
- Paper-style live viewer, saved paper figures, coverage plots, and optional GIF animation.

The controller intentionally does **not** use these as direct control priors in the unknown-cargo caging baseline:

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
- Vendored MAS platform under `platforms/mas_public`
- MAS `config_loader` supports `controller.type: dtransport`
- MAS `ControllerModule` can build `DecentralizedTransportController`
- MAS `dtransport.yaml` supports `virtual_object`
- `compute(WorldState)` returns MAS `ControlCommand`
- Velocity and yaw-rate limits are clamped by both system and dtransport limits

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

### Stage 3: MAS Dry-Run

Completed:

- `platforms/mas_public/apps/dbact/run_dtransport_dry_run.py`
- `platforms/mas_public/apps/dbact/run_controller_module_dtransport_dry_run.py`
- World-bound checks.
- Dry-run trajectory output.
- Dry-run command/state/event CSV outputs.
- Automatic dry-run robot initialization.
- Clamp-to-world-bounds mode.
- ControllerModule-level dry-run through MAS helper methods.

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

### Stage 4: OptiTrack Read-Only Bridge

Current Stage 4 goal:

```text
Motive / NatNet or MockNatNet
  -> NatNetRigidBody
  -> RobotState
  -> WorldState
  -> CSV log
```

The repository includes a safe read-only logger:

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

### Seven-S1 Command Smoke Test

The repository now includes a low-speed command-streaming smoke path for seven RoboMaster S1 robots:

- `src/dbact/agent_control.py` defines agent state providers, CVT/DBACT command policies, world-to-body command conversion, velocity limits, a mock backend, and an optional `S1RoboMasterBackend`.
- `scripts/run_seven_s1_cvt_test.py` runs in mock mode by default, supports `--real` for seven configured S1 robots, supports `--controller cvt` and `--controller dbact-box`, and includes a `--connect-only` hardware check.
- `tests/test_agent_control.py` covers command limiting, command-frame conversion, mock backend behavior, and both command policies.

This smoke test is useful for validating command generation and conservative S1 command streaming. It is not a substitute for OptiTrack feedback or physical safety checks unless a real state provider is connected and verified.

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
|-- docs/                      # Architecture, algorithm, roadmap, health, inventory
`-- platforms/mas_public/      # Vendored MAS / OptiTrack / RoboMaster platform
```

Detailed file inventory:

```text
docs/repository_inventory_2026-06-08.md
```

## Healthy File Policy

The GitHub repository should contain source code, configs, docs, tests, platform integration code, and the local Codex environment file. It should not contain generated experiment data, temporary test directories, bytecode, plots, CSV logs, caches, or local virtual environments.

Generated or local-only paths that should stay out of Git:

```text
runs/
outputs/
*.csv
*.png
*.gif
*.mp4
*.log
__pycache__/
.pytest_cache/
.venv/
platforms/mas_public/data/
platforms/mas_public/logs/
platforms/mas_public/**/__pycache__/
```

Note: `src/dbact.egg-info/*` is currently tracked historical packaging metadata. It is not normally edited by hand; future cleanup can remove it from tracking if the repository policy is tightened.

## Essential Files

### Root Project

| File | Why it matters |
| --- | --- |
| `.codex/environments/environment.toml` | Local Codex project environment setup. |
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
| `src/dbact/boundary_density.py` | Gaussian density around boundary-offset cage targets and target-region points. |
| `src/dbact/local_cvt.py` | Local weighted CVT centroid approximation. |
| `src/dbact/local_cbf_qp.py` | Optional QP-backed CBF safety filter with projection fallback. |
| `src/dbact/controller.py` | End-to-end caging and coverage controller pipeline. |
| `src/dbact/agent_control.py` | Seven-S1 command policies, safety limiting, state providers, and mock / RoboMaster backends. |
| `src/dbact/transport_dynamics.py` | Simplified simulation caging / pushing dynamics. |
| `src/dbact/metrics.py` | Coverage, recruitment, safety, and path metrics. |

### Simulation

| File or directory | Role |
| --- | --- |
| `src/dbact_sim/run_sim.py` | CLI entry point for simulation, live viewer, figure output, and animation. |
| `src/dbact_sim/environment.py` | Simulation loop, logging, output generation. |
| `src/dbact_sim/scenarios.py` | YAML scenario loading and object/agent construction. |
| `src/dbact_sim/visualization.py` | Snapshot, trajectory, coverage, paper-frame, live, and GIF plots. |
| `configs/sim/*.yaml` | Circle, rectangle, L-shape, nonconvex, caging, coverage, and moving-cargo scenarios. |
| `scripts/run_all_scenarios.py` | Batch simulation runner with live/animation options. |
| `scripts/run_seven_s1_cvt_test.py` | Mock or real seven-S1 low-speed command smoke test. |

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
| `platforms/mas_public/apps/dbact/*.py` | DBACT-specific MAS dry-run and OptiTrack read-only tools. |

## Installation

The project is currently developed in the Conda environment named `dbact`.

```powershell
conda activate dbact
python --version
```

The known working environment is Python 3.10.x; the latest local health check used Python 3.10.20.

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

MAS platform tests:

```powershell
conda activate dbact
cd platforms\mas_public
python -m pytest -q apps\pytest_tests
```

If pytest cannot access the default Windows temporary cache directory, provide an explicit temporary base:

```powershell
python -m pytest -q --basetemp %TEMP%\dbact_pytest_tmp tests
```

## Run Simulations

Single scenario:

```powershell
conda activate dbact
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape
```

Real-time paper-style viewer:

```powershell
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape --live
```

`--live` opens an interactive matplotlib window while the simulation is running. The window shows the same paper-style layout as the saved `FIG_*.png` files: robot positions, local Voronoi/CVT cells, cargo cage targets, local safety/communication ranges, and the 3D density field `Phi`. The window stays open at the end so the final state can be inspected; add `--live-close-at-end` for non-blocking batch runs.

Expected outputs:

```text
runs/l_shape/
|-- agent_positions.csv
|-- coverage_rates.csv
|-- trajectories.csv
|-- metrics.json
|-- coverage_rate_curve.png
|-- final_snapshot.png
|-- trajectory.png
`-- figures/
    |-- FIG_0.png
    |-- FIG_<middle>.png
    `-- FIG_<final>.png
```

Optional animation output:

```powershell
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape --animate
```

Moving irregular cargo demo:

```powershell
python -m dbact_sim.run_sim --config configs/sim/paper_like_irregular_moving_cargo.yaml --steps 520 --output runs/paper_like_irregular_moving_cargo --live --figure-frames 0,31,82,124,520
```

Decentralized coverage demo:

```powershell
python -m dbact_sim.run_sim --config configs/sim/decentralized_cvt_coverage.yaml --steps 220 --output runs/decentralized_cvt_coverage --figure-frames 0,55,110,165,220
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

Batch with live windows:

```powershell
python scripts/run_all_scenarios.py --live --live-close-at-end
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

## Run Seven-S1 Command Smoke Test

Mock mode from the repository root:

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

For first hardware runs, keep physical access to an emergency stop and prefer `--connect-only` before any movement command.

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
| `docs/stage4_optitrack_readonly.md` | Stage 4 OptiTrack read-only bridge status and blocker. |
| `docs/MAS_INTEGRATION.md` | MAS integration guide. |
| `docs/ROADMAP.md` | Staged roadmap. |
| `docs/daily_health_2026-06-08.md` | Latest full branch and working-tree health report. |
| `docs/repository_inventory_2026-06-08.md` | Detailed local repository file inventory. |
| `docs/daily_health_2026-06-03.md` | Historical branch and working-tree health report. |
| `docs/repository_inventory_2026-06-03.md` | Historical local repository file inventory. |
| `platforms/mas_public/docs/*.md` | MAS platform hardware, config, usage, and debug notes. |

## Roadmap

Next engineering steps:

1. Create and enable Motive rigid bodies for each robot so NatNet reports `raw_bodies > 0`.
2. Run `log_optitrack_world_state.py` against real Motive/NatNet with robots only.
3. Confirm axes, yaw, rigid-body names/IDs, and world bounds.
4. Run `run_seven_s1_cvt_test.py --real --connect-only`, then very-low-speed command streaming only after physical safety checks.
5. Add object/cargo OptiTrack observation support.
6. Run `OptiTrack -> ControllerModule -> ControlCommand` dry-run with robot output disabled.
7. Run low-speed caging-only RoboMaster experiment.
8. Add visualization for real experiments and compare simulation vs hardware logs.
9. Replace virtual/known object with a real boundary-observation pipeline.

Research work still open:

- Formal QP tuning and solver selection for CBF filtering.
- Adaptive recruitment and boundary gap detection.
- Object pose and boundary estimation from accumulated local observations.
- Nonholonomic/contact-rich dynamics.
- Real unknown-object transport beyond caging.

## License

MIT License.
