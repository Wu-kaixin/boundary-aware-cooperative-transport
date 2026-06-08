# Repository Inventory - 2026-06-08

Generated from the local repository root:

```text
E:\DBACT\boundary-aware-cooperative-transport
```

This inventory lists the source, config, test, documentation, and platform files that should be visible to GitHub after the 2026-06-08 update. Generated outputs such as `runs/`, `outputs/`, `.pytest_cache/`, `__pycache__/`, virtual environments, experiment CSVs, logs, and plot artifacts are intentionally excluded by `.gitignore` and are not part of the committed source inventory.

## Summary

| Area | Files | Purpose |
| --- | ---: | --- |
| Root project metadata | 7 | Codex environment config, ignore rules, dependency lists, license, top-level READMEs. |
| Tracked package metadata | 5 | Existing `dbact.egg-info` metadata tracked in the repository. |
| Root DBACT source | 13 | Core algorithm, local sensing, density, CVT, CBF, metrics, types, and robot command orchestration. |
| Root simulation source | 5 | Scenario loading, simulation loop, CLI, visualization. |
| MAS adapter source | 3 | Hardware-independent bridge into MAS-style control. |
| Simulation configs | 15 | Baselines, caging scenarios, coverage scenario, paper-style moving cargo demo. |
| MAS configs | 3 | Root-level MAS adapter configs. |
| Root scripts | 4 | Batch scenario runner, mock MAS pipeline, tree utility, seven-S1 command smoke test. |
| Root tests | 7 | DBACT unit, smoke, MAS adapter, coverage-mode, and agent-control tests. |
| Root docs | 13 | Architecture, algorithm, roadmap, stage notes, daily health, inventory. |
| Vendored MAS platform | 100 | Platform ignore rules, apps, configs, docs, tests, source, and NatNet client files. |

Total Git-visible source/inventory files after this update: 175.

## Root Project

```text
.codex/environments/environment.toml
.gitignore
LICENSE
README.md
README_DBACT.md
pyproject.toml
requirements.txt
```

## Tracked Package Metadata

```text
src/dbact.egg-info/PKG-INFO
src/dbact.egg-info/SOURCES.txt
src/dbact.egg-info/dependency_links.txt
src/dbact.egg-info/requires.txt
src/dbact.egg-info/top_level.txt
```

## Root DBACT Source

```text
src/dbact/__init__.py
src/dbact/agent_control.py
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
```

## Root Simulation Source

```text
src/dbact_sim/__init__.py
src/dbact_sim/environment.py
src/dbact_sim/run_sim.py
src/dbact_sim/scenarios.py
src/dbact_sim/visualization.py
```

## MAS Adapter Source

```text
src/mas_adapter/__init__.py
src/mas_adapter/decentralized_transport_controller.py
src/mas_adapter/object_observer.py
```

## Configs

```text
configs/mas/controller.yaml
configs/mas/dtransport.yaml
configs/mas/dtransport_mock.yaml
configs/sim/baseline_unknown_polygon_caging.yaml
configs/sim/baseline_unknown_polygon_caging_tight.yaml
configs/sim/circle.yaml
configs/sim/decentralized_cvt_coverage.yaml
configs/sim/l_shape.yaml
configs/sim/multi_object.yaml
configs/sim/nonconvex.yaml
configs/sim/one_circle_caging.yaml
configs/sim/one_nonconvex_polygon_caging.yaml
configs/sim/one_nonconvex_polygon_caging_tight.yaml
configs/sim/one_polygon_caging.yaml
configs/sim/one_rectangle_polygon_caging.yaml
configs/sim/one_rectangle_polygon_caging_tight.yaml
configs/sim/paper_like_irregular_moving_cargo.yaml
configs/sim/rectangle.yaml
```

## Scripts

```text
scripts/make_repo_tree.py
scripts/run_all_scenarios.py
scripts/run_mock_mas_pipeline.py
scripts/run_seven_s1_cvt_test.py
```

## Root Tests

```text
tests/test_agent_control.py
tests/test_cargo.py
tests/test_controller_smoke.py
tests/test_decentralized_coverage.py
tests/test_density.py
tests/test_mas_adapter_import.py
tests/test_mas_adapter_mock_pipeline.py
```

## Documentation

```text
docs/ALGORITHM.md
docs/ARCHITECTURE.md
docs/MAS_INTEGRATION.md
docs/ROADMAP.md
docs/daily_health_2026-05-30.md
docs/daily_health_2026-06-03.md
docs/daily_health_2026-06-08.md
docs/repository_inventory_2026-06-03.md
docs/repository_inventory_2026-06-08.md
docs/stage1_results.md
docs/stage2_mas_virtual_object.md
docs/stage3_mas_dry_run.md
docs/stage4_optitrack_readonly.md
```

## Vendored MAS Platform

### Platform Metadata And Docs

```text
platforms/mas_public/.gitignore
platforms/mas_public/README.md
platforms/mas_public/pyproject.toml
platforms/mas_public/requirements.txt
platforms/mas_public/docs/config_description.md
platforms/mas_public/docs/hardware_setup.md
platforms/mas_public/docs/usage_and_debug.md
```

### Platform Apps

```text
platforms/mas_public/apps/check_experiment.py
platforms/mas_public/apps/plot_experiment.py
platforms/mas_public/apps/run_controller.py
platforms/mas_public/apps/run_optitrack.py
platforms/mas_public/apps/run_robot_comm.py
platforms/mas_public/apps/run_supervisor.py
platforms/mas_public/apps/dbact/log_optitrack_world_state.py
platforms/mas_public/apps/dbact/run_controller_module_dtransport_dry_run.py
platforms/mas_public/apps/dbact/run_dtransport_dry_run.py
```

### Platform Manual Tests

```text
platforms/mas_public/apps/manual_tests/mock_optitrack.py
platforms/mas_public/apps/manual_tests/mock_robot.py
platforms/mas_public/apps/manual_tests/test_closed_loop_io.py
platforms/mas_public/apps/manual_tests/test_optitrack_module.py
platforms/mas_public/apps/manual_tests/test_robot_module.py
```

### Platform Pytest Tests

```text
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
```

### Platform Configs

```text
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
```

### Platform Source

```text
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
```

### Third Party NatNet Client

```text
platforms/mas_public/third_party/natnet_client/DataDescriptions.py
platforms/mas_public/third_party/natnet_client/MoCapData.py
platforms/mas_public/third_party/natnet_client/NatNetClient.py
```

## Current Git-Visible Working Tree Changes

The inventory was prepared while `main` had local changes intended to be committed with this documentation update:

```text
M  README.md
M  docs/daily_health_2026-06-03.md
A  docs/daily_health_2026-06-08.md
A  docs/repository_inventory_2026-06-08.md
A  scripts/run_seven_s1_cvt_test.py
A  src/dbact/agent_control.py
A  tests/test_agent_control.py
```

## Generated Or Local-Only Paths

These paths are intentionally not part of the committed source inventory:

```text
runs/
outputs/
.pytest_cache/
__pycache__/
.venv/
platforms/mas_public/data/
platforms/mas_public/logs/
*.csv
*.log
*.png
*.gif
*.mp4
```
