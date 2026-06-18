# DBACT: Boundary-Aware Cooperative Transport

[English](README.en.md) | [繁體中文](README.zh-TW.md) | [日本語](README.ja.md)

## Project Context

This repository is a research software stack for decentralized cooperative transport with multiple mobile robots.

The project asks a practical robotics question:

> Can multiple robots form a useful caging and transport structure around an object whose complete shape, center, radius, and required team size are not given to the controller?

The immediate purpose is narrower than a full physical transport system:

> Build a reproducible DBACT simulation stack, connect it to MAS-style controller interfaces, produce visual experimental evidence, and prepare a safe path toward RoboMaster S1 + OptiTrack validation.

The research lineage is:

```text
Cooperative-Transport-Multi-Agent-System
        -> DBACT: Decentralized Boundary-Aware Cooperative Transportation
        -> MAS adapter / OptiTrack read-only / RoboMaster S1 smoke path
```

This repository should be read as an active research prototype. The simulation and dry-run paths are working; the full physical transport experiment is still a staged validation target.

## Current Research Decision

The current project direction is:

- Keep `main` as the maintained branch.
- Treat simulation and MAS dry-runs as the primary validation surface.
- Do not claim a completed physical transport experiment yet.
- Do not send real robot commands before read-only OptiTrack logging, mock pipeline checks, and low-speed command smoke tests pass.
- Keep visualization as a first-class output, because the project value depends on making caging, coverage, density, and safety behavior understandable.

The action item is therefore:

```text
Maintain a clear simulation-to-hardware path:
local boundary simulation
  -> visual result generation
  -> MAS-compatible dry-run
  -> OptiTrack read-only validation
  -> low-speed RoboMaster S1 smoke testing
  -> future closed-loop physical transport
```

## Current Project Scope

This repository currently focuses on:

```text
unknown polygon caging
+ local boundary sensing
+ boundary-aware density
+ local CVT target allocation
+ local CBF-style safety filtering
+ simplified caging / pushing transport dynamics
+ simulation metrics and visualizations
+ MAS-compatible ControlCommand generation
+ OptiTrack read-only logging path
+ RoboMaster S1 command smoke path
```

Out of scope for the current validated stage:

```text
complete physical transport validation
real cargo perception from arbitrary sensors
force-controlled contact dynamics
paper-grade QP solver integration for every path
large-scale hardware deployment
fully automated process-launcher experiments
```

## Controller And Simulation Model

DBACT models each robot as a local decision maker. Each agent has:

- position and velocity;
- sensing range;
- communication range;
- safety radius / minimum inter-agent distance;
- local boundary observations;
- local neighbor states.

The controller intentionally avoids direct access to:

```text
cargo.center
cargo.radius
cargo.vertices
cargo.closest_boundary()
global robot assignment
global all-pairs QP
predefined team size
```

The simulator may use true cargo geometry to generate local sensor observations and offline metrics, but the controller path is kept boundary-observation based.

## DBACT Boundary-Aware Pipeline

The main DBACT idea is:

> Move robots toward useful boundary-adjacent cage targets, not toward a known object center.

Current DBACT flow:

1. generate local boundary observations;
2. estimate outward boundary normals;
3. create cage target points outside the object;
4. build a boundary-aware Gaussian density field;
5. compute local weighted CVT centroids using nearby robots;
6. apply local CBF-style safety filtering;
7. output robot-level velocity commands;
8. export trajectories, coverage curves, figures, and optional animations.

The cage target rule is:

```text
q_target = b + d_cage * n_out
```

The inter-agent safety constraint is:

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

Main implementation:

```text
src/dbact/controller.py
src/dbact/local_sensing.py
src/dbact/boundary_density.py
src/dbact/local_cvt.py
src/dbact/local_cbf_qp.py
src/dbact/transport_dynamics.py
src/dbact_sim/run_sim.py
src/mas_adapter/decentralized_transport_controller.py
```

## Visualization Priority

The project value depends strongly on visualization.

The visual output should make the simulation understandable at a glance:

- robot trajectories;
- unknown cargo shape;
- cage target region;
- local CVT / Voronoi structure;
- boundary-aware density surface;
- boundary coverage curve;
- minimum distance and safety behavior;
- MAS dry-run trajectory evidence.

The selected README assets are stored in `docs/assets/` so they render correctly on GitHub.

![DBACT moving cargo demo](docs/assets/dbact-moving-cargo.gif)

| Density + Local CVT | Trajectory |
| --- | --- |
| ![DBACT density and local CVT frame](docs/assets/dbact-density-cvt-frame.png) | ![DBACT trajectory](docs/assets/dbact-trajectory.png) |

| Coverage Curve | Final Snapshot |
| --- | --- |
| ![DBACT coverage curve](docs/assets/dbact-coverage-curve.png) | ![DBACT final snapshot](docs/assets/dbact-final-snapshot.png) |

If new images, GIFs, or videos should appear on GitHub, copy them from `runs/` into `docs/assets/` and reference that stable path. Generated files under `runs/` remain ignored by Git.

## Experimental Results

Stage 1 validates unknown polygon caging without direct cargo-geometry access inside the controller.

Original baseline results:

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging` | arbitrary polygon | 12 | 900 | 0.7625 | 6 / 12 | 0.3446 m |
| `one_rectangle_polygon_caging` | rectangle polygon | 12 | 900 | 0.7000 | 6 / 12 | 0.3446 m |
| `one_nonconvex_polygon_caging` | nonconvex polygon | 14 | 1000 | 0.90625 | 9 / 14 | 0.3393 m |

Tight baseline results:

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging_tight` | arbitrary polygon | 12 | 900 | 0.95625 | 11 / 12 | 0.3450 m |
| `one_rectangle_polygon_caging_tight` | rectangle polygon | 12 | 900 | 0.99375 | 9 / 12 | 0.3450 m |
| `one_nonconvex_polygon_caging_tight` | nonconvex polygon | 14 | 1000 | 0.9750 | 13 / 14 | 0.3370 m |

Moving irregular cargo demo:

| Metric | Value |
| --- | ---: |
| Final time | 25.95 s |
| Final coverage | 0.83125 |
| Cargo displacement | 1.539 m |
| Recruited agents | 6 |
| Minimum inter-agent distance | 0.3571 m |
| Mean path length | 4.6999 m |

The tight caging settings improve boundary coverage while keeping the minimum inter-agent distance above 0.33 m in the reported Stage 1 benchmarks.

## Repository Structure

```text
boundary-aware-cooperative-transport/
|-- configs/
|   |-- sim/
|   `-- mas/
|-- docs/
|   |-- assets/
|   |-- ARCHITECTURE.md
|   |-- ALGORITHM.md
|   |-- MAS_INTEGRATION.md
|   |-- ROADMAP.md
|   `-- stage1_results.md
|-- src/
|   |-- dbact/
|   |-- dbact_sim/
|   `-- mas_adapter/
|-- scripts/
|   |-- run_all_scenarios.py
|   |-- run_mock_mas_pipeline.py
|   `-- run_seven_s1_cvt_test.py
|-- tests/
|-- platforms/
|   `-- mas_public/
|-- README.md
|-- README.en.md
|-- README.zh-TW.md
|-- README.ja.md
|-- requirements.txt
|-- pyproject.toml
`-- LICENSE
```

## Conda Setup

The known working local environment is a Conda environment named `dbact`.

Create and install:

```bash
conda create -n dbact python=3.10
conda activate dbact
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .[dev]
```

For the vendored MAS platform:

```bash
conda activate dbact
cd platforms/mas_public
python -m pip install -r requirements.txt
```

## Run Simulations

Moving irregular cargo demo:

```bash
python -m dbact_sim.run_sim --config configs/sim/paper_like_irregular_moving_cargo.yaml --steps 520 --output runs/paper_like_irregular_moving_cargo --animate
```

L-shape scenario with paper-style figures:

```bash
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape
```

Batch run:

```bash
python scripts/run_all_scenarios.py --steps 400 --animate
```

## Outputs

Each simulation run saves:

- `trajectories.csv`;
- `agent_positions.csv`;
- `coverage_rates.csv`;
- `metrics.json` when metrics export is enabled;
- `final_snapshot.png`;
- `trajectory.png`;
- `coverage_rate_curve.png`;
- `figures/FIG_*.png`;
- optional `animation.gif`.

Generated outputs are ignored by Git. Curated README visuals are tracked under `docs/assets/`.

## MAS And Hardware Validation

Mock MAS pipeline:

```bash
python scripts/run_mock_mas_pipeline.py --steps 80 --dt 0.05 --print-every 20 --output runs/mock_mas_pipeline
```

MAS dry-run:

```bash
cd platforms/mas_public
python apps/dbact/run_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data/dry_runs/dtransport_auto_init --clamp-to-world-bounds
```

ControllerModule-level dry-run:

```bash
cd platforms/mas_public
python apps/dbact/run_controller_module_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data/dry_runs/controller_module_dtransport
```

OptiTrack read-only check:

```bash
cd platforms/mas_public
python apps/dbact/log_optitrack_world_state.py --mock --frames 50 --hz 100 --print-every 10 --output data/optitrack_readonly/mock_world_states.csv
```

RoboMaster S1 mock command smoke test:

```bash
python scripts/run_seven_s1_cvt_test.py --duration 3
```

Safety rules:

- Run read-only OptiTrack logging before enabling controller output.
- Verify robot ID to rigid-body mapping one robot at a time.
- Use very low speed limits for the first physical run.
- Keep a physical emergency stop available during hardware tests.
- Inspect command and state logs after each run.

## Reports And Documentation

Existing documentation:

```text
docs/ARCHITECTURE.md
docs/ALGORITHM.md
docs/MAS_INTEGRATION.md
docs/ROADMAP.md
docs/stage1_results.md
docs/stage2_mas_virtual_object.md
docs/stage3_mas_dry_run.md
docs/stage4_optitrack_readonly.md
docs/daily_health_2026-06-18.md
docs/assets/README.md
```

These reports should be read as staged research evidence, not as a final physical-experiment claim.

## Tests

Run root tests:

```bash
python -m pytest -q tests
```

Compile check:

```bash
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
```

MAS platform tests:

```bash
python -m pytest -q --rootdir platforms/mas_public platforms/mas_public/apps/pytest_tests
```

The latest verified local health report is recorded in:

```text
docs/daily_health_2026-06-18.md
```

## Near-Term Plan

Recommended next implementation targets:

1. Preserve `docs/assets/` as the stable GitHub visualization surface.
2. Add more side-by-side scenario comparison figures.
3. Improve moving-cargo transport metrics and summary dashboards.
4. Replace virtual object assumptions with a real boundary-observation pipeline.
5. Validate Motive rigid bodies until robot poses are stable.
6. Run low-speed caging-only physical experiments after read-only and dry-run checks.

## Research Interpretation

A positive result means DBACT-style local boundary sensing and local allocation can produce useful caging behavior for unknown-shaped objects.

A weak result is also useful. It identifies which component should be improved next: local sensing, density shaping, CVT target allocation, safety filtering, or physical contact modeling.

The current repository is therefore designed to create reliable simulation evidence and a safe hardware-validation basis before claiming full real-world cooperative transport.

## License

This project is released under the MIT License. See [LICENSE](LICENSE) for details.
