<div align="center">

# DBACT: Decentralized Boundary-Aware Cooperative Transport

Reproducible decentralized multi-robot caging and transport experiments with metrics, reports, MAS dry-runs, and visualizations.

[English](README.md) | [繁體中文](README.zh-TW.md) | [日本語](README.ja.md)

![License](https://img.shields.io/badge/License-MIT-green.svg)
![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)
![Tests](https://img.shields.io/badge/Tests-16%20passed-brightgreen.svg)
![Version](https://img.shields.io/badge/Version-0.1.0-informational.svg)
![Visualization](https://img.shields.io/badge/Visualization-Matplotlib-orange.svg)
![Platform](https://img.shields.io/badge/Platform-MAS%20%7C%20RoboMaster%20S1-lightgrey.svg)

</div>

DBACT is a research prototype for **Decentralized Boundary-Aware Cooperative Transport**. It studies how multiple mobile robots can form a useful caging and transport structure around an object whose complete shape, center, radius, and required team size are not given to the controller.

The repository combines a standalone simulation stack, boundary-aware local control, metrics, GitHub-renderable visual artifacts, a MAS-compatible controller adapter, OptiTrack read-only tooling, and conservative RoboMaster S1 command smoke tests.

> This repository is a research prototype, not a completed physical transport product. Simulation and dry-run paths are working; full physical transport remains a staged validation target.

---

## Visual Showcase

![DBACT moving cargo demo](docs/assets/dbact-moving-cargo.gif)

> A tracked GIF artifact generated from simulation replay. Unlike local `runs/*.gif` files, this file is committed under `docs/assets/`, so it renders directly on GitHub.

![DBACT density and local CVT frame](docs/assets/dbact-density-cvt-frame.png)

> Paper-style frame showing unknown cargo, local CVT / Voronoi structure, robot safety regions, and boundary-aware density surface.

---

## Media Gallery

All inline media below points to committed repository files under `docs/assets/`, so GitHub can render them without local run artifacts.

| Animation | Density + Local CVT |
| --- | --- |
| <img src="docs/assets/dbact-moving-cargo.gif" alt="DBACT moving cargo animation" width="100%"> | <img src="docs/assets/dbact-density-cvt-frame.png" alt="DBACT density and local CVT frame" width="100%"> |

| Trajectory | Coverage Curve |
| --- | --- |
| <img src="docs/assets/dbact-trajectory.png" alt="DBACT trajectory" width="100%"> | <img src="docs/assets/dbact-coverage-curve.png" alt="DBACT coverage curve" width="100%"> |

| Final Snapshot | Asset Manifest |
| --- | --- |
| <img src="docs/assets/dbact-final-snapshot.png" alt="DBACT final snapshot" width="100%"> | [`docs/assets/README.md`](docs/assets/README.md) |

Generated PNG, GIF, CSV, and MP4 artifacts are still produced under `runs/` or `platforms/mas_public/data/` by default and are intentionally ignored by Git. For GitHub display, copy selected figures into `docs/assets/`, or publish larger videos through GitHub Releases.

---

## Project Snapshot

| Item | Details |
| --- | --- |
| Project name | DBACT: Decentralized Boundary-Aware Cooperative Transport |
| Purpose | Test local boundary-aware caging and transport around unknown-shaped objects. |
| Core stack | Python 3.9+, NumPy, PyYAML, Matplotlib, pytest |
| Main scenarios | `paper_like_irregular_moving_cargo.yaml`, `l_shape.yaml`, `nonconvex.yaml`, `multi_object.yaml` |
| Output types | CSV trajectories, coverage metrics, JSON summaries, PNG figures, GIF animations |
| Integration path | DBACT simulation -> MAS dry-run -> OptiTrack read-only -> RoboMaster S1 smoke test |
| Current status | Simulation and dry-runs working; full physical experiment not yet complete |

---

## Features

- **Decentralized boundary-aware control**: robots use local boundary observations and neighbor states rather than global object geometry.
- **Unknown-object caging**: the controller avoids direct use of `cargo.center`, `cargo.radius`, `cargo.vertices`, and closest-boundary queries.
- **Local CVT allocation**: each robot computes a local weighted centroid using itself and nearby neighbors.
- **CBF-style safety filtering**: inter-agent distance constraints and velocity limits keep the caging process conservative.
- **Visualization-first workflow**: simulations export trajectories, coverage curves, final snapshots, paper-style frames, and optional GIF animations.
- **Hardware-oriented staging**: MAS adapter, OptiTrack read-only logging, and RoboMaster S1 smoke tests prepare a safe path toward real experiments.

---

## Results & Visualizations

### Stage 1 Unknown Polygon Caging

Stage 1 validates that caging can be formed around arbitrary polygonal cargo without direct complete-cargo access inside the controller.

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging` | arbitrary polygon | 12 | 900 | 0.7625 | 6 / 12 | 0.3446 m |
| `one_rectangle_polygon_caging` | rectangle polygon | 12 | 900 | 0.7000 | 6 / 12 | 0.3446 m |
| `one_nonconvex_polygon_caging` | nonconvex polygon | 14 | 1000 | 0.90625 | 9 / 14 | 0.3393 m |

### Tight Baseline Results

The tight baseline improves caging compactness by reducing cage offset and narrowing the density field.

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Recruited Agents | Min Inter-Agent Distance |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `baseline_unknown_polygon_caging_tight` | arbitrary polygon | 12 | 900 | 0.95625 | 11 / 12 | 0.3450 m |
| `one_rectangle_polygon_caging_tight` | rectangle polygon | 12 | 900 | 0.99375 | 9 / 12 | 0.3450 m |
| `one_nonconvex_polygon_caging_tight` | nonconvex polygon | 14 | 1000 | 0.9750 | 13 / 14 | 0.3370 m |

### Moving Irregular Cargo Demo

| Metric | Value |
| --- | ---: |
| Final time | 25.95 s |
| Final coverage | 0.83125 |
| Cargo displacement | 1.539 m |
| Recruited agents | 6 |
| Minimum inter-agent distance | 0.3571 m |
| Mean path length | 4.6999 m |

**Interpretation**

- Tight caging improves boundary coverage while maintaining minimum inter-agent distance above 0.33 m in the reported Stage 1 benchmarks.
- The moving-cargo demo shows caging and transport-like displacement, but physical contact dynamics are still simplified.
- Current results are simulation evidence and MAS dry-run evidence, not a final claim of full real-world transport.

---

## Quick Start

### 1. Clone

```bash
git clone https://github.com/Wu-kaixin/boundary-aware-cooperative-transport.git
cd boundary-aware-cooperative-transport
```

### 2. Create an Environment

Conda:

```bash
conda create -n dbact python=3.10
conda activate dbact
python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .[dev]
```

Windows PowerShell virtual environment:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -U pip
python -m pip install -e ".[dev]"
```

macOS / Linux virtual environment:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -e ".[dev]"
```

### 3. One-line Smoke Experiment

```bash
python -m dbact_sim.run_sim --config configs/sim/paper_like_irregular_moving_cargo.yaml --steps 520 --output runs/paper_like_irregular_moving_cargo --animate
```

Important outputs:

- `runs/paper_like_irregular_moving_cargo/animation.gif`
- `runs/paper_like_irregular_moving_cargo/trajectory.png`
- `runs/paper_like_irregular_moving_cargo/final_snapshot.png`
- `runs/paper_like_irregular_moving_cargo/coverage_rate_curve.png`
- `runs/paper_like_irregular_moving_cargo/metrics.json`
- `runs/paper_like_irregular_moving_cargo/figures/FIG_520.png`

---

## How It Works

1. **Load scenario configuration**
   YAML files define domain size, cargo geometry, robot initial states, sensing range, communication range, transport direction, and controller parameters.

2. **Generate local boundary observations**
   The simulator uses cargo geometry to generate local boundary observations, but the controller does not directly consume full cargo shape.

3. **Create cage targets**
   Each observed boundary point `b` is shifted outward by a cage offset:

```text
q_target = b + d_cage * n_out
```

4. **Build boundary-aware density**
   Cage targets become Gaussian density peaks that attract robots toward useful boundary-adjacent locations.

5. **Run local CVT allocation**
   Each robot computes a local weighted centroid using itself and neighbors within communication range.

6. **Apply safety filtering**
   The CBF-style filter keeps inter-agent distance above the configured minimum:

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

7. **Save replay, metrics, and figures**
   Simulation runs write CSV logs, metrics, final snapshots, trajectory plots, coverage curves, paper-style frames, and optional animations.

---

## Repository Structure

```text
boundary-aware-cooperative-transport/
|-- configs/                         # Simulation and MAS configuration
|   |-- sim/
|   `-- mas/
|-- src/
|   |-- dbact/                       # Core controller, sensing, density, CVT, safety, metrics
|   |-- dbact_sim/                   # Simulation environment, scenarios, visualization, CLI
|   `-- mas_adapter/                 # MAS-compatible controller adapter
|-- scripts/                         # Batch runs, mock MAS pipeline, RoboMaster S1 smoke tests
|-- docs/                            # Architecture, algorithm notes, reports, staged validation
|   |-- assets/                      # Tracked GitHub-renderable README media
|   |-- ARCHITECTURE.md
|   |-- ALGORITHM.md
|   |-- MAS_INTEGRATION.md
|   `-- stage1_results.md
|-- platforms/mas_public/            # Vendored MAS platform code
|-- runs/                            # Local generated runs, ignored by Git
|-- tests/                           # Unit and smoke tests
|-- README.md
|-- README.en.md
|-- README.zh-TW.md
`-- README.ja.md
```

---

## Useful Commands

Run standard scenarios:

```bash
python scripts/run_all_scenarios.py --steps 400 --animate
```

Run an L-shape scenario:

```bash
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 400 --output runs/l_shape
```

Run the mock MAS pipeline:

```bash
python scripts/run_mock_mas_pipeline.py --steps 80 --dt 0.05 --print-every 20 --output runs/mock_mas_pipeline
```

Run MAS dry-run:

```bash
cd platforms/mas_public
python apps/dbact/run_dtransport_dry_run.py --steps 80 --dt 0.05 --print-every 20 --output data/dry_runs/dtransport_auto_init --clamp-to-world-bounds
```

Run OptiTrack read-only check:

```bash
cd platforms/mas_public
python apps/dbact/log_optitrack_world_state.py --mock --frames 50 --hz 100 --print-every 10 --output data/optitrack_readonly/mock_world_states.csv
```

Run RoboMaster S1 mock command smoke test:

```bash
python scripts/run_seven_s1_cvt_test.py --duration 3
```

Run tests:

```bash
python -m pytest -q tests
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
python -m pytest -q --rootdir platforms/mas_public platforms/mas_public/apps/pytest_tests
```

---

## Current Research Direction

The next useful work is staged validation, not broad feature expansion. Priority items are:

- preserve `docs/assets/` as the stable GitHub media surface;
- add side-by-side scenario comparison figures;
- improve moving-cargo transport metrics and dashboard summaries;
- replace virtual-object assumptions with a real boundary-observation pipeline;
- validate Motive rigid bodies until robot poses are stable;
- run low-speed caging-only physical experiments only after read-only logging and dry-runs pass.

---

## Safety Notes

- Run read-only OptiTrack logging before enabling controller output.
- Verify robot ID to rigid-body mapping one robot at a time.
- Use very low speed limits for the first physical run.
- Keep a physical emergency stop available during hardware tests.
- Inspect command and state logs after every run.

---

## Contributing & License

Contributions are welcome through Issues and Pull Requests. New scenarios, clearer visualizations, stronger metrics, and better staged hardware validation are especially useful.

This project is released under the [MIT License](LICENSE).
