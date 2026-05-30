# DBACT: Decentralized Boundary-Aware Cooperative Transportation

DBACT is a research-oriented Python framework for cooperative multi-robot transport of unknown, arbitrary-shaped objects. It studies how a robot team can discover an object's boundary from local observations, self-allocate around the object, maintain safety, and produce coordinated caging / pushing behavior without relying on prior object geometry.

The repository contains a runnable simulation stack, a modular controller implementation, scenario configurations, tests, documentation, and an adapter path toward an OptiTrack + RoboMaster S1 multi-robot platform.

```text
local boundary sensing
  -> boundary-aware density construction
  -> local CVT-based allocation
  -> local CBF-style safety filtering
  -> caging / pushing transport dynamics
```

## Highlights

- **Unknown-object operation**: the controller is structured around local boundary observations rather than direct access to global object shape.
- **Boundary-aware coordination**: boundary points are offset along outward normals to create cage targets, then converted into a density field for spatial allocation.
- **Decentralized information flow**: each agent uses its own observations plus neighbor-shared observations within communication range.
- **Local safety filtering**: velocity commands pass through a lightweight CBF-style projection layer for inter-agent separation.
- **Simulation-to-platform bridge**: `src/mas_adapter` connects the DBACT controller to a MAS-public-style control interface for staged RoboMaster S1 integration.
- **Reproducible scenarios**: circle, rectangle, L-shape, non-convex, caging, and multi-object configurations are included under `configs/sim`.

## Repository Structure

```text
.
|-- src/dbact/                 # Core DBACT algorithm modules
|-- src/dbact_sim/             # Simulation environment, scenarios, metrics, visualization
|-- src/mas_adapter/           # MAS-public compatible controller adapter
|-- configs/sim/               # Simulation scenario YAML files
|-- configs/mas/               # DBACT configuration for MAS-style integration
|-- scripts/                   # Batch runs and mock MAS pipeline utilities
|-- tests/                     # Root DBACT unit and smoke tests
|-- docs/                      # Architecture, algorithm notes, integration roadmap
`-- platforms/mas_public/      # OptiTrack + RoboMaster S1 platform subtree
```

## Installation

Python 3.9 or newer is supported. Python 3.10+ is recommended for development.

```bash
python -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate

python -m pip install --upgrade pip
pip install -r requirements.txt
pip install -e .
```

For a minimal editable install with only package metadata dependencies:

```bash
pip install -e .[dev]
```

## Quick Verification

```bash
python -m pytest
python -m compileall -q src tests scripts platforms/mas_public/src platforms/mas_public/apps
```

Expected root test status:

```text
6 passed
```

## Run a Simulation

```bash
python -m dbact_sim.run_sim \
  --config configs/sim/l_shape.yaml \
  --steps 400 \
  --output runs/l_shape
```

The simulator writes trajectories, metrics, and plots:

```text
runs/l_shape/
|-- trajectories.csv
|-- metrics.json
|-- final_snapshot.png
`-- trajectory.png
```

Run the default scenario batch:

```bash
python scripts/run_all_scenarios.py
```

Useful single-scenario commands:

```bash
python -m dbact_sim.run_sim --config configs/sim/circle.yaml --steps 300 --output runs/circle
python -m dbact_sim.run_sim --config configs/sim/rectangle.yaml --steps 400 --output runs/rectangle
python -m dbact_sim.run_sim --config configs/sim/l_shape.yaml --steps 500 --output runs/l_shape
python -m dbact_sim.run_sim --config configs/sim/nonconvex.yaml --steps 500 --output runs/nonconvex
python -m dbact_sim.run_sim --config configs/sim/multi_object.yaml --steps 600 --output runs/multi_object
```

## Mock MAS Pipeline

Before running on real hardware, the MAS adapter can be validated with a virtual object and a mock world state:

```bash
python scripts/run_mock_mas_pipeline.py \
  --steps 80 \
  --dt 0.05 \
  --print-every 20 \
  --output runs/mock_mas_pipeline
```

This exercises the integration path:

```text
mock WorldState
  -> DecentralizedTransportController
  -> ObjectObserver virtual object
  -> DBACTController
  -> planar velocity commands
  -> integrated mock WorldState
```

Expected outputs:

```text
runs/mock_mas_pipeline/
|-- states.csv
|-- commands.csv
`-- mock_trajectory.png
```

## Core Modules

| Module | Purpose |
| --- | --- |
| `cargo.py` | Polygon cargo representation for simulation and sensing |
| `local_sensing.py` | Local boundary observations within sensor range |
| `boundary_map.py` | Per-agent local memory and neighbor observation exchange |
| `boundary_density.py` | Gaussian density over boundary-offset cage targets |
| `local_cvt.py` | Local weighted CVT centroid approximation |
| `local_cbf_qp.py` | Lightweight CBF-style velocity projection |
| `transport_dynamics.py` | Simplified caging / pushing object dynamics |
| `controller.py` | End-to-end DBACT controller pipeline |
| `metrics.py` | Coverage, path length, recruitment, and safety metrics |

## Algorithm Notes

For each local boundary observation `(b, n_out)`, DBACT creates a cage target outside the object:

```text
q_target = b + d_cage * n_out
```

The controller then constructs a Gaussian density field around these targets and computes a local weighted CVT centroid using only the robot and communication-range neighbors. Inter-agent safety is enforced with the constraint:

```text
h_ij = ||p_i - p_j||^2 - d_min^2 >= 0
```

The current implementation uses an iterative half-plane projection as a lightweight CBF-style filter. This keeps the default simulator dependency-light while preserving a clean path toward a formal QP solver.

## MAS / RoboMaster S1 Integration

The root DBACT package remains hardware-independent. Hardware-facing functionality is isolated in the platform subtree and adapter layer:

```text
src/mas_adapter/decentralized_transport_controller.py
  MAS-public-style DBACT controller wrapper

src/mas_adapter/object_observer.py
  virtual object observer now; real object observer later

platforms/mas_public/
  OptiTrack input, ZeroMQ messaging, controller process,
  robot communication, logging, plotting, and supervisor orchestration
```

Recommended integration sequence:

1. Validate the root DBACT tests and simulations.
2. Run `scripts/run_mock_mas_pipeline.py` in virtual-object mode.
3. Register the adapter in the MAS-public controller stack.
4. Validate MAS mock IO.
5. Move to OptiTrack / RoboMaster S1 experiments with physical safety procedures in place.

Detailed notes:

- `docs/MAS_INTEGRATION.md`
- `docs/stage2_mas_virtual_object.md`
- `platforms/mas_public/README.md`

## Development Health Checks

Parse all YAML configurations:

```bash
python - <<'PY'
from pathlib import Path
import yaml

paths = list(Path("configs").rglob("*.yaml"))
paths += list(Path("platforms/mas_public/configs").rglob("*.yaml"))

for path in paths:
    yaml.safe_load(path.read_text(encoding="utf-8"))
    print("ok", path)
PY
```

Platform-specific tests live in `platforms/mas_public/apps/pytest_tests`. They require platform dependencies such as `pandas` and `pyzmq`; hardware-facing dependencies are listed in `platforms/mas_public/requirements.txt`.

## Current Status

Implemented:

- Arbitrary polygon cargo simulation
- Local boundary sensing and local boundary memory
- Boundary-aware density construction
- Local CVT-based spatial allocation
- Local CBF-style safety filter
- Simplified caging / pushing dynamics
- MAS adapter import tests and mock pipeline tests
- OptiTrack + RoboMaster S1 platform subtree

Open research and engineering work:

- Replace the lightweight projection filter with a formal QP solver
- Add boundary gap detection and adaptive recruitment
- Estimate object pose from accumulated local boundary memory
- Add nonholonomic robot and contact-rich physical dynamics
- Complete real-hardware DBACT experiments on the MAS / RoboMaster S1 stack

## Documentation

- `docs/ARCHITECTURE.md`: package layout and data flow
- `docs/ALGORITHM.md`: algorithm notes
- `docs/MAS_INTEGRATION.md`: MAS-public integration plan
- `docs/stage2_mas_virtual_object.md`: virtual-object MAS pipeline progress
- `docs/ROADMAP.md`: staged project roadmap

## License

MIT License.
