# Architecture

DBACT is split into three layers:

```text
src/dbact      core algorithm modules
src/dbact_sim  standalone simulation and visualization
src/mas_adapter integration layer for MAS-public / RoboMaster S1
```

## Core Modules

- `cargo.py`: arbitrary-shape cargo represented as polygon.
- `local_sensing.py`: local boundary observation inside sensor range.
- `boundary_map.py`: per-agent local memory and neighbor-shared observations.
- `boundary_density.py`: Gaussian density around boundary-offset cage targets.
- `local_cvt.py`: local weighted CVT approximation.
- `local_cbf_qp.py`: local CBF-style safety filter.
- `transport_dynamics.py`: simplified caging-pushing object dynamics.
- `controller.py`: high-level DBACT controller.

## Data Flow

```text
AgentState + Cargo polygon
  -> LocalBoundarySensor
  -> LocalBoundaryMap
  -> BoundaryAwareDensity
  -> LocalCVT centroid
  -> LocalCBFQP filter
  -> ControlCommand
```
