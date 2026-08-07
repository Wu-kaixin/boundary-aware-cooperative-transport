# Architecture

DBACT is split into three layers:

```text
src/dbact       core algorithm modules (paper-v1 focus)
src/dbact_sim   standalone simulation and visualization
src/mas_adapter hardware-agnostic integration (frozen for paper-v1)
platforms/      RoboMaster / OptiTrack stack (frozen; keep interfaces only)
```

## Core Modules

- `local_sensing.py` / `dbact_sim/raycast_sensor.py`: ray-cast local measurements `z_ik=(b̂,n̂,c,t)`; PCA normals.
- `boundary_map.py`: voxel dedup, confidence fusion, TTL + age decay.
- `boundary_density.py`: boundary-measure-induced density around cage targets `ξ=b+d_c n`.
- `local_cvt.py`: limited local CVT on `D ∩ B(p_i, R_ℓ)`.
- `distributed_cbf.py` (`local_cbf_qp.py` re-export): responsibility-splitting CBF + object-boundary CBF.
- `controller.py`: composes modules; baselines via `method` (`dbact|arm|oracle|no_cbf`).
- `transport_dynamics.py`: `scripted` or `pymunk` backends.
- `dbact_sim/rigid_body_world.py`: PyMunk planar contact world.
- `cargo.py` / `metrics.py` / `types.py`: geometry, paper metrics, shared datatypes.

## Data Flow

```text
Simulator polygon (ground truth, sensor only)
  -> RayCast / LocalBoundarySensor
  -> LocalBoundaryMap
  -> Boundary-measure Density
  -> Limited Local CVT centroid
  -> Distributed CBF-QP
  -> ControlCommand
  -> scripted | pymunk transport
```

## Paper experiments

Configs: `configs/sim/paper/`. Batch runner: `scripts/run_paper_matrix.py`.
