# Stage 1 Results: Unknown Polygon Caging Baseline

## Environment

- OS: Windows
- Conda environment: dbact
- Python: 3.10.20
- Project: boundary-aware-cooperative-transport
- Controller mode: caging-only
- Transport bias: disabled
- Cargo prior in controller: disabled
- CBF scope: agent-agent only

## Results

| Scenario | Cargo Type | Agents | Steps | Final Coverage | Min Inter-Agent Distance |
|---|---:|---:|---:|---:|---:|
| baseline_unknown_polygon_caging | arbitrary polygon | 12 | 900 | 0.7625 | 0.3446 |
| one_rectangle_polygon_caging | rectangle polygon | 12 | 900 | 0.7000 | 0.3446 |
| one_nonconvex_polygon_caging | nonconvex polygon | 14 | 1000 | 0.90625 | 0.3393 |

## Key Claim

The caging behavior is achieved without direct use of cargo center, radius, polygon vertices, or closest-boundary queries inside the controller.

The controller uses only:

```text
local BoundaryObservation
neighbor agent states
boundary-aware density
local CVT
agent-agent CBF