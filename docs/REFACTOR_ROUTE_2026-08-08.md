# DBACT paper-driven refactor route and implementation status

## Decision order

The project now follows this order:

1. Literature audit
2. PyMunk L-shape decision spike
3. Title and venue decision
4. Local sensing
5. Boundary map
6. Boundary-measure density
7. Strict limited local CVT
8. Hard distributed CBF
9. Physics-based transport
10. Reproducible experiment matrix

This reverses the old risk order: paper identity is decided before most implementation work.

## Gate results

| Gate | Evidence | Result |
| --- | --- | --- |
| Literature intersection | `docs/LITERATURE_AUDIT_2026-08-08.md` | No complete direct predecessor located; novelty must be claimed as a coupled construction |
| Mechanical transport spike | `scripts/spike_pymunk_l_shape.py` | PASS: 12 point robots moved an L shape 1.920 m with negligible lateral drift/yaw |
| Paper identity | Spike + end-to-end L-shape result | Keep contact-driven cooperative transport; do not reduce the work to enclosure only |
| Venue | Theory/evidence fit | L-CSS regular submission; RAS extension/fallback |

## Implemented module contracts

### 1. Local sensing

File: `src/dbact/local_sensing.py`

- Nearest-intersection ray casting across all cargoes handles self-occlusion and inter-object occlusion.
- Local PCA estimates tangents and signs normals using the robot-outside convention.
- Confidence combines range, PCA planarity and local support; it is no longer identically one.
- BLAKE2-based frame seeds replace Python's process-randomized string hash.
- Only the simulator sensor reads true polygons. The controller receives `BoundaryObservation` records.

### 2. Boundary map

File: `src/dbact/boundary_map.py`

- Spatial voxel upsert prevents relay duplication from multiplying density mass.
- Re-observation refreshes the timestamp even when an older sample has higher confidence.
- The 160-entry cap now applies to unique voxels, not raw packets; repeated 24-point frames no longer erase the TTL horizon in 0.1 s.
- Confidence/age fusion and robust translation compensation prevent moving cargo from leaving a stale world-frame density trail.

### 3. Boundary-measure density

File: `src/dbact/boundary_density.py`

`phi_i(q,t) = phi_0 + sum_k Delta s_k c_k exp(-lambda age_k) (1 + kappa g_k) K_sigma(q-xi_k)`

The total discrete mass follows observed boundary length rather than packet count. Gap weighting is an ablation-controlled term.

### 4. Strict local CVT

File: `src/dbact/local_cvt.py`

- Integration domain is exactly `D intersect B(p_i, R_l)`.
- Samples are anchored to a world grid with fixed `grid_spacing` in metres.
- Paper and simulation configs use `grid_spacing: 0.08`; variable bounding-box resolution has been removed from the paper controller.

### 5. Hard distributed CBF

File: `src/dbact/distributed_cbf.py`

- Pairwise responsibility splitting removes dependence on neighbor control inputs.
- Safety slack variables are deleted.
- The optimization is a true QP with an axis-aligned input box; OSQP is requested explicitly.
- The controller verifies returned constraint residuals. For the static safe-set assumptions, certified `u=0` is used if a numerical projection would violate constraints.
- Object constraints include boundary estimation margin, optional boundary-speed bound, and a contact allowance used only with the rigid contact backend.

Static safe-set feasibility certificate:

- If every `h_ij >= 0` and `h_iO >= 0`, `u=0` satisfies `2(p_i-p_j)^T u + (gamma/2)h_ij >= 0` and `n_hat^T u + alpha h_iO >= 0`.
- The input box contains zero; therefore the local hard QP is feasible without slack.
- Moving-boundary feasibility is conditional on the disturbance bound and input authority; it must not be presented as unconditional.

### 6. Contact transport

Files: `src/dbact_sim/rigid_body_world.py`, `src/dbact/transport_dynamics.py`

- Concave cargo polygons are ear-clipped into convex PyMunk shapes attached to one dynamic body.
- Kinematic agent states are advanced once by the physics world (the earlier double-integration bug is removed).
- Paper transport uses a two-stage task: enclosure first, then a common task velocity `v_d` after an explicit high-level activation time.
- The local map compensates cargo translation while rigid-body contact supplies all object displacement.

## Validation results

Environment available to this run: Python 3.12 runtime with project dependencies installed in an isolated workspace target. The requested `conda dbact` executable was not exposed in the hosted container, so a reproducible `environment.yml` is supplied for the user's Python 3.10 conda environment.

### Tests

- `33 passed`
- Coverage: reproducible sensing seed, L-shape occlusion, TTL-effective voxel memory, motion compensation, density weights, fixed-grid local CVT, hard-QP constraints, concave-body physics, controller/MAS smoke tests.

### Mechanical L-shape spike

- Forward displacement: 1.9198 m
- Lateral drift: approximately 0.000011 m
- Final yaw magnitude: approximately 0.000022 rad
- Result: PASS

### End-to-end unknown concave object

Config: `configs/sim/paper/pymunk_l_shape_transport.yaml`, 300 steps, seed 1.

- Final boundary coverage: 0.5125
- Cargo displacement: 0.6230 m
- Enclosure time: 4.10 s
- Minimum inter-agent distance: 0.3056 m (`d_min=0.28 m`)
- CBF calls: 3600
- Hard-QP infeasible calls: 0
- Mean CBF solve time: about 6.97 ms/call
- Scenario success: true

These are engineering validation results, not yet a statistically sufficient paper table.

## Experiment infrastructure

- Config-driven multi-seed runner: `scripts/run_paper_matrix.py`
- Per-run CSV and JSON outputs, aggregated JSON, always-written long CSV, optional Parquet, and a run manifest.
- One-command figure: `scripts/plot_paper_matrix.py`
- Baselines: ARM-style detecting-agent density, oracle boundary, no-CBF, full DBACT.
- Ablations: packet dropout, normal error, gap weighting/map sharing through YAML switches.

## Remaining paper gates

1. Prove or precisely state the slow-time-varying density/map-disagreement practical-stability result. Do not promote frozen-map Lloyd descent beyond a lemma.
2. Run the preregistered large matrix (at least 10-30 seeds per cell, N=8/12/16/32, convex and concave random shapes) and report confidence intervals/effect sizes.
3. Add a rotation-aware map compensation ablation; current compensation estimates translation only.
4. Verify the same commands in the user's `conda dbact` Python 3.10 environment using `environment.yml`.
5. Complete a database-level novelty audit before writing any "first" claim.

## Reproduction commands (conda dbact)

```bash
conda env update -n dbact -f environment.yml
conda run -n dbact python -m pytest -q
conda run -n dbact python scripts/spike_pymunk_l_shape.py --output runs/spikes/pymunk_l_shape
conda run -n dbact python -m dbact_sim.run_sim --config configs/sim/paper/pymunk_l_shape_transport.yaml --steps 300 --output runs/validation/pymunk_l_shape_transport
conda run -n dbact python scripts/run_paper_matrix.py --seeds 10 --steps 300 --output runs/paper_matrix
conda run -n dbact python scripts/plot_paper_matrix.py --input runs/paper_matrix/all_runs.csv --output runs/paper_matrix/method_comparison.png
```
