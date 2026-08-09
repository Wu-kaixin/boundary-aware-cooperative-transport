# Stage 6 — headless pipeline profile and optimization

## Result

On the fixed three-case runtime ablation, the optimized headless simulator runs
at **22.16–35.24 control frames/s** (mean **28.69 fps**, median **28.67 fps**).
The matched pre-optimization mean was 16.65 fps, so the measured mean gain is
72.26%.  These are wall-clock measurements on the development machine, not
portable timing guarantees.

| case | seed | baseline fps | final fps | change | final classification |
|---|---:|---:|---:|---:|---|
| circle | 0 | 24.30 | 35.24 | +45.05% | SUCCESS |
| concave random, 7 vertices | 2 | 14.42 | 28.67 | +98.82% | CONTRACT_FAILURE |
| polygon, 32 vertices | 2 | 11.25 | 22.16 | +97.00% | MAP_INCOMPLETE |

All three final episodes have hard-QP fallback = 0, infeasible = 0, and rho
relaxation = 0.  The two failed episodes remain failures: no eligibility,
coverage, rotation, or directional-efficiency threshold was reduced.

## What changed

- Batched voxel-key quantization replaces one `round/astype` allocation per
  relayed cell.
- Rigid map compensation transforms and re-keys a complete object map in array
  batches while retaining the original ordered collision-fusion rule.
- Each voxel caches its exact exponential-decay expiry time.  Pruning skips the
  complete stale scan until an expiry can actually have occurred and returns
  early whenever total cells fit the per-object capacity.
- Fused map reads use the map layer's maintained unit-normal invariant instead
  of normalizing every copied observation again.
- A relayed cell that cannot change timestamp, confidence, arc mass, or weight is
  ignored before fusion work.
- Complete planning-map snapshots run at a lower rate during independent sweep
  and boundary mapping.  They return to one exchange per perception update for
  the explicit rendezvous/gossip interval used by the discovery proof.
- Filtered Monte Carlo runs now use the shape's global matrix index, so a
  one-shape performance run exactly preserves its full-matrix position, yaw,
  density, and friction stratum.

Local ray perception continues every 3 physics frames and the safety QP still
runs every physics frame.  The raw local scan, not the multi-rate planning map,
defines object CBF rows.

## Reproduction

The local editable install pointed at a different Git worktree, so every retained
Stage-6 measurement explicitly bound imports to this branch:

```powershell
$env:PYTHONPATH=(Resolve-Path 'src').Path
python scripts/run_arbitrary_shape_monte_carlo.py --seeds 0 --shapes circle --max-steps 1500 --output runs/stage6_circle_final_v2
python scripts/run_arbitrary_shape_monte_carlo.py --seeds 2 --shapes concave_random7 --max-steps 1500 --output runs/stage6_concave7_final_v2
python scripts/run_arbitrary_shape_monte_carlo.py --seeds 2 --shapes polygon32 --max-steps 1500 --output runs/stage6_polygon32_final
```

The machine-readable result is
`docs/results/PERFORMANCE_STAGE6.json`.  cProfile traces and full per-frame run
directories remain ignored because they are large, reproducible intermediates.

## Interpretation and limitation

This stage establishes the requested **>20 fps** headless target on the simple,
concave, and high-vertex stress cases selected for the ablation.  It does not
prove a platform-independent runtime bound.  The preferred 30–40+ fps range is
reached for the circle case, not uniformly across arbitrary geometry.  More
important, performance does not imply correctness: the concave case still
violates directional/phase contracts and the 32-vertex case is still rejected
for an incomplete map (with excessive rotation also recorded by C3).
