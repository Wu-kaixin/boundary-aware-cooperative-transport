# README Visual Assets

This folder contains curated, Git-tracked visual assets used by the Simplified Chinese README.

Generated simulation outputs under `runs/` are intentionally ignored by Git, so README files should reference assets from this folder when the image or GIF must render on GitHub.

Current paper and closed-loop figures must come from the consolidation commit's traces. `paper_like_irregular_moving_cargo.yaml` is labelled "Not a paper configuration"; do not use those frames as the README showcase.

| File | Source | Purpose |
| --- | --- | --- |
| `closed_loop_d_seed2.gif` | `runs/readme/near_seed_2/closed_loop.gif` | Closed loop: discovery, enclosure, directional transport, stop. |
| `closed_loop_d_seed4.gif` | `runs/readme/near_seed_4/closed_loop.gif` | Closed loop, second sampled task direction. |
| `closed_loop_d_seed8.gif` | `runs/readme/near_seed_8/closed_loop.gif` | Closed loop, third sampled task direction. |
| `search_d_seed7.gif` | `runs/readme/search_seed7/closed_loop.gif` | Far-field lane sweep, token relay and convergence. |
| `d10-post-detection-stages.png` | `runs/readme/d10_diag/figA_segments.png` | D10-DIAG: where the frames between detection and contact-ready go. |
| `d10-coverage-and-gap.png` | `runs/readme/d10_diag/figB_coverage.png` | D10-DIAG: union map coverage, strict coverage, largest unobserved arc. |
| `d10-gate-tradeoff.png` | `runs/readme/d10_enc/figF_gate_tradeoff.png` | D10-ENC: transition delay against the enclosure each candidate gate certifies. |
| `static-deployment-comparison.png` | `scripts/build_consolidation_report.py` from `runs/paper/jeh_matrix` | Static oracle-map theory validation, L/rectangle/C, seed 2. |

Historical, not a paper configuration (`configs/sim/paper_like_irregular_moving_cargo.yaml`):

| File | Source | Purpose |
| --- | --- | --- |
| `dbact-moving-cargo.gif` | `runs/paper_like_irregular_moving_cargo/animation.gif` | Withdrawn coverage-era animation. Do not use as the README hero. |
| `dbact-density-cvt-frame.png` | `runs/paper_like_irregular_moving_cargo/figures/FIG_520.png` | Withdrawn paper-style frame. |
| `dbact-coverage-curve.png` | `runs/paper_like_irregular_moving_cargo/coverage_rate_curve.png` | Withdrawn coverage curve. |
| `dbact-trajectory.png` | `runs/paper_like_irregular_moving_cargo/trajectory.png` | Withdrawn trajectory still. |
| `dbact-final-snapshot.png` | `runs/paper_like_irregular_moving_cargo/final_snapshot.png` | Withdrawn final snapshot. |
