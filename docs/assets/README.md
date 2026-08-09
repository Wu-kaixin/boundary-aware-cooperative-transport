# README Visual Assets

This folder contains curated, Git-tracked visual assets used by the multilingual README files.

Generated simulation outputs under `runs/` are intentionally ignored by Git, so README files should reference assets from this folder when the image or GIF must render on GitHub.

| File | Source | Purpose |
| --- | --- | --- |
| `dbact-full-workspace-v3-500.gif` | `runs/full_workspace_visual_seed0/closed_loop_500.gif` | Certificate-eligible random polygon over a complete-domain 500-step run; 251 rendered states at stride 2. |
| `dbact-full-workspace-v3-500-final.png` | `runs/full_workspace_visual_seed0/final_snapshot.png` | Final state of the full-workspace seed-0 release artifact. |
| `dbact-closed-loop-v3-500.gif` | `runs/v3_demo_seed0/closed_loop_500.gif` | Initial state plus all 500 v3 search/enclosure/transport/hold steps. |
| `dbact-closed-loop-v3-500-final.png` | `runs/v3_demo_seed0/final_snapshot.png` | Seed-0 bounded transport final state. |
| `dbact-closed-loop-v3-500-sweep.png` | `runs/v3_sweep_12_final/closed_loop_sweep.png` | Phase deadlines, outcome gates, and throughput for twelve seeds. |
| `dbact-moving-cargo.gif` | `runs/paper_like_irregular_moving_cargo/animation.gif` | Hero animation for the project overview. |
| `dbact-density-cvt-frame.png` | `runs/paper_like_irregular_moving_cargo/figures/FIG_520.png` | Paper-style frame showing workspace, local CVT/Voronoi structure, and density surface. |
| `dbact-coverage-curve.png` | `runs/paper_like_irregular_moving_cargo/coverage_rate_curve.png` | Coverage curve for the moving irregular cargo scenario. |
| `dbact-trajectory.png` | `runs/paper_like_irregular_moving_cargo/trajectory.png` | Agent trajectories and final cargo state. |
| `dbact-final-snapshot.png` | `runs/paper_like_irregular_moving_cargo/final_snapshot.png` | Final simulation snapshot. |
