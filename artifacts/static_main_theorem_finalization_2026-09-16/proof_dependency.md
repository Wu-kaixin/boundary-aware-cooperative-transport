# Proof dependency

Edges that are **not** dependencies: measured fail-frame counts, measured max \(b\), measured \(\lVert\Pi_F(0)\rVert\), trajectory \(\bar E\) as \(B_E\), \(J_{\mathrm{proxy}}=\sum\lVert u_i\rVert^2\).

| ID | Claim | Depends on | Proof | Code | Check |
|---|---|---|---|---|---|
| P0 | Checkable initial set + (RT) | model params | `main_theorem.md` §0 | `assert_theorem_params`, spawn | P0 dump in priors JSON |
| RT | \(R_{\mathrm{row}}-u_{\max}\Delta\ge r_{\mathrm{safe}}+\rho/\gamma\) | P0 scalars | `barrier_invariance_derivation.md` A3 | `range_truncation_holds` | `test_paper_range_truncation_numbers_hold_and_are_the_filter_contract` |
| L-seg | Convex dist-to-segment CBF | convex analysis, \(p\notin F\), \(\gamma\Delta\le 1\), cap inactive | same A1 | `_nearest_feature_rows` | `test_nearest_feature_flat_edge`, `test_feature_cover_discrete_inequality_across_a_convex_corner` |
| L-cover | Φ contains one-step nearest edges in range | 1-Lipschitz | same | `_feature_cover_reach` \(=2u_{\max}\Delta\) | `test_feature_cover_includes_adjacent_edge_near_a_convex_corner` |
| L-range | Range-excluded edges stay above margin | (RT), 1-Lipschitz | A3 | `object_row_range` vs `max_speed*dt` | same range-truncation test |
| L-margin | \(d_\partial\ge r_{\mathrm{safe}}+\rho/\gamma\) on the hold | L-seg, L-cover, L-range | A1–A3 | `object_row_mode=nearest_feature` | unit cover tests; not a trajectory count |
| L-cross | No first entry into \(S\) | L-margin, continuity, P0 exterior | A4 | signed distance positive outside | first-crossing is a proof, not a monitor |
| L-0obj | \(0\in F_\rho^{\mathrm{object}}\) ⇔ \(h_{\mathrm{true}}\ge\rho/\gamma\) (or empty Φ) | L-margin, L-cross | A5 | `zero_input_feasible_with_rho` | `test_zero_in_F_rho_iff_true_h_meets_rho_over_gamma` |
| L-agg | Aggregate infinite plane can jump | convex-corner geometry | `barrier_update_diagnosis.md` | `_aggregate_face` | C5/L11/L17 dumps (historical) |
| L-agent | Pairwise sample+hold | Wang–Ames–Egerstedt shared row, \(\gamma_a\Delta\le 1\) | `theorem_mode.py` | `_agent_rows` | pairwise tests |
| L-omit | Omitted pairs safe | \(d_{\min}+2u_{\max}\Delta\), dropout 0 | `omitted_pair_radius` | `omitted_neighbor_certificate` | hold-segment gate is a detector |
| L-wall | Hold in \(D\) | \(D\) rectangle | `wall_halfplanes` | `enable_wall_rows` | abort on out-of-domain |
| P-joint | Joint \(0\in F_\rho\), exact QP exists, \(\mathcal K_0\) | L-0obj, L-agent, L-wall, cap, P0 | `main_theorem.md` §3 | `filter_velocity` stack; `qp2d` | Stage C/D/E: implementation check only |
| L-reach | \(\lVert\xi-p\rVert\nleq R+s\sigma\) | AABB square | `aabb_disk_corner_reach` | `evaluated_source_reach` | `test_bbox_cull_does_not_imply_r_plus_s_sigma` |
| L-proj | Projection optimality, no duplicate extra | Euclidean projection on \(B(0,R)\) | `projection_error_lemma.md` | `moment_form_E_bound_with_mass_fallback`, `project_to_disk` | `test_projected_centroid_uses_raw_moments_and_disk_projection_optimality` |
| L-BE | \(B_E=\min(\)moment+fallback, \(4R^2M)\) | L-reach, L-proj, restrict, \(M_2\) | `assemble_prior` | priors JSON | `priors_recomputed/prior_screen.json` |
| L-star | (★) \(J\) bound | sat then QP, \(\mathcal K_0\) from P-joint, \(a>0\) | `discrete_dissipation_derivation.md` Thm C | `audit_discrete_dissipation.py` | in-K0 slacks recorded, not a proof |
| T-main | Safety + recursive feasibility + a priori \(J\) | P0, P-joint, L-margin, L-star, L-BE | `main_theorem.md` §5 | theorem_mode cascade | three-shape certificate beats geometry |

A6 (exact vs float) is a **label split**, not a lemma used by T-main: T-main is exact arithmetic. Float `optimal` flags are implementation observations.
