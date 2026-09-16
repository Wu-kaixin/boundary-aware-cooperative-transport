# Proof dependency

| ID | Claim | Depends on | Proof | Code | Check |
|---|---|---|---|---|---|
| P0 | Checkable initial set | model params | `main_theorem.md` §0 | `guarantees.py`, spawn in `scenarios.py` | P0 dump in priors JSON |
| L-plane | Fixed affine discrete CBF | P0, γΔ≤1, v_obj=0, no cap bind | `barrier_invariance_derivation.md` | `_finish_object_rows` | unit: flat edge |
| L-seg | Convex dist-to-segment CBF | convex analysis | same | `_nearest_feature_rows` | `test_nearest_feature_flat_edge` |
| L-cover | Φ contains one-step nearest edges | 1-Lipschitz | same | `_feature_cover_reach` = 2 u_max Δ | `test_feature_cover_includes_adjacent_edge` |
| L-obj | h_true invariant on the hold | L-seg, L-cover, cap inactive | same | `object_row_mode=nearest_feature` | `test_feature_cover_discrete_inequality_across_a_convex_corner` |
| L-0obj | 0∈F_ρ object ⇔ h_true≥ρ/γ | L-obj | same | `zero_input_feasible_with_rho` | `test_zero_in_F_rho_iff_true_h_meets_rho_over_gamma`; C5 geometry test |
| L-agg | Aggregate infinite plane can jump | geometry of convex corners | `barrier_update_diagnosis.md` | `_aggregate_face`, window W, face cosine | C5/L11/L17 diagnosis dumps |
| L-agent | Pairwise sample+hold | Wang–Ames–Egerstedt shared row | `theorem_mode.py` header | `_agent_rows` | `test_pairwise_barrier_is_maintained` |
| L-omit | Omitted pairs safe | d_min+2 u_max Δ | `omitted_pair_radius` | `theorem_mode.py` | hold-segment gate |
| L-wall | Hold in D | D rectangle convex | `wall_halfplanes` | `enable_wall_rows` | abort on out-of-domain |
| P-joint | 0∈F_ρ joint, QP exists | L-0obj, L-agent, L-wall, cap | `main_theorem.md` §3 | `filter_velocity` stack; `qp2d` | Stage C/D: 9600 optimal, empty K0-fail (implementation check) |
| L-reach | ‖ξ−p‖ ≰ R+sσ | AABB square | `edge_green_integral.aabb_disk_corner_reach` | `evaluated_source_reach` | `test_bbox_cull_does_not_imply_r_plus_s_sigma` |
| L-proj | Projection extra on trap/float | moment identity | `moment_form_E_bound_with_mass_fallback` | `project_to_disk` | `test_projected_centroid_moment_identity_charges_quadrature_once_more` |
| L-BE | B_E = min(moment+extra, 4R²M) | L-reach, L-proj, restrict, M2 | `apriori_centroid_bound.assemble_prior` | priors JSON | `priors_recomputed/prior_screen.json` |
| L-star | (★) J bound | sat then QP, K0, a>0 | `discrete_dissipation_derivation.md` | `audit_discrete_dissipation.py` | in-K0 slacks recorded, not a proof |
| T-main | Safety + recursive feasibility + a priori J | P0, P-joint, L-obj, L-star, L-BE | `main_theorem.md` §5 | theorem_mode cascade | matrix + independent seeds; C-shape J-vs-geom **fails** at rigorous level |

Edges that are **not** dependencies: measured fail-frame counts, measured max b, measured ‖Π_F(0)‖, trajectory Ē as B_E.
