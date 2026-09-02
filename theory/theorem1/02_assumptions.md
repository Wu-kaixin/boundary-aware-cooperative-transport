# Assumptions A1--A10

1. **A1 (domain/tube).** `D` is a fixed rectangle. The solution remains in a
   fixed compact configuration tube `X` for the interval claimed.
2. **A2 (separation/locality).** Pairwise separation is at least `d_s>0` and
   `R_comm >= 2 R_l`. The latter is fail-closed in theorem mode.
3. **A3 (boundary motion).** `Gamma(t)` is piecewise `C^2`, closed, and has
   perimeter at most `P_max`. Version 1 uses a rigid boundary with bounded twist.
4. **A4 (density).** `phi_0>0`, `sigma>0`, and the unnormalized Gaussian in the
   notation file is used.
5. **A5 (map certificate).** Weights are nonnegative. A runtime/offline
   certificate supplies `epsilon_b`, `epsilon_n`, `epsilon_d`, voxel size `v`,
   maximum arc step `h_s`, `E_w`, dropped mass `M_drop`, and tail separation
   `s_sigma`. The configured `density.influence_sigmas` equals `s_sigma`, so the
   `restrict()` tail is auditable. These are upper bounds, not estimates silently
   treated as exact.
6. **A6 (quadrature).** LocalCVT uses rectangular midpoint cells with maximum
   side length `h`; the certificate verifies `eta_m<m_min`.
7. **A7 (unmodelled density bias).** Gap/frontier/exploration and transport-side
   density biases are disabled in theorem mode, or their stacked command effect
   is included explicitly in `delta`.
8. **A8 (local regularity).** In a symmetry-reduced convex chart of the selected
   branch/tube, `nabla^2 H_red*(z,t) >= m_H I` uniformly, with `m_H>0`. No branch
   or interval enclosure was supplied in this task, so A8 is **CONDITIONAL**.
9. **A9 (nondegeneracy).** Voronoi ties and the `r=R_l` threshold occur only on
   zero-measure sets, and the solution stays on the same regular branch tube.
10. **A10 (implementation disturbance).** ZOH, numerical, tracking, and safety
    deviations from the ideal field have certified stacked-norm upper bounds.

## Status note

The analytic implication A1--A10 => Theorem 1 is proved. A5, A8, A9, and A10
are not facts about every run; they are explicit assumptions/certificate gates.
Consequently the result is a complete **conditional local theorem**, not an
unconditional system-success claim.
