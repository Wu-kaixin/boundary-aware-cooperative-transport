# Scope and frozen claim

## Headline claim

Bounded local-map disagreement and bounded reference variation imply practical
stability of the boundary-induced Local-CVT allocation dynamics around a
nondegenerate local CVT branch.

The theorem concerns only the continuous-time ENCLOSE allocation law

\[
\dot p_i=u_i^{\rm nom}=-k_c(p_i-\widehat c_i)
\]

and bounded deviations of the realized command from the ideal Lloyd field.  It
uses the saturated limited-range objective

\[
H^*(P,t)=\int_D\min_i\min\{\|q-p_i\|^2,R_l^2\}\,\phi^*(q,t)\,dq.
\]

The statement is local in a fixed compact regular tube `X` around a selected
local minimizer branch `C*(t)`. Its PL and quadratic-growth constants are not
asserted outside that tube.

## Deliberate exclusions

This package does **not** prove:

- global CVT optimality or arbitrary-shape global convergence;
- finite-time SEARCH-to-HOLD completion;
- geometric or topological caging;
- contact mechanics or unconditional cooperative-transport success;
- CBF forward invariance when feasibility, speed-cap, model-error, or
  safety-filter-deviation contracts are violated;
- a uniform reduced-Hessian bound for an unspecified CVT branch.

Collision safety belongs to a separate theorem/appendix and is not a dependency
of Theorem 1.
