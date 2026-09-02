# Notation

All spatial norms are Euclidean. Stacked vectors use the Euclidean product norm,
`||d||^2 = sum_i ||d_i||^2`.

| Symbol | Frozen meaning |
|---|---|
| `D` | fixed compact rectangle in `R^2`, width `W_D`, height `H_D` |
| `P=(p_1,...,p_N)` | stacked robot configuration in `R^(2N)` |
| `R_l`, `R_comm` | local integration and communication radii |
| `Gamma(t)`, `P_Gamma` | true boundary and its perimeter, `P_Gamma <= P_max` |
| `xi(s,t)` | true offset-boundary target |
| `nu_t*` | pushforward `xi(.,t)_# ds` of arc length |
| `K_sigma(x)` | `exp(-||x||^2/(2 sigma^2))`, not normalized |
| `phi*(q,t)` | `phi_0 + K_sigma * nu_t*` |
| `phihat_i(q,t)` | robot-i density after mapping, restriction, and weights |
| `V_i^l(P)` | `Vor_i(P) intersect D intersect B(p_i,R_l)` |
| `m_i*`, `c_i*` | ideal mass and centroid of `V_i^l` under `phi*` |
| `chat_i` | implementation midpoint-grid centroid |
| `H*(P,t)` | saturated limited-range cost frozen in the scope file |
| `C*(t)` | selected local minimizer branch/set inside regular tube `X` |
| `V(P,t)` | `H*(P,t)-H_min*(t)` |

## Kernel constants

\[
\|K_\sigma\|_1=2\pi\sigma^2,\qquad
\|\nabla K_\sigma\|_1=\sqrt2\,\pi^{3/2}\sigma,\qquad
\|\nabla K_\sigma\|_\infty=e^{-1/2}/\sigma.
\]

## Units

With arc length measured in metres, `K_sigma` dimensionless, and `phi_0` in
metres, `phi` has units m, density `L1` errors and cell masses have units m^3,
shifted first moments have units m^4, `H` has units m^5, `grad H` has units m^4,
and command disturbances have units m/s.  The ledger records these units rather
than silently mixing geometric length and density mass.
