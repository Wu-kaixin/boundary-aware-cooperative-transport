# Proposition 1 - ideal Local-CVT gradient and descent

## Statement

At every regular configuration covered by A1--A4 and A9,

\[
\nabla_{p_i}H^*(P,t)=2m_i^*(p_i-c_i^*).
\tag{P1-a}
\]

Thus the ideal Lloyd command satisfies

\[
u_i^*=-k_c(p_i-c_i^*)=-{k_c\over2m_i^*}\nabla_{p_i}H^*.
\]

For `A(P,t)=diag_i(k_c/(2m_i*) I_2)` and
`a_0=k_c/(2m_max)`, `A>=a_0 I`. With the reference frozen,

\[
\dot H^*=-\nabla H^{*T}A\nabla H^*\le-a_0\|\nabla H^*\|^2.
\tag{P1-b}
\]

## Proof

Partition `D` into the ordinary Voronoi cells and use
`F_R(r)=min(r^2,R_l^2)`. Away from the zero-measure tie and threshold sets,
the active integrand for robot `i` has derivative `2(p_i-q)` inside
`V_i^l` and zero where it is saturated.

Reynolds boundary terms cancel for two separate reasons. Across a shared
Voronoi face the two competing values of the minimum agree, so the equal and
opposite fluxes cancel. Across the moving circle `r=R_l`, the inner quadratic
value and outer saturated value are both `R_l^2`, so that flux also cancels.
The workspace boundary is fixed. Consequently only the volume derivative
remains:

\[
\nabla_{p_i}H^*=2\int_{V_i^l}(p_i-q)\phi^*(q,t)dq
=2m_i^*(p_i-c_i^*),
\]

which proves (P1-a). Since Lemma 2 gives `m_i*<=m_max`, every diagonal block of
`A` is at least `a_0 I_2`. Substitution of `Pdot=-A grad H*` gives (P1-b). QED.

Finally, if `q in B(p_i,R_l)` and robot `j` beats robot `i` at `q`, then
`||p_j-p_i||<=||p_j-q||+||q-p_i||<=2R_l<=R_comm`. Hence the communication
neighbor set contains every possible local competitor. This justifies equality
with the ideal local cell; it is not an approximation claim.

Wolfram checked the integrand derivative and an exact fixed-square example only.
Those checks are not substitutes for the moving-boundary cancellation argument.
