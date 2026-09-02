# Proposition 3 - reference rate and local regularity

## Reference-rate statement

For `xi=b+dn`, the product rule gives

\[
\dot\xi=\dot b+\dot d\,n+d\dot n.
\]

Under the rigid-body A3 certificate,

\[
v_{\xi,\max}=\bar v_O+\bar\omega_O(R_O+d_{\max}+L_d),
\]

where version 1 takes constant offset, `L_d=0`. The convolution translation
inequality then yields

\[
\nu_\phi:=\|\partial_t\phi^*\|_1
\le L_{K,1}P_{\max}v_{\xi,\max}.
\tag{P3-a}
\]

Because `0<=min_i F_R<=R_l^2`,

\[
|\partial_tH^*(P,t)|\le R_l^2\nu_\phi.
\]

Let `H_min*(t)=min_{P in X}H*(P,t)` for fixed compact `X`. The elementary
inequality `|min f-min g|<=sup|f-g|` makes the optimal value Lipschitz with the
same bound almost everywhere. Therefore

\[
|\dot H_{\min}^*(t)|\le R_l^2\nu_\phi,\qquad
\boxed{\nu_H=2R_l^2\nu_\phi.}
\tag{P3-b}
\]

This proof does not cover deformation, changing perimeter, or split/merge
events. Those require a direct measure-rate assumption.

## Local regularity statement

On the symmetry-reduced convex chart/tube in A8, a uniform bound
`nabla^2 H_red* >= m_H I` implies strong convexity. Integrating the Hessian along
segments from the selected minimizer gives quadratic growth, and the standard
strong-convexity gradient inequality gives the PL bound:

\[
{1\over2}\|\nabla H^*\|^2\ge\mu(H^*-H_{\min}^*),\qquad
H^*-H_{\min}^*\ge{\alpha\over2}\operatorname{dist}^2(P,C^*(t)),
\]

with `mu=alpha=m_H` (or smaller certified constants).

This implication is proved, but its premise A8 is **CONDITIONAL** here: no
specific nondegenerate branch, cell combinatorics, reduced gauge, and complete
parameter tube were supplied. Wolfram verified the generic quadratic algebra;
there is no interval certificate and no floating-point eigenvalue is presented
as proof.
