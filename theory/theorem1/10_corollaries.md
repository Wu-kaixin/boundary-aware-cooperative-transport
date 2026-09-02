# Corollaries

## Frozen perfect-information corollary

If all implementation errors vanish and the reference is frozen, then
`delta=nu_H=B=0`. Theorem 1 reduces to

\[
\dot V\le-\lambda_HV,\qquad
V(t)\le e^{-\lambda_H(t-t_0)}V(t_0),\qquad
\operatorname{dist}(P(t),C^*)\to0.
\]

The generator and tests fail if the all-zero/frozen input does not return
exactly `delta=nu_H=B=0`.

## Sampled-data corollary (M7)

Let

\[
P_{k+1}=P_k+\Delta t[-A_kg_k+d_k],\quad g_k=\nabla H^*(P_k,t_k),
\]

with `a_0 I<=A_k<=a_1 I`, `a_1=k_c/(2m_min)`, `||d_k||<=delta`, and a
spatially `L_H`-Lipschitz gradient. Define

\[
\bar a=a_0-{L_Ha_1^2\Delta t\over2},\quad
C_\delta=\delta(1+L_Ha_1\Delta t),
\]

\[
B_d={C_\delta^2\over2\bar a}+{L_H\over2}\delta^2\Delta t+\nu_H.
\]

If

\[
\Delta t<{2a_0\over L_Ha_1^2},\qquad0<\bar a\mu\Delta t\le1,
\]

then

\[
V_{k+1}\le(1-\bar a\mu\Delta t)V_k+B_d\Delta t
\]

and

\[
\limsup_{k\to\infty}\operatorname{dist}(P_k,C_k^*)
\le\sqrt{{2B_d\over\alpha\bar a\mu}}.
\]

### Proof

The descent lemma and reference-rate bound give

\[
V_{k+1}-V_k\le\Delta t[-a_0x^2+\delta x]
+{L_H\Delta t^2\over2}(a_1x+\delta)^2+\nu_H\Delta t,
\quad x=\|g_k\|.
\]

Expanding produces
`Delta t[-abar x^2+C_delta x+(L_H/2)delta^2 Delta t+nu_H]`.
Young's inequality leaves `-(abar/2)x^2+B_d`; local PL produces the stated
recursion. Summing its geometric series and applying quadratic growth proves the
ultimate-radius formula. Wolfram checked the expansion and coefficient gates.
