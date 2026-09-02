# Theorem 1 - local practical stability

## Statement

Assume A1--A10, Lemmas 1--2, and Propositions 1--3 on an interval on which the
solution remains in the regular tube `X`. Define

\[
V(P,t)=H^*(P,t)-H_{\min}^*(t),\quad
a_0={k_c\over2m_{\max}},\quad
\lambda_H=a_0\mu,
\]

\[
B={\delta^2\over2a_0}+\nu_H.
\]

Then, for every `t>=t_0` in that interval,

\[
\boxed{V(t)\le e^{-\lambda_H(t-t_0)}V(t_0)
+{B\over\lambda_H}(1-e^{-\lambda_H(t-t_0)})}
\tag{T1-a}
\]

and

\[
\boxed{\limsup_{t\to\infty}\operatorname{dist}(P(t),C^*(t))
\le\sqrt{{2B\over\alpha\lambda_H}}.}
\tag{T1-b}
\]

The limit statement applies when the regular-tube hypotheses continue for all
future time.

## Proof with every inequality identified

At almost every regular time,

\[
\begin{aligned}
\dot V
&=\nabla H^{*T}\dot P+\partial_tH^*-\dot H_{\min}^*\\
&\le-\nabla H^{*T}A\nabla H^*+\|\nabla H^*\|\,\|d\|+\nu_H
&&\text{(Propositions 1 and 3)}\\
&\le-a_0\|\nabla H^*\|^2+\delta\|\nabla H^*\|+\nu_H
&&\text{(Proposition 2 and }A\succeq a_0I\text{)}\\
&\le-{a_0\over2}\|\nabla H^*\|^2+{\delta^2\over2a_0}+\nu_H
&&\text{(Young)}\\
&\le-a_0\mu V+B=-\lambda_HV+B
&&\text{(local PL in Proposition 3).}
\end{aligned}
\]

The scalar comparison lemma for `Vdot<=-lambda_H V+B` yields (T1-a). Taking
`limsup` gives `limsup V<=B/lambda_H`. Local quadratic growth in Proposition 3
gives `dist(P,C*)<=sqrt(2V/alpha)`, hence (T1-b). QED.

## Status

The implication is analytically proved and its scalar algebra is Wolfram
verified. The theorem remains **CONDITIONAL** on the run/tube certificates and,
in particular, on A8. It is not a global PL theorem and says nothing about
formal caging or unconditional transport success.
