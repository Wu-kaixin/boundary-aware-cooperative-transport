# Proposition 2 - aggregate implementation disturbance

## Statement and proof

Let the realized stacked dynamics be

\[
\dot P=-A(P,t)\nabla H^*(P,t)+d(t).
\]

The centroid part of robot `i` differs from the ideal command by at most
`k_c epsilon_c,i` by Lemma 2. Write the remaining stacked deviations as ZOH,
numerical, tracking, and safety-filter components. The triangle inequality in
the frozen stacked Euclidean norm gives

\[
\boxed{\|d(t)\|\le\delta:=k_c\sqrt{\sum_i\epsilon_{c,i}^2}
+\epsilon_{\rm zoh}+\epsilon_{\rm num}+\epsilon_{\rm trk}
+\epsilon_{\rm sf}.}
\tag{P2}
\]

The production safety filter clips each robot command in Euclidean norm. If both
the nominal and filtered commands satisfy `||u_i||<=u_max`, then

\[
\|u_i^{sf}-u_i^{nom}\|\le2u_{\max},\qquad
\epsilon_{sf}\le2\sqrt N u_{\max}.
\]

The Python ledger also exposes the per-agent ceiling `2u_max`. For a hypothetical
componentwise box cap, the stacked Euclidean ceiling is instead
`2 sqrt(2N) u_max`; the norm convention is never inferred from the symbol alone.

The analytic ceiling is deliberately conservative. A measured tightening may
replace it only when its provenance and coverage interval are recorded. Any
theorem-mode gap/frontier/lead-side bias must be zero or appear as another term
in (P2). QED.
