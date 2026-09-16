# Observer error versus dissipation anomalies

Two different objects.

## Controller-error certificate

\(B_E\) bounds \(\sum m_i^\star\|\hat c_i-c_i^\star\|^2\) from model, geometry, and P0. It does not include polar-observer quadrature. Status: rigorous (moment form + fallback, projection optimality, legal source reach).

## Evaluation observer

`UniformOffsetObserver` (polar offset grid, default `ntheta=128`, `nradial=12`) estimates \(H^\star,g^\star,m^\star,c^\star\) used in the JEH matrix. That remainder is `numerical_a_priori`. It is not \(B_E\) and is not a strict covering certificate of (★).

## Dissipation slacks

The discrete identity compared in `run_fresh_case` is

\[
\Delta H - \bigl(\Delta {g^\star}^\top U + \Delta^2 \sum m^\star\|U\|^2\bigr).
\]

Positive slack of size \(\sim 10^{-5}\) appears on in-\(\mathcal K_0\) frames (example: L-shape seed 5, frame 142, slack \(1.38\cdot 10^{-5}\), `in_K0=true`). That scale matches polar quadrature, not a sign-definite violation of (★). Frames with slack \(>10^{-6}\) are listed as `dissipation_anomaly_frames` and remain **undetermined in sign** at the observer envelope. They are not counterexamples and not a successful strict numerical covering.

Constraint dumps are written only on K0 failure or slack \(>10^{-3}\), so observer-scale slacks do not saturate disk I/O.

## Labels

| Quantity | Label |
|---|---|
| Certificate \(B_E\), \(B_{H0}=R^2M_{\mathrm{plane}}\), \(B_{J,\mathrm{prior}}\) | rigorous (exact arithmetic) |
| Polar \(H^\star(P_0)\), P0H column | numerical_a_priori |
| Trajectory \(\bar J,\bar E,\bar H\) | post_hoc numerical estimate |
| Slack \(\sim 10^{-5}\) | undetermined at observer envelope |
| Float `optimal` flag | implementation observation |
