# Lemma 2 - mass, midpoint quadrature, and centroid perturbation

## Statement

Assume A1--A6 and the exact-cell locality contract `R_comm>=2R_l`. Put

\[
r_0=\min\{R_l,d_s/2,W_D/2,H_D/2\},\quad
m_{\min}=\phi_0\pi r_0^2/4>0,
\]

\[
m_{\max}=\pi R_l^2(\phi_0+P_{\max}),\quad
\widehat\phi_{\max}=\phi_0+P_{\max}+E_{w,i},\quad
\widehat L_\phi\le (P_{\max}+E_{w,i})e^{-1/2}/\sigma.
\]

For midpoint mesh side at most `h`, let `r_h=h/sqrt(2)`,
`A_V<=pi R_l^2`, `P_V<=2 pi R_l`, and

\[
\eta_m=\widehat L_\phi r_hA_V+\widehat\phi_{\max}(2P_Vr_h+\pi r_h^2),
\]

\[
\eta_a=(\widehat\phi_{\max}+R_l\widehat L_\phi)r_hA_V
 +R_l\widehat\phi_{\max}(2P_Vr_h+\pi r_h^2).
\]

If `eta_m<m_min`, define

\[
\epsilon_q={\eta_a+R_l\eta_m\over m_{\min}-\eta_m}.
\]

Then the implementation centroid obeys

\[
\boxed{\|\widehat c_i-c_i^*\|\le
\epsilon_{c,i}:={2R_l\epsilon_{\phi,i}\over m_{\min}}
 +\epsilon_{q,i}+\epsilon_{G,i}.}
\tag{L2}
\]

In strict synchronized theorem mode, `epsilon_G=0`. Delayed or missing neighbor
positions require a separate geometric cell-error certificate.

## Proof

1. **Mass floor.** Separation implies every point of
   `B(p_i,d_s/2)` is no farther from `p_i` than from another robot. Intersecting
   with the local disk and rectangle leaves at least a quarter disk of radius
   `r_0`; the worst rectangular location is a corner. Since `phi*>=phi_0`, its
   mass is at least `m_min`. The same floor holds for the mapped density because
   theorem mode preserves the positive base density.

2. **Mass ceiling and Lipschitz constant.** `K_sigma<=1` and
   `P_Gamma<=P_max` imply `phi*<=phi_0+P_max`. The local cell area is at most
   `pi R_l^2`, proving `m_i*<=m_max`. The measure decomposition in Lemma 1
   gives mapped total weight at most `P_max+E_w`. Differentiating its convolution
   and using `||grad K_sigma||_infinity=e^(-1/2)/sigma` gives the displayed
   common mapped-density bounds.

3. **Midpoint mass error.** A midpoint cell wholly inside the exact local cell
   contributes at most `Lhat_phi r_h` times its area. Cells cut by the cell boundary
   lie in a radius-`r_h` parallel set whose area is bounded by
   `2P_V r_h+pi r_h^2`. Bounding their density by `phihat_max` proves `eta_m`.
   The cell is convex (intersection of half-planes, a disk, and a rectangle), so
   `P_V<=2 pi R_l` is a valid conservative perimeter bound.

4. **First-moment error.** Work with the shifted moment
   `a=integral_V (q-p_i) phi(q)dq`, whose integrand is Lipschitz with constant
   at most `phihat_max+R_l Lhat_phi` and magnitude at most `R_l phihat_max`. Repeating
   Step 3 proves `||ahat-a||<=eta_a`.

5. **Ratio perturbation.** If `mhat` is the midpoint mass, then
   `mhat>=m_min-eta_m>0` and

   \[
   \left\|{\widehat a\over\widehat m}-{a\over m}\right\|
   \le{\eta_a\over\widehat m}
      +{\|a\|\,|m-\widehat m|\over m\widehat m}
   \le {\eta_a+R_l\eta_m\over m_{\min}-\eta_m}=\epsilon_q.
   \]

6. **Density perturbation.** On the same exact cell, Lemma 1 gives mass error
   at most `epsilon_phi` and shifted-moment error at most
   `R_l epsilon_phi`. Both ideal and mapped continuum masses have the base-density
   floor `m_min`; applying the ratio identity in both directions gives
   `||c_tilde-c*||<=2R_l epsilon_phi/m_min`.

7. Add the density, midpoint, and separately certified geometric-cell errors.
   This is (L2). QED.

The denominator gate is fail-closed in the Python certificate. Wolfram checked
its positivity, the scalar ratio identity, and monotonicity of the displayed
error functions; the norm bounds and geometry remain analytic arguments.
