# Lemma 1 - boundary/map to density consistency

## Statement

Let

\[
\nu_t^*=\xi(\cdot,t)_\#ds,
\quad \nu_{h,t}^*=\sum_k\Delta s_k\,\delta_{\xi_k^*},
\quad \widehat\nu_{i,t}=\sum_k\widehat w_{ik}\,\delta_{\widehat\xi_{ik}}.
\]

Assume A3--A5, `0<=Delta s_k<=h_s`, and that every source discarded by
`density.restrict()` is at least `s_sigma sigma` from every query in
`B(p_i,R_l) intersect D`. Define

\[
\epsilon_{\xi,i}=\epsilon_{b,i}+\epsilon_{d,i}
 +2d_{\max}\sin(\epsilon_{n,i}/2),\qquad
\epsilon_{x,i}=\epsilon_{\xi,i}+\sqrt2v,
\]

and

\[
E_{w,i}=L_{\rm miss,i}+M_{\rm spur,i}
 +\sum_{k\in\mathrm{matched}}|\widehat w_{ik}-\Delta s_k|.
\]

Then

\[
\|\widehat\phi_i-\phi^*\|_{L^1(B(p_i,R_l)\cap D)}\le\epsilon_{\phi,i},
\]

where

\[
\boxed{\epsilon_{\phi,i}=
L_{K,1}P_{\max}(\epsilon_{\xi,i}+\sqrt2v+L_T h_s/2)
+2\pi\sigma^2E_{w,i}+\epsilon_{\rm tail,i}}
\tag{L1}
\]

and

\[
\epsilon_{\rm tail,i}\le
\pi R_l^2M_{\rm drop,i}e^{-s_\sigma^2/2}.
\]

The comparison is made **after convolution**. It does not claim convergence of
a continuous arc-length measure to a Dirac measure in total variation.

## Proof

1. **Offset-target error.** For unit normals making angle at most
   `epsilon_n`, `||nhat-n||=sqrt(2-2 cos epsilon_n)=2 sin(epsilon_n/2)`.
   The triangle inequality applied to `xi=b+dn` therefore gives the stated
   `epsilon_xi`. Replacing a target by a representative in the same square voxel
   adds at most its diagonal `sqrt(2)v`, giving `epsilon_x`.

2. **Translation inequality.** For any `a,b in R^2`, the fundamental theorem of
   calculus along the segment from `a` to `b`, followed by Tonelli, gives

   \[
   \|K_\sigma(\cdot-a)-K_\sigma(\cdot-b)\|_1
   \le \|\nabla K_\sigma\|_1\|a-b\|.
   \]

3. **Curve quadrature.** On an arc of length `Delta s_k`, choose the registered
   point at the arc midpoint (or assume the certificate supplies the same
   average-distance bound). If `xi` is `L_T`-Lipschitz in arc length, the
   translation inequality gives an error at most
   `L_K,1 L_T Delta s_k^2/2`. Summing, using
   `sum Delta s_k^2 <= h_s sum Delta s_k` and
   `sum Delta s_k=P_Gamma<=P_max`, yields

   \[
   \|K_\sigma*\nu_h^*-K_\sigma*\nu^*\|_1
   \le \tfrac12L_{K,1}L_TP_{\max}h_s.
   \]

4. **Map displacement and weight mismatch.** Match exact and observed atoms.
   Moving all matched target mass by at most `epsilon_x` costs at most
   `L_K,1 P_max epsilon_x` by Step 2. Missing, spurious, and matched-weight
   discrepancies have total variation at most `E_w`; convolution therefore
   contributes at most `||K_sigma||_1 E_w=2 pi sigma^2 E_w`.

5. **Restriction tail.** A discarded atom of weight `w` contributes at most
   `w exp(-s_sigma^2/2)` pointwise on the query disk. Summing discarded weights
   and integrating over an area no larger than `pi R_l^2` gives the stated tail.
   Gaussian support is infinite, so this term cannot be set to zero merely
   because `density.restrict()` was called.

6. Add Steps 3--5 by the triangle inequality and restrict the global `L1`
   estimate to `B(p_i,R_l) intersect D`. The common base density cancels. This is
   exactly (L1). QED.

## Evidence boundary

Wolfram exactly checked the three kernel constants, the normal-angle identity,
and the step-square inequality. Pushforwards, translation in `L1`, curve
quadrature, atom matching, and the tail decomposition are the analytic proof
above, not CAS outputs.
