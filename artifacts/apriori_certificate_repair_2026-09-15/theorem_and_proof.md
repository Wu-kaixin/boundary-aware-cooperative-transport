# Theorem and proof — a priori centroid certificate repair

Branch: `feat/apriori-certificate-repair` (from `7b9fb36`).  
Scope: static sampled theorem_mode, N=16, frozen object, oracle map, Δ=0.05, 600 steps.  
Not claimed: unknown-object transport, global convergence, N=16 Hessian.

Certificate levels: **rigorous** / **rigorous_on_K0** / **numerical_a_priori** / **constants_only** / **post_hoc**.

---

## 1. Identity (unchanged)

$$
J_k=\sum_i m_{i,k}^\star\|u_{i,k}\|^2,\qquad
E_k=\sum_i m_{i,k}^\star\|\hat c_{i,k}-c_{i,k}^\star\|^2,
$$

$a=2/k_c-\Delta>0$. Under Theorem C of `discrete_dissipation_derivation.md` (exact sat+QP cascade, full window in $\mathcal{K}_0$):

$$
\bar J_K\le \frac{2H^\star(P_0)}{aK\Delta}+\frac{4\bar E_K}{a^2}.
$$

Priors: $E_k\le B_E$, $H^\star(P_0)\le B_{H0}$, $B_{J,\mathrm{prior}}=2B_{H0}/(aK\Delta)+4B_E/a^2$.  
Comparator: $B_{J,\mathrm{geom}}=M_{\mathrm{plane}}u_{\max}^2$, $M_{\mathrm{plane}}=\phi_0|D|+2\pi\sigma^2 L$.

$\mathcal{K}_0$: every QP status is `optimal` **and** $u=0$ lies in the *original* (unclamped) $\rho$-constraint set **and** $a>0$. Missing flags fail closed. Trajectory $\bar E$ is post-hoc.

**Label rule.** Numerical observer $H(P_0)$ never yields `rigorous_on_K0`. If $\mathcal{K}_0$ fails, a numerical comparison against geometry is `constants_only`.

---

## 2. Analytic second-derivative majorants

Kernel $k(u)=\exp(-\|u\|^2/(2\sigma^2))$, unit-speed $u(s)=u_0+\tau s$, $\|\tau\|=1$.

### 2.1 Kernel and $\sigma^2 k$

$$
\frac{d^2 k}{ds^2}=\frac{k}{\sigma^2}\bigl(z^2-1\bigr),\qquad z=(u\cdot\tau)/\sigma.
$$

Then $|d^2 k/ds^2|\le \sigma^{-2}\exp(-z^2/2)|z^2-1|$. The scalar $g(z)=\exp(-z^2/2)|z^2-1|$ has $g(0)=1$ and the only other critical value $2e^{-3/2}<1$. Hence

$$
\bigl|d^2 k/ds^2\bigr|\le \frac{1}{\sigma^2},\qquad
\bigl|d^2(\sigma^2 k)/ds^2\bigr|\le 1.
$$

Domain: $\sigma>0$, all positions, all unit directions. The second bound is dimensionless and $\sigma$-independent. These replace the previous NMaximize envelope $15(0.2/\sigma)^4$, whose scaling was dimensionally inconsistent with both integrands.

### 2.2 Plane antiderivative $F1$

At $\sigma=1$, $F1=\sqrt{2\pi}\,\Phi(x)\exp(-y^2/2)$ with $\Phi$ the standard normal CDF, $0<F1\le\sqrt{2\pi}\exp(-y^2/2)$. The Hessian contraction along $\tau=(\cos\theta,\sin\theta)$ satisfies

$$
\bigl|\tau^\top \nabla^2 F1\,\tau\bigr|
\le \frac{\varphi}{\sqrt{e}}+\sqrt{2\pi},
$$

where $\varphi=(1+\sqrt{5})/2$. The first summand is $\max(c^2+2|cs|)/\sqrt{e}$ (maximum of $c^2+|\sin 2\theta|$ equals the golden ratio). The second is $s^2 F1|y^2-1|\le\sqrt{2\pi}$ because $\max_y e^{-y^2/2}|y^2-1|=1$. Scaling $F1_\sigma(x,y)=\sigma F1_1(x/\sigma,y/\sigma)$ gives

$$
\bigl|d^2 F1/ds^2\bigr|\le \frac{C}{\sigma},\qquad C=\varphi/\sqrt{e}+\sqrt{2\pi}\approx 3.488.
$$

At the paper value $\sigma=0.2$, $C/\sigma\approx 17.44$. Units: 1/m.

Numerical maximisation may be used to *locate* peaks; it is not the certificate.

---

## 3. Clipped-edge quadrature remainder

The integrator replaces $B(p_i,R)$ by an inscribed regular $n$-gon and clips by the domain box and Voronoi half-planes. Clipped edges include wall and Voronoi chords. Any convex subset of the closed disk has

* diameter $\le 2R$ (hence every edge length $\le 2R$);
* perimeter $\le 2\pi R$.

The unclipped chord $2R\sin(\pi/n)$ is **not** a bound after clipping. L-shape seed 2 at $n=256$ has declared chord $\approx 0.01963\,\mathrm{m}$ and realised max edge $\approx 1.40\,\mathrm{m}$, with $16/16$ cells exceeding the chord.

Panels on an edge of length $L$ are $m=\lceil L/h_\star\rceil$ with a **frozen** maximum step $h_\star$ (default $0.004\,\mathrm{m}$). Then $h=L/m\le h_\star$ on every realised edge, so the composite-trapezoid remainder of one scalar integrand is $\le L h_\star^2 M_2/12$. Summing lengths $\le 2\pi R$ per cell:

$$
|\delta m_i|_{\mathrm{trap}}\le W\cdot (2\pi R)\,h_\star^2 M_{2,F1}/12,
$$

with $W=\sum_j w_j\ge 0$ (oracle: $W=L$). No $n_{\mathrm{sources}}$ unit-weight inflation. Two kernel-centred moment components use $M_2=1$.

Runtime allocation by actual length is allowed because it is dominated by the same a priori $h_\star$. Trajectory max-edge is **not** an input.

---

## 4. Robot-centred moments and culling

Green returns $\mu_{\xi}=\int(q-\xi)k\,dA$ and $m=\int k\,dA$. Robot-centred $\mu_p=\mu_{\xi}+(\xi-p)m$, so

$$
\|\delta\mu_p\|\le\|\delta\mu_\xi\|+\|\xi-p\|\,|\delta m|.
$$

The integrator skips sources with $\mathrm{dist}(\xi,\mathrm{bbox}(P))>s\sigma$. Every evaluated source therefore satisfies $\|\xi-p_i\|\le R+s\sigma$ ($P\subset B(p_i,R)$). Omitted mass on the truncated Voronoi partition is the plane tail $2\pi\sigma^2 e^{-s^2/2}W$ (no $N$ factor), charged separately from `density.restrict` (influence $n_\sigma=3$). Restrict and culling are both included; overlap is conservative.

The continuous infinite-line bound $\phi\le\phi_0+n_e\sigma\sqrt{2\pi}$ is **not** a bound on the discrete mixture. Valid pointwise majorants:

* $k\le 1$ $\Rightarrow$ $\phi\le\phi_0+W$;
* per-edge spacing $\delta$: $\phi\le\phi_0+n_e(\delta+\sigma\sqrt{2\pi})$.

The disk-versus-$n$-gon area gap uses the minimum of those two.

Floating-point addends are charged separately from analytic truncation (machine epsilon times a panel/edge count majorant).

---

## 5. Approximate centroids

Exact nonnegative $\phi^\star$ on $\Omega_i\subset B(p_i,R)$ has $c_i^\star\in\mathrm{conv}(\Omega_i)$. A Green centroid need not. The implementation:

1. if numerical mass $\le 10^{-12}$, uses the site $p_i$;
2. otherwise projects $\hat c$ onto $B(p_i,R)$.

Projection onto a closed convex set containing $c^\star$ is non-expansive, so it cannot increase $\|\hat c-c^\star\|$. Fallback cells satisfy $m^\star\le|\delta m|+\varepsilon$, hence $E_i\le R^2 m^\star$. The certificate uses

$$
B_E=\min\Bigl(2R\bigl(\textstyle\sum\|\delta\mu_p\|+R\textstyle\sum|\delta m|\bigr)+R^2\bigl(\textstyle\sum|\delta m|+N\varepsilon\bigr),\; 4R^2 M_{\mathrm{plane}}\Bigr).
$$

---

## 6. Safety margin clamp (not the baseline)

Rows are $a^\top u\ge b$. The option `theorem_clamp_margin_to_keep_zero` replaces some object $b>0$ by $0$ when the $\rho$-free RHS already admits $u=0$. That **relaxes** the $\rho$-margin; it does not preserve the original $a^\top u\ge b$ with $\rho$. This round's baseline is **unclamped**. Original and effective RHS are both logged. $\mathcal{K}_0$ is judged on the original $\rho$ set: $0\notin F$ is distinguished from $F=\emptyset$ (QP infeasible / empty).

Sample-and-hold: wall rows keep $P_k+\Delta U_k\in D$; pairwise omitted neighbours must lie outside $d_{\min}+2u_{\max}\Delta$.

---

## 7. Frozen constants (this package)

$h_\star=0.004\,\mathrm{m}$, $n_{\mathrm{gon}}=256$, cull $s=6$, $\delta=0.04$, $n_\sigma=3$, $N=16$, $R=0.8$, $\sigma=0.2$, $\Delta=0.05$, $k_c=0.9$, $a\approx 2.172$, $K=600$, $u_{\max}=0.35$, $\rho=0.02$ unchanged.

Constant-level screen (rigorous $B_{H0}=R^2 M_{\mathrm{plane}}$, repaired $B_E$):

| shape | $B_E$ | $B_{J,\mathrm{prior}}$ | $B_{J,\mathrm{geom}}$ | beats |
|---|---|---|---|---|
| rectangle | 0.1352 | 0.1435 | 0.1803 | yes |
| L-shape | 0.1787 | 0.1883 | 0.2295 | yes |
| C-shape | 0.1899 | 0.1978 | 0.2295 | yes |

These numbers are **not** inherited from the PARTIAL 2026-09-15 closure (those used an invalid chord remainder and, in README, a numerical $H_0$ column).

---

## 8. $\mathcal{K}_0$ and Route B (blocking, not CLOSED)

Identity (★) requires every sample in $\mathcal{K}_0$: QP `optimal` **and** $0\in F_\rho$ on the **unclamped** rows. That fails on:

* C-shape seed 5 (development): frames 228–229 have $0\notin F_{\mathrm{hard}}$ (object $b>0$ even without $\rho$); 230–232 are margin-only. $F$ is nonempty (9600/9600 optimal).
* L-shape seeds 11 and 17 (validation): frames 238–240 (agent 15) and 318–322 (agent 14), all margin-only, $0\in F_{\mathrm{hard}}$, $F$ nonempty.

Route B: for $0\notin F$ with $F\neq\emptyset$, the extra dissipation term involves $\|\Pi_F(0)\|$. The only a priori envelope from the input constraint is $\|\Pi_F(0)\|\le u_{\max}=0.35$. Substituting it recovers a $J$ bound of order $M_{\mathrm{plane}}u_{\max}^2$ and **cancels** $B_{J,\mathrm{prior}}<B_{J,\mathrm{geom}}$. A tighter envelope would need a uniform lower bound on the object CBF $h$; that is an unproved invariant of $P_0$, and trajectory-measured $b$ or $\|u\|$ are forbidden as priors. Clamp hides some margin-only frames by relaxing $\rho$ and is not the original-safety baseline.

Dissipation slacks $\sim 10^{-5}$ are **not** theorem counterexamples: polar $H,g,m,c$ remain numerical, with no certified observer envelope that settles the sign.

This package therefore stops at **PARTIAL** with that blocking condition. Recursive feasibility / control redesign is a different theorem, not a repair of (★) on the present cascade.
