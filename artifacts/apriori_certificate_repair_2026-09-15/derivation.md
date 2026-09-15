# A priori centroid-error bound for discrete dissipation (★)

Branch: `feat/apriori-centroid-bound` (base `feat/sampled-theorem-mode` @ `a531515`)  
Date: 2026-09-15  
Scope: static sampled coverage, frozen $\phi^\star$, sample-and-hold $P_{k+1}=P_k+\Delta u_k$.  
**Not claimed:** N=16 Hessian / local stability; unknown-object transport; mixing with research-v3 certificate `theorem_mode`.

Status labels: **proved** / **conditional on $\mathcal{K}_0$** / **numerical a priori** (geometry or $P_0$, Lipschitz remainder) / **post-hoc** (trajectory) / **not a strict quadrature certificate**.

---

## 1. Identity being instantiated

Mass-weighted indicators (controller grid centroid $\hat c$ vs observer centroid $c^\star$):

$$
J_k=\sum_i m_{i,k}^\star\|u_{i,k}\|^2,\qquad
E_k=\sum_i m_{i,k}^\star\|\hat c_{i,k}-c_{i,k}^\star\|^2,
$$

$a=2/k_c-\Delta>0$. Under Theorem C of `discrete_dissipation_derivation.md` (exact cascade, full window in $\mathcal{K}_0$):

$$
\bar J_K\le \frac{2H^\star(P_0)}{aK\Delta}+\frac{4\bar E_K}{a^2}.\tag{★}
$$

Substituting trajectory $\bar E$ is **post-hoc**. This note supplies $B_E\ge\bar E_K$ from parameters, geometry and $P_0$ only, and

$$
B_{J,\mathrm{prior}}=\frac{2B_{H0}}{aK\Delta}+\frac{4B_E}{a^2}.
$$

Geometric comparator (not $N m_+ u_{\max}^2$):

$$
B_{J,\mathrm{geom}}=(M_{\mathrm{tot}}+\varepsilon_M)\,u_{\max}^2,
$$

with $M_{\mathrm{tot}}$ the observer domain mass and $\varepsilon_M=10^{-10}$ above `scipy.quad` `epsabs=1e-11`. A coarser envelope is the plane mass $M_{\mathrm{plane}}=\phi_0|D|+2\pi\sigma^2 L$.

$\mathcal{K}_0$: every agent QP status is `optimal` **and** `zero_input_feasible_with_rho` is true (missing flag fails closed) **and** $a>0$. If any step leaves $\mathcal{K}_0$, the full-horizon bound is **inapplicable**. Failures are not deleted; $\rho$ is not reduced.

---

## 2. Two densities (oracle does not make the error zero)

**Controller** $\phi_{\mathrm{ctrl}}$: `oracle_boundary_view` midpoint samples of true edges, spacing $\delta=$ `theorem_oracle_spacing`, offset $d_c$, kernel $k(q,\xi)=\exp(-\|q-\xi\|^2/(2\sigma^2))$, plus floor $\phi_0$. `density.restrict` drops sources with $\|b_k-p\|>R+d_c+n_\sigma\sigma$, $n_\sigma=$ `influence_sigmas` (default 3).

**Observer** $\phi^\star$: analytic erf line integrals of the same kernel on the continuous offset edge measure (`UniformOffsetObserver.density`), independent of $\delta$ and of the CVT grid. Polar Gauss–Legendre evaluates truncated-cell $H,g,m,c$ from **one** `evaluate()` call.

Shared geometry and offset law; **not** the same numerical object.

Gaussian kernel constants (Wolfram, $\sigma$ general):

| quantity | formula | units / notes |
|---|---|---|
| $\max\|\nabla k\|$ | $1/(\sigma\sqrt{e})$ | 1/m |
| $\int_{\mathbb{R}^2} k\,dA$ | $2\pi\sigma^2$ | m² |
| $\int_{\mathbb{R}^2}\|\nabla k\|\,dA$ | $\pi\sigma\sqrt{2\pi}$ | m |
| $\int_{\mathbb{R}^2}\|\partial^2 k/\partial x^2\|\,dA$ | $4\sqrt{2\pi/e}$ | dimensionless ($\sigma$-independent) |

$\mathrm{Lip}(\phi)\le L/(\sigma\sqrt{e})$ with $L=$ polygon perimeter.

---

## 3. Moment form (no $1/m_-$)

Both $\hat c_i$ and $c_i^\star$ lie in $B(p_i,R)$ (grid samples are in the closed disk). Then $\|e_i\|\le 2R$ and

$$
m_i^\star\|e_i\|^2\le 2R\,\|m_i^\star e_i\|,\qquad
m_i^\star e_i=(\hat\mu_i'-\mu_i^{\star\prime})-\hat c_i'( \hat m_i-m_i^\star),
$$

hence

$$
E_k\le 2R\Big(\sum_i\|\delta\mu_i'\|+R\sum_i|\delta m_i|\Big).\tag{M}
$$

The moment form (M) is **grid-dependent** and **avoids $1/m_-$**. Direct numerical evaluation on the present constants shows that the $N$-fold Lip-over-disk tube is **looser than** $4R^2M$ at $n\in\{20,40,80\}$. The certificate therefore uses

$$
B_E=\min\bigl(2R(\textstyle\sum\|\delta\mu'\|+R\textstyle\sum|\delta m|),\ 4R^2 M\bigr),
$$

which is still a valid upper bound. When the min is the diameter term, $B_E$ does **not** improve under grid refinement; the grid-dependent summands are reported anyway so their decrease can be checked. Lipschitz covering remainders of the form $\mathrm{Lip}\cdot h_{\mathrm{mesh}}\cdot|D|$ were computed and **discarded** (they exceed $4R^2M$ by themselves and are not a useful majorant).

---

## 4. Error budget

### 4.1 Oracle midpoint vs continuous edge (floor w.r.t. CVT $n$)

On each panel of length $h\le\delta$ the midpoint matches the first along-edge moment. Taylor + $\int|\partial^2 k/\partial s^2|$ gives

$$
\|\phi_{\mathrm{ctrl}}-\phi^\star\|_{L^1(\mathbb{R}^2)}\le \sqrt{2\pi/e}\,L\,\delta^2/6.
$$

Covered cells partition a subset of $D$, so $\sum_i|\delta m_{\mathrm{den},i}|\le$ this $L^1$ number and $\sum_i\|\delta\mu_{\mathrm{den},i}\|\le R$ times the same. **Decreases with $\delta$, not with CVT $n$.** A Cartesian mesh of $D$ is stored as a numerical check; mesh consistency is **not** a strict certificate.

### 4.2 `restrict` tail (floor w.r.t. $n$ and $\delta$)

On $B(p,R)$, every dropped kernel satisfies $k<e^{-n_\sigma^2/2}$. Pointwise $|\Delta\phi|\le L e^{-n_\sigma^2/2}$. Disks overlap, so the rigorous mass sum uses $N$ times one-cell area:

$$
\sum_i|\delta m_{\mathrm{rst},i}|\le N\,L e^{-n_\sigma^2/2}\,\pi R^2.
$$

A tighter **numerical a priori** majorant maximises $\int_{B(p,R)}|\phi-\phi_{\mathrm{restrict}}|$ over a lattice of sites in $D$ (not the closed-loop trajectory) plus a Lipschitz site-grid remainder. **Does not vanish as $n\to\infty$** unless $n_\sigma\to\infty$.

### 4.3 Endpoint CVT grid (decreases with $n$)

`LocalCVT` uses `linspace` on the box of the reach disk, $h=2R/(n-1)$ (an upper bound: domain clipping only shrinks the box). Membership is the closed disk and Voronoi test; each kept node carries area $h^2$ (endpoint / rectangle hybrid, **not** trapezoid with $1/2$ edge weights). $\Omega_i=V_i\cap B(p_i,R)$ is convex, $\mathrm{Perim}(\Omega_i)\le 2\pi R$. Davenport-type tube of width $\sqrt{2}h$:

$$
e_{\mathrm{area}}(h)=2\pi R\cdot\sqrt{2}h+\pi(\sqrt{2}h)^2+4h^2.
$$

Per cell, $|\delta m|\le \phi_{\max} e_{\mathrm{area}}+\mathrm{Lip}(\phi)\sqrt{2}h\,\pi R^2$, and analogously for the first moment relative to $p_i$. Disks overlap $\Rightarrow$ multiply by $N$. **This is the only certificate term that is guaranteed to decrease as the controller grid is refined.** $\phi_{\max}$ is taken as $\max(\phi_0+L,\;\mathrm{mesh\ max})$.

### 4.4 Observer polar quadrature (evaluation grid, not controller grid)

$H,g,m,c$ always come from the same observer. The experiment **fixes** the evaluation polar grid (default $n_\theta=128$, $n_r=12$) across controller $n\in\{20,40,80\}$. A finer polar observer is compared at $P_0$ (and recorded). Differences are a **numerical error assessment**; they are **not** a strict certificate (`no_rigorous_quadrature_error_bound`). The polar remainder is **excluded from the rigorous $B_E$** and included only in the numerical-a-priori budget.

`observer.total_mass` uses `scipy.quad`, independent of $(n_\theta,n_r)$.

---

## 5. $B_{H0}$

**Proved:** $H^\star(P)\le R^2 M_{\mathrm{plane}}$ for every $P$.  
**Numerical a priori:** $H^\star(P_0)$ from the evaluation observer plus the $P_0$ polar envelope. $P_0$ is an initial condition, not a trajectory mean of $E$.

For paper defaults, $2B_{H0}/(aK\Delta)$ is $O(10^{-2})$ against $B_{J,\mathrm{geom}}\approx M u_{\max}^2=O(10^{-1})$. The $H_0$ term is not the term that decides whether (★) beats geometry; $B_E$ is.

---

## 6. What refinement does and does not do

| source | as $n_{\mathrm{CVT}}\uparrow$ | as $\delta\downarrow$ | as $n_\sigma\uparrow$ | as polar eval $\uparrow$ |
|---|---|---|---|---|
| oracle midpoint | floor | decreases | — | — |
| restrict tail | floor | floor | decreases | — |
| endpoint grid tube | decreases $O(h)$ | — | — | — |
| observer polar | — | — | — | decreases (eval only) |
| diameter $4R^2M$ | floor | floor | floor | floor |

---

## 7. Open issues (not closed this round)

1. **Polar quadrature remainder** for truncated Voronoi cells is not enclosed; only multi-resolution differences at $P_0$.
2. The **$N$-fold restrict and grid tubes** are honest but loose (disk overlap). They typically keep rigorous $B_E$ above the $E^\star$ needed to beat $M u_{\max}^2$.
3. No inequality is claimed on steps outside $\mathcal{K}_0$. A hypothetical extra term for $0\notin F_\rho$ would need a new proof; trajectory-measured extras remain post-hoc.
4. Mesh / site-grid majorants are **numerical a priori**, not strict certificates.
5. $J$ is mass-weighted command energy, **not** $\|g^\star\|^2$ and **not** coverage convergence.

---

## 8. Re-used hypotheses from the sampled-theorem derivation

Lemma A (frozen truncated partition majorant), $a>0$, saturation $\lambda_i\le k_c$, projection optimality only on $\mathcal{K}_0$. Unchanged. This branch does not restatement-replace continuous Theorem 1.

---

## 9. This run (2026-09-15, 27 cases)

All 27 finished 30 s with no abort. Geometric $J$ uses $M_{\mathrm{obs}}+10^{-10}$, not $N m_+ u_{\max}^2$.

**Certificate.** On every case `active_rigorous_bound = diameter_4R2M`. $B_{J,\mathrm{prior}}\approx 4.10$ (L/C) or $3.22$ (rectangle) versus $B_{J,\mathrm{geom}}\approx 0.230$ or $0.180$. **No prior beats geometry.** Post-hoc (trajectory $\bar E$) is $\approx 0.025$–$0.034$ and **does** beat geometry on all 24 applicable cases; that remains post-hoc.

**Measured $E$.** Falls with controller $n$ at fixed eval grid (seed-mean): L $1.32\times10^{-4}\to 4.2\times10^{-6}$; rectangle $1.40\times10^{-4}\to 1.7\times10^{-6}$; C $2.10\times10^{-4}\to 5.8\times10^{-6}$ from $n=20$ to $80$. $\bar J$ and $H^\star(P_0)$ do not. So the *implemented* centroid error is grid-dominated and already small; the *proof* does not capture that.

**$\mathcal{K}_0$.** C-shape seed 5 is inapplicable at every $n$, agent 14, `zero_input_feasible_with_rho` false. Frames: $n=20$ → 241–246 (historical match); $n=40$ → 224–228; $n=80$ → 232–236. Failures were not deleted; $\rho$ was not reduced. No new inequality for $0\notin F_\rho$ is claimed.

**P₀ polar envelope (L seed 2).** $|ΔH|\approx 6.1\times10^{-6}$, $|Δ\sum m|\approx 2.8\times10^{-4}$ between (128,12) and (256,16). Numerical assessment only.

