> 精确算术下的静态采样安全和先验界在所声明的 oracle-map 范围内闭合；控制残差与有意义部署驻点的联系、局部未知边界接口以及部署对搬运的作用仍是当前论文需要补齐的内容。

# Feature-cover barrier invariance (range truncation, first crossing, joint feasibility)

Status: **proved** for the exact-arithmetic sampled cascade under P0 and the parameter inequalities of §0. Monitor abort is not a substitute for these lemmas. Floating-point QP status is §A6, not the exact theorem.

Code: `SafetyFilter._nearest_feature_rows`, `object_row_mode=nearest_feature`, `range_truncation_holds`. ρ, \(r_{\mathrm{safe}}\), \(u_{\max}\), \(d_{\min}\) unchanged.

## 0. Setup

Frozen simple closed polygon \(S\subset\mathbb{R}^2\) (compact, nonempty interior), oracle vertices, \(v_{\mathrm{obj}}=0\). Sampling period \(\Delta>0\), speed limit \(u_{\max}>0\), \(\gamma=\gamma_{\mathrm{obj}}>0\), \(0\le\gamma\Delta\le 1\), \(\rho>0\), \(r_{\mathrm{safe}}>0\), finite range \(R_{\mathrm{row}}>0\).

Robot centre \(p\), hold \(p(\tau)=p+\tau u\) for \(\tau\in[0,\Delta]\), \(\lVert u\rVert\le u_{\max}\). Signed distance \(\mathrm{sd}(\cdot,S)\) is positive in the unbounded complementary component (exterior), zero on \(\partial S\), negative in \(\mathrm{int}(S)\). Unsigned boundary distance \(d_\partial(p)=\mathrm{dist}(p,\partial S)=\min_F\mathrm{dist}(p,F)\), the minimum of Euclidean distances to the finitely many closed edges \(F\) of \(S\).

On the **closed exterior** \(\{\mathrm{sd}\ge 0\}\) one has \(\mathrm{sd}=d_\partial\). Inside, \(\mathrm{sd}=-d_\partial\). In particular \(d_\partial\) alone does not distinguish interior from exterior: first crossing is §A4.

Barrier
\[
h_{\mathrm{true}}(p):=\mathrm{sd}(p,S)-r_{\mathrm{safe}}.
\]
Each edge \(F\) is a compact segment, hence closed convex. \(\mathrm{dist}(\cdot,F)\) is convex and \(1\)-Lipschitz, and (Fréchet) differentiable off \(F\), with
\[
\nabla\mathrm{dist}(p,F)=\frac{p-q}{\lVert p-q\rVert},
\]
where \(q=\Pi_F(p)\) is the unique closest point (radial at an endpoint; the outward unit normal of the supporting line when \(q\) is in the relative interior and \(p\notin F\)). At a point of \(F\) the distance is \(0\) and a subgradient is any unit vector; the code then uses the oriented edge normal `_edge_plane_row`. Under P0 one has \(d_\partial(p)\ge r_{\mathrm{safe}}+\rho/\gamma>0\), so the robot is **not** on any edge at a sampling instant, and every covered gradient exists.

\[
h_F(p):=\mathrm{dist}(p,F)-r_{\mathrm{safe}},\qquad n_F(p):=\nabla\mathrm{dist}(p,F)\quad(p\notin F).
\]

Cover
\[
\Phi(p):=\{F:\mathrm{dist}(p,F)\le d_\partial(p)+2u_{\max}\Delta\}
\cap\{F:\mathrm{dist}(p,F)\le R_{\mathrm{row}}\}.
\]
Three disjoint classes of edges: \(\Phi(p)\); cover-excluded (in range, outside the \(2u_{\max}\Delta\) neighbourhood of \(d_\partial\)); range-excluded (\(\mathrm{dist}(p,F)>R_{\mathrm{row}}\)).

**Parameter restriction used below (checked in code, not inferred from a numerical gap):**
\[
R_{\mathrm{row}}-u_{\max}\Delta\ge r_{\mathrm{safe}}+\rho/\gamma. \tag{RT}
\]
Paper scalars: \(0.60-0.35\cdot 0.05=0.5825\) and \(0.065+0.02/8=0.0675\). The predicate `range_truncation_holds` compares **the same** filter parameters.

Also \(0\le\gamma_{\mathrm{agent}}\Delta\le 1\), \(a=2/k_c-\Delta>0\).

## A1. Edges in \(\Phi(p)\)

Fix \(F\in\Phi(p)\) and \(p\notin F\). Convexity of \(h_F\) gives, for \(\tau\in[0,\Delta]\),
\[
h_F(p+\tau u)\ge h_F(p)+\tau\, n_F(p)^\top u.
\]
The implemented uncapped row is \(n_F(p)^\top u\ge -\gamma h_F(p)+\rho\) (static \(v_{\mathrm{obj}}=0\)). Hence
\[
h_F(p+\tau u)\ge(1-\gamma\tau)h_F(p)+\rho\tau.
\]
If \(h_F(p)\ge\rho/\gamma\) and \(0\le\gamma\tau\le 1\), the right-hand side is at least \(\rho/\gamma\).

**Cap inactivity.** `_cap_to_reachable` replaces a right-hand side \(b\) by \(\min(b,\max(r,0))\) with \(r\) a reachable inner product. When \(h_F\ge\rho/\gamma\), the uncapped RHS \(-\gamma h_F+\rho\le 0\), so the cap does not change the row. The implemented constraint is the uncapped discrete CBF.

**Initial positive spacing.** P0 gives \(h_{\mathrm{true}}(p)\ge\rho/\gamma\), and \(h_F\ge h_{\mathrm{true}}\) on the exterior, so every covered row starts with \(h_F\ge\rho/\gamma\) and \(p\notin F\).

Duplicate \((n,h)\) rows are collapsed; this does not drop a geometrically distinct edge.

## A2. Cover-excluded edges (in range)

If \(F\notin\Phi(p)\) and \(\mathrm{dist}(p,F)\le R_{\mathrm{row}}\), then
\[
\mathrm{dist}(p,F)>d_\partial(p)+2u_{\max}\Delta.
\]
\(1\)-Lipschitz:
\[
\mathrm{dist}(p+\tau u,F)\ge\mathrm{dist}(p,F)-\tau u_{\max}
>d_\partial(p)+2u_{\max}\Delta-\tau u_{\max}
\ge d_\partial(p)+u_{\max}\Delta.
\]
Thus, throughout the hold,
\[
h_F(p+\tau u)>d_\partial(p)+u_{\max}\Delta-r_{\mathrm{safe}}.
\]
If the robot remains in the closed exterior (A4), \(d_\partial(p)=h_{\mathrm{true}}(p)+r_{\mathrm{safe}}\), so
\[
h_F(p+\tau u)>h_{\mathrm{true}}(p)+u_{\max}\Delta.
\]
This is a **lower bound**, not a discrete CBF contraction for that edge. It is enough for the safety margin once \(h_{\mathrm{true}}(p)\ge\rho/\gamma\) and \(u_{\max}\Delta\ge 0\).

## A3. Range-excluded edges

If \(\mathrm{dist}(p,F)>R_{\mathrm{row}}\),
\[
\mathrm{dist}(p+\tau u,F)\ge\mathrm{dist}(p,F)-\tau u_{\max}>R_{\mathrm{row}}-u_{\max}\Delta,
\]
hence \(h_F(p+\tau u)>R_{\mathrm{row}}-u_{\max}\Delta-r_{\mathrm{safe}}\). Under (RT),
\[
h_F(p+\tau u)>\rho/\gamma.
\]
No row is written for these edges. The argument does **not** identify them with cover-distance exclusions (the error in the previous note).

## Safety margin versus strong contraction

Let \(d(\tau)=\min_F\mathrm{dist}(p+\tau u,F)=d_\partial(p(\tau))\). Combining A1–A3, for \(\tau\in[0,\Delta]\),
\[
d(\tau)
\ge\min\Bigl(
\min_{F\in\Phi}h_F(p+\tau u)+r_{\mathrm{safe}},\;
d_\partial(p)+u_{\max}\Delta,\;
R_{\mathrm{row}}-u_{\max}\Delta
\Bigr).
\]
Under P0, \(h_F(p)\ge\rho/\gamma\) on \(\Phi\), A1 gives \(\min_\Phi h_F(p+\tau u)\ge\rho/\gamma\), A2/A3 give the other two arguments \(\ge r_{\mathrm{safe}}+\rho/\gamma\) by (RT) and \(d_\partial(p)\ge r_{\mathrm{safe}}+\rho/\gamma\). Therefore
\[
d(\tau)\ge r_{\mathrm{safe}}+\rho/\gamma. \tag{M}
\]
This is the **margin invariance** used for recursive feasibility. It does **not**, by itself, assert
\[
h_{\mathrm{true}}(p+\tau u)\ge(1-\gamma\tau)h_{\mathrm{true}}(p)+\rho\tau \tag{C}
\]
for every configuration.

**Corollary (strong inequality under extra comparisons).** Suppose the robot is in the closed exterior, \(\Phi(p)\) is nonempty (so \(d_\partial(p)\le R_{\mathrm{row}}\) and \(h_{\mathrm{true}}(p)=\min_\Phi h_F(p)\)), and
\[
(1-\gamma\tau)h_{\mathrm{true}}(p)+\rho\tau
\le\min\bigl(h_{\mathrm{true}}(p)+u_{\max}\Delta,\; R_{\mathrm{row}}-u_{\max}\Delta-r_{\mathrm{safe}}\bigr)
\]
for all \(\tau\in[0,\Delta]\). Then (C) holds, because the min in the lower bound of \(h_{\mathrm{true}}\) is the A1 term. Paper numbers: \(h_{\mathrm{true}}(p)\le R_{\mathrm{row}}-r_{\mathrm{safe}}=0.535\), \(\gamma\Delta=0.4\), and
\[
\gamma(R_{\mathrm{row}}-r_{\mathrm{safe}})-\rho=8\cdot 0.535-0.02=4.26\ge u_{\max}=0.35,
\]
so the comparison holds uniformly on the theorem parameter set when \(\Phi\neq\emptyset\).

If \(\Phi=\emptyset\), the nearest edge is out of range, \(d_\partial(p)>R_{\mathrm{row}}\), and (M) follows from A3 plus \(1\)-Lipschitz of \(d_\partial\). Inequality (C) is **not** claimed in that regime; recursive feasibility only needs (M).

## A4. First crossing (exterior is not assumed along the hold)

Hypothesis at a sampling instant: \(p(0)\) lies in the **open exterior** \(\{\mathrm{sd}>0\}\) with (M) at \(\tau=0\), i.e. \(d_\partial(p(0))\ge r_{\mathrm{safe}}+\rho/\gamma>0\).

The hold \(p(\cdot)\) is continuous. The set \(\partial S=\{d_\partial=0\}\) is closed. Any first entry into \(\mathrm{int}(S)\) requires a first time \(\tau^\star\in(0,\Delta]\) with \(p(\tau^\star)\in\partial S\), hence \(d_\partial(p(\tau^\star))=0\).

On \([0,\tau^\star)\) the trajectory stays in the closed exterior, where A1–A3 apply with unsigned distances only, and (M) gives \(d_\partial(p(\tau))\ge r_{\mathrm{safe}}+\rho/\gamma>0\). By continuity, \(d_\partial(p(\tau^\star))\ge r_{\mathrm{safe}}+\rho/\gamma>0\), contradicting \(p(\tau^\star)\in\partial S\). Therefore there is no first crossing: the whole hold remains in the open exterior, \(\mathrm{sd}=d_\partial\), and (M) is \(h_{\mathrm{true}}(p(\tau))\ge\rho/\gamma\).

This argument does **not** assume “the robot stays outside” as a hypothesis. Exterior at \(\tau=0\) plus (M) on unsigned distances plus continuity excludes \(\partial S\).

**Empty \(\Phi\).** Outside, \(\Phi=\emptyset\) iff the nearest edge has distance \(>R_{\mathrm{row}}\) (the nearest edge always meets the cover-distance test \(d_\partial+2u_{\max}\Delta\)). Then A3 plus \(1\)-Lipschitz of \(d_\partial\) give (M); no object row is written, and none is required.

**Far from the object.** Same as empty \(\Phi\).

**Nearest-feature switch.** During the hold the argmin of \(\mathrm{dist}(\cdot,F)\) may jump (convex corners, parallel sides, ties). A1 controls every \(F\in\Phi(p)\) that can become nearest inside \(B(p,u_{\max}\Delta)\) while remaining in range; A2/A3 control the rest. A single nearest-edge row at time \(0\) would miss a neighbour that becomes nearest.

**Tied nearest edges.** Both enter \(\Phi\) whenever they meet the cover and range tests; both rows are written (modulo duplicate collapse of identical \((n,h)\)).

**Endpoints.** Off the segment, \(\nabla\mathrm{dist}\) is the radial unit vector toward the nearest endpoint, which is a valid gradient of the convex function. P0 keeps \(p\notin F\).

**Nonconvex / reflex corners.** Distance-to-segment remains convex **per edge**. The true \(d_\partial=\min_F\mathrm{dist}(\cdot,F)\) is not convex, and is not represented by infinite supporting planes of incident edges. Reflex two-plane infinite AND is not used: in a concave crook it can lie strictly below \(\mathrm{sd}\) and destroy \(0\in F_\rho\) while \(h_{\mathrm{true}}\ge\rho/\gamma\). Aggregate infinite planes of a non-nearest hull edge are the C5/L11/L17 mechanism and are not this construction.

## A5. Joint recursive feasibility

**P0 (checkable).** For every robot \(i=1,\ldots,N\): \(p_i\in D\) (closed rectangle); pairwise \(\lVert p_i-p_j\rVert\ge d_{\min}\); \(\mathrm{sd}(p_i,S)\ge r_{\mathrm{safe}}+\rho/\gamma_{\mathrm{obj}}\); oracle map; no communication dropout; \(\mathrm{comm\_range}\ge 2R\); \(\gamma_{\mathrm{obj}}\Delta\le 1\); \(\gamma_{\mathrm{agent}}\Delta\le 1\); \(a>0\); (RT); frozen \(S\); \(v_{\mathrm{obj}}=0\).

**One-step feasible set** for robot \(i\) at a sample, with positions frozen,
\[
F_{\rho,i}
=\{u:\lVert u\rVert\le u_{\max}\}
\cap\{\text{agent rows}\}
\cap\{\text{wall rows}\}
\cap\{\text{object rows with }\rho\}.
\]
Object rows: A1 on \(\Phi(p_i)\). Agent rows: shared-responsibility
\[
2(p_i-p_j)^\top u_i\ge -(\gamma_{\mathrm{agent}}/2)\,h_{ij},\qquad
h_{ij}=\lVert p_i-p_j\rVert^2-d_{\min}^2,
\]
for every neighbour \(j\) in the communication ball. Wall rows: \(p_i+\Delta u\in D\). Speed ball contains \(0\).

**Zero input.** \(u_i=0\) satisfies:

- speed ball;
- every agent row iff \(h_{ij}\ge 0\), i.e. \(\lVert p_i-p_j\rVert\ge d_{\min}\) for listed neighbours;
- every wall row iff \(p_i\in D\) (because \(b_{\mathrm{wall}}\le 0\) precisely then);
- every object row iff \(h_F\ge\rho/\gamma\) on \(\Phi(p_i)\), which on the exterior is equivalent to \(h_{\mathrm{true}}(p_i)\ge\rho/\gamma\) (covered \(h_F\ge h_{\mathrm{true}}\), and the nearest edge is in \(\Phi\) whenever it is in range; if out of range, there are no object rows and the object family is empty).

Cap inactive by A1 whenever \(h_F\ge\rho/\gamma\).

**Omitted pairs.** Neighbour lists are all pairs with \(\lVert p_i-p_j\rVert\le\mathrm{comm\_range}\), undirected, dropout \(=0\). A pair omitted from both lists has distance \(>\mathrm{comm\_range}\ge 2R=1.6\,\mathrm{m}\). The one-step collision radius is \(d_{\min}+2u_{\max}\Delta=0.315\,\mathrm{m}\). Omitted pairs cannot violate \(d_{\min}\) on the hold (`omitted_pair_radius`, `omitted_neighbor_certificate`). Asymmetry of directed lists cannot occur with dropout \(0\) and a symmetric threshold. Pairs inside the communication ball are constrained on **both** QPs.

**Agent hold.** For two linear motions the minimum of \(\lVert p_i(\tau)-p_j(\tau)\rVert\) on \([0,\Delta]\) is exact (`hold_segment_min_distance`). Shared-responsibility plus \(\gamma_{\mathrm{agent}}\Delta\le 1\) preserves \(h_{ij}\ge 0\) at the next sample; the hold-segment gate is an independent monitor, not the pairwise proof.

**Walls.** \(D\) is a closed rectangle, hence convex. The four linear rows are equivalent to \(p+\Delta u\in D\). The segment \(p+[0,\Delta]u\) then lies in \(D\).

**Update order.** At sample \(t_k\) every QP is assembled from the **same** frozen \(P_k\) (shared responsibility does not require neighbour inputs). Commands are then held simultaneously. Adjacent timesteps of one trajectory are not parallelised. Per-agent QPs may be conceptually simultaneous; they do not mutate \(P_k\) between agents.

**Induction.** P0 \(\Rightarrow\) \(0\in F_{\rho,i}\) for every \(i\) (conjunction of the four families, not a single object halfspace) \(\Rightarrow\) the feasible set is a nonempty closed convex subset of the speed ball \(\Rightarrow\) the Euclidean projection QP attains a unique minimiser \(U_{k,i}\) \(\Rightarrow\) A1–A4 and the agent/wall lemmas keep the P0-type invariant at \(t_{k+1}\). Recursive feasibility of the **joint** constraint family follows. In particular \(0\in F_\rho\) at every sample, which is the predicate `zero_input_feasible_with_rho` together with solver status `optimal` in exact arithmetic: this **is** \(\mathcal K_0\), deduced from P0, not assumed as a future trajectory property.

Single object-row feasibility does **not** replace this conjunction.

**Idle implementation tiers.** YAML may enable margin clamp or object-barrier scaling. On the invariant set every original-\(\rho\) right-hand side already satisfies \(b\le 0\), so clamp does not fire, scaling is not reached, and `forbid_fallback` does not fire. The proved cascade is saturation then projection onto the **unclamped** \(F_\rho\). Those tiers are not part of the theorem.

## A6. Exact algorithm versus numerical execution

The planar solver `solve_min_norm_2d` enumerates the unconstrained point, speed-circle scalings, single-row projections, and pairwise intersections, and selects a least-cost candidate feasible within tolerance \(10^{-7}\). In exact real arithmetic, a nonempty closed convex set in \(\mathbb{R}^2\) makes this the Euclidean projection. In IEEE-754, “status = optimal” means a candidate passed the float test; it is **not** a proof that the returned vector is the exact projection.

The main theorem is the exact-arithmetic cascade. An implementation corollary records float `optimal` flags and dissipation slacks as **observations**. They do not enlarge the theorem and are not counterexamples when they sit inside the solver/observer envelope.

**Monitor.** `hold_segment_object_clearance` is a Lipschitz lower bound used as a **fault detector**. It is not Lemma A1–A4. `forbid_fallback` plus `TheoremModeAbort` **stops** the case; the theorem does not claim safety of a continued trajectory after abort. On the proved set abort does not occur, so “safe or abort” is not a safety proof and is not used as one.

## Correspondence

| Symbol | Code |
|---|---|
| \(\mathrm{dist}(p,F)\) | `closest_point_on_segment` |
| \(\Phi\) | `_nearest_feature_rows`, `cover = unsigned + 2 max_speed dt` |
| (RT) | `range_truncation_holds` |
| \(n_F,h_F\) | \((p-q)/d\), \(d-r_{\mathrm{safe}}\) |
| RHS / cap | `_finish_object_rows`, `_cap_to_reachable` |
| joint stack | `filter_velocity`: agent, wall, object, speed |
| omitted pairs | `omitted_neighbor_certificate` |
| walls | `wall_halfplanes` |
| abort | `TheoremModeAbort`; not a safety certificate |
