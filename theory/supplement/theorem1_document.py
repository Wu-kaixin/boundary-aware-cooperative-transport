"""The Theorem 1 proof supplement, authored once in LaTeX.

Every formula in this file is LaTeX.  Two renderers consume it:

* ``render_latex.py``  -> ``theorem1_supplement.tex`` (canonical typeset source)
* ``build_supplement_docx.py`` -> a Word file whose equations are native OMML
  objects, converted from the same LaTeX strings.

So the .tex and the .docx cannot disagree about a formula or a proof step.

Block grammar
-------------
``("h1"|"h2"|"h3", text)``      headings; ``text`` may contain ``$...$``
``("para", text)``              paragraph; ``$...$`` marks inline math
``("eq", latex, tag)``          display equation, ``tag`` may be ``None``
``("chain", [(latex, why)])``   aligned derivation, one justification per line
``("bullets", [text])``         list; ``$...$`` allowed
``("table", kind, rows)``       ``kind`` selects column widths and styling
``("callout", head, text)``     boxed remark
``("pagebreak",)``
"""

from __future__ import annotations


HEAD = "64e1905e092d19fd9dcbf0e77fbd39f55f0562ba"

TITLE = "Proofs and Supporting Results"
KICKER = "DBACT THEOREM 1"
SUBTITLE = (
    "Boundary-measure-induced Local-CVT practical stability near a selected "
    "nondegenerate local branch"
)
DOC_TITLE = "DBACT Theorem 1 - Proofs and Supporting Results"
DOC_AUTHOR = "Boundary-aware cooperative transport project"
DOC_SUBJECT = "Local conditional practical-stability theorem and verification record"

RUNNING_LEFT = "DBACT THEOREM 1  /  PROOF SUPPLEMENT"
RUNNING_RIGHT = "LOCAL - CONDITIONAL - AUDITABLE"

CALLOUT_HEAD = "CLAIM BOUNDARY"
CALLOUT_BODY = (
    "No global PL. No formal caging. No finite-time phase completion. "
    "No unconditional safety or cooperative-transport success."
)

STATUS_ROWS = [
    ["Branch", "DBACT-research-v3"],
    ["Baseline HEAD", HEAD],
    ["Proof status", "Conditional local theorem; A8 interval certificate not supplied"],
    ["Wolfram", "Connector exact checks passed; 9 local scripts exit 0"],
    ["Regression", "477 passed, 3 skipped"],
]

LEDGER_CONSTANTS = [
    "kernel_l1", "kernel_grad_l1", "kernel_grad_linf", "epsilon_xi",
    "epsilon_phi", "m_min", "m_max", "grid_spacing", "eta_m", "eta_a",
    "epsilon_q", "epsilon_c", "delta", "nu_phi", "nu_H", "a0",
    "lambda_H", "B", "ultimate_radius", "sampled_Bd", "sampled_ultimate_radius",
]


def document(cert: dict) -> list[tuple]:
    """Return the whole supplement as an ordered list of blocks."""
    blocks: list[tuple] = []
    add = blocks.append

    # ---------------------------------------------------------------- #
    # 1. Scope
    # ---------------------------------------------------------------- #
    add(("h1", "1. Scope and frozen claim"))
    add(("para",
         "Bounded local-map disagreement and bounded reference variation imply "
         "practical stability of the boundary-induced Local-CVT allocation "
         "dynamics around a nondegenerate local CVT branch."))
    add(("para",
         "The agents are first-order, $\\dot p_i = u_i$, and the nominal ENCLOSE "
         "law is $u_i^{\\mathrm{nom}} = -k_c\\,(p_i - \\widehat c_i)$. The ideal "
         "objective is the \\emph{saturated} limited-range coverage cost"))
    add(("eq", r"F_R(r) = \min\{r^2,\ R_\ell^2\},\qquad "
               r"H^*(P,t) = \int_D \min_i F_R\bigl(\|q-p_i\|\bigr)\,\phi^*(q,t)\,dq",
         None))
    add(("para",
         "Saturation is not cosmetic. With the unsaturated truncated cost "
         "$\\int_{V_i^\\ell}\\|q-p_i\\|^2\\phi^*\\,dq$ the integration domain moves "
         "with $p_i$, and the Reynolds boundary term on the circle "
         "$\\|q-p_i\\| = R_\\ell$ does not cancel; the standard CVT gradient "
         "identity is then false. Section 5 shows the cancellation that "
         "saturation buys."))
    add(("h2", "1.1 Deliberate exclusions"))
    add(("para", "This supplement does not prove any of the following."))
    add(("bullets", [
        "global CVT optimality, or convergence from arbitrary initial configurations;",
        "finite-time SEARCH-to-HOLD completion;",
        "geometric or topological caging;",
        "contact mechanics, or unconditional cooperative-transport success;",
        "CBF forward invariance when feasibility, speed-cap, or deviation contracts are violated;",
        "a uniform reduced-Hessian bound for an unspecified local branch.",
    ]))
    add(("para",
         "Collision safety is a separate result and is deliberately not a "
         "dependency of Theorem 1."))

    # ---------------------------------------------------------------- #
    # 2. Notation
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "2. Notation"))
    add(("para",
         "$D \\subset \\mathbb{R}^2$ is a fixed compact rectangle of width $W_D$ "
         "and height $H_D$; $P = (p_1,\\dots,p_N) \\in \\mathbb{R}^{2N}$ stacks the "
         "agent positions. All spatial norms are Euclidean, and the stacked norm "
         "is the Euclidean product norm"))
    add(("eq", r"\|d\|^2 = \sum_{i=1}^{N}\|d_i\|^2", None))
    add(("para",
         "$\\Gamma(t)$ is the true object boundary, with perimeter "
         "$P_\\Gamma \\le P_{\\max}$, and $\\xi(s,t)$ is the offset-boundary target "
         "parameterised by arc length $s$. The ideal boundary measure is the "
         "pushforward of arc length, and the ideal density is its Gaussian "
         "convolution above a positive floor:"))
    add(("eq", r"\nu_t^* = \xi(\cdot,t)_{\#}\,ds,\qquad "
               r"\phi^*(q,t) = \phi_0 + \bigl(K_\sigma * \nu_t^*\bigr)(q) "
               r"= \phi_0 + \int_{\Gamma(t)} K_\sigma\bigl(q-\xi(s,t)\bigr)\,ds",
         None))
    add(("para",
         "The kernel is the \\emph{unnormalised} Gaussian "
         "$K_\\sigma(x) = \\exp\\bigl(-\\|x\\|^2/(2\\sigma^2)\\bigr)$. The local cell, "
         "its mass and its centroid are"))
    add(("eq", r"V_i^\ell(P) = \mathrm{Vor}_i(P)\cap D\cap B(p_i,R_\ell),\qquad "
               r"m_i^* = \int_{V_i^\ell}\phi^*\,dq,\qquad "
               r"c_i^* = \frac{1}{m_i^*}\int_{V_i^\ell} q\,\phi^*(q,t)\,dq",
         None))
    add(("para",
         "$\\widehat\\phi_i$ is agent $i$'s mapped density, $\\widehat c_i$ its "
         "numerically computed centroid, $\\mathcal{C}^*(t)$ the selected local "
         "minimiser branch inside a fixed compact regular tube $X$, and"))
    add(("eq", r"V(P,t) = H^*(P,t) - H^*_{\min}(t) \ \ge\ 0,\qquad "
               r"H^*_{\min}(t) = \min_{P\in X} H^*(P,t)", None))

    add(("h2", "2.1 Kernel constants"))
    add(("para",
         "These three constants are used throughout; all are Wolfram-verified "
         "(entries W-RR-01 to W-RR-03) and derived in Step 1 of Lemma 1."))
    add(("eq", r"\|K_\sigma\|_1 = 2\pi\sigma^2,\qquad "
               r"\|\nabla K_\sigma\|_1 = \sqrt{2}\,\pi^{3/2}\sigma,\qquad "
               r"\|\nabla K_\sigma\|_\infty = \frac{e^{-1/2}}{\sigma}",
         "K"))

    add(("h2", "2.2 Units"))
    add(("para",
         "Arc length is in metres and $K_\\sigma$ is dimensionless, so $\\phi$ has "
         "units of m, an $L^1$ density error and a cell mass have units "
         "$\\mathrm{m}^3$, a shifted first moment $\\mathrm{m}^4$, $H^*$ has "
         "$\\mathrm{m}^5$, $\\nabla H^*$ has $\\mathrm{m}^4$, and a command "
         "disturbance has $\\mathrm{m}/\\mathrm{s}$. The ledger in Appendix A "
         "carries the unit of every constant so that geometric length and "
         "density mass are never silently added."))

    # ---------------------------------------------------------------- #
    # 3. Assumptions
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "3. Assumptions A1-A10"))
    add(("table", "assumptions", [
        ["ID", "Assumption", "How it is discharged"],
        ["A1", "$D$ is a fixed rectangle and the solution remains in a fixed compact "
               "configuration tube $X$ on the claimed interval.",
         "assumption / invariant-domain check"],
        ["A2", "Pairwise separation is at least $d_s>0$, and $R_{\\mathrm{comm}} \\ge 2R_\\ell$.",
         "runtime certificate + config validator (fail-closed)"],
        ["A3", "$\\Gamma(t)$ is closed and piecewise $C^2$ with $P_\\Gamma \\le P_{\\max}$; "
               "version 1 uses a rigid boundary with bounded twist.",
         "assumption / scenario certificate"],
        ["A4", "$\\phi_0>0$, $\\sigma>0$, and the unnormalised Gaussian above is used.",
         "analytic + config"],
        ["A5", "Map weights are nonnegative and a certificate supplies "
               "$\\varepsilon_b,\\varepsilon_n,\\varepsilon_d$, voxel $v$, $h_s$, $E_w$, "
               "$M_{\\mathrm{drop}}$ and $s_\\sigma$ as upper bounds.",
         "runtime / offline certificate"],
        ["A6", "Local CVT uses a rectangular midpoint grid of side at most $h$, and "
               "$\\eta_m < m_{\\min}$.",
         "code contract + fail-closed test"],
        ["A7", "Gap, frontier and transport density biases are disabled in theorem "
               "mode, or their stacked command effect is included in $\\delta$.",
         "config + perturbation ledger"],
        ["A8", "On a symmetry-reduced convex chart of the selected branch, "
               "$\\nabla^2 H^*_{\\mathrm{red}}(z,t) \\succeq m_H I$ uniformly, $m_H>0$.",
         "CONDITIONAL - no interval certificate supplied"],
        ["A9", "Voronoi ties and the threshold set $\\|q-p_i\\|=R_\\ell$ have measure "
               "zero, and the solution stays on one regular branch tube.",
         "local theorem assumption"],
        ["A10", "ZOH, numerical, tracking and safety-filter deviations from the ideal "
                "field have certified stacked-norm bounds.",
         "analytic + runtime certificate"],
    ]))
    add(("callout", "STATUS OF THE ASSUMPTION SET",
         "The implication A1-A10 $\\Rightarrow$ Theorem 1 is proved. A5, A8, A9 and "
         "A10 are not facts about every run: they are explicit assumptions and "
         "certificate gates. The result is therefore a complete conditional local "
         "theorem, not an unconditional system-success claim."))

    # ---------------------------------------------------------------- #
    # 4. Lemma 1
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "4. Lemma 1 - boundary and map to density consistency"))
    add(("h2", "4.1 Statement"))
    add(("para",
         "Assume A3-A5, let the arc partition satisfy $0 \\le \\Delta s_k \\le h_s$, "
         "and suppose every source discarded by the restriction step is at least "
         "$s_\\sigma\\sigma$ away from every query point in "
         "$B(p_i,R_\\ell)\\cap D$. Define the target, weight and tail errors"))
    add(("eq", r"\varepsilon_{\xi,i} = \varepsilon_{b,i} + \varepsilon_{d,i} "
               r"+ 2 d_{\max}\sin\!\left(\frac{\varepsilon_{n,i}}{2}\right),"
               r"\qquad \varepsilon_{x,i} = \varepsilon_{\xi,i} + \sqrt{2}\,v", None))
    add(("eq", r"E_{w,i} = L_{\mathrm{miss},i} + M_{\mathrm{spur},i} "
               r"+ \sum_{k\,\in\,\mathrm{matched}} \bigl|\widehat w_{ik} - \Delta s_k\bigr|",
         None))
    add(("para", "Then"))
    add(("eq", r"\bigl\|\widehat\phi_i - \phi^*\bigr\|_{L^1(B(p_i,R_\ell)\cap D)} "
               r"\ \le\ \varepsilon_{\phi,i} "
               r"\ :=\ L_{K,1} P_{\max}\!\left(\varepsilon_{\xi,i} + \sqrt{2}\,v "
               r"+ \frac{L_T h_s}{2}\right) + 2\pi\sigma^2 E_{w,i} "
               r"+ \varepsilon_{\mathrm{tail},i}", "L1"))
    add(("para", "with $L_{K,1} := \\|\\nabla K_\\sigma\\|_1$ and"))
    add(("eq", r"\varepsilon_{\mathrm{tail},i} \ \le\ \pi R_\ell^2\,"
               r"M_{\mathrm{drop},i}\,e^{-s_\sigma^2/2}", None))
    add(("callout", "WHY THREE MEASURES",
         "The comparison is made after convolution, through an exact discrete "
         "intermediate measure. It is never claimed that a continuous arc-length "
         "measure converges to a sum of Dirac masses in total variation - that "
         "statement is false."))
    add(("eq", r"\nu_t^*\ \longrightarrow\ "
               r"\nu_{h,t}^* = \sum_k \Delta s_k\,\delta_{\xi_k^*}\ \longrightarrow\ "
               r"\widehat\nu_{i,t} = \sum_k \widehat w_{ik}\,\delta_{\widehat\xi_{ik}}",
         None))

    add(("h2", "4.2 Proof"))

    add(("h3", "Step 1. The three kernel constants"))
    add(("para", "The mass separates into one-dimensional Gaussians:"))
    add(("eq", r"\|K_\sigma\|_1 = \int_{\mathbb{R}^2} e^{-\|x\|^2/(2\sigma^2)}dx "
               r"= \left(\int_{\mathbb{R}} e^{-x^2/(2\sigma^2)}dx\right)^{\!2} "
               r"= \bigl(\sigma\sqrt{2\pi}\bigr)^2 = 2\pi\sigma^2", None))
    add(("para",
         "Since $\\nabla K_\\sigma(x) = -\\,\\sigma^{-2} x\\,K_\\sigma(x)$, its magnitude "
         "is radial, $\\|\\nabla K_\\sigma(x)\\| = (r/\\sigma^2)e^{-r^2/(2\\sigma^2)}$ "
         "with $r = \\|x\\|$. Integrating in polar coordinates,"))
    add(("eq", r"\|\nabla K_\sigma\|_1 = \int_0^{\infty}\frac{r}{\sigma^2}"
               r"e^{-r^2/(2\sigma^2)}\,2\pi r\,dr "
               r"= \frac{2\pi}{\sigma^2}\int_0^{\infty} r^2 e^{-r^2/(2\sigma^2)}dr "
               r"= \frac{2\pi}{\sigma^2}\cdot\sigma^3\sqrt{\frac{\pi}{2}} "
               r"= \sqrt{2}\,\pi^{3/2}\sigma", None))
    add(("para",
         "For the sup-norm, write $g(r) = (r/\\sigma^2)e^{-r^2/(2\\sigma^2)}$. Then"))
    add(("eq", r"g'(r) = \frac{1}{\sigma^2}e^{-r^2/(2\sigma^2)}"
               r"\left(1 - \frac{r^2}{\sigma^2}\right) = 0 "
               r"\ \Longleftrightarrow\ r = \sigma \quad (r>0)", None))
    add(("para",
         "and $g(0) = 0$, $g(r)\\to 0$ as $r\\to\\infty$, so the interior "
         "stationary point is the maximum:"))
    add(("eq", r"\|\nabla K_\sigma\|_\infty = g(\sigma) "
               r"= \frac{\sigma}{\sigma^2}e^{-1/2} = \frac{e^{-1/2}}{\sigma}", None))

    add(("h3", "Step 2. Translation inequality in $L^1$"))
    add(("para",
         "This is the workhorse of the whole lemma: moving a source by a distance "
         "costs at most $\\|\\nabla K_\\sigma\\|_1$ times that distance, in $L^1$. "
         "For $a,b \\in \\mathbb{R}^2$ the fundamental theorem of calculus along the "
         "segment $\\tau \\mapsto b + \\tau(a-b)$ gives"))
    add(("eq", r"K_\sigma(x-a) - K_\sigma(x-b) "
               r"= -\int_0^1 \nabla K_\sigma\bigl(x - b - \tau(a-b)\bigr)\cdot(a-b)\,d\tau",
         None))
    add(("para",
         "Taking absolute values, integrating in $x$ and exchanging the order of "
         "integration by Tonelli (the integrand is nonnegative),"))
    add(("eq", r"\bigl\|K_\sigma(\cdot-a)-K_\sigma(\cdot-b)\bigr\|_1 "
               r"\le \|a-b\|\int_0^1\!\!\int_{\mathbb{R}^2}"
               r"\bigl\|\nabla K_\sigma\bigl(x-b-\tau(a-b)\bigr)\bigr\|\,dx\,d\tau "
               r"= \|\nabla K_\sigma\|_1\,\|a-b\|", "T"))
    add(("para",
         "the inner integral being translation-invariant and therefore equal to "
         "$\\|\\nabla K_\\sigma\\|_1$ for every $\\tau$."))

    add(("h3", "Step 3. Target error"))
    add(("para",
         "With $\\xi = b + d\\,n$ and the observed $\\widehat\\xi = \\widehat b "
         "+ \\widehat d\\,\\widehat n$, the triangle inequality gives"))
    add(("eq", r"\|\widehat\xi - \xi\| \le \|\widehat b - b\| "
               r"+ |\widehat d - d| + d_{\max}\,\|\widehat n - n\|", None))
    add(("para",
         "Both normals are unit vectors, so if they subtend an angle $\\theta$ then"))
    add(("eq", r"\|\widehat n - n\| = \sqrt{2 - 2\cos\theta} = 2\sin\frac{\theta}{2},"
               r"\qquad 0\le\theta\le\pi", None))
    add(("para",
         "which is monotone in $\\theta$; substituting $\\theta \\le \\varepsilon_n$ "
         "yields $\\varepsilon_\\xi$. Replacing a target by a representative of the "
         "same square voxel of side $v$ moves it by at most the voxel diagonal "
         "$\\sqrt2\\,v$, which gives $\\varepsilon_x = \\varepsilon_\\xi + \\sqrt2\\,v$."))
    add(("callout", "SCOPE OF THE VOXEL TERM",
         "$\\sqrt2\\,v$ is valid only while the fused representative stays inside "
         "its own voxel. If the fusion law can move it outside, the bound must be "
         "re-derived from that law; it must not be reused as written."))

    add(("h3", "Step 4. Curve quadrature"))
    add(("para",
         "Compare the exact discrete measure with the continuous one. On arc $k$, "
         "of length $\\Delta s_k$ and with representative $\\xi_k^*$,"))
    add(("eq", r"K_\sigma * \nu_h^* - K_\sigma * \nu^* "
               r"= \sum_k \int_{\mathrm{arc}_k}\Bigl[K_\sigma(\cdot-\xi_k^*) "
               r"- K_\sigma\bigl(\cdot-\xi(s)\bigr)\Bigr]ds", None))
    add(("para",
         "Apply (T) inside the sum. If $\\xi$ is $L_T$-Lipschitz in arc length "
         "then $\\|\\xi_k^* - \\xi(s)\\| \\le L_T|s - s_k|$, and for any "
         "representative in the arc $\\int_{\\mathrm{arc}_k}|s-s_k|\\,ds "
         "\\le \\Delta s_k^2/2$. Hence"))
    add(("eq", r"\bigl\|K_\sigma * \nu_h^* - K_\sigma * \nu^*\bigr\|_1 "
               r"\le L_{K,1}L_T\sum_k \frac{\Delta s_k^2}{2}", None))
    add(("para",
         "Because $0 \\le \\Delta s_k \\le h_s$ we have "
         "$\\Delta s_k^2 \\le h_s\\Delta s_k$, so the sum telescopes into the "
         "perimeter (Wolfram entry W-RR-05):"))
    add(("eq", r"\sum_k \Delta s_k^2 \ \le\ h_s\sum_k \Delta s_k "
               r"\ =\ h_s P_\Gamma \ \le\ h_s P_{\max} "
               r"\quad\Longrightarrow\quad "
               r"\bigl\|K_\sigma * \nu_h^* - K_\sigma * \nu^*\bigr\|_1 "
               r"\le \tfrac12 L_{K,1}L_T P_{\max} h_s", None))
    add(("para",
         "Choosing the arc midpoint would give the sharper factor "
         "$\\Delta s_k^2/4$; the certificate uses the conservative $1/2$ so that "
         "it stays valid for any registered representative inside the arc."))

    add(("h3", "Step 5. Map displacement and weight mismatch"))
    add(("para",
         "Match the observed atoms to the exact ones and split the difference "
         "into a displacement part, a matched-weight part, and "
         "missing/spurious mass:"))
    add(("eq", r"\widehat\nu_i - \nu_h^* "
               r"= \underbrace{\sum_{\mathrm{matched}}\Delta s_k"
               r"\bigl(\delta_{\widehat\xi_k}-\delta_{\xi_k^*}\bigr)}_{\text{displacement}} "
               r"+ \underbrace{\sum_{\mathrm{matched}}"
               r"\bigl(\widehat w_k-\Delta s_k\bigr)\delta_{\widehat\xi_k}}_{\text{weight}} "
               r"- \sum_{\mathrm{missed}}\Delta s_k\,\delta_{\xi_k^*} "
               r"+ \sum_{\mathrm{spurious}}\widehat w_k\,\delta_{\widehat\xi_k}", None))
    add(("para", "Convolve and bound each group. By (T), the displacement group costs"))
    add(("eq", r"\Bigl\|\sum_{\mathrm{matched}}\Delta s_k"
               r"\bigl(K_\sigma(\cdot-\widehat\xi_k)-K_\sigma(\cdot-\xi_k^*)\bigr)\Bigr\|_1 "
               r"\le L_{K,1}\,\varepsilon_x \sum_k \Delta s_k "
               r"\le L_{K,1}P_{\max}\,\varepsilon_x", None))
    add(("para",
         "The remaining three groups are bounded by their total variation times "
         "the kernel mass, because $\\|w\\,K_\\sigma(\\cdot-y)\\|_1 = |w|\\,\\|K_\\sigma\\|_1$:"))
    add(("eq", r"\bigl\|K_\sigma * (\text{weight} + \text{missed} + \text{spurious})\bigr\|_1 "
               r"\le \|K_\sigma\|_1\Bigl(L_{\mathrm{miss}} + M_{\mathrm{spur}} "
               r"+ \sum_{\mathrm{matched}}|\widehat w_k - \Delta s_k|\Bigr) "
               r"= 2\pi\sigma^2 E_w", None))

    add(("h3", "Step 6. Restriction tail"))
    add(("para",
         "The restriction step discards far sources. It does not do so for free: "
         "the Gaussian has no compact support. A discarded atom of weight "
         "$\\widehat w_k$ lies at distance at least $s_\\sigma\\sigma$ from every "
         "query $q$ in the disk, so pointwise"))
    add(("eq", r"K_\sigma(q-\widehat\xi_k) "
               r"= \exp\!\left(-\frac{\|q-\widehat\xi_k\|^2}{2\sigma^2}\right) "
               r"\le \exp\!\left(-\frac{s_\sigma^2}{2}\right)", None))
    add(("para",
         "Summing the discarded weights and integrating over an area no larger "
         "than $\\pi R_\\ell^2$,"))
    add(("eq", r"\varepsilon_{\mathrm{tail},i} "
               r"= \int_{B(p_i,R_\ell)\cap D}\sum_{\mathrm{dropped}}"
               r"\widehat w_k K_\sigma(q-\widehat\xi_k)\,dq "
               r"\le \pi R_\ell^2 M_{\mathrm{drop},i}\,e^{-s_\sigma^2/2}", None))

    add(("h3", "Step 7. Assembly"))
    add(("para",
         "The common base density $\\phi_0$ appears in both $\\widehat\\phi_i$ and "
         "$\\phi^*$ and cancels. Adding Steps 4, 5 and 6 by the triangle "
         "inequality and restricting the global $L^1$ estimate to "
         "$B(p_i,R_\\ell)\\cap D$,"))
    add(("chain", [
        (r"\bigl\|\widehat\phi_i-\phi^*\bigr\|_{L^1} "
         r"\le \bigl\|K_\sigma*\widehat\nu_i - K_\sigma*\nu_h^*\bigr\|_1 "
         r"+ \bigl\|K_\sigma*\nu_h^* - K_\sigma*\nu^*\bigr\|_1 "
         r"+ \varepsilon_{\mathrm{tail},i}", "triangle inequality"),
        (r"\le L_{K,1}P_{\max}\varepsilon_{x,i} + 2\pi\sigma^2 E_{w,i} "
         r"+ \tfrac12 L_{K,1}L_T P_{\max}h_s + \varepsilon_{\mathrm{tail},i}",
         "Steps 4-6"),
        (r"= L_{K,1}P_{\max}\!\left(\varepsilon_{\xi,i}+\sqrt2\,v "
         r"+ \frac{L_T h_s}{2}\right) + 2\pi\sigma^2 E_{w,i} "
         r"+ \varepsilon_{\mathrm{tail},i} \;=\; \varepsilon_{\phi,i}",
         "definition of $\\varepsilon_{x,i}$"),
    ]))
    add(("para", "which is (L1). $\\qquad\\blacksquare$"))

    add(("h2", "4.3 Evidence boundary"))
    add(("para",
         "Wolfram checked the two kernel integrals exactly, and the sup-norm, the "
         "normal-angle identity and the bounded-step inequality symbolically. The "
         "pushforward construction, the translation inequality (T), the curve "
         "quadrature, the atom matching of Step 5 and the tail decomposition are "
         "the analytic argument above; no computer-algebra output is offered in "
         "their place."))

    # ---------------------------------------------------------------- #
    # 5. Lemma 2
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "5. Lemma 2 - mass, quadrature and centroid perturbation"))
    add(("h2", "5.1 Statement"))
    add(("para", "Assume A1-A6 and the exact-cell contract "
                 "$R_{\\mathrm{comm}} \\ge 2R_\\ell$. Put"))
    add(("eq", r"r_0 = \min\Bigl\{R_\ell,\ \tfrac{d_s}{2},\ \tfrac{W_D}{2},"
               r"\ \tfrac{H_D}{2}\Bigr\},\qquad "
               r"m_{\min} = \frac{\phi_0\pi r_0^2}{4} > 0,\qquad "
               r"m_{\max} = \pi R_\ell^2\bigl(\phi_0+P_{\max}\bigr)", None))
    add(("eq", r"\widehat\phi_{\max} = \phi_0+P_{\max}+E_{w,i},\qquad "
               r"\widehat L_\phi \le \bigl(P_{\max}+E_{w,i}\bigr)\frac{e^{-1/2}}{\sigma}",
         None))
    add(("para",
         "For a midpoint mesh of side at most $h$ set $r_h = h/\\sqrt2$ (the "
         "half-diagonal), $A_V \\le \\pi R_\\ell^2$ and $P_V \\le 2\\pi R_\\ell$, and "
         "define"))
    add(("eq", r"\eta_m = \widehat L_\phi\, r_h A_V "
               r"+ \widehat\phi_{\max}\bigl(2P_V r_h + \pi r_h^2\bigr)", None))
    add(("eq", r"\eta_a = \bigl(\widehat\phi_{\max}+R_\ell \widehat L_\phi\bigr) r_h A_V "
               r"+ R_\ell\widehat\phi_{\max}\bigl(2P_V r_h + \pi r_h^2\bigr)", None))
    add(("para", "If the gate $\\eta_m < m_{\\min}$ holds, set"))
    add(("eq", r"\varepsilon_q = \frac{\eta_a + R_\ell\,\eta_m}{m_{\min}-\eta_m}", None))
    add(("para", "Then the implementation centroid obeys"))
    add(("eq", r"\bigl\|\widehat c_i - c_i^*\bigr\| \ \le\ \varepsilon_{c,i} "
               r"\ :=\ \frac{2R_\ell\,\varepsilon_{\phi,i}}{m_{\min}} "
               r"+ \varepsilon_{q,i} + \varepsilon_{G,i}", "L2"))
    add(("para",
         "In strict synchronised theorem mode $\\varepsilon_{G,i}=0$. Delayed or "
         "missing neighbour positions require a separate geometric cell-error "
         "certificate; that error must not be hidden inside "
         "$\\varepsilon_{\\phi,i}$."))

    add(("h2", "5.2 Proof"))

    add(("h3", "Step 1. Mass floor"))
    add(("para",
         "First, separation alone puts a whole disk inside agent $i$'s Voronoi "
         "cell. Let $\\|q-p_i\\| < d_s/2$. For any $j \\ne i$,"))
    add(("eq", r"\|q-p_j\| \ \ge\ \|p_i-p_j\| - \|q-p_i\| "
               r"\ \ge\ d_s - \frac{d_s}{2} \ =\ \frac{d_s}{2} \ >\ \|q-p_i\|", None))
    add(("para",
         "so $q$ is strictly closer to $p_i$, i.e. $B(p_i,d_s/2)\\subset "
         "\\mathrm{Vor}_i(P)$. Intersecting with the local disk and the rectangle, "
         "$V_i^\\ell \\supseteq B(p_i,r_0)\\cap D$. The worst placement of $p_i$ in a "
         "rectangle is a corner, which retains a quarter disk, so "
         "$|V_i^\\ell| \\ge \\pi r_0^2/4$. Since $\\phi^* \\ge \\phi_0$,"))
    add(("eq", r"m_i^* = \int_{V_i^\ell}\phi^*\,dq "
               r"\ \ge\ \phi_0\cdot\frac{\pi r_0^2}{4} \ =\ m_{\min} \ >\ 0", None))
    add(("callout", "NON-RECTANGULAR WORKSPACES",
         "The quarter-disk constant is specific to a rectangle. For another "
         "workspace, replace it with a uniform interior-cone constant; the "
         "quarter-disk value must not be carried over."))

    add(("h3", "Step 2. Mass ceiling and mapped-density bounds"))
    add(("para", "Since $K_\\sigma \\le 1$ pointwise and $P_\\Gamma \\le P_{\\max}$,"))
    add(("eq", r"\phi^*(q,t) = \phi_0 + \int_{\Gamma(t)}K_\sigma(q-\xi)\,ds "
               r"\ \le\ \phi_0 + P_\Gamma \ \le\ \phi_0 + P_{\max}", None))
    add(("para", "and $|V_i^\\ell| \\le |B(p_i,R_\\ell)| = \\pi R_\\ell^2$, hence"))
    add(("eq", r"m_i^* \ \le\ \pi R_\ell^2\bigl(\phi_0+P_{\max}\bigr) \ =\ m_{\max}", None))
    add(("para",
         "The measure decomposition of Lemma 1 bounds the mapped total weight by "
         "$P_{\\max}+E_w$, which gives $\\widehat\\phi_{\\max}$ by the same argument. "
         "Differentiating under the integral sign and using "
         "$\\|\\nabla K_\\sigma\\|_\\infty = e^{-1/2}/\\sigma$ gives the Lipschitz "
         "constant $\\widehat L_\\phi$."))

    add(("h3", "Step 3. Midpoint mass error"))
    add(("para",
         "Split the midpoint cells into those lying wholly inside $V_i^\\ell$ and "
         "those cut by its boundary. For an interior cell $Q$ with midpoint "
         "$q_Q$, Lipschitz continuity gives"))
    add(("eq", r"\left|\int_Q \phi\,dq - \phi(q_Q)\,|Q|\right| "
               r"\le \widehat L_\phi \max_{q\in Q}\|q-q_Q\|\,|Q| "
               r"= \widehat L_\phi\, r_h\,|Q|", None))
    add(("para",
         "because the farthest point of a cell from its midpoint is a corner, at "
         "distance $r_h = h/\\sqrt2$. Summing over interior cells contributes at "
         "most $\\widehat L_\\phi r_h A_V$."))
    add(("para",
         "The cut cells all lie within distance $r_h$ of $\\partial V_i^\\ell$. "
         "$V_i^\\ell$ is convex, being an intersection of half-planes, a disk and a "
         "rectangle, so the inner and outer parallel sets of that width have "
         "combined area at most $2P_V r_h + \\pi r_h^2$. Bounding the density there "
         "by $\\widehat\\phi_{\\max}$ and adding the two groups gives $\\eta_m$. "
         "Convexity also justifies $P_V \\le 2\\pi R_\\ell$, the perimeter of the "
         "enclosing disk."))

    add(("h3", "Step 4. First-moment error"))
    add(("para",
         "Repeat Step 3 for the \\emph{shifted} first moment, which is what the "
         "centroid actually needs:"))
    add(("eq", r"a = \int_{V_i^\ell}(q-p_i)\,\phi(q)\,dq", None))
    add(("para", "Its integrand $f(q) = (q-p_i)\\phi(q)$ satisfies, on the cell,"))
    add(("eq", r"\|f(q)\| \le R_\ell\,\widehat\phi_{\max},\qquad "
               r"\mathrm{Lip}(f) \le \widehat\phi_{\max} + R_\ell \widehat L_\phi",
         None))
    add(("para",
         "since $\\|q-p_i\\| \\le R_\\ell$ there. Substituting these two constants "
         "for $\\widehat L_\\phi$ and $\\widehat\\phi_{\\max}$ in Step 3 yields "
         "$\\|\\widehat a - a\\| \\le \\eta_a$."))

    add(("h3", "Step 5. Ratio perturbation"))
    add(("para",
         "The centroid is a ratio, so the two errors must be combined through a "
         "quotient bound rather than added. Write $\\widehat c - p_i "
         "= \\widehat a/\\widehat m$ and $c - p_i = a/m$. Then"))
    add(("eq", r"\frac{\widehat a}{\widehat m} - \frac{a}{m} "
               r"= \frac{\widehat a - a}{\widehat m} "
               r"+ a\left(\frac{1}{\widehat m}-\frac{1}{m}\right) "
               r"= \frac{\widehat a - a}{\widehat m} "
               r"+ \frac{a\,(m-\widehat m)}{m\,\widehat m}", None))
    add(("para",
         "The gate $\\eta_m < m_{\\min}$ makes the denominator safely positive, "
         "$\\widehat m \\ge m - \\eta_m \\ge m_{\\min}-\\eta_m > 0$, and "
         "$\\|a\\| \\le R_\\ell m$ because $\\|q-p_i\\|\\le R_\\ell$ on the cell. "
         "Therefore"))
    add(("chain", [
        (r"\left\|\frac{\widehat a}{\widehat m}-\frac{a}{m}\right\| "
         r"\le \frac{\|\widehat a-a\|}{\widehat m} "
         r"+ \frac{\|a\|}{m}\cdot\frac{|m-\widehat m|}{\widehat m}", "triangle inequality"),
        (r"\le \frac{\eta_a}{m_{\min}-\eta_m} "
         r"+ R_\ell\,\frac{\eta_m}{m_{\min}-\eta_m}", "Steps 3-4 and $\\|a\\|\\le R_\\ell m$"),
        (r"= \frac{\eta_a + R_\ell \eta_m}{m_{\min}-\eta_m} = \varepsilon_q",
         "definition"),
    ]))

    add(("h3", "Step 6. Density perturbation on the same cell"))
    add(("para",
         "Now hold the cell fixed and change only the density. Lemma 1 gives"))
    add(("eq", r"\bigl|\widetilde m - m^*\bigr| "
               r"= \left|\int_{V_i^\ell}(\widehat\phi_i-\phi^*)\,dq\right| "
               r"\le \varepsilon_\phi,\qquad "
               r"\bigl\|\widetilde a - a^*\bigr\| "
               r"= \left\|\int_{V_i^\ell}(q-p_i)(\widehat\phi_i-\phi^*)\,dq\right\| "
               r"\le R_\ell\,\varepsilon_\phi", None))
    add(("para",
         "Both densities keep the base-density floor, so both masses are at least "
         "$m_{\\min}$. Applying the same ratio identity as in Step 5,"))
    add(("eq", r"\bigl\|\widetilde c - c^*\bigr\| "
               r"\le \frac{R_\ell\varepsilon_\phi}{m_{\min}} "
               r"+ R_\ell\cdot\frac{\varepsilon_\phi}{m_{\min}} "
               r"= \frac{2R_\ell\,\varepsilon_\phi}{m_{\min}}", None))

    add(("h3", "Step 7. Assembly"))
    add(("para",
         "Adding the density error of Step 6, the midpoint error of Step 5 and "
         "the separately certified geometric cell error gives (L2). "
         "$\\qquad\\blacksquare$"))
    add(("para",
         "The denominator gate is fail-closed in the certificate generator: a "
         "configuration with $\\eta_m \\ge m_{\\min}$ produces no certificate at "
         "all rather than a large one."))

    # ---------------------------------------------------------------- #
    # 6. Proposition 1
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "6. Proposition 1 - ideal Local-CVT gradient and descent"))
    add(("h2", "6.1 Statement"))
    add(("para",
         "At every regular configuration covered by A1-A4 and A9,"))
    add(("eq", r"\nabla_{p_i}H^*(P,t) = 2\,m_i^*\,\bigl(p_i - c_i^*\bigr)", "P1-a"))
    add(("para", "so the ideal Lloyd command is a scaled gradient step,"))
    add(("eq", r"u_i^* = -k_c\bigl(p_i-c_i^*\bigr) "
               r"= -\frac{k_c}{2m_i^*}\,\nabla_{p_i}H^*", None))
    add(("para", "and, with $A(P,t) = \\mathrm{diag}_i\\bigl(\\tfrac{k_c}{2m_i^*}I_2\\bigr)$,"))
    add(("eq", r"A(P,t) \succeq a_0 I,\quad a_0 = \frac{k_c}{2m_{\max}},"
               r"\qquad \dot H^* = -\nabla H^{*\top}A\,\nabla H^* "
               r"\le -a_0\bigl\|\nabla H^*\bigr\|^2", "P1-b"))
    add(("para", "the last identity holding with the reference frozen."))

    add(("h2", "6.2 Proof"))
    add(("h3", "Step 1. Partition the integral"))
    add(("para",
         "Away from the zero-measure tie and threshold sets of A9, the domain "
         "splits into the local cells and the saturated remainder "
         "$U = D\\setminus\\bigcup_j B(p_j,R_\\ell)$, on which the running minimum "
         "equals $R_\\ell^2$:"))
    add(("eq", r"H^* = \sum_j \int_{V_j^\ell}\|q-p_j\|^2\,\phi^*\,dq "
               r"\;+\; R_\ell^2\int_U \phi^*\,dq", None))

    add(("h3", "Step 2. Volume term"))
    add(("para",
         "Differentiating the integrand of the $j=i$ term with respect to $p_i$,"))
    add(("eq", r"\frac{\partial}{\partial p_i}\|q-p_i\|^2 = 2\,(p_i-q) "
               r"\quad\Longrightarrow\quad "
               r"\int_{V_i^\ell}2(p_i-q)\phi^*\,dq "
               r"= 2\left[m_i^* p_i - \int_{V_i^\ell}q\,\phi^*dq\right] "
               r"= 2m_i^*\bigl(p_i-c_i^*\bigr)", None))
    add(("para",
         "No other volume term depends on $p_i$: for $j\\ne i$ the integrand "
         "$\\|q-p_j\\|^2$ does not involve $p_i$, and the $U$ integrand is the "
         "constant $R_\\ell^2$."))

    add(("h3", "Step 3. The moving-boundary terms cancel"))
    add(("para",
         "This is the step that the saturated cost exists to make work. Moving "
         "$p_i$ deforms $\\partial V_i^\\ell$, so Reynolds' transport theorem "
         "contributes a flux for each moving piece of boundary. There are three "
         "kinds, and each vanishes for its own reason."))
    add(("bullets", [
        "\\emph{Shared Voronoi faces.} On the face between cells $i$ and $j$ we "
        "have $\\|q-p_i\\| = \\|q-p_j\\|$, so the two cells carry the same "
        "integrand value there. Whatever area cell $i$ gains, cell $j$ loses, "
        "with equal and opposite normal velocity; the two fluxes cancel exactly.",
        "\\emph{The moving circle} $\\|q-p_i\\| = R_\\ell$. Just inside, the "
        "integrand is $\\|q-p_i\\|^2 = R_\\ell^2$; just outside, the saturated "
        "value is also $R_\\ell^2$. The integrand is continuous across the "
        "circle, so the flux cancels. Without saturation the outer value would "
        "be $0$, the jump would be $R_\\ell^2$, and a term proportional to "
        "$R_\\ell^2\\oint\\phi^*$ would survive.",
        "\\emph{The workspace boundary} $\\partial D$ is fixed, so its normal "
        "velocity is zero.",
    ]))
    add(("para", "Only the volume derivative survives, which is (P1-a)."))

    add(("h3", "Step 4. Descent"))
    add(("para",
         "By Lemma 2, $m_i^* \\le m_{\\max}$, so every diagonal block satisfies "
         "$\\tfrac{k_c}{2m_i^*} \\ge \\tfrac{k_c}{2m_{\\max}} = a_0$, i.e. "
         "$A \\succeq a_0 I$. Substituting $\\dot P = -A\\nabla H^*$ with the "
         "reference frozen,"))
    add(("eq", r"\dot H^* = \nabla H^{*\top}\dot P = -\nabla H^{*\top}A\nabla H^* "
               r"\le -a_0\|\nabla H^*\|^2 \ \le\ 0", None))

    add(("h3", "Step 5. The implemented cell is exact, not approximate"))
    add(("para",
         "Suppose $q \\in B(p_i,R_\\ell)$ and agent $j$ beats agent $i$ at $q$, i.e. "
         "$\\|q-p_j\\| \\le \\|q-p_i\\| \\le R_\\ell$. Then"))
    add(("eq", r"\|p_j-p_i\| \le \|p_j-q\| + \|q-p_i\| \le 2R_\ell "
               r"\ \le\ R_{\mathrm{comm}}", None))
    add(("para",
         "so every possible local competitor is already inside the communication "
         "neighbourhood. Under A2 the implemented cell therefore \\emph{equals} "
         "$V_i^\\ell$; this is an identity, not an approximation. "
         "$\\qquad\\blacksquare$"))
    add(("callout", "IF $R_{\\mathrm{comm}} = R_\\ell$",
         "The argument above needs $2R_\\ell$. A theorem-mode configuration with "
         "$R_{\\mathrm{comm}} < 2R_\\ell$ is rejected by the validator, and the "
         "exact-cell claim would have to be marked BLOCKED rather than weakened "
         "silently."))
    add(("para",
         "Wolfram checked the integrand derivative and one exact fixed-square "
         "example. Those are sanity checks; the shape-derivative cancellation in "
         "Step 3 is the proof."))

    # ---------------------------------------------------------------- #
    # 7. Proposition 2
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "7. Proposition 2 - aggregate implementation disturbance"))
    add(("h2", "7.1 Statement"))
    add(("para", "Write the realised stacked dynamics as an ideal field plus a disturbance:"))
    add(("eq", r"\dot P = -A(P,t)\,\nabla H^*(P,t) + d(t),\qquad \|d(t)\|\le\delta", None))
    add(("eq", r"\delta = k_c\sqrt{\sum_{i=1}^N \varepsilon_{c,i}^2} "
               r"+ \varepsilon_{\mathrm{zoh}} + \varepsilon_{\mathrm{num}} "
               r"+ \varepsilon_{\mathrm{trk}} + \varepsilon_{\mathrm{sf}}", "P2"))

    add(("h2", "7.2 Proof"))
    add(("h3", "Step 1. Centroid component"))
    add(("para",
         "The nominal law and the ideal field differ only through the centroid "
         "each uses, so the gain multiplies the Lemma 2 error directly:"))
    add(("eq", r"u_i^{\mathrm{nom}} - u_i^* "
               r"= -k_c\bigl(p_i-\widehat c_i\bigr) + k_c\bigl(p_i-c_i^*\bigr) "
               r"= k_c\bigl(\widehat c_i - c_i^*\bigr) "
               r"\quad\Longrightarrow\quad "
               r"\bigl\|u_i^{\mathrm{nom}}-u_i^*\bigr\| \le k_c\,\varepsilon_{c,i}",
         None))
    add(("para", "Stacking over the $N$ agents in the frozen product norm,"))
    add(("eq", r"\Bigl\|\bigl(k_c(\widehat c_i-c_i^*)\bigr)_{i=1}^N\Bigr\| "
               r"= k_c\sqrt{\sum_i\bigl\|\widehat c_i-c_i^*\bigr\|^2} "
               r"\le k_c\sqrt{\sum_i \varepsilon_{c,i}^2}", None))

    add(("h3", "Step 2. Remaining deviations"))
    add(("para",
         "Sample-and-hold, solver/integration, low-level tracking and "
         "safety-projection deviations are each bounded in the same stacked norm "
         "and added by the triangle inequality, which gives (P2)."))

    add(("h3", "Step 3. Analytic ceiling for the safety filter"))
    add(("para",
         "The safety term admits a bound that needs no model of the filter at "
         "all, only the speed cap that both the nominal and the filtered command "
         "respect. If $\\|u_i\\| \\le u_{\\max}$ for both, then"))
    add(("eq", r"\bigl\|u_i^{\mathrm{sf}}-u_i^{\mathrm{nom}}\bigr\| "
               r"\le \bigl\|u_i^{\mathrm{sf}}\bigr\| + \bigl\|u_i^{\mathrm{nom}}\bigr\| "
               r"\le 2u_{\max} "
               r"\quad\Longrightarrow\quad "
               r"\varepsilon_{\mathrm{sf}} \le \sqrt{N\,(2u_{\max})^2} "
               r"= 2\sqrt{N}\,u_{\max}", None))
    add(("para",
         "The norm convention is read from the implementation, never guessed "
         "from the symbol. If the cap were componentwise, "
         "$\\|u_i\\|_\\infty \\le u_{\\max}$, then $\\|u_i\\|_2 \\le \\sqrt2 u_{\\max}$ "
         "and the stacked ceiling would instead be "
         "$2\\sqrt{2N}\\,u_{\\max}$. $\\qquad\\blacksquare$"))
    add(("callout", "THIS CEILING IS DELIBERATELY CRUDE",
         "$2\\sqrt N u_{\\max}$ assumes the filter may reverse every command. A "
         "measured tightening may replace it only when its provenance and "
         "coverage interval are recorded in the ledger. Any theorem-mode "
         "gap, frontier or lead-side bias must be zero or appear as a further "
         "term in (P2)."))

    # ---------------------------------------------------------------- #
    # 8. Proposition 3
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "8. Proposition 3 - reference rate and local regularity"))
    add(("h2", "8.1 Target and density rate"))
    add(("para",
         "Differentiating the offset target $\\xi = b + d\\,n$ by the product rule,"))
    add(("eq", r"\dot\xi = \dot b + \dot d\,n + d\,\dot n", None))
    add(("para",
         "Under the rigid-body certificate A3, a point of the boundary at radius "
         "at most $R_O$ from the object centre moves at speed at most "
         "$\\bar v_O + \\bar\\omega_O R_O$, the unit normal rotates at rate at most "
         "$\\bar\\omega_O$, and the offset varies at rate at most "
         "$\\bar\\omega_O L_d$. Hence"))
    add(("eq", r"\|\dot\xi\| \le \bar v_O + \bar\omega_O\bigl(R_O + d_{\max} + L_d\bigr) "
               r"=: v_{\xi,\max}", None))
    add(("para",
         "with $L_d = 0$ for the constant offset used in version 1. Differentiating "
         "the density under the integral sign and applying (T),"))
    add(("eq", r"\partial_t\phi^*(q,t) "
               r"= -\int_{\Gamma(t)}\nabla K_\sigma\bigl(q-\xi(s,t)\bigr)\cdot"
               r"\dot\xi(s,t)\,ds "
               r"\quad\Longrightarrow\quad "
               r"\nu_\phi := \bigl\|\partial_t\phi^*\bigr\|_1 "
               r"\le L_{K,1}P_{\max}\,v_{\xi,\max}", "P3-a"))

    add(("h2", "8.2 Cost and optimal-value rate"))
    add(("para",
         "Because the saturated integrand is bounded, "
         "$0 \\le \\min_i F_R \\le R_\\ell^2$, the cost rate inherits the density rate:"))
    add(("eq", r"\bigl|\partial_t H^*(P,t)\bigr| "
               r"= \left|\int_D \min_i F_R\bigl(\|q-p_i\|\bigr)\,"
               r"\partial_t\phi^*(q,t)\,dq\right| "
               r"\le R_\ell^2\int_D\bigl|\partial_t\phi^*\bigr|dq "
               r"= R_\ell^2\,\nu_\phi", None))
    add(("para",
         "The optimal value inherits it too. For a fixed compact $X$ and any two "
         "times, the elementary inequality "
         "$\\bigl|\\min f - \\min g\\bigr| \\le \\sup|f-g|$ gives"))
    add(("eq", r"\bigl|H^*_{\min}(t_2)-H^*_{\min}(t_1)\bigr| "
               r"\le \sup_{P\in X}\bigl|H^*(P,t_2)-H^*(P,t_1)\bigr| "
               r"\le R_\ell^2\nu_\phi\,|t_2-t_1|", None))
    add(("para",
         "so $H^*_{\\min}$ is Lipschitz and, being Lipschitz, differentiable almost "
         "everywhere with $\\bigl|\\dot H^*_{\\min}\\bigr| \\le R_\\ell^2\\nu_\\phi$. "
         "Adding the two contributions bounds the drift of $V = H^*-H^*_{\\min}$:"))
    add(("eq", r"\bigl|\partial_t H^* - \dot H^*_{\min}\bigr| \le \nu_H "
               r":= 2R_\ell^2\,\nu_\phi", "P3-b"))
    add(("callout", "RIGID BOUNDARIES ONLY",
         "This derivation uses a rigid twist. Deformation, a time-varying "
         "perimeter, or split/merge events break it; those cases need a direct "
         "measure-rate assumption on $\\partial_t\\nu_t^*$ instead."))

    add(("h2", "8.3 Local PL and quadratic growth (M5)"))
    add(("para",
         "Suppose A8 holds: on a convex, symmetry-reduced chart of the selected "
         "branch, $\\nabla^2 H^*_{\\mathrm{red}}(z,t) \\succeq m_H I$ with $m_H>0$. "
         "Write $f$ for $H^*$ in those coordinates and $z^*$ for a minimiser. "
         "Integrating the Hessian twice along the segment from $z^*$ to $z$ gives "
         "the standard strong-convexity inequality"))
    add(("eq", r"f(y) \ \ge\ f(z) + \nabla f(z)^\top(y-z) "
               r"+ \frac{m_H}{2}\|y-z\|^2 \qquad \forall\,y,z", None))
    add(("para", "Putting $y = z^*$ gives quadratic growth,"))
    add(("eq", r"H^* - H^*_{\min} \ \ge\ \frac{\alpha}{2}\,"
               r"\operatorname{dist}^2\bigl(P,\mathcal{C}^*(t)\bigr),\qquad \alpha = m_H",
         "QG"))
    add(("para",
         "and minimising the right-hand side over $y$ instead, at "
         "$y = z - \\nabla f(z)/m_H$, gives"))
    add(("chain", [
        (r"H^*_{\min} \ \ge\ H^*(z) - \frac{\|\nabla f(z)\|^2}{2m_H}",
         "minimise the quadratic lower bound over $y$"),
        (r"\tfrac12\bigl\|\nabla H^*\bigr\|^2 \ \ge\ "
         r"\mu\bigl[H^*-H^*_{\min}\bigr],\qquad \mu = m_H", "rearrange"),
    ]))
    add(("para",
         "which is the local Polyak-Lojasiewicz inequality used in Theorem 1. "
         "$\\qquad\\blacksquare$"))
    add(("callout", "A8 IS CONDITIONAL, AND THAT IS THE HONEST STATUS",
         "The implication above is proved. Its premise is not established here: "
         "no specific nondegenerate branch, cell combinatorics, reduced gauge, or "
         "complete parameter tube was supplied, so no validated interval "
         "enclosure of the reduced Hessian exists. A floating-point eigenvalue at "
         "one configuration would be NUMERIC-SANITY evidence and is not presented "
         "as proof. No global PL claim is made for a nonconvex CVT cost."))

    # ---------------------------------------------------------------- #
    # 9. Theorem 1
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "9. Theorem 1 - local practical stability"))
    add(("h2", "9.1 Statement"))
    add(("para",
         "Assume A1-A10, Lemmas 1-2 and Propositions 1-3, on an interval "
         "throughout which the solution remains in the regular tube $X$. Define"))
    add(("eq", r"V(P,t) = H^*(P,t)-H^*_{\min}(t)\ \ge\ 0,\qquad "
               r"a_0 = \frac{k_c}{2m_{\max}},\qquad "
               r"\lambda_H = a_0\mu,\qquad "
               r"B = \frac{\delta^2}{2a_0} + \nu_H", None))
    add(("para", "Then for every $t \\ge t_0$ in that interval"))
    add(("eq", r"V(t)\ \le\ e^{-\lambda_H(t-t_0)}V(t_0) "
               r"+ \frac{B}{\lambda_H}\Bigl(1-e^{-\lambda_H(t-t_0)}\Bigr)", "T1-a"))
    add(("para", "and, if the regular-tube hypotheses persist for all future time,"))
    add(("eq", r"\limsup_{t\to\infty}\operatorname{dist}"
               r"\bigl(P(t),\mathcal{C}^*(t)\bigr) "
               r"\ \le\ \sqrt{\frac{2B}{\alpha\,\lambda_H}}", "T1-b"))

    add(("h2", "9.2 Proof - every inequality named"))
    add(("para",
         "Differentiate $V$ along the solution at an almost-every regular time "
         "and bound each term by the result that produces it."))
    add(("chain", [
        (r"\dot V = \nabla H^{*\top}\dot P + \partial_t H^* - \dot H^*_{\min}",
         "chain rule; $H^*_{\\min}$ differentiable a.e. by Prop. 3"),
        (r"= \nabla H^{*\top}\bigl(-A\nabla H^* + d\bigr) "
         r"+ \partial_t H^* - \dot H^*_{\min}", "substitute the dynamics"),
        (r"\le -\nabla H^{*\top}A\,\nabla H^* "
         r"+ \bigl\|\nabla H^*\bigr\|\,\|d\| + \nu_H",
         "Cauchy-Schwarz, and (P3-b)"),
        (r"\le -a_0\bigl\|\nabla H^*\bigr\|^2 "
         r"+ \delta\bigl\|\nabla H^*\bigr\| + \nu_H",
         "$A \\succeq a_0 I$ by (P1-b); $\\|d\\|\\le\\delta$ by (P2)"),
        (r"\le -a_0\bigl\|\nabla H^*\bigr\|^2 "
         r"+ \frac{a_0}{2}\bigl\|\nabla H^*\bigr\|^2 + \frac{\delta^2}{2a_0} + \nu_H",
         "Young: $\\delta x \\le \\tfrac{a_0}{2}x^2 + \\tfrac{\\delta^2}{2a_0}$"),
        (r"= -\frac{a_0}{2}\bigl\|\nabla H^*\bigr\|^2 + B",
         "definition of $B$"),
        (r"\le -a_0\mu\,V + B \;=\; -\lambda_H V + B",
         "local PL: $\\tfrac12\\|\\nabla H^*\\|^2 \\ge \\mu V$"),
    ]))
    add(("para",
         "The scalar comparison lemma applied to "
         "$\\dot V \\le -\\lambda_H V + B$ integrates this to (T1-a): the solution "
         "of the corresponding equality is exactly the right-hand side of (T1-a), "
         "and a differential inequality is dominated by it. Letting "
         "$t\\to\\infty$ in (T1-a), the exponential terms vanish and"))
    add(("eq", r"\limsup_{t\to\infty} V(t) \ \le\ \frac{B}{\lambda_H}", None))
    add(("para", "Finally, quadratic growth (QG) converts the Lyapunov level into a distance,"))
    add(("eq", r"\operatorname{dist}\bigl(P,\mathcal{C}^*\bigr) "
               r"\le \sqrt{\frac{2V}{\alpha}} "
               r"\quad\Longrightarrow\quad "
               r"\limsup_{t\to\infty}\operatorname{dist}\bigl(P(t),\mathcal{C}^*(t)\bigr) "
               r"\le \sqrt{\frac{2B}{\alpha\lambda_H}}", None))
    add(("para", "which is (T1-b). $\\qquad\\blacksquare$"))

    add(("h2", "9.3 Status"))
    add(("para",
         "The implication is analytically complete and its scalar algebra is "
         "Wolfram-verified (Young, the comparison solution, and the strong "
         "convexity to PL step). The theorem remains CONDITIONAL on A5 and "
         "A8-A10 and on the run/tube certificates. It is local. It is not a "
         "global PL theorem, and it says nothing about caging or transport "
         "success."))

    # ---------------------------------------------------------------- #
    # 10. Corollaries
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "10. Corollaries"))
    add(("h2", "10.1 Frozen perfect information"))
    add(("para",
         "If every implementation error vanishes and the reference is frozen, "
         "then $\\delta = 0$ and $\\nu_H = 0$, so $B = 0$ and (T1-a) collapses to "
         "pure exponential decay:"))
    add(("eq", r"\dot V \le -\lambda_H V "
               r"\quad\Longrightarrow\quad "
               r"V(t) \le e^{-\lambda_H(t-t_0)}V(t_0) "
               r"\quad\Longrightarrow\quad "
               r"\operatorname{dist}\bigl(P(t),\mathcal{C}^*\bigr)\to 0", None))
    add(("para",
         "This is a sanity gate, not a decoration: the constant generator and the "
         "unit tests require exactly $B = 0$ and a zero ultimate radius in this "
         "case. If they did not, the formulas or their wiring would be wrong and "
         "Theorem 1 could not be marked complete."))

    add(("h2", "10.2 Sampled-data corollary (M7)"))
    add(("para",
         "Let the implementation take explicit steps of size $\\Delta t$,"))
    add(("eq", r"P_{k+1} = P_k + \Delta t\bigl[-A_k g_k + d_k\bigr],\qquad "
               r"g_k = \nabla H^*(P_k,t_k),\qquad "
               r"a_0 I \preceq A_k \preceq a_1 I,\quad a_1 = \frac{k_c}{2m_{\min}}",
         None))
    add(("para",
         "with $\\|d_k\\| \\le \\delta$ and $\\nabla H^*$ spatially "
         "$L_H$-Lipschitz on $X$. Define"))
    add(("eq", r"\bar a = a_0 - \frac{L_H a_1^2 \Delta t}{2},\qquad "
               r"C_\delta = \delta\bigl(1 + L_H a_1 \Delta t\bigr),\qquad "
               r"B_d = \frac{C_\delta^2}{2\bar a} + \frac{L_H}{2}\delta^2\Delta t + \nu_H",
         None))
    add(("para", "Then, under the step gates"))
    add(("eq", r"\Delta t < \frac{2a_0}{L_H a_1^2},\qquad 0 < \bar a\,\mu\,\Delta t \le 1",
         "M7-gate"))
    add(("para", "the Lyapunov level contracts geometrically,"))
    add(("eq", r"V_{k+1} \le \bigl(1-\bar a\mu\Delta t\bigr)V_k + B_d\,\Delta t,"
               r"\qquad "
               r"\limsup_{k\to\infty}\operatorname{dist}\bigl(P_k,\mathcal{C}_k^*\bigr) "
               r"\le \sqrt{\frac{2B_d}{\alpha\,\bar a\,\mu}}", "M7"))

    add(("h3", "Proof"))
    add(("para",
         "Apply the descent lemma to the $L_H$-Lipschitz gradient, add the "
         "reference drift over one step, and write $x = \\|g_k\\|$:"))
    add(("chain", [
        (r"V_{k+1}-V_k \le g_k^\top\bigl(P_{k+1}-P_k\bigr) "
         r"+ \frac{L_H}{2}\bigl\|P_{k+1}-P_k\bigr\|^2 + \nu_H\Delta t",
         "descent lemma and (P3-b)"),
        (r"\le -a_0\Delta t\,x^2 + \delta\Delta t\,x "
         r"+ \frac{L_H}{2}\Delta t^2\bigl(a_1 x + \delta\bigr)^2 + \nu_H\Delta t",
         "$a_0 I \\preceq A_k \\preceq a_1 I$, $\\|d_k\\|\\le\\delta$"),
        (r"= \Delta t\left[-\bar a\,x^2 + C_\delta\,x "
         r"+ \frac{L_H}{2}\delta^2\Delta t + \nu_H\right]",
         "expand the square and collect powers of $x$"),
        (r"\le \Delta t\left[-\frac{\bar a}{2}x^2 + \frac{C_\delta^2}{2\bar a} "
         r"+ \frac{L_H}{2}\delta^2\Delta t + \nu_H\right]",
         "Young, with $\\bar a > 0$"),
        (r"= \Delta t\left[-\frac{\bar a}{2}x^2 + B_d\right]",
         "definition of $B_d$"),
        (r"\le \Delta t\bigl[-\bar a\mu V_k + B_d\bigr]",
         "local PL: $\\tfrac12 x^2 \\ge \\mu V_k$"),
    ]))
    add(("para",
         "Rearranging gives the recursion in (M7). The first gate is exactly the "
         "condition $\\bar a > 0$ needed for the Young step; the second keeps the "
         "contraction factor in $[0,1)$. Summing the resulting geometric series,"))
    add(("eq", r"\limsup_{k\to\infty}V_k "
               r"\le \frac{B_d\Delta t}{\bar a\mu\Delta t} = \frac{B_d}{\bar a\mu}",
         None))
    add(("para",
         "and quadratic growth converts this to the stated ultimate radius. "
         "$\\qquad\\blacksquare$"))
    add(("para",
         "Wolfram expanded the descent algebra and checked the coefficient "
         "collection; a randomised positive-parameter test independently compares "
         "the certified bound against the direct one-step expression."))

    # ---------------------------------------------------------------- #
    # 11. Limits
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "11. Limits and evidence status"))
    add(("table", "limits", [
        ["Result", "Status", "Limiting condition"],
        ["Lemma 1", "PROVED + CAS", "A5 map bounds and tail separation"],
        ["Lemma 2", "PROVED + CAS", "rectangle, midpoint gate, exact-cell contract"],
        ["Proposition 1", "PROVED", "regular tie/threshold sets; saturated cost"],
        ["Proposition 2", "PROVED", "declared stacked disturbance components"],
        ["Proposition 3", "CONDITIONAL", "rate proved; A8 branch coercivity assumed"],
        ["Theorem 1", "CONDITIONAL", "complete implication under A1-A10"],
        ["Sampled corollary", "PROVED", "explicit step gates"],
    ]))
    add(("h2", "11.1 Evidence classes"))
    add(("bullets", [
        "PROVED: the analytic implication is written out in this supplement.",
        "VERIFIED: Wolfram reproduced the identified integral or algebraic "
        "subclaim, and nothing beyond it.",
        "CONDITIONAL: a named branch, tube, map bound, or runtime deviation "
        "certificate is still required.",
        "NUMERIC-SANITY: debugging evidence only. No such result is used as "
        "proof anywhere in this package, and there are no such entries.",
    ]))
    add(("h2", "11.2 Fail-closed runtime gates"))
    add(("para",
         "Each of these refuses to emit a certificate rather than emitting a "
         "degraded one, and each is covered by a test that feeds it a "
         "deliberately invalid configuration:"))
    add(("bullets", [
        "locality: a theorem-mode configuration with "
        "$R_{\\mathrm{comm}} < 2R_\\ell$ is rejected;",
        "arc length: a boundary observation without positive $\\Delta s_k$ is "
        "rejected, and the unit-weight fallback is forbidden;",
        "quadrature rule: anything other than a midpoint grid is rejected;",
        "quadrature gate: $\\eta_m \\ge m_{\\min}$ is rejected;",
        "bias: a nonzero gap or exploration gain is rejected unless separately "
        "budgeted into $\\delta$;",
        "sampled step: $\\Delta t$ violating (M7-gate) is rejected.",
    ]))
    add(("h2", "11.3 Claim audit"))
    add(("para",
         "This supplement contains no global PL statement, no formal caging "
         "claim, no finite-time phase-completion result, and no unconditional "
         "transport or safety guarantee. The numeric certificate in Appendix A "
         "validates the dependency wiring and the gates; its ultimate radius is "
         "deliberately conservative and may be numerically vacuous. It is not "
         "experimental evidence and is not used to claim transport performance."))

    # ---------------------------------------------------------------- #
    # Appendix A
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "Appendix A. Constant ledger"))
    add(("para",
         "Every value below is produced by one code path from the frozen "
         "formulas; none is entered by hand. The chain from raw sensing bounds to "
         "the ultimate radius is"))
    add(("eq", r"\bigl(\varepsilon_b,\varepsilon_n,\varepsilon_d,v,h_s,E_w,"
               r"M_{\mathrm{drop}}\bigr) "
               r"\ \longrightarrow\ \varepsilon_\phi "
               r"\ \longrightarrow\ \varepsilon_c "
               r"\ \longrightarrow\ \delta "
               r"\ \longrightarrow\ B "
               r"\ \longrightarrow\ \sqrt{\frac{2B}{\alpha\lambda_H}}", None))
    rows = [["Constant", "Value", "Unit", "Status"]]
    for name in LEDGER_CONSTANTS:
        item = cert["constants"][name]
        rows.append([name, f"{item['value']:.6g}", item["unit"], item["status"]])
    add(("table", "ledger", rows))

    # ---------------------------------------------------------------- #
    # Appendix B
    # ---------------------------------------------------------------- #
    add(("pagebreak",))
    add(("h1", "Appendix B. Wolfram verification manifest"))
    add(("para",
         "Connector: Wolfram (version not exposed). Local runtime: WolframScript "
         "1.14.0; all nine committed scripts executed with exit code 0. Exact "
         "inputs, assumptions and outputs are archived under "
         "theory/wolfram/results."))
    add(("table", "wolfram", [
        ["ID", "Target", "Evidence", "Verdict"],
        ["W-L1-01/02", "Gaussian $L^1$ integrals", "EXACT", "PASS"],
        ["W-L1-03", "gradient radial maximum", "SYMBOLIC", "PASS"],
        ["W-L1-04/05", "normal and bounded-step identities", "SYMBOLIC", "PASS"],
        ["W-L2-01/02", "ratio, monotonicity, mass bounds", "SYMBOLIC", "PASS"],
        ["W-P1-01", "integrand derivative, fixed-cell check", "SYMBOLIC", "PASS"],
        ["W-P3-01", "reference-rate algebra", "SYMBOLIC", "PASS"],
        ["W-T1-01", "Young and comparison ODE", "SYMBOLIC", "PASS"],
        ["W-C1-01", "sampled-data descent expansion", "SYMBOLIC", "PASS"],
        ["W-M5-01", "generic quadratic implication", "SYMBOLIC", "PASS"],
        ["W-M5-02", "actual branch Hessian box", "INTERVAL", "CONDITIONAL"],
    ]))
    add(("para",
         "Evidence rule: an exact or symbolic output supports only the stated "
         "integral or algebraic identity. Interval evidence would have to cover "
         "the complete selected tube to support A8, and none does."))
    add(("h2", "What computer algebra did not verify"))
    add(("bullets", [
        "the pushforward construction, the $L^1$ translation inequality, the "
        "curve quadrature and the atom matching of Lemma 1;",
        "the Voronoi shape derivative and the moving-boundary flux cancellation "
        "of Proposition 1;",
        "any uniform lower bound on a reduced Hessian for a named branch, which "
        "is why A8 and therefore Theorem 1 remain conditional.",
    ]))
    add(("h2", "Reference"))
    add(("para",
         "J. Cortes, S. Martinez and F. Bullo, \"Spatially-distributed coverage "
         "optimization and control with limited-range interactions\", ESAIM: "
         "COCV, 11(4):691-719, 2005. DOI 10.1051/cocv:2005024."))

    return blocks
