"""Frozen content of the Theorem 1 proof supplement.

The supplement has two renderers -- a ReportLab PDF and a Word document -- and
both must say exactly the same thing.  Keeping the prose here, behind a small
emitter protocol, is what makes that structural rather than a promise.  The PDF
renderer's output is byte-identical (modulo the embedded timestamp) to the one
built before this text was extracted.
"""

from __future__ import annotations

from typing import Protocol


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
    ["Regression", "470 passed, 3 skipped"],
]

TOC_ITEMS = [
    "1  Scope and frozen claim",
    "2  Notation and assumptions",
    "3  Lemma 1 - density consistency",
    "4  Lemma 2 - centroid perturbation",
    "5  Proposition 1 - ideal gradient",
    "6  Proposition 2 - aggregate disturbance",
    "7  Proposition 3 - reference rate and local regularity",
    "8  Theorem 1 - practical stability",
    "9  Corollaries",
    "10 Limits and evidence status",
    "Appendix A  Constant ledger",
    "Appendix B  Wolfram manifest",
]

LEDGER_CONSTANTS = [
    "kernel_l1", "kernel_grad_l1", "kernel_grad_linf", "epsilon_xi",
    "epsilon_phi", "m_min", "m_max", "grid_spacing", "eta_m", "eta_a",
    "epsilon_q", "epsilon_c", "delta", "nu_phi", "nu_H", "a0",
    "lambda_H", "B", "ultimate_radius", "sampled_Bd", "sampled_ultimate_radius",
]


class Emitter(Protocol):
    """What a renderer must provide for :func:`compose` to run."""

    def title_block(self) -> None: ...
    def section(self, title: str) -> None: ...
    def subsection(self, title: str) -> None: ...
    def lead(self, text: str) -> None: ...
    def body(self, text: str) -> None: ...
    def bullets(self, items: list[str]) -> None: ...
    def equation(self, text: str) -> None: ...
    def toc(self, items: list[str]) -> None: ...
    def callout(self, head: str, text: str) -> None: ...
    def table(self, kind: str, rows: list[list[str]]) -> None: ...
    def spacer(self, points: float) -> None: ...
    def pagebreak(self) -> None: ...


def compose(e: Emitter, cert: dict) -> None:
    """Emit the whole supplement, in order, against ``e``."""

    e.title_block()
    e.table("status", STATUS_ROWS)
    e.spacer(0.35 * 72)
    e.callout(CALLOUT_HEAD, CALLOUT_BODY)
    e.pagebreak()

    e.section("Contents and dependency chain")
    e.toc(TOC_ITEMS)
    e.spacer(14)
    e.equation(
        "boundary/map certificate\n        |\n        v\nLemma 1 -> Lemma 2 -> Proposition 2\n"
        "                |             /\n                v            /\n"
        "        Proposition 1 ------+\n                |\n"
        "        Proposition 3 (A8 conditional)\n                |\n                v\n"
        "             Theorem 1 -> corollaries"
    )
    e.body(
        "The chain contains no SEARCH, caging, contact, CBF-invariance, or "
        "transport-success node."
    )

    e.pagebreak()
    e.section("1. Scope and frozen claim")
    e.lead(
        "Bounded local-map disagreement and bounded reference variation imply practical stability "
        "of the boundary-induced Local-CVT allocation dynamics around a nondegenerate local CVT branch."
    )
    e.body(
        "The system is first order, p_i' = u_i. The nominal ENCLOSE law is "
        "u_i^nom = -k_c(p_i - c_hat_i). The ideal objective uses a saturated performance "
        "function; dropping saturation would introduce an uncancelled moving-boundary term."
    )
    e.equation("F_R(r) = min(r^2, R_l^2)\nH*(P,t) = integral_D min_i F_R(||q-p_i||) phi*(q,t) dq")
    e.subsection("Deliberate exclusions")
    e.bullets([
        "no global CVT optimality or arbitrary-shape global convergence;",
        "no finite-time SEARCH-to-HOLD guarantee;",
        "no formal geometric or topological caging;",
        "no contact-transport theorem or unconditional cooperative-transport success;",
        "no safety claim outside feasibility and deviation contracts;",
        "no uniform Hessian bound for an unspecified local branch.",
    ])

    e.section("2. Notation and assumptions")
    e.body(
        "D is a fixed rectangle; P stacks N planar robot positions. Gamma(t) is the true boundary "
        "with perimeter at most P_max. The true offset target pushes arc length forward to nu*_t; "
        "phi* is its convolution with the unnormalized Gaussian plus phi_0."
    )
    e.equation(
        "K_sigma(x) = exp(-||x||^2/(2 sigma^2))\nnu*_t = xi(.,t)_# ds\n"
        "phi*(q,t) = phi_0 + (K_sigma * nu*_t)(q)\n"
        "V_i^l = Vor_i(P) intersect D intersect B(p_i,R_l)"
    )
    e.equation(
        "||K_sigma||_1 = 2 pi sigma^2\n||grad K_sigma||_1 = sqrt(2) pi^(3/2) sigma\n"
        "||grad K_sigma||_inf = exp(-1/2)/sigma"
    )
    e.subsection("A1-A10")
    e.bullets([
        "A1. D is rectangular and the trajectory stays in fixed compact tube X.",
        "A2. Separation is at least d_s>0 and R_comm >= 2 R_l.",
        "A3. The closed piecewise-C2 boundary is rigid in version 1 and P_Gamma <= P_max.",
        "A4. phi_0>0, sigma>0, and the stated unnormalized Gaussian is used.",
        "A5. A conservative map certificate supplies every boundary/normal/offset/voxel/weight/tail bound.",
        "A6. LocalCVT uses midpoint cells and eta_m < m_min.",
        "A7. Density biases are disabled or explicitly included in delta.",
        "A8. A symmetry-reduced selected branch has uniform reduced Hessian >= m_H I (conditional here).",
        "A9. Degenerate tie/threshold sets have measure zero; the solution remains on the branch tube.",
        "A10. ZOH, numerical, tracking, and safety deviations have stacked-norm certificates.",
    ])

    e.pagebreak()
    e.section("3. Lemma 1 - boundary/map to density")
    e.body(
        "Introduce an exact discrete intermediate measure. The comparison is performed after "
        "convolution; no total-variation convergence between a continuous arc-length measure and "
        "a Dirac measure is asserted."
    )
    e.equation(
        "nu*_t -> nu*_{h,t} = sum_k Delta s_k delta_{xi*_k}\n"
        "      -> nu_hat_{i,t} = sum_k w_hat_ik delta_{xi_hat_ik}"
    )
    e.equation(
        "epsilon_xi = epsilon_b + epsilon_d + 2 d_max sin(epsilon_n/2)\n"
        "E_w = L_miss + M_spur + sum_matched |w_hat_k - Delta s_k|"
    )
    e.equation(
        "epsilon_phi = L_K,1 P_max (epsilon_xi + sqrt(2)v + L_T h_s/2)\n"
        "              + 2 pi sigma^2 E_w + epsilon_tail          (L1)\n"
        "epsilon_tail <= pi R_l^2 M_drop exp(-s_sigma^2/2)"
    )
    e.subsection("Proof")
    e.body(
        "Unit normals separated by angle theta differ by 2 sin(theta/2); the triangle inequality "
        "and a square-voxel diagonal give the target error. The fundamental theorem of calculus "
        "along a translation segment, followed by Tonelli, gives "
        "||K(.-a)-K(.-b)||_1 <= ||grad K||_1 ||a-b||."
    )
    e.body(
        "On each arc, the midpoint curve approximation costs at most L_K,1 L_T Delta s_k^2/2. "
        "Summing uses sum Delta s_k^2 <= h_s sum Delta s_k <= h_s P_max. Matched target "
        "displacement costs at most L_K,1 P_max(epsilon_xi+sqrt(2)v), and total "
        "missing/spurious/weight discrepancy costs ||K||_1 E_w."
    )
    e.body(
        "Every discarded Gaussian source is at least s_sigma sigma from every query in the disk. "
        "Its pointwise contribution is therefore at most exp(-s_sigma^2/2); summing dropped mass "
        "and integrating over at most pi R_l^2 proves the tail. The triangle inequality proves (L1)."
    )
    e.subsection("Evidence boundary")
    e.body(
        "Wolfram exactly checked the kernel integrals and symbolically checked the angle and step "
        "identities. Pushforward, translation, curve quadrature, measure decomposition, and tail "
        "control are the human analytic proof above."
    )

    e.pagebreak()
    e.section("4. Lemma 2 - mass and centroid perturbation")
    e.equation(
        "r0 = min(R_l, d_s/2, W_D/2, H_D/2)\nm_min = phi_0 pi r0^2 / 4 > 0\n"
        "m_max = pi R_l^2 (phi_0 + P_max)\nphi_hat_max = phi_0 + P_max + E_w\n"
        "L_hat_phi <= (P_max + E_w) exp(-1/2)/sigma"
    )
    e.equation(
        "r_h = h/sqrt(2),  A_V <= pi R_l^2,  P_V <= 2 pi R_l\n"
        "eta_m = L_hat_phi r_h A_V + phi_hat_max(2 P_V r_h + pi r_h^2)\n"
        "eta_a = (phi_hat_max+R_l L_hat_phi)r_h A_V\n"
        "        + R_l phi_hat_max(2 P_V r_h + pi r_h^2)\n"
        "epsilon_q = (eta_a + R_l eta_m)/(m_min - eta_m)"
    )
    e.equation(
        "||c_hat_i-c*_i|| <= 2 R_l epsilon_phi_i/m_min\n"
        "                       + epsilon_q_i + epsilon_G_i      (L2)"
    )
    e.subsection("Proof")
    e.body(
        "Separation puts B(p_i,d_s/2) in robot i's Voronoi cell. The intersection with the local "
        "disk and rectangle contains at least a quarter disk of radius r0; phi*>=phi0 gives m_min. "
        "Kernel boundedness and disk area give the ideal m_max. Lemma 1 bounds mapped total weight "
        "by P_max+E_w; differentiating its convolution gives the displayed mapped-density bounds."
    )
    e.body(
        "A midpoint cell entirely inside the exact convex cell contributes at most L_hat_phi r_h "
        "times area. Boundary-cut cells occupy a parallel strip of area at most "
        "2P_V r_h + pi r_h^2. This proves eta_m. Repeating the argument for the shifted first "
        "moment a=integral(q-p_i)phi dq proves eta_a."
    )
    e.body(
        "Because m_hat >= m_min-eta_m>0, the ratio identity gives epsilon_q. Lemma 1 changes "
        "continuum mass by at most epsilon_phi and shifted moment by at most R_l epsilon_phi; both "
        "densities retain the base-density floor, giving 2R_l epsilon_phi/m_min. Add the separately "
        "certified geometric-cell term. The denominator gate fails closed."
    )

    e.pagebreak()
    e.section("5. Proposition 1 - ideal gradient and descent")
    e.equation(
        "grad_{p_i} H* = 2 m*_i (p_i-c*_i)                 (P1-a)\n"
        "u*_i = -k_c(p_i-c*_i) = -[k_c/(2m*_i)] grad_{p_i}H*"
    )
    e.body(
        "Inside V_i^l the active integrand derivative is 2(p_i-q); outside the disk it is zero "
        "because the cost is saturated. On a shared Voronoi face, competing minimum values agree "
        "and the two fluxes cancel. On r=R_l, the inner and outer costs both equal R_l^2, so the "
        "circle flux cancels. The workspace boundary is fixed and the exceptional tie/threshold "
        "sets have measure zero. Only the volume derivative remains, proving P1-a."
    )
    e.equation(
        "A(P,t) = diag_i(k_c/(2m*_i) I_2) >= a0 I\na0 = k_c/(2m_max)\n"
        "Hdot* = -grad H*^T A grad H* <= -a0 ||grad H*||^2"
    )
    e.body(
        "If a point in B(p_i,R_l) is won by robot j, then ||p_j-p_i|| <= 2R_l <= R_comm. Thus the "
        "communication set contains every local competitor and the cell is exact. The Wolfram "
        "fixed-square gradient check is only a sanity example; it is not the shape-derivative proof."
    )

    e.section("6. Proposition 2 - aggregate disturbance")
    e.equation(
        "Pdot = -A(P,t) grad H*(P,t) + d(t)\n||d(t)|| <= delta\n"
        "delta = k_c sqrt(sum_i epsilon_c_i^2) + epsilon_zoh + epsilon_num\n"
        "        + epsilon_trk + epsilon_sf                         (P2)"
    )
    e.body(
        "Lemma 2 bounds the centroid component, and the stacked triangle inequality adds the "
        "remaining deviations. Production commands use a per-agent Euclidean speed cap, so "
        "||u_i^sf-u_i^nom||<=2u_max and epsilon_sf<=2sqrt(N)u_max. A componentwise box cap would "
        "require 2sqrt(2N)u_max. The certificate records the chosen convention."
    )

    e.pagebreak()
    e.section("7. Proposition 3 - reference rate and regularity")
    e.equation(
        "xi = b + d n,   xi_dot = b_dot + d_dot n + d n_dot\n"
        "v_xi,max = v_O_bar + omega_O_bar(R_O+d_max+L_d)\n"
        "nu_phi <= L_K,1 P_max v_xi,max\n"
        "nu_H = 2 R_l^2 nu_phi                                  (P3)"
    )
    e.body(
        "The rigid-twist bound controls every target speed; convolution translation controls the "
        "density rate. Since 0<=min_i F_R<=R_l^2, |partial_t H*|<=R_l^2 nu_phi. On fixed compact X, "
        "|min f-min g|<=sup|f-g| supplies the same bound for H_min*, producing nu_H. Deformation, "
        "perimeter change, or split/merge events require a new measure-rate assumption."
    )
    e.subsection("M5 decision")
    e.equation(
        "1/2 ||grad H*||^2 >= mu (H*-H_min*)\n"
        "H*-H_min* >= (alpha/2) dist^2(P,C*(t))"
    )
    e.body(
        "A uniform reduced Hessian lower bound on a convex symmetry-reduced chart implies both "
        "relations with mu=alpha=m_H (or smaller constants). The implication is proved. Its premise "
        "is conditional: no selected branch, gauge, cell combinatorics, or complete parameter tube "
        "was supplied for validated interval subdivision. No floating-point eigenvalue is used as proof."
    )

    e.pagebreak()
    e.section("8. Theorem 1 - local practical stability")
    e.equation(
        "V(P,t) = H*(P,t)-H_min*(t) >= 0\na0 = k_c/(2m_max),   lambda_H = a0 mu\n"
        "B = delta^2/(2a0) + nu_H"
    )
    e.equation(
        "V(t) <= exp[-lambda_H(t-t0)] V(t0)\n"
        "        + (B/lambda_H)(1-exp[-lambda_H(t-t0)])           (T1-a)\n"
        "limsup dist(P(t),C*(t)) <= sqrt(2B/(alpha lambda_H))     (T1-b)"
    )
    e.subsection("Proof - every inequality named")
    e.equation(
        "Vdot = grad H*^T Pdot + partial_t H* - H_min*_dot\n"
        " <= -grad H*^T A grad H* + ||grad H*|| ||d|| + nu_H\n"
        "      [Propositions 1 and 3]\n"
        " <= -a0 ||grad H*||^2 + delta ||grad H*|| + nu_H\n"
        "      [Proposition 2 and A >= a0 I]\n"
        " <= -(a0/2)||grad H*||^2 + delta^2/(2a0) + nu_H\n"
        "      [Young]\n"
        " <= -a0 mu V + B = -lambda_H V + B\n"
        "      [local PL]"
    )
    e.body(
        "The scalar comparison lemma yields T1-a. Hence limsup V<=B/lambda_H. Local quadratic "
        "growth gives dist(P,C*)<=sqrt(2V/alpha), which yields T1-b. The infinite-time limit is "
        "asserted only if the trajectory continues to satisfy the regular-tube premises."
    )
    e.subsection("Status")
    e.body(
        "The analytic implication is complete, and Wolfram verifies the scalar algebra. The result "
        "remains conditional on A5, A8-A10 and the run/tube certificates. It is local, not global."
    )

    e.pagebreak()
    e.section("9. Corollaries")
    e.subsection("Frozen perfect information")
    e.equation(
        "delta=0, nu_H=0  =>  B=0\n"
        "V(t) <= exp[-lambda_H(t-t0)] V(t0),  dist(P,C*) -> 0"
    )
    e.body(
        "The generator and unit tests require exactly zero forcing and zero ultimate radius in "
        "this case."
    )
    e.subsection("Sampled data")
    e.equation(
        "P_{k+1}=P_k+dt[-A_k g_k+d_k],  a0 I<=A_k<=a1 I\nabar=a0-(L_H a1^2 dt)/2\n"
        "C_delta=delta(1+L_H a1 dt)\nB_d=C_delta^2/(2abar)+(L_H/2)delta^2 dt+nu_H"
    )
    e.equation(
        "dt < 2a0/(L_H a1^2),  0 < abar mu dt <= 1\n"
        "V_{k+1} <= (1-abar mu dt)V_k+B_d dt\n"
        "limsup dist(P_k,C*_k) <= sqrt(2B_d/(alpha abar mu))"
    )
    e.body(
        "The descent lemma first gives dt[-a0 x^2+delta x]+(L_H dt^2/2)(a1 x+delta)^2+nu_H dt. "
        "Expansion, Young, and local PL give the recursion. The geometric series and quadratic "
        "growth give the ultimate radius. Wolfram checked the expansion; randomized "
        "positive-parameter tests independently compare the direct one-step expression with the "
        "certified upper bound."
    )

    e.pagebreak()
    e.section("10. Limits and evidence status")
    e.table("limits", [
        ["Result", "Status", "Limiting condition"],
        ["Lemma 1", "PROVED + CAS", "A5 map bounds and tail separation"],
        ["Lemma 2", "PROVED + CAS", "rectangle, midpoint gate, exact-cell contract"],
        ["Proposition 1", "PROVED", "regular tie/threshold sets; saturated cost"],
        ["Proposition 2", "PROVED", "declared stacked disturbance components"],
        ["Proposition 3", "CONDITIONAL", "rate proved; A8 branch coercivity assumed"],
        ["Theorem 1", "CONDITIONAL", "complete implication under A1-A10"],
        ["Sampled corollary", "PROVED", "explicit step gates"],
    ])
    e.spacer(10)
    e.body(
        "The illustrative certificate validates the dependency wiring and gates. Its numeric radius "
        "is intentionally reported as conservative and may be vacuous; it is not experimental "
        "evidence and is not used to claim transport performance."
    )
    e.subsection("Evidence classes and runtime gates")
    e.bullets([
        "PROVED: the analytic implication is written in this supplement.",
        "VERIFIED: Wolfram reproduced only the identified integral or algebraic subclaim.",
        "CONDITIONAL: a named branch, tube, map bound, or runtime deviation certificate remains required.",
        "NUMERIC-SANITY: debugging evidence only; no such result is used as proof here.",
        "Fail-closed gates: locality, positive arc length, midpoint rule, eta_m<m_min, disabled unbudgeted biases, and sampled step size.",
    ])
    e.subsection("Claim audit")
    e.body(
        "The supplement deliberately contains no global PL statement, no formal caging claim, no "
        "finite-time phase-completion result, and no unconditional transport or safety guarantee."
    )

    e.pagebreak()
    e.section("Appendix A. Constant ledger")
    rows = [["Constant", "Value", "Unit", "Status"]]
    for name in LEDGER_CONSTANTS:
        item = cert["constants"][name]
        rows.append([name, f"{item['value']:.6g}", item["unit"], item["status"]])
    e.table("ledger", rows)
    e.spacer(8)
    e.equation(
        "(epsilon_b, epsilon_n, epsilon_d, v, h_s, E_w, M_drop)\n"
        "  -> epsilon_phi -> epsilon_c -> delta -> B\n"
        "  -> sqrt(2B/(alpha lambda_H))"
    )

    e.pagebreak()
    e.section("Appendix B. Wolfram verification manifest")
    e.body(
        "Connector: Wolfram (version not exposed). Local runtime: WolframScript 1.14.0. All nine "
        "included scripts executed with exit code 0. Exact connector outputs and assumptions are "
        "archived in theory/wolfram/results."
    )
    e.table("wolfram", [
        ["ID", "Target", "Evidence", "Verdict"],
        ["W-L1-01/02", "Gaussian L1 integrals", "EXACT", "PASS"],
        ["W-L1-03", "gradient radial maximum", "SYMBOLIC", "PASS"],
        ["W-L1-04/05", "normal/step identities", "SYMBOLIC", "PASS"],
        ["W-L2-01/02", "ratio, monotonicity, mass", "SYMBOLIC", "PASS"],
        ["W-P1-01", "integrand and fixed-cell check", "SYMBOLIC", "PASS"],
        ["W-P3-01", "reference-rate algebra", "SYMBOLIC", "PASS"],
        ["W-T1-01", "Young and comparison ODE", "SYMBOLIC", "PASS"],
        ["W-C1-01", "sampled-data expansion", "SYMBOLIC", "PASS"],
        ["W-M5-01", "generic quadratic implication", "SYMBOLIC", "PASS"],
        ["W-M5-02", "actual branch Hessian box", "INTERVAL", "CONDITIONAL"],
    ])
    e.spacer(10)
    e.body(
        "Evidence rule: exact/symbolic output supports only the stated integral or algebra. "
        "Interval evidence would need to cover the complete selected tube. Numeric sanity is "
        "debugging evidence only. There are no numeric-sanity entries in this package."
    )
    e.subsection("Reference")
    e.body(
        "J. Cortes, S. Martinez, and F. Bullo, 'Spatially-distributed coverage optimization and "
        "control with limited-range interactions,' ESAIM: COCV, 11(4):691-719, 2005. "
        "DOI 10.1051/cocv:2005024."
    )
