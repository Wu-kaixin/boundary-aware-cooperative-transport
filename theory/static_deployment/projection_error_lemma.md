> 精确算术下的静态采样安全和先验界在所声明的 oracle-map 范围内闭合；控制残差与有意义部署驻点的联系、局部未知边界接口以及部署对搬运的作用仍是当前论文需要补齐的内容。

# Projection-error lemma (raw Green moments)

Status: **proved** under exact Euclidean projection onto the closed disk in robot-centred coordinates. Floating-point `project_to_disk` is not identified with that operator.

This note removes the duplicate charge
\(2R(\lVert\delta\mu_{\mathrm{quad}}\rVert+R\lvert\delta m_{\mathrm{quad}}\rvert)\)
that previously appeared as a “projection protrusion” on top of a moment bound already written in the raw Green moments. The controller, \(h^\star=0.004\), and \(n_{\mathrm{gon}}=256\) are unchanged. Only the analysis budget changes.

## 1. Coordinates and objects

Fix a robot site \(p\in\mathbb{R}^2\) and radius \(R>0\). All vectors in this lemma are **robot-centred**: the origin is \(p\), and the local disk is the closed Euclidean ball \(B(0,R)\). World-frame first moments convert by \(\mu_{\mathrm{world}}=\mu_{\mathrm{robot}}+p\,m\); mixing frames invalidates the identity below.

For one truncated cell \(\Omega\subset B(p,R)\):

- Exact reference mass \(m^\star=\int_\Omega\phi^\star\ge 0\) and moment \(\mu^\star=\int_\Omega q\,\phi^\star\) (robot-centred). If \(m^\star>0\), the exact centroid is \(c^\star=\mu^\star/m^\star\). Nonnegative \(\phi^\star\) and \(\Omega\subset B(0,R)\) imply \(c^\star\in B(0,R)\).
- Numerical mass \(\hat m>0\) above the implementation floor `mass_floor`, and **raw** numerical (Green / trapezoid) moment \(\hat\mu\).
- Raw numerical centroid \(x=\hat\mu/\hat m\).
- Implemented centroid \(y=\Pi_{B(0,R)}(x)\), the Euclidean projection onto the closed disk.
- Error \(e=y-c^\star\), \(\delta m=\hat m-m^\star\), \(\delta\mu=\hat\mu-\mu^\star\) (raw Green, **not** \(y\hat m-\mu^\star\)).

If \(\hat m\le\texttt{mass_floor}\), the implementation replaces \(y\) by the site \(0\). That branch is §5, not this identity.

If \(m^\star=0\), the mass-weighted contribution \(m^\star\lVert e\rVert^2\) is defined to be \(0\). The quotient \(c^\star=\mu^\star/m^\star\) is not formed.

## 2. Algebraic identity

Assume \(m^\star>0\) and \(\hat m>0\). Then
\[
m^\star e=\delta\mu-y\,\delta m+\hat m(y-x).
\]
Proof. The right-hand side expands as
\[
\hat\mu-\mu^\star-y(\hat m-m^\star)+\hat m y-\hat m x
=\hat m x-\mu^\star-y\hat m+y m^\star+\hat m y-\hat m x
=m^\star(y-\mu^\star/m^\star)=m^\star e.
\]
This identity does **not** use projection. It remains valid if \(y\) is any replacement of \(x\).

## 3. Euclidean projection optimality

\(B(0,R)\) is closed, convex, and nonempty. The Euclidean projection \(y=\Pi_{B(0,R)}(x)\) is characterised by
\[
\langle x-y,\,z-y\rangle\le 0\qquad\text{for every }z\in B(0,R).
\]
Take \(z=c^\star\in B(0,R)\):
\[
\langle x-y,\,c^\star-y\rangle\le 0\iff\langle y-x,\,e\rangle\le 0.
\]

Inner product of the identity with \(e\):
\[
m^\star\lVert e\rVert^2
=\langle\delta\mu,e\rangle-\langle y,e\rangle\delta m+\hat m\langle y-x,e\rangle
\le \langle\delta\mu,e\rangle-\langle y,e\rangle\delta m,
\]
the last term being \(\le 0\). Hence
\[
m^\star\lVert e\rVert^2
\le\lVert e\rVert\bigl(\lVert\delta\mu\rVert+\lVert y\rVert\lvert\delta m\rvert\bigr)
\le\lVert e\rVert\bigl(\lVert\delta\mu\rVert+R\lvert\delta m\rvert\bigr),
\]
using \(\lVert y\rVert\le R\). In particular \(\lVert e\rVert\le 2R\), so
\[
m^\star\lVert e\rVert^2
\le 2R\bigl(\lVert\delta\mu\rVert+R\lvert\delta m\rvert\bigr).
\]
Summing over cells (cells with \(m^\star=0\) contribute \(0\)) yields the same moment form already implemented as `moment_form_E_bound`, **using the raw Green \(\delta\mu\)**.

The discarded extra \(2R(\lVert\delta\mu_{\mathrm{quad}}\rVert+R\lvert\delta m_{\mathrm{quad}}\rvert)\) was a second charge of the protrusion \(\hat m(y-x)\). Projection optimality shows that term has the wrong sign in the inner product with \(e\), so it cannot be added on top of a bound already written in \(\delta\mu=\hat\mu-\mu^\star\).

Trapezoid / float remainders remain inside \(\sum\lvert\delta m\rvert\) and \(\sum\lVert\delta\mu\rVert\) **once**. They are not deleted from the budget.

## 4. Distinguishing \(\hat\mu\) and \(y\hat m\)

The identity uses the raw Green moment \(\hat\mu=\hat m\,x\). Substituting the projected moment \(y\hat m\) in place of \(\hat\mu\) produces a different vector and would double-count \(\hat m(y-x)\). Code correspondence:

- `mixture_mass_centroid_over_polygon` returns a mass and a **world-frame** moment; the caller centres by subtracting \(p m\).
- If the raw centroid leaves the disk, `project_to_disk` produces \(y\). The prior charges \(\delta\mu\) from the Green remainder, not from \(y\hat m-\mu^\star\).

## 5. Mass fallback

If \(\hat m\le\texttt{mass_floor}\), the implementation sets \(y=0\) (the site) and does not form \(x\). The identity of §2 is not used. The existing fallback
\[
R^2\Bigl(\sum\lvert\delta m\rvert+N\varepsilon\Bigr)
\]
covers \(m^\star\lVert 0-c^\star\rVert^2\le R^2 m^\star\) together with a floor \(\varepsilon\) on every cell. This term is **kept**. It is not a second projection charge.

## 6. Exact projection versus floating-point `project_to_disk`

The characterisation \(\langle x-y,c^\star-y\rangle\le 0\) is for the exact Euclidean projection. The implementation `project_to_disk` uses IEEE-754 norms and a threshold \(10^{-15}\). That operator is not proved identical to \(\Pi_{B(0,R)}\). The main theorem is exact-arithmetic. A numerical remainder for float projection is **not** closed here; it is an implementation gap, not a licence to restore the duplicate extra.

## 7. What this lemma does not do

- It does not change the closed-loop map \((P_k,M_k)\mapsto U_k\).
- It does not lower \(h^\star\) or raise panel counts.
- It does not claim that polar observer \((H,g,m,c)\) errors are included in \(B_E\). Those stay `numerical_a_priori`.
- It does not replace the diameter bound \(4R^2 M_{\mathrm{plane}}\). The certificate uses \(\min(\text{moment+fallback},\,4R^2 M_{\mathrm{plane}})\).
