# Exact connector queries

The connector was given complete, stateless Wolfram Language associations. The
included scripts are the canonical exact inputs. They cover:

1. `Integrate` for `||K_sigma||_1` and `||grad K_sigma||_1`.
2. derivative, critical point, and endpoint values for
   `||grad K_sigma||_infinity`.
3. the normal-angle and bounded-step identities.
4. centroid ratio identity, denominator positivity, and monotonicity.
5. integrand differentiation plus an exact fixed-square gradient example.
6. offset-reference product rule and `nu_H` substitution.
7. Young's inequality and the scalar comparison ODE solution.
8. sampled-data descent-lemma expansion, Young remainder, and step gate.
9. the generic strong-convex quadratic implication; no actual CVT Hessian box.

See `scripts/*.wl` for the literal inputs. No natural-language-only response is
accepted as a certificate.
