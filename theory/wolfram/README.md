# Wolfram verification

The files in `scripts/` are the reproducible Wolfram Language inputs. Connector
outputs are transcribed verbatim into `results/` with assumptions, evidence
class, verdict, UTC provenance, and a human note that limits each conclusion.

Evidence classes:

- `SYMBOLIC` / `EXACT`: supports the stated algebra or integral only.
- `INTERVAL`: requires coverage of a complete declared parameter tube.
- `NUMERIC-SANITY`: debugging evidence only, never a theorem proof.

The connector passed all requested exact algebra/integral checks. It emitted
some non-fatal front-end messages when simplifying quantified expressions; the
record keeps those messages and relies only on the displayed exact outputs.
There is no M5 interval certificate because no concrete selected branch, gauge,
cell combinatorics, and parameter box were supplied. A8 therefore remains
conditional.

Local reproducibility uses WolframScript 1.14.0:

```text
wolframscript -file theory/wolfram/scripts/01_kernel_constants.wl
```
