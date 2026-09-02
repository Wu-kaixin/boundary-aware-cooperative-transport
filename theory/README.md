# DBACT Theorem 1 verification package

This directory freezes and audits the claim that bounded local-map disagreement
and bounded reference variation imply **practical stability of the
boundary-measure-induced Local-CVT allocation dynamics near a selected,
nondegenerate local CVT branch**.  The result is local and conditional on
Assumptions A1--A10.  It is not a global convergence, caging, safety, or
transport-success theorem.

## P0 preflight record

- Branch: `DBACT-research-v3`
- Baseline HEAD: `64e1905e092d19fd9dcbf0e77fbd39f55f0562ba`
- Remote comparison after `git fetch origin --prune`: local and
  `origin/DBACT-research-v3` were identical (`0 0` ahead/behind).
- Initial worktree: clean.
- Applicable `AGENTS.md`: none found.
- Python: 3.13.13 (`D:\miniconda\python.exe`).
- Baseline command:
  `python -m pytest tests -q --basetemp tmp/pytest-baseline -p no:cacheprovider`
- Baseline result: **449 passed, 3 skipped in 31.54 s**.
- Preflight recorded: 2026-09-02T11:09:21.219Z.

## Final verification record

- Final regression command:
  `python -m pytest tests -q --basetemp tmp/pytest-final -p no:cacheprovider`
- Final regression result: **470 passed, 3 skipped in 31.43 s**.
- Targeted theorem/density/LocalCVT/map tests: **70 passed in 13.16 s**.
- Certificate generation: completed with `status=conditional`; every fail-closed
  gate passed for the illustrative theorem-mode configuration.
- Wolfram: all nine scripts re-executed with WolframScript 1.14.0 and exit code
  zero; the actual-branch reduced-Hessian interval obligation remains conditional.
- Static checks: Python compilation, all JSON/YAML parses, and
  `git diff --check` passed.
- PDF QA: the final 12-page supplement was rendered at 144 dpi with Poppler and
  every page was visually inspected for clipping, overlap, and legibility.
- Final verification recorded: 2026-09-02T11:52:52.1215570Z.

The uploaded execution specification was read as an implementation contract,
not as a source of independent mathematical evidence.  The DOCX could be
structurally extracted, but its visual render was unavailable because
LibreOffice/soffice is not installed on this host.

## Independent re-run record (2026-09-02T12:15:36Z)

The package above was re-executed end to end by a second agent, on the same
clean `64e1905` worktree, to check that the recorded verdicts are reproducible
rather than transcribed.

- Full regression re-run: **470 passed, 3 skipped** — matches the record above.
- `tests/theory` re-run: 13 passed, then **20 passed** after the supplement
  parity tests below were added.
- Certificate regenerated: `status=conditional`, byte-stable apart from its
  timestamp.
- **Every constant in `analytic_constants.json` was recomputed from the frozen
  formulas by an independent script**; all 23 agreed to floating-point
  round-off.  The ledger is therefore generated, not hand-entered.
- All six fail-closed gates were probed with deliberately invalid configs and
  every one refused to emit a certificate: locality (`R_comm < 2 R_l`), missing
  arc length, non-midpoint rule, `eta_m >= m_min`, enabled density bias, and an
  oversized sampled step.
- Wolfram: 11 obligations re-derived through the connector
  (`results/W-RERUN_connector_session.md`), plus all nine `.wl` scripts
  re-executed locally under WolframScript 1.14.0 with exit code 0 and outputs
  identical to the archived ones.  `W-M5-02` is still CONDITIONAL.
- The then-current PDF was rebuilt and came out byte-identical once the
  embedded `CreationDate`/`ID` were normalised, so that build was
  deterministic. (That ASCII-formula PDF has since been replaced; see the
  rewrite below.)
- Both documents were rendered at 2x and inspected page by page, with no
  clipping or overlap. The DOCX render used the locally installed Word;
  LibreOffice is still absent. Current sizes: PDF 26 pages (A4, from LaTeX),
  DOCX 22 pages (US Letter).

## Supplement rewrite: real mathematics, one LaTeX source

The first supplement typeset its formulas as monospace ASCII (`grad H*^T`,
`sqrt(2B/(alpha lambda_H))`) and compressed each proof into a few sentences.
It was rewritten.

- Every formula is now authored in **LaTeX**, once, in
  `supplement/theorem1_document.py`.
- `render_latex.py` emits `theorem1_supplement.tex` and typesets it with
  Tectonic to produce the PDF.
- `build_supplement_docx.py` converts the same LaTeX to **native Word equation
  objects** (LaTeX -> MathML -> OMML via Office's `MML2OMML.XSL`), so the
  .docx contains real, editable mathematics rather than pictures or ASCII.
- Each lemma, proposition and the theorem now carries a numbered, step-by-step
  derivation; every inequality in the Lyapunov chain and the sampled-data
  expansion names the result that justifies it.

### The conversion needs a guard, and the guard found real bugs

`MML2OMML.XSL` drops constructs **silently**: the formula still renders, and
still looks plausible. `latex_to_omml` therefore compares the significant
characters of the MathML against those of the OMML and fails the build on any
loss. Running it over all 283 equations caught four classes of corruption:

| Construct | Silent result | Fix |
|---|---|---|
| `\|d\|^2` | the `d` vanished | group the fence body |
| `\|\nabla H^*\|` | the gradient vanished | group `\nabla` |
| `\pi^{3/2}` | became `\pi^{32}` | group the slash |
| `\left(a+b+c\right)` | became `(abc)` | group the fence body |

`\bigl\|` also survived as literal backslash-pipe, `\qquad`/`\quad`/`\;` were
dropped so equations ran together, and MathML accents arrived as OMML *limits*
(a hat drawn high and detached) rather than accents. All are normalised or
post-processed, and `tests/theory/test_supplement_content.py` checks both that
the traps are neutralised and that the guard still detects the raw ones -- a
regression in the guard would otherwise make those tests pass vacuously.

Re-run environment: Python 3.13.13, python-docx 1.2.0, latex2mathml 3.81.0,
lxml 6.1.2, Tectonic 0.15.0, pypdfium2 5.13.0 for page rendering. None is a
runtime dependency of `dbact`. Word 2016 was used to render the .docx for
visual QA; the build itself does not need Word, only its stylesheet.

## Package map

- `theorem1/`: scope, notation, assumptions, dependency graph, proofs,
  corollaries, constant ledger, and proof-obligation ledger.
- `wolfram/`: reproducible Wolfram Language scripts, exact queries, captured
  connector results, and a verification manifest.
- `python/`: the single numerical constant/certificate implementation.
- `certificates/`: generated analytic constants and the certificate schema.
- `supplement/`: standalone professor-facing document. `theorem1_document.py`
  holds the frozen prose and every LaTeX formula; `render_latex.py` produces
  the .tex and the PDF, `build_supplement_docx.py` the Word file.

## Evidence policy

`PROVED` means an analytic argument is present. `VERIFIED` means a specified
algebraic/integral subclaim was reproduced by Wolfram. `CONDITIONAL` means the
proof is complete only after the named assumption or runtime certificate is
supplied. `NUMERIC-SANITY` is never promoted to proof. In particular, A8 remains
conditional because no selected branch, reduced gauge, or full parameter tube
was supplied for a validated interval Hessian lower bound.

## Reproduction

```text
python theory/python/generate_certificate.py --config theory/theorem1/theorem_mode.yaml
python -m pytest tests/theory -q
python -m pytest tests -q
wolframscript -file theory/wolfram/scripts/01_kernel_constants.wl
python theory/supplement/render_latex.py            # .tex + PDF (needs tectonic)
python theory/supplement/build_supplement_docx.py   # .docx with Word equations
```

`theorem1_supplement.tex` is generated, not hand-edited: change
`theorem1_document.py` and re-run the two builders. The PDF is typeset from
that .tex by Tectonic. Page rendering for visual QA used pypdfium2, since
Poppler is not on this host's PATH.
