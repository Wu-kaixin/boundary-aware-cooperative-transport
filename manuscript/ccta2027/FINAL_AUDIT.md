# Final local draft audit — 2026-11-04

**Result: local research-draft deliverables verified.** This is not acceptance of a new transport algorithm, dynamic safety guarantee, or formal conference submission.

| Required deliverable | Evidence and result |
|---|---|
| Trustworthy and reproducible baseline | Current source/config/environment identity rechecked; protected YAML and runtime unchanged. Core regression: 1113 passed, 2 skipped; MAS: 106 passed. |
| Scientific question and contribution decision | [Research summary](RESEARCH_SUMMARY.md); deployment-to-transport gap and directional-capability analysis, with novelty and information limits. |
| Multishape full-task comparison | Complete historical populations remain separate; current frozen protocol has 60/60 terminal receipts, zero execution errors, 58 spatial completions, 56 without original hard safety failure. |
| Mathematics and assumptions | Contact equations, QP set/projection distinction and dual bound in main; restricted identifiability/accounting in supplement; source mapping in [index](RESULTS_INDEX.md). |
| Success, failure and limitation report | Both incomplete current cases and both penetration failures retained; rejected candidate branches and interrupted full constraint audit explicitly identified. |
| Figures, media and reproduction | Matplotlib figures, four new fully decoded MP4s, verified 81-frame GIF, Inkscape vector export, raw-replay hashes and fresh-output commands in [media guide](MEDIA_AND_CLAIMS.md). |
| English source, references and PDF | Main 8 pages, supplement 6 pages, both Letter. Twenty main references resolve; fonts embedded, zero Type 3; no overfull, missing-glyph or undefined-reference warning. |

## Verification details

- Main final pages 1–8 were rendered and inspected. Final edits changed only pages 4 and 6 relative to the earlier fully reviewed eight-page render; all other page PNG hashes are identical. The changed pages were inspected again.
- Supplement was re-rendered after fixing float-only heading pages. Every final page was inspected for readable mathematics, labels, captions and cropping. This supplement is a local reading companion, not a claim of conference attachment compliance.
- New media final-state previews were inspected; start and intermediate frames were additionally checked. MP4 decoder checks cover all frames, while visual sampling does not claim inspection of every animation frame.
- Main font objects: 21; supplement: 23. Recursive PDF resource/font-descriptor checks use the root environment's `pypdf`; `pdffonts` is unavailable in the local Poppler subset. Poppler rendered the pages.
- `git diff --check` and syntax parsing of the seven new research/presentation helpers pass. Scientific helpers also completed their built-in source, input/product identity, denominator and numerical consistency assertions during actual execution.
- The first core test attempt failed during temporary-directory setup under the sandbox. Its logs remain separate; the approved rerun in a fresh repository-local test directory completed with the counts above. No software failure is concealed by discarding that receipt.
- The first supplementary build lacked cached `article.cls`; it was changed to the existing official `ieeeconf` class in one-column mode. The successful build still emits a Fontconfig configuration message, but recursive font inspection confirms embedded fonts and visual rendering shows the intended glyphs. Main underfull spacing warnings are retained and inspected; no overfull warning remains.
- Inkscape 1.4.4 was downloaded from its official release link as a local portable tool. Its noninteractive SVG-to-PDF export succeeded and was visually checked. GTK display/recent-file warnings are retained as environment messages; no system-wide installation or solver change occurred.

Machine-readable results: `runs/research/2026-11-04-ccta2027-draft-refresh-v1/final_audit_v1/audit.json`. The same directory's `release_manifest.json` identifies final local deliverables. Raw current validation remains in `runs/research/2026-11-04-resource-noise-validation-v1/`; no historical run or source snapshot was rewritten.

## Scope left deliberately unclaimed

No complete finite-support constraint PASS; no resumed rejected algorithm; no fully decentralized supervision; no arbitrary-shape/noise reliability or continuous-time contact-safety theorem; no hardware validation, remote push, public upload or formal submission. Author identities remain pending. These are explicit scientific and publication boundaries, not missing local-draft deliverables.
