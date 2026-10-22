# Verified sources and claim checks

Checked 22 October 2026. These four primary sources support the specific related
work paragraphs in `main.tex`; this is not an exhaustive novelty review.

| Citation | Primary source | Checked use |
| --- | --- | --- |
| Cortes et al., 2004 | [Author-hosted corrected paper](https://motion.me.ucsb.edu/pdf/2002j-cmkb.pdf) | Voronoi/centroidal coverage framework; first page confirms journal, volume and pages |
| Ames et al., 2017 | [Caltech author repository](https://authors.library.caltech.edu/records/jnhr0-1ww05) | CBF-QP safety/performance framework; journal metadata and DOI |
| Pereira et al., 2004 | [Publisher record](https://journals.sagepub.com/doi/10.1177/0278364904045477) | Decentralized object-closure manipulation; abstract and bibliographic metadata only, no detailed algorithm comparison |
| Vardharajan et al., 2021 | [Author preprint](https://arxiv.org/abs/2106.03132) | Sequential placement around unknown shapes, transport, simulation and physical evaluation; cited as preprint to avoid inventing final proceedings metadata |

The related-work paragraphs paraphrase these sources; they do not reproduce
their text or claim to inherit their hypotheses or guarantees.

The [official CCTA call](https://ccta2027.ieeecss.org/authors/call-for-papers)
confirms standard two-column IEEE Proceedings format, eight initial pages and
six accepted pages before overlength fees. The draft uses `IEEEtran` conference
mode, consistent with the [IEEE template guidance](https://www.overleaf.com/latex/templates/ieee-conference-template/grfzhhncsfqn).
Author identities and affiliations remain unset; this is a review draft.

| Main-text assertion | Evidence / qualification |
| --- | --- |
| Static safety | `theory/static_deployment/main_theorem.md`, feature-cover and P0 hypotheses stated in text |
| Projected residual | `theory/static_deployment/applicability_and_residual.md`; no global geometric stationarity claim |
| 169 cases, static paths, completion, readiness, guard frequency | `docs/CCTA2027_EXPERIMENTS.md` and immutable `docs/results/ccta2027_draft_v2/` receipts |
| Failure outcome decomposition | `docs/results/reliability_2026-10-22/v2_failure_classification.json`; geometric symptoms, causal attribution unresolved |
| Repairs | Current controller and `tests/test_transport_reliability.py`; HOLD/command-invalidation fixes separated from disabled brake/feedback experiments and excluded from v2 scientific tables |
| Unproved extensions | Local coverage certificate, floating arithmetic budget, moving contact safety explicitly left open |
