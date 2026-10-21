# CCTA 2027 manuscript structure

This is an implementation-to-evidence map, not an authored submission or an approved title.
Official guidance is in [CCTA2027.md](../../docs/CCTA2027.md). The six-page budget is a project
choice within the published initial eight-page cap.

The current experimental revision and its acceptance boundaries are specified in
[CCTA2027_EXPERIMENTS.md](../../docs/CCTA2027_EXPERIMENTS.md).

| Section | Approx. pages | Implementation / evidence |
| --- | ---: | --- |
| Introduction and related work | 0.75 | `docs/paper_storyline.md`; verified literature still required |
| Model and information permissions | 0.75 | `dbact.types`, `sensing/`, `contracts.py`; oracle versus local sensing |
| Boundary-aware deployment and safety | 1.5 | `deployment/density.py`, `cvt.py`, `safety/filter.py`, `control/controller.py` |
| Delimited analysis | 0.75 | `theory/static_deployment/`; geometry, sampling and error assumptions |
| Experiments and limits | 1.5 | `experiments/`, `dbact_sim/experiments/`, independent evaluator and evidence matrix |
| Conclusion and references | 0.75 | Conclusions bounded by populations; no fabricated references |

Preparation, physical contact, safety and task completion need distinct measurements. Keep
local spacing and censored pairs visible. Static results cannot substitute for dynamic contact
validation. Captions must name their source batch and quantity. Archived October evidence
predates this refactor and the unvalidated local candidates.

Before creating `main.tex`, settle the storyline, verify related work, freeze a new full study
and retrieve current official IEEE template guidance. No submission-compliance claim is made
about a nonexistent manuscript.
