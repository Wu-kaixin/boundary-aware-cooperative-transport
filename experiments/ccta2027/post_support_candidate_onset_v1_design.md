# Actual-trajectory propulsion onset diagnostic

This is a read-only analysis of all eight completed episodes in the frozen
post-acquisition finite-support comparison. It changes no controller, parameter,
physics, gate or active audit helper. The original full audit remains live.
These four conditions have already been used in development; repeated frames and
agents are not independent trials. This is not a new physical experiment.

## Questions and fixed selection

Locate the first change in the actual candidate trajectories, separating nominal
intent, filtering, contact force, reference acquisition and task progress. Analyze
all four matched pairs in protocol order; keep all failures and missing events.
Use a fixed numerical classification tolerance of 1e-7 m/s only for diagnostic
velocity signs, without changing any controller threshold.

For every episode retain a compact record for every frame: phase/reference status,
true/estimated progress, goal-projected actual object velocity, positive/reverse
force, support admissions, declared pushers, positive nominal commands and their
executed signs, and solver-status counts. Select the first frozen-reference frame,
first finite-support admission, first frozen declared-pusher state with positive
nominal goal velocity but nonpositive executed goal velocity, and first positive
physical force after reference freeze. Keep all 16 agents at each selected frame.
Also record full-trajectory maxima and final values, without changing selectors
after observing the outcomes. No state selected here is called a causal failure
solely because it has a negative command.

For each matched pair locate the first different executed command in their common
recorded frame range. At that frame compare pre-command robot/cargo poses from
replay.npz, nominal commands, reference/phase data, and saved diagnostics. The
preceding commands and physical states must be exactly equal before calling this
an initial divergence under common inputs. Do not compare later different
trajectories as same-input controller counterfactuals.

If further row-level analysis is needed, replay both controllers from frame zero
through the initial divergence, without any physics step; match every saved
command, diagnostic, solver statistic, phase response and map snapshot. Copy the
prepared constraint systems and permitted support inputs at that frame. Only
after exact replay succeeds, evaluate original/candidate directional capability
using the already verified full-system optimizer. A velocity capability statement
is not a force, physical-safety or whole-trajectory causal certificate.

## Data integrity

Use a bounded-memory reader for the existing top-level {steps: [...], summary: ...}
JSON format. Test nested strings/escapes, arbitrary chunk boundaries, truncation,
invalid separators, duplicate/unknown top-level keys and trailing data against
the standard JSON decoder. Exhaust each episode and verify frame indices, count,
all commands against replay position increments where applicable, and saved
robot positions/progress against the NPZ data. Verify parsed summary with the
original summary (respecting the actual owning writer's schema).

Before data analysis, preserve this design, new scripts/tests, hashes of all input
files and the frozen source/environment/config identities in a fresh runs/
directory. Retain unsuccessful attempts. Do not modify packages or the running
analysis adapter. Check the frozen identity before and after. The new tool must
not silently turn a truncated file into a completed episode.

## Decision

Use the first verified divergence to identify which constraint/nominal input
changes first, then test a specific contact-compatible mechanism in its own
design. Do not remove margins, use evaluation truth for control, promote the
failed candidate, or claim that fixed-state certificates explain all four full
trajectories. Formal full-episode acceptance still belongs to the original audit.
