# Initial-geometry task reference: design and implementation plan

Date: 2026-11-02. Status: design before candidate implementation/outcomes.
User authorization: synchronize the research branch, then autonomously pursue a
task-consistent displacement reference, far-field stopping, and a matched test of
the contribution of boundary deployment. This is a simulation study.

## Existing evidence

Baseline commit 27d17ad contains all prior studies. Root environment Python
3.12.14/PyYAML 6.0.2 is retained for this development study; the lock lists 6.0.3.
Prior runs and their byte identities remain unchanged. A fresh baseline check
passed 829 core tests (2 historical skips), MAS tests, smoke, and resume.

The eight original arms of reference_origin_v1 are development conditions.
The pre-contact capture probe used reconstructed same-input scans, checking all
scan counts. A trigger at robot_radius + 3*range_noise_std + 2*max_speed*dt
(0.195 m here) preceded all true cargo translation/rotation in these eight runs.
Far C and skew initial observations are nearly linear: minor spatial RMS about
0.008--0.023 m with 0.01 m range noise. Zero bootstrap alone cannot supply the
missing geometric information. These observations are diagnoses, not successes.

## Intended change and alternatives

The chosen research direction is an initial observed-geometry reference with
an explicit observation acquisition period. The object starts at rest and is
driven only by robot contact, as in the existing simulation. Keep a conservative
observed boundary standoff during acquisition; preserve original physical and
evaluation parameters. Gather permitted own/one-hop scans into per-agent initial
templates while the object remains untouched, then freeze the reference at zero
displacement. Never inherit the old map displacement or its 1.24 gain.

Rejected alternatives: further gain/continuity/window tuning does not repair
reference semantics; using task.start, cargo pose, full shape or evaluation
contact would violate the information model; making a boundary point the new
evaluation target would change the task. Full global-map centralization is not
selected. The existing common phase supervisor may aggregate readiness bits;
local registration uses own/one-hop permitted boundary data.

Each tracker estimates a rigid transform relative to initial geometry. A
reference point derived from observed geometry is explicitly distinguished from
the unknown physical center. Moving between reference points must use
t + (R-I)(q-a). The extra error at an offset center is bounded conditionally by
2*abs(sin(theta/2))*distance(q, center), not by the registration residual. With
incomplete geometry no unconditional center bound is asserted. A design that
only improves material-point tracking does not count as solving task stopping.

## Boundaries and unresolved implementation choices

- Preserve the 0.12 m independent task tolerance, 0.02 m controller arrival band,
  40-frame physical linear-speed settling, safety evaluator, dt=0.05 and maximum
  600 physical seconds. G500 remains separate. No hardware commands.
- Common reference configuration: the existing coordinate-consistent safety,
  vector goal, observed-reference position/velocity, radial pivot, contact patch
  distance and 0.45 motion-prior development configuration. Failed continuity,
  early-anchor and support-only candidates are not promoted as proven fixes.
- Initial template storage is bounded and voxelized at 2*range_noise_std. Keep
  observation timestamps and acquisition/freeze diagnostics. Empty or weak
  geometry must abstain. Test rigid-frame covariance and permission boundaries.
- A newly joining robot cannot silently initialize at zero after contact. It must
  use a previously captured initial template or abstain.
- Determine the least invasive acquisition control supported by the existing
  deployment implementation. Start with extra observed standoff and a readiness
  condition on spatial and normal support; if deployment cannot acquire support,
  report that outcome before specifying a distinct active-observation candidate.
- A common geometric center is not assumed observable merely because the ICP
  Jacobian has rank three. Choose the reference readout and its uncertainty model
  based on permissible geometry, and retain failures under rotation.

## Sequence and gates

1. Complete GitHub synchronization (done: 27d17ad).
2. Test and implement the smallest standalone initial-reference component. Use
   exact synthetic transforms to test translation, rotation/reference point,
   rejected line support, missing scans, immutable templates and invalid epochs.
3. Freeze an offline comparison before candidate replay on the eight original
   development trajectories. Examine availability, center-relative error and the
   old false-HOLD instant. Old trajectories cannot show a new acquisition policy;
   distinguish feasibility probes from a claimed closed-loop improvement.
4. Freeze a bounded acquisition development comparison if needed, with identical
   initial states and all failures retained. New acquisition geometry requires
   new data, not reconstructed claims of changed historical observations.
5. Only a candidate with supported reference semantics and useful fresh feedback
   enters a matched closed-loop stopping comparison. Rescue far skew, retain all
   seven prior completions, no new hard-safety failures, no false HOLD and at least
   old 6/8 G500 are the development adoption targets, not guaranteed outcomes.
6. After selecting/fixing the final system, freeze unused scene blocks and compare
   boundary_cvt/local_spacing/nearest with identical information, common transport
   control and budgets. Report task completion and common-time process metrics,
   not only method-specific readiness snapshots. Independent validation cannot
   reuse the eight diagnostic conditions as independent evidence.

## Implementation tasks

- [x] Preserve current modifications and synchronize the paper branch.
- [x] Probe original scan availability and pre-contact stationarity, without physics.
- [ ] Resolve the reference/acquisition interface; add failing mechanism tests.
- [ ] Implement an opt-in component under src/dbact/control and controller wiring.
- [ ] Add a fresh-output study helper under scripts and frozen protocols here.
- [ ] Run the bounded development stages and retain every receipt/source snapshot.
- [ ] Review results before finalizing an independent comparison protocol.
- [ ] Run current regression, publish compact evidence and update the branch.

Engineering and experimental execution outputs go to new paths under
runs/maintenance/2026-11-02-task-reference-v1 and runs/research/. No historical
protocol, receipt, raw trajectory or conclusion may be relabeled. Non-adoption is
a valid experimental result and must be reported as unresolved user objectives.
