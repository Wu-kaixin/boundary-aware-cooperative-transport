# Acquisition failure trace v1

Purpose: explain the reference-map candidate's skew-box acquisition failure before
changing a controller mechanism. Preserve all best numeric parameters, guards,
original receipts and failed physical gates. This is an offline diagnosis with
zero new physical episodes, not a new success-rate experiment.

Compare the two existing skew trajectories from reference_map_v1: the original
translation reference through its freeze frame791 (39.5s), and the rejected map
candidate through frame1201 (60s deadline). Reconstruct each controller from its
original frozen runtime, inject saved pre-physics poses, and require exact
commands, all diagnostics, solver counters, phase/guard/response fields and the
recorded map. A mismatched reconstruction is invalid evidence and retained.

Record every agent's capture count, normal conditioning, spatial minor RMS,
largest normal-angle gap and exact original support predicate. Record incoming
own/one-hop packets and lost new cells at the existing4096-cell capacity. Never
relax the original32-point,1e-3 conditioning,3-noise spatial or pi-angle tests.
Record nominal targets, modes, neighbors, pre-physics positions and fresh sensor
counts. Evaluate capture capacity exhaustion versus missing viewing directions;
an aggregate map coverage value alone cannot prove the capture predicate.

For every scaled QP retain the original matrices and distinguish object-only,
robot-only and joint feasibility, with the original speed cap; also test the
rho-free rows and zero estimated object motion diagnostically. Retain first and
worst full witnesses. True object geometry is observer-only and must not enter
any original controller, map, capture or constraint calculation.

A20-frame preflight for each arm checks reconstruction plumbing. The fixed full
prefixes follow only if it passes; do not restart a live handle or overwrite an
attempt. Store raw trace and script snapshots under a new runs/research directory.
Publish the complete prefix denominators, counters, all support failures and
any negative hypotheses. Draw a mechanism conclusion only from these traces;
a controller remedy needs a separate design and prospective physical protocol.
