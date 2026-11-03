# Same-voter map increments: prospective mechanism and validation

The exact 989-frame trace identified a concrete failure at frame989: agent_05
changes from voter09 to voters06/09. Their current x displacements are 0.183816
and 1.111963 m. The absolute geometric median changes by -0.451596 m although
voter09 advances +0.012478 m. Three recent returns (frames976/981/983, source03)
had true errors 10.8--14.8 mm; their final fused cell is 438.753 mm outside.
This is an estimator-roster offset entering map motion. The upstream biased
reference readings are not yet repaired or certified by this mechanism.

Candidate `reference_map_bridge=False` by default; valid only with the existing
initial translation-reference map. No numeric parameter, physical model, speed,
margin, stop gate, random seed or step budget changes.

Each receiver stores only its own/one-hop fresh readings from the preceding
available frame. During stationary acquisition use the original zero epoch.
The first fresh moving reference anchors as before. On overlapping rosters,
translate history by the geometric median of SAME-voter displacement increments;
use only those common voters for the existing fresh filtered velocity aggregate.
New voters can contribute increments after a consecutive observation. This is
rotation equivariant and cancels constant per-voter offsets; it does not cancel
time-varying/common bias or unobserved rotation.

Missing reference means the existing unavailable/reacquisition behavior and no
new stored map data. After a gap or disjoint roster, old map geometry has no
supported relative bridge: discard target-object cells and anchor only current
one-hop returns. Do not shift old points with the new absolute readout. Preserve
other objects, own sensor/registration calls and the existing support/row rules.
No history is used as a current stop vote; navigation and phase inputs unchanged.

Tests must cover the actual989 roster discrepancy; same-roster constant biases;
rotation equivariance; no duplicate or out-of-neighborhood voter; missing/disjoint
rosters and reacquisition; object-specific history reset; default-off identity;
invalid contracts. The diagnostics expose actual sources, increment, state and
discard counts so every physical command's map update can be audited afterwards.

Prospective physical pilot: exactly four reused conditions (near circle, near L,
far C open-away pos0, far skew box s0/r0/pos0), each with the prior local-pair arm
as control and bridge=True candidate. Eight NEW episodes, same seeds/initial
states, shuffled schedule seed20261103, four workers/one internal thread,
12000steps/600s maximum per episode and original settling. No adaptive tuning.
All controls must reproduce original25arrays and scientific fields exactly.

Mechanism gate: complete8; controls exact4; all candidates safe-complete with
valid zero epoch and zero false HOLD/hard failure; preserve every control G500
pass; no matched acquisition/post-acquisition scaling increase; skew
post-acquisition scaling strictly below its original1; audit all bridge updates.
Full gate additionally requires zero candidate scalings, G5004/4 and formal
TRANSPORT4/4. Report all endpoint errors, time, rotations, scaling counts,
quorum/phase timing and failures. No default adoption in this small development
pilot even if a gate passes. It does not independently establish deployment
benefit, general reliability, fully decentralized supervision or hardware safety.
