# Exact row diagnosis of all map-bridge scaling events

Base bceb1c4. No runtime/numeric changes or new physical episodes. Source is the
completed map_reference_bridge_v1 physical protocol, de7e895a..; original
candidate and failures remain unadopted. Four-condition experiment is NOT rerun.

Replay the skew candidate through saved517, covering all18scaled robot steps:
original acquisition402/agent05 and17post-acquisition agent14 steps469--517.
Separate20-frame preflight. Compare EVERY command, full diagnostics, cumulative
solver statistics, phase/guards/response and map points to the executed receipt.
Capture every robot's original A/b/h, hard-row kinds, normal motion, nominal and
filtered commands, parameters/domain, all current permitted packets and original
map inputs at each selected frame. Do not omit failed or inconvenient states.

Truth only AFTER commands are fixed: robot clearance/normal/closest point;
each input map point's true clearance, closest point and normal; original cargo
pose/shape for finite-edge projection analysis. Never feed truth to control.

Predefine saved-state comparisons: capped full, uncapped full, uncapped no-rho,
uncapped zero-motion full and zero-motion/no-rho; preserve all agent/wall rows
and original speed bound. Decompose object-only, agent-only and joint systems,
including inclusion-minimal conflicts. Use exact2D plus independent CLARABEL.
For17aggregate states additionally recompute rows with (a) true projections of
input points only, (b) true nearest normals only, (c) both, (d) true nearest
boundary row at the robot. Retain the received velocity model and original rho;
these truth-assisted substitutions are diagnosis, NOT deployable interventions.
Report when a numerical comparison cannot be verified; it is not infeasibility.

Analyze the original selected map geometry and projection onto true finite edges
to distinguish unsupported plane extension, noisy normal/point estimates,
motion demand and agent-row conflict. Only then propose an observation-only
correction and freeze a separate physical validation if justified. No adoption,
broader safety, statistical generalization or deployment-contribution claim.
