# Read-only contact compatibility diagnostic on completed controls

The eight-episode physical candidate run is still active; all four controls are
complete and independently reproduce the old bridge trajectories. No candidate
terminal outcome has been inspected. This diagnostic creates zero new physical
episodes and does not modify the frozen run, gates, parameters or source.

For EACH of the four control conditions, select two event rules from all saved
steps after initial reference freeze and before BRAKE/HOLD: (1) the first step
with positive goal-projected physical contact force, and (2) the maximum summed
positive goal-projected contact force (earliest index breaks ties). If no eligible
event exists, retain that condition as missing. Deduplicate overlapping frames
but retain both rule labels. The selection never depends on new-row feasibility.

Exactly replay the control prefix through its latest selected step. Verify all
commands, full diagnostics, solver statistics, phases/guards/responses and map
points against its saved receipt. At selected frames prepare an independent
counterfactual filter from the original nominal command/params/map-motion input,
the current local own/one-hop packets and the published finite-support admission.
Do not replace the original controller/filter state or physical commands.
Keep original hard rows, speed, rho, reach capping and all fallback rules.

After original commands are fixed, evaluate all16 robot states per selected
frame. Primary subset: every agent with positive goal-projected physical contact
force in that saved hold. Record original/counterfactual commands, status/scale,
original command residuals against new capped/full uncapped object rows, contact
force, stage/pusher flags, all admission/fallback reasons and source roster.
Also retain the full16-agent denominator, all selected/missing conditions and
the original/new matrices in raw data. Received-speed and geometric assumptions
remain empirical. A changed hypothetical command does not establish a new force
or a closed-loop failure; this is a mechanism diagnostic to interpret the ongoing
pilot, not a replacement for its actual outcomes.
