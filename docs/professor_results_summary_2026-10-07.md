# Simulation results for Professors Ogura and Bai

7 October 2026. Provisional. No manuscript title is chosen here.

## Completed
The work is on local branch `research/simulation-evidence-closure`, based on main `9693dfd`. It was not pushed. Metrics now separate clearance lower bounds, sampled clearances, and observed path violations. Controller phase times, physical-contact latches, progress errors, Euclidean endpoint error, settling, G500, and watchdog stops are stored separately. Empty events stay censored.

Reproduced archives: 9 static local/oracle pairs and 9 transport pairs. Development comparison: the same 9 conditions with boundary CVT, nearest observed target, and a local spacing engineering baseline. Ablations and two diagnostic batches were run on development cases only. A 36-episode frozen test, written before execution, then ran to completion (33 simulations and 3 retained initialization errors). Serial and parallel replays of one C-shape pair matched aside from wall time.

## Measured effects
Boundary CVT reaches the controller CONTACT_READY phase sooner than nearest observed target. On the frozen test, 10 pairs give a mean advantage of 4.37 s and a median of 0.77 s. The paired bootstrap interval lies below zero. Two pairs never produce a difference: one because the scaled L-shape starts inside a robot radius, and one because nearest target never becomes ready. Local spacing is as fast or faster. CVT is slower than spacing on 9 of 11 test pairs, and that interval includes zero.

Task completion is poor for every method once scale, distance, and search layouts vary. CVT, nearest target, and spacing complete 1, 1, and 2 of 12 test episodes. Many other episodes settle outside the 0.12 m endpoint tolerance. Absolute progress error does not show a reliable CVT advantage. G500 passes more often for CVT (4/12 versus 0/12 and 1/12), but G500 is not the task-completion metric.

The reproduced C-shape seed 8 failure is a phase latch. CVT pushes the object well past the goal, with sustained forward force, but never gets four robots onto the push side, so braking never starts. The other two targets do brake on that same initial state. A diagnostic change to the phase law can stop the overrun. That is a downstream fix, not evidence for the configuration rule.

A static scan-segment safety candidate removes the observed sub-65 mm path clearances on the nine local episodes. Robots also move much less. This was not transferred into transport, and it is not a certificate for unseen boundary.

## Not supported
Improved cooperative transport of unknown geometry is not supported. Faster preparation than the spacing baseline is not supported. A sooner quorum of positive-force contacts is not supported; that latch rarely fires, including on successes. Continuous safety of moving contact is not established. The density exploration ablation tested a gain that was already zero.

## Decisions left open
Whether the paper should be scoped to preparation time versus nearest-target control, with transport and spacing reported as limits. Whether a downstream phase recovery belongs in a later study. Whether a nonzero exploration gain, a direction factor, or partial visibility should be added as a new named protocol. Gate 6 v2 is still missing, so seeds 12–31 were not reused. The RAS reference PDFs were not in the workspace; no algorithm from those papers is claimed.
