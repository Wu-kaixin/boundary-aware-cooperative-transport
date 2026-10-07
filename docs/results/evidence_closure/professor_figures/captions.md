Figure 1. One robot's range returns on C-shape seed 8 at the initial instant. The dashed outline is evaluation geometry and is not a controller input. The figure shows why configuration has to be built from a partial boundary. It does not show a completed transport.

Figure 2. The modules that the current controller actually executes: local ranges, offset targets, a limited-range CVT centroid, the shared safety filter, and a brake that can leave CONTACT_READY without four push-side robots. Pause and watchdog are not task success. This is not a proof of the method.

Figure 3. The same C seed 8 CVT deployment before and after the downstream brake fix. The before run never brakes and passes the target by about 1.79 m. The after run brakes and stops near the target. This is not a comparison against spacing or nearest-target control.

Figure 4. Boundary CVT, nearest observed target, and local spacing on the development shapes and seeds, all using the controller after the brake and initial-clearance fixes. Endpoint error above 0.12 m is not task completion. Red marks a watchdog. The sample is the development set only.

Figure 5. Static L seed 2 with sampled map rows and with fresh segments added. Path clearance and robot displacement are separate. Crossing below 65 mm is an observed static violation. Restored motion is not a safety certificate, and a low clearance with large displacement is not a successful deployment.
