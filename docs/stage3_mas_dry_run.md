\# Stage 3: MAS Dry-Run / Virtual Object Experiment



\## Goal



Stage 3 validates the vendored MAS platform integration without using physical robots, OptiTrack, RoboMaster SDK runtime, or network messaging.



The dry-run pipeline is:



```text

MAS config loader

&#x20; -> dtransport controller

&#x20; -> synthetic WorldState

&#x20; -> MAS ControlCommand

&#x20; -> integrated mock robot states

&#x20; -> states.csv / commands.csv / events.csv / trajectory.png

