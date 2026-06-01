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

## ControllerModule-Level Dry-Run

Stage 3.8 adds a ControllerModule-level dry-run script:

```text
platforms/mas_public/apps/dbact/run_controller_module_dtransport_dry_run.py


1. 新增 run_controller_module_dtransport_dry_run.py
2. 说明它和 run_dtransport_dry_run.py 的区别
3. 说明它复用了 ControllerModule helper methods
4. 说明它仍然不启动 ZMQ run loop
5. 记录验证结果：commands.csv 中 controller_mode=dbact_cage
6. 说明下一步才是 ZMQ mock publisher/subscriber dry-run

