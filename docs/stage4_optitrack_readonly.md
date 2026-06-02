\# Stage 4: OptiTrack Read-Only Integration



\## Current status



Stage 4 focuses on connecting real Motive / OptiTrack data to the MAS platform without sending commands to RoboMaster robots.



Validated so far:



\- NatNet Python SDK files restored.

\- NatNetClient can be imported.

\- NatNetAdapter is selected instead of MockNatNetAdapter.

\- Motive connection succeeds through 127.0.0.1 Unicast.

\- Python receives continuous MoCap frames.

\- Read-only logger writes WorldState CSV headers.

\- Raw rigid body diagnostic option `--print-raw-bodies` is available.



Current blocker:



\- Motive has not yet created or streamed robot rigid bodies.

\- The logger receives MoCap frames but reports `raw\_bodies=0` and `robots=0`.



\## Next step



Create and enable rigid bodies in Motive:



\- Rigid Body 001 -> robot\_1

\- Rigid Body 002 -> robot\_2

\- Rigid Body 003 -> robot\_3



Then run:



```powershell

cd E:\\DBACT\\boundary-aware-cooperative-transport\\platforms\\mas\_public



python apps\\dbact\\log\_optitrack\_world\_state.py --frames 300 --hz 100 --print-every 30 --print-raw-bodies --output data\\optitrack\_readonly\\real\_world\_states.csv

