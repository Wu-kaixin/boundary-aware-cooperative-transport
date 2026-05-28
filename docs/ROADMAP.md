# Roadmap

## Stage 1: Simulation baseline

- [x] Arbitrary polygon cargo model
- [x] Local boundary sensing
- [x] Boundary-aware density field
- [x] Local CVT approximation
- [x] Local CBF safety filter
- [x] Simplified caging-pushing dynamics

## Stage 2: Algorithm improvement

- [ ] Replace half-plane projection with formal QP solver
- [ ] Add boundary gap detection and explicit adaptive recruitment
- [ ] Add object pose estimation from local boundary memory
- [ ] Add nonholonomic robot model
- [ ] Add contact force/friction model

## Stage 3: MAS integration

- [ ] Run adapter in MAS mock environment
- [ ] Implement OptiTrack object observer
- [ ] Add experiment recorder fields for object pose
- [ ] Add emergency stop and contact-loss behavior

## Stage 4: Paper-quality experiments

- [ ] Circle / rectangle / L-shape / non-convex benchmark
- [ ] Compare against baseline CVT + fixed circular AOI
- [ ] Ablation: no CBF, no boundary density, no communication
- [ ] Real RoboMaster S1 experiment
