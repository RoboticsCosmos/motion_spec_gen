
## Changes to be made / Bugs to be fixed:

- [ ] handle getLinkOrientation (orientation coordinate) properly (1d)
- [ ] Handle the as-seen-by attribute required for the transforms before passing data to solvers better. 
- [ ] root_accelerations for the rne solver during initial tau computation is missing.
- [ ] Compute all the setpoints/ current values at the beginning of the control loop.
- [ ] handle embed map for multi-degree controllers
  
## Changes made / Bugs fixed:

- [x] added getLinkQuaternion function to get the quaternion of the link.
- [x] error in orientation computation to be handled using KDL::diff internally.
- [x] added 3d orientation control support (partially done)
- [x] fixed control_loop_dt for all the controllers (maybe its a hack)