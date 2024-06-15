
## Changes to be made / Bugs to be fixed:

- [ ] handle getLinkOrientation (orientation coordinate) properly (1d vs quaternion)
- [ ] error in orientation computation to be handled using KDL::diff internally.
- [ ] Handle the as-seen-by attribute required for the transforms before passing data to solvers better. 
- [ ] root_accelerations for the rne solver during initial tau computation is missing.
- [ ] Compute all the setpoints/ current values at the beginning of the control loop.
  
## Changes made / Bugs fixed:

- [x] bug fix in vector computation for the position and orinetation coordinates