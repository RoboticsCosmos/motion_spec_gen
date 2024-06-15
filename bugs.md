
## Changes to be made / Bugs to be fixed:

- [ ] control_loop_dt has to be used for all controllers, if no factor specified. Ref freddy_uc1_ref.
- [ ] Handle the as-seen-by attribute required for the transforms before passing data to solvers better. 
- [ ] root_accelerations for the rne solver during initial tau computation is missing.
- [ ] error in orientation computation to be handled using KDL::diff internally.
- [ ] Compute all the setpoints/ current values at the beginning of the control loop.
  
## Changes made / Bugs fixed:

- [x] num_constraints is not correctly calculated during IR gen. Ref freddy_uc1_ref
- [x] transform_wrench is not properly populated during stst gen. Ref freddy_uc1_ref.
- [x] add initial_tau computations for the arms using rne which is later used for initial control mode change. Ref freddy_uc1_ref.