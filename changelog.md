
## Changes to be made / Bugs to be fixed:

- [ ] handle getLinkOrientation (orientation coordinate) properly (1d)
- [ ] Handle the as-seen-by attribute required for the transforms before passing data to solvers better. 
- [ ] root_accelerations for the rne solver during initial tau computation is missing.
- [ ] handle embed map for multi-degree controllers
- [ ] update "measured" object for the controllers to not be redundtant with compute_variables
- [ ] compute setpoints (variable) at the beginning of the control loop
- [ ] bug in the achd solver beta population based on the active constraints !!!
  
## Changes made / Bugs fixed:

- [x] added pid controller for the base force control