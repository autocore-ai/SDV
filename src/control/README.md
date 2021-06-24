Control {#autoware-control-design}
=======

# Domain Description

The `control` sub-directory contains nodes and libraries related to the controller functionalities
of the vehicle.
Packages under this directory enable the autonomous vehicle to follow a reference trajectory
appropriately by converting the input trajectory into longitudinal and lateral commands.
To ensure safety, testing framework modules and libraries which report errors and system statistics
regarding the controller may also be packages under this directory.

# Subpages

- @subpage controller-base-design
- @subpage controller-design
- @subpage controller-reference-implementation
- @subpage controller-testing

- Model-Predictive Control Controller
  - @subpage mpc-controller-design
- Pure Pursuit Controller
  - @subpage pure-pursuit
  - @subpage pure-pursuit-nodes

- @subpage trajectory-follower-package-design
  - @subpage latlon-muxer-design
  - Longitudinal Controller (Not yet implemented)
  - Lateral Controller (Not yet implemented)