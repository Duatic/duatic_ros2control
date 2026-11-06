^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package duatic_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-11-06)
------------------
* feat!: remove the safety scaling controller, which never worked reliably (#39)
* feat: add a Cartesian pose controller (#45)
* feat: choose between dynamics and gravity compensation in the gravity compensation controller,
  with introspection and a dry run mode (#27)
* feat: a manual brake release mode in the brake release controller (#30)
* feat: brake safety support (#20)
* feat: publish the joint name in the drive state
* feat: support ROS 2 lyrical (#17)
* fix(controllers): reserve ordered interface vectors before filling them (#64)
* fix(force_torque_broadcaster): prevent a segfault on startup (#63)
* fix: target twist publication and documentation (#61)
* fix: add the missing #include <unordered_set> (#41)
* fix: correct the sign in the brake release controller (#30)
* fix: reload the Pinocchio model in the gravity compensation and brake release controllers (#28)
* fix: no higher priority for translation than rotation in the Cartesian pose controller
* docs: document the controllers (#12, #18)
* docs(control): typo corrected for default parameter value (#51)
* docs(control): reference to the ProxSuite library (#48)
