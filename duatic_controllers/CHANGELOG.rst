1.0.0 (2026-11-06)
------------------
* feat: add Cartesian pose controller (#45)
* fix(controllers): reserve ordered interface vectors before filling them (#64)
* fix(force_torque_broadcaster): prevent segfault on startup (#63)
* fix: target twist publication and documentation (#61)
* fix: add #include <unordered_set> (#41)
* docs(control): typo corrected for default parameter value (#51)
* docs(control): reference to the ProxSuite library (#48)
* removed safety scaling controller - it never really worked properly (#39)
* wire in joint name into message
* Fix sign issue in brake release controller (#30)
* Gravity Compensation Only (#27)
* Bugfix: pinocchio model reloading (#28)
* Feature/brake safety support (#20)
* Add controller documentation (#12)
* migrated controllers doc from dynaarm repo (#18)
* add support for lyrical (#17)
* CartesianPoseController - Do not have higher prio for translation
