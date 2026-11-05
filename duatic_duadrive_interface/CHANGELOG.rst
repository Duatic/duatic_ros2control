0.1.0 (2026-11-05)
------------------
* feat(control): read and apply joint limits from the URDF (#35)
* fix: update get_package_share_directory -> get_package_share_path (#40)
* fix(interface): incorrect mode conditions (#36)
* fix: abort when attaching slave to master fails (#31)
* Abort startup if drive is in Fatal State + Abort startup if errors could not be cleared (#33)
* Hotfix - fix issue with mock hardware
* Add communication timeout which freezes the system in the error case (#32)
* enforce disable mode in case freeze was desired but the brake is active (#24)
* added a hw description validation on startup to detect missalignments between code and model (#23)
* Feature/brake safety support (#20)
* add support for lyrical (#17)
* explicitly add -fPIC
* remove the part of the safety scaling which actually writes to the hardware as it created some
  issues
* fix unused variable warning
* Feature/migrate controllers (#11)
* enable position limit only for joints where it is appropriate (#6)
* fix joint torque state field
* extended the logging statement from the last commit
* added raw statusword to the debug line
* put the whole system into enforced freeze in case of a single component error (#9)
* print the current state enum
* Print drive status at startup (#7)
* Feature/explain limiter better (#5)
* Fix/improve limiter (#4)
* wire in per drive freezemode (#3)
* defer fsm setting at startup (#1)
* fix interface handling
* Added position command feedback for non position control modes, added state interface field for
  current drive control mode,
* please pre-commit
* remove gain interfaces from mode selection
* fix bug in joint limiter
* fix some warnings
* fix pid gain selection mode
* added support for JointVelocity mode in mock
* improve logger naming, added support for initial positions for mocked interfaces
* rework advanced command limiter and handle freeze mode explicitly
* fix advanced position command limiting logic
* disable advanced position command limiter
* wire in advanced joint command limiting algorithm
* added comment
* fix interface initialization bug at startup
* fix several issues
* wire in correct setup of initial position command
* added debug printout
* made pre-commit happy
* added extract_interface_name method
* correctly select the right drive mode
* rework to new urdf structure
* added duadrive_interface.urdf.xacro
* remove cmake based standard tests
* rework duadrive_interface to use a consisten namespace and provide a coupled kinematics hardware
  interface
* Update internal include paths
* update package.xml
