^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package duatic_duadrive_interface
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-11-06)
------------------
* feat(control): read and apply joint limits from the URDF (#35)
* feat: brake safety support; with the brake active, a requested freeze enforces disable mode
  instead (#20, #24)
* feat: freeze the whole system when one component fails, with a freeze mode per drive, and on a
  communication timeout (#3, #9, #32)
* feat: abort startup when a drive is in the Fatal state or its errors cannot be cleared (#33)
* feat: validate the hardware description against the code at startup (#23)
* feat: an advanced joint command limiter, with position limits only on the joints they apply to
  (#4, #5, #6)
* feat: position command feedback in non-position control modes, and a state interface for the
  current drive control mode
* feat: a coupled kinematics hardware interface under one consistent namespace, with
  duadrive_interface.urdf.xacro for the new URDF structure
* feat(mock): JointVelocity mode and initial positions for mocked interfaces
* feat: log the drive status and statusword at startup (#7)
* feat: support ROS 2 lyrical (#17)
* fix: abort when attaching a slave to the master fails (#31)
* fix(interface): correct the mode conditions (#36)
* fix: use get_package_share_path instead of get_package_share_directory (#40)
* fix: the mock hardware
* fix: select the right drive mode and PID gains, without the gain interfaces in mode selection
* fix: the joint torque state field
* fix: initialize the interfaces and the initial position command correctly at startup
* fix: defer setting the drive state machine at startup (#1)
* fix: stop the safety scaling from writing to the hardware
* refactor: move the controllers into duatic_controllers (#11)
* refactor: follow the rename to ecat_sdk_master (#66)
