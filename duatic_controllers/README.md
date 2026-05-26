# duatic_controllers

This package contains a large variety of [ros2control controllers](https://control.ros.org/rolling/doc/getting_started/getting_started.html#controllers) specifically designed and tuned for the [Duatic](https://www.duatic.com/) robots.

Please note that product specific controllers are still contained in the product specific `duatic_<product name>_controllers` package.

## License

The same BSD-3-Clause license as for the whole [duatic_ros2control](github.com/Duatic/duatic_ros2control) repository applies.


## Overview

The controllers can be grouped into Duatic product specific controllers and general purpose controllers.\
In the following we always use the exported name without the `duatic_controllers/` prefix.

### Duatic product specific controllers

| Name | Description |
| ---- | ----------- |
| [PIDTuner](./doc/PIDTuner.md) | Allows to tune at runtime the DuaDrive actuator PID gains|
| StatusBroadcaster| Publish DuaDrive specific information as ROS2 topic |
| [BrakeReleaseController](./doc/BrakeReleaseController.md) | Controller which is used to disengage the brakes on some DuaDrive models |
| SafetyScalingController | Allows at runtime to scale down maximum velocity and maximum torque by a configurable factor |
| FreeDriveController | Supporting controller for freedrive mode |
| [FreezeController](./doc/FreezeController.md) | Instantly freezes all drives (Safe Operating Stop) |


### General purpose controllers

| Name | Description |
| ---- | ----------- |
| ForceTorqueBroadcaster | A virtual FTS sensor calculating a wrench at the TCP using the joint torques |
| CartesianPoseController | A cartesian pose controller which handles the DynaArm kinematics more joyfully |
| [GravityCompensationController](./doc/GravityCompensationController.md) | A feed forward controller which tries to keep the current dynamics |
