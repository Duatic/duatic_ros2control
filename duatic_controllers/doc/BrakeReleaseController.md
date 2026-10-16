# BrakeReleaseController

Use this controller to deactivate the brakes at startup on any Duatic DuaDrive equipped with brakes.

__Why is this controller needed?__\
The DuaDrives use pin brakes. If the full weight of an arm is loaded onto a pin brake it is possible that the pin cannot be pulled.
For this case the pin needs to be unloaded.


## Usage

1. Add entry in controllers.yaml and in your launch infrastructure. Do not directly activate it during startup
2. Make sure to deactivate the `FreezeController`
3. Activate the `BrakeReleaseController`
4. If it was for some reason not successful -> Retry (deactivate -> activate). See [Checking the release](#checking-the-release)
5. If it did not deactivate itself, deactivate it to release the joint interfaces and the effort references

__NOTE:__ Both modes assume the [GravityCompensationController](./GravityCompensationController.md) is up and running.
The "auto" mode chains onto it: it claims its `effort` reference interfaces, so it cannot be activated without it.
The "manual" mode only claims `target_brake_state` and reads `brake_released`, so it leaves every joint interface to other controllers.


## How does it work ?

### Mode: "auto"

The controller excites the brakes and sweeps a torque offset per drive, on top of the gravity compensation.
The pin sits on the motor of the drive, so in a coupled arm, where one drive turns several joints, the offset is planned in drive torque and mapped to joint torques with `coupling`.
Drive torque is the torque at the output of the drive, after its gearbox and before any coupling or transmission outside the drive.
The pin itself sees the drive torque divided by the gear ratio of the drive, which the controller does not know, so the same `max_torque` loads the pin of a drive with a smaller gear ratio more.
On an uncoupled joint the drive torque is the joint torque. On the DynaArm the shoulder_flexion joint gets the torque of its own drive plus that of the elbow drive, and gearing outside the drives scales it further.
The joint torques go into the GravityCompensationController's effort references.
While searching, the position and integral gains of the drives are 0, so the offset alone acts on the pin; the drive's damping stays.
On the side where the pin is loaded the offset only presses it harder.
In the free direction the offset lifts the load off the pin and the solenoid pulls it.

`joints` names the drives, which carry the names of the joints. It has to list every joint that a swept drive turns, since each of them gets a share of the offset.

To load the pins as little as possible, the offset sweeps at `torque_rate` through a quarter of `max_torque` in the positive direction, then the negative one, then half and finally the full `max_torque` in both directions, and back to 0.
It changes direction by ramping through 0, never by a step.
A level is only left once the drive no longer moves in the pushed direction faster than `stop_velocity`: a joint pressing on its pin stops at once, while a free joint keeps moving against the drive's damping until it has travelled far enough for its release to be detected.
A free joint is therefore handed back while it still moves at about offset divided by the drive's damping gain.

As soon as a joint reports `brake_released`, its offset is removed, its brake is put into hold, its gains are restored and it is held at its current position.
A joint whose sweep ends without a release gives up and is held the same way, but its brake stays excited, since a coupled drive that is still searching may unload its pin later.
Once no joint is searching any more, the remaining brakes are put into hold.
If every drive reports `brake_released`, the controller deactivates itself. Otherwise it logs the joints that are not released and keeps holding them until it is deactivated.

### Mode: "manual"

In this mode the controller simply performs the unlock lifecycle by:

"excite brakes" -> user moves the joints until every drive reports `brake_released` -> deactivate itself

The brake of each drive is put into hold as soon as that drive reports `brake_released`.
While waiting, the controller logs every 3 seconds which drives are not released yet.

## Checking the release

The drive only reports whether its solenoid is energized, not whether the pin came out.
The hardware interface therefore infers the release from motion. The pin sits on the motor side of the gearbox, so while it is engaged the motor can only move within the pin clearance.
Once the motor travels further than `brake_clearance`, the `brake_released` state interface becomes 1.0, independent of the commanded solenoid state.
The [StatusBroadcaster](./StatusBroadcaster.md) publishes it as `brake_released` for every drive.

It stays 1.0 until the stack is restarted.
Drives without a brake always report 1.0.

`brake_clearance` is a parameter of the `duadrive_interface_joint` xacro macro, in rad of motor travel, with a default of 1.2.

## Interaction

### Mode: "auto"

Activate, the controller deactivates itself once all brakes are released

### Mode: "manual"

Activate, manually move all joints around until the controller reports all brakes released and deactivates itself

## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `joints` | List of joint names as described in the URDF the controller shall handle | - | may not be empty |
| `torque_rate` | Rate in mode "auto" in [Nm/s] at which the torque offset changes | 10.0 | > 0.0 |
| `max_torque` | Largest torque offset in mode "auto" in [Nm] per drive, as drive torque: at the output of the drive, after its gearbox and before any coupling or transmission outside the drive. Tried as a quarter, a half and the full value in both directions | 3.0 | > 0.0 |
| `stop_velocity` | Drive speed in mode "auto" in [rad/s], in drive coordinates, below which a drive counts as stopped in the pushed direction, so the sweep moves on to its next level. A coupling with gearing outside the drives scales it against the joint speed | 0.02 | > 0.0 |
| `coupling` | Row-major matrix in mode "auto" mapping the torque of each drive to the torques of the joints, both in the order of `joints`, as the coupled to serial torque mapping of the robot's kinematics | [] (every joint has its own drive) | `joints` squared entries |
| `gravity_compensation_controller` | Name of the GravityCompensationController whose effort references carry the offset in mode "auto" | "gravity_compensation_controller" | - |
| `mode` | Select as an alternative manual mode. Read-only, so a change needs the controller to be reloaded | "auto" | "auto" or "manual" |

## Notes

To deactivate itself, the controller calls the `switch_controller` service of the controller manager named `controller_manager` in its own namespace.
