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
5. If it did not deactivate itself, deactivate it to release the `position/velocity` interfaces

__NOTE:__ The "manual" mode assumes the  [GravityCompensationController](./GravityCompensationController.md) is up and running.
The "auto" mode leaves the `effort` interfaces unclaimed, so the GravityCompensationController may stay active and its feedforward torque helps unloading the pins.
The "manual" mode only claims `target_brake_state` and reads `brake_released`, so it leaves every joint interface to other controllers.


## How does it work ?

### Mode: "auto"

The controller excites the brakes and moves the position target of every joint slowly (`probe_velocity`) in the positive direction.
On the side where the pin is loaded the joint is blocked, so the position error and with it the motor torque keep growing.
In the free direction the motor lifts the load off the pin, the solenoid pulls it and the joint follows the target.

To load the pins as little as possible, the torque is escalated: each direction is tried until the measured drive torque in that direction reaches a quarter of `max_torque`, then both directions again with half and finally with the full `max_torque`.
`max_torque` is the only limit on an attempt, so a drive that builds up no torque keeps searching until the controller is deactivated.
When an attempt ends, the target jumps back to the measured position, which takes the extra torque off the pin at once, before it moves in the other direction.

As soon as a joint reports `brake_released`, it is held at its current position.
After the last attempt a joint gives up and is held where it is.
Once no joint is searching any more, the brakes are put into hold.
If every drive reports `brake_released`, the controller deactivates itself. Otherwise it logs the joints that are not released and keeps holding them until it is deactivated.

The search relies on the ros2_control convention that a positive measured torque pushes the joint in the positive direction.

### Mode: "manual"

In this mode the controller simply performs the unlock lifecycle by:

"excite brakes" -> user moves the joints until every drive reports `brake_released` -> "put drives into hold" -> deactivate itself

While waiting, the controller logs every 3 seconds which drives are not released yet.
If not all drives are released within "timeout" seconds, it puts the drives into hold and stays active.

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
| `probe_velocity` | Speed of the position target in mode "auto" in [rad/s], also sets how fast the torque grows while blocked | 0.6 | > 0.0 |
| `max_torque` | Largest measured drive torque in mode "auto" in [Nm], including the torque of other controllers such as the GravityCompensationController. Tried as a quarter, a half and the full value in both directions | 3.0 | > 0.0 |
| `mode` | Select as an alternative manual mode. Read-only, so a change needs the controller to be reloaded | "auto" | "auto" or "manual" |
| `timeout`| Time in manual mode after which brakes are put into hold mode even if not all are released | 50.0 | >= 0.0 |

## Notes

To deactivate itself, the controller calls the `switch_controller` service of the controller manager named `controller_manager` in its own namespace.
