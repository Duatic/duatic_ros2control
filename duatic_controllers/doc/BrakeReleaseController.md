# BrakeReleaseController

Use this controller to deactivate the brakes at startup on any Duatic DuaDrive equipped with brakes.

__Why is this controller needed?__\
The DuaDrives use pin brakes. If the full weight of an arm is loaded onto a pin brake it is possible that the pin cannot be pulled.
For this case the pin needs to be unloaded.


## Usage

1. Add entry in controllers.yaml and in your launch infrastructure. Do not directly activate it during startup
2. Make sure to deactivate the `FreezeController`
3. Activate the `BrakeReleaseController`
4. If it was for some reason not successful -> Retry (deactivate -> activate)
5. Deactivate the controller to release the `position/velocity` interfaces

__NOTE:__ The "manual" mode assumes the  [GravityCompensationController](./GravityCompensationController.md) is up and running


## How does it work ?

### Mode: "auto"

The controller calculates the currently needed joint torques to hold the arm in its current configuration.
Therefore a properly configured URDF is needed.

It then commands a small position "kick" into the same direction of the corresponding torque.
This unloads the brakes and helps releasing them.

### Mode: "manual"

In this mode the controller simply performs the unlock lifecycle by:

"excite brakes" -> give user "timeout" seconds to move joints -> "put drives into hold"

## Interaction

### Mode: "auto"

None, only activate, deactivate

### Mode: "manual"

Activate, manually move all joint around to unlock all the brakes, deactivate

## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `joints` | List of joint names as described in the URDF the controller shall handle | - | may not be empty |
| `position_kick | Distance each actuator shall move against gravity in [rad]. | 0.005 | [0.0, 0.2] |
| `mode` | Select as an alternative manual mode | "auto" | "auto" or "manual" |
| `timeout`| Time in manual mode until brakes are put into hold mode | 50.0 | >= 0.0 |

## Notes

This controller needs a correctly configured URDF including masses
