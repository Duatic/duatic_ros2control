# BrakeReleaseController

Use this controller to deactivate the brakes at startup on any Duatic DuaDrive equipped with brakes.


## Usage

1. Add entry in controllers.yaml and in your launch infrastructure. Do not directly activate it during startup
2. Make sure to deactivate the `FreezeController`
3. Activate the `BrakeReleaseController`
4. If it was for some reason not successful -> Retry (deactivate -> activate)
5. Deactivate the controller to release the `position/velocity` interfaces


## How does it work ?

The controller calculates the currently needed joint torques to hold the arm in its current configuration.
Therefore a properly configured URDF is needed.

It then commands a small position "kick" into the same direction of the corresponding torque.
This unloads the brakes and helps releasing them.

## Interaction

None, only activate, deactivate

## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `joints` | List of joint names as described in the URDF the controller shall handle | - | may not be empty |
| `position_kick | Distance each actuator shall move against gravity in [rad]. | 0.005 | [0.0, 0.2] |


## Notes

This controller needs a correctly configured URDF including masses
