# BrakeReleaseController

Use this controller to deactivate the brakes at startup on any Duatic DuaDrive equipped with brakes.

__Why is this controller needed?__\
The DuaDrives use pin brakes. If the full weight of an arm is loaded onto a pin brake it is possible that the pin cannot be pulled.
For this case the pin needs to be unloaded.


## Usage

1. Add entry in controllers.yaml and in your launch infrastructure. Do not directly activate it during startup
2. Make sure to deactivate the `FreezeController`
3. Activate the `BrakeReleaseController`

__NOTE:__ Both modes assume the [GravityCompensationController](./GravityCompensationController.md) is up and running.
The "auto" mode chains onto it: it claims its `effort` reference interfaces, so it cannot be activated without it.
The "manual" mode only claims `target_brake_state` and reads `brake_released`, so it leaves every joint interface to other controllers.


## How does it work ?

### Mode: "auto"

The controller excites the brakes and sweeps a torque offset per drive, on top of the gravity compensation. To load the pins as little as possible, the offset sweeps at `torque_rate` through a quarter of `max_drive_torque` in the positive direction, then the negative one, then half and finally the full `max_drive_torque` in both directions, and back to 0. A torque level is considered as finished when the drive no longer moves in the pushed direction faster than `stop_velocity`. As soon as a joint reports `brake_released`, its offset is removed and its brake is put into hold. Once no joint is sweeping anymore, the remaining brakes are put into hold too. If every drive reports `brake_released`, the controller deactivates itself. Otherwise it logs the joints that are not released.

### Mode: "manual"

In this mode the controller simply performs the unlock lifecycle by:

"excite brakes" -> user moves the joints until every drive reports `brake_released` -> deactivate itself

The brake of each drive is put into hold as soon as that drive reports `brake_released`.
While waiting, the controller logs every 3 seconds which drives are not released yet.

## Checking the release

The hardware interface infers the brake release from motion. The pin sits on the motor side of the gearbox, so while it is engaged the motor can only move within the pin clearance. Once the motor travels further than `brake_clearance`, the `brake_released` state interface becomes 1.0.

## Interaction

### Mode: "auto"

Activate, the controller deactivates itself once all brakes are released

### Mode: "manual"

Activate, manually move all joints around until the controller reports all brakes released and deactivates itself

## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `joints` | List of joint names as described in the URDF the controller shall handle | - | may not be empty |
| `mode` | Select as an alternative manual mode | "auto" | "auto" or "manual" |
| `torque_rate` | Rate in mode "auto" in [Nm/s] at which the torque offset changes | 20.0 | > 0.0 |
| `max_drive_torque` | Largest torque offset in mode "auto" in [Nm] per drive, as drive torque: at the output of the drive, after its gearbox and before any coupling or transmission outside the drive. Tried as a quarter, a half and the full value in both directions | 12.0 | > 0.0 |
| `stop_velocity` | Drive speed in mode "auto" in [rad/s], in drive coordinates, below which a drive counts as stopped in the pushed direction, so the sweep moves on to its next level. A coupling with gearing outside the drives scales it against the joint speed | 0.02 | > 0.0 |
| `gravity_compensation_controller_name` | Name of the GravityCompensationController whose effort references carry the offset in mode "auto" | "gravity_compensation_controller" | - |

## Notes

This controller needs a correctly configured URDF including masses.
