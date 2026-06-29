# GravityCompensationController

A passive controller which calculates the necessary torques to keep the arm in the current state (includes dynamics) and commands them to the effort interfaces

## Usage

1. Create an entry in your controllers.yaml and add it to your launch file infrastructure.
You may activate this controller on startup
2. Activate the controller -> If no other controller is loaded and the brakes are released you should now be able the freely move around the controller

This controller can be run in parallel to all other controllers and calculates the necessary torque feed-forward value.

### Configuration example

```
gravity_compensation_controller:
  ros__parameters:
    joints:
      - shoulder_rotation
      - shoulder_flexion
      - elbow_flexion
      - forearm_rotation
      - wrist_flexion
      - wrist_rotation
```

### Interaction

None, only activate, deactivate


## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `joints`| List of joints used by the controller | - | May not be empty |
| `enable_state_topic` | Publish a status message with the currently commanded torques | true | - |
| `enable_startup_check`| Perform a check in the first half second to prevent large changes in joint position | true | - |
| `max_jump_startup` | Maximum allowed jump in the half second after startup | 0.5 |

## Notes

* This controller can be used a free drive controller. The actual `FreeDriveController` only provides a dampening on the motion in order to make the handling smoother
* A correctly configured URDF is needed (masses, inertias)
    * The `startup check` may help to prevent unintended motions due to wrong URDFs
* We recommend for all other control modes (e.g. position velocity via JTC) to run this controller in parallel.

## References

* [Pinocchio library](https://github.com/stack-of-tasks/pinocchio)
