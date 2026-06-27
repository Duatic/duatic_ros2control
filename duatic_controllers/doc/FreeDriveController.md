# FreeDriveController

A controller which enhances the "free drive feeling" already achieved by the [GravityCompensationController](./GravityCompensationController.md)

## Usage

1. Create an entry in your controllers.yaml and add it to your launch file infrastructure.
You shall not activate this controller on startup
2. Unfreeze the arm and activate the controller in addition to the `GravityCompensationController`.
3. Free drive motion should be more pleasant now

### Configuration example

```
freedrive_controller:
  ros__parameters:
    joints:
      - shoulder_rotation
      - shoulder_flexion
      - elbow_flexion
      - forearm_rotation
      - wrist_flexion
      - wrist_rotation
    d_gain: 0.005
```

## Interaction

None, only activate, deactivate

## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `joints` | List of joint names as described in the URDF the controller shall handle | - | may not be empty |
| `d_gain` | A stronger d-gain dampens the motion more | 0.001 | - |

## Notes

* This controller used to be necessary because the interface selection was handled differently - now it is only there to dampen the motion which can improve the free drive feeling
