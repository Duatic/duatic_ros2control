# ForceTorqueBroadcaster

A virtual FTS sensor which simulates a real FTS sensor based on the actuator torque measurements.

## Usage

1. Create an entry in your controllers.yaml and add it to your launch file infrastructure.
You may activate this controller on startup
2. Activate the controller -> You shall now see a `/<controller_name>/wrench` and `/<controller_name>/torques` topic

### Configuration example

```
virtual_fts:
  ros__parameters:
    joints:
      - shoulder_rotation
      - shoulder_flexion
      - elbow_flexion
      - forearm_rotation
      - wrist_flexion
      - wrist_rotation
    endeffector_frame: "flange"
    use_svd_solver: true
```


### Interaction

Activate the controller an subscribe to the following topics:

*  `/<controller_name>/wrench`   - Reading a FTS sensor would produce at the corresponding frame location
*  `/<controller_name>/torques`  - Torques without the gravity component (external force impact on the arm)

### Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `joints`| List of joints used by the controller | - | May not be empty |
| `endeffector_frame` | Name of the end effector frame in the URDF | /flange | - |
| `lambda_regularization` | Regularization parameter for DLS solver approach | 0.0001 | - |
| `use_svd_solve` | Use SVD solver instead of DLS solver | false | - |

## Notes

* The quality of the reading vastly depends on the accuracy of the torque readings and especially of the URDF
* During our tests the virtual FTS readings proved to be way less drifting around compared to direct measurement approaches
