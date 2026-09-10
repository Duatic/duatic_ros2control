# CartesianPoseController

A controller which drives a Cartesian pose of a `target_frame` toward a target pose given relative to a specified `base_frame`.

__NOTE:__ This controller requires a `controller_manager` `update_rate` of at least 500Hz.

## Usage

1. Create an entry in your controllers.yaml and add it to your launch file infrastructure. Set `base_frame` and `target_frame` accordingly. Do not activate on startup.
2. Activate the controller -> It claims the position (and, if `command_velocities` is true, also velocity) interfaces of all joints between `base_frame` and `target_frame`.
3. Publish a target pose -> The end effector should now move towards it, respecting the configured cartesian velocity/acceleration limits.

### Configuration example

```
cartesian_pose_controller:
  ros__parameters:
    base_frame: "base_link"
    target_frame: "flange"
    command_velocities: true
    limits:
      velocity:
        linear: 1.0
        angular: 6.283
      acceleration:
        linear: 2.0
        angular: 12.566
```

### Interaction

Publish target poses relative to `base_frame` on `/<topic_prefix>/<target_frame>/<target_topic_suffix>` (`geometry_msgs/msg/PoseStamped`).

If `topic_pub_frequency` is greater than zero, the current target pose/twist relative to `base_frame` are published on `/<topic_prefix>/<target_frame>/pose` and `/<topic_prefix>/<target_frame>/twist`.

`topic_prefix` defaults to the controller name if left empty.

## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `base_frame` | Name of the base frame | `base_link` | - |
| `target_frame` | Name of the target frame to be controlled | - | may not be empty |
| `target_filter` | Exponential target filter time constant [s] | 0.1 | [0.0, 10.0] |
| `command_velocities` | If true, also claims and commands the joints' velocity interface | true | - |
| `limits.velocity.linear` | Maximum linear velocity [m/s] | 1.0 | [0.0, 10.0] |
| `limits.velocity.angular` | Maximum angular velocity [rad/s] | 6.283 | [0.0, 62.83] |
| `limits.acceleration.linear` | Maximum linear acceleration [m/s^2] | 2.0 | [0.0, 100.0] |
| `limits.acceleration.angular` | Maximum angular acceleration [rad/s^2] | 12.466 | [0.0, 628.3] |
| `topic_prefix` | Prefix to all topics. If empty, the controller name is used | - | - |
| `target_topic_suffix` | Suffix added to the target frame topic for receiving control inputs | `target` | - |
| `topic_pub_frequency` | Frequency of the published end effector pose/twist topics. Zero disables publishing | 10.0 | [0.0, 1000.0] |

### Debug parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `enable_introspection` | Enable ros2control introspection for internal state monitoring | false | - |
| `dry_run` | If true, the controller will not claim command interfaces | false | - |

### Expert-only parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `velocity_feedback` | Filter weight for the joint velocity feedback. 0 disables it | 0.001 | [0.0, 1.0] |
| `motion_horizon` | Time horizon [s] used to scale the IK solver's joint displacement variable | 0.01 | [0.001, 1.0] |
| `ik_damping` | Weight to dampen all joint motions and stabilize the IK solution | 0.01 | [0.000001, 1.0] |
| `ik_meter_to_revolution_error_correlation` | Correlation between linear (one meter) and angular (one revolution) errors in the IK solver | 25.0 | [1.0, 100.0] |
| `ik_precision` | Precision of the inverse kinematics solver | 0.0001 | [0.000001, 0.1] |
| `ik_max_iterations` | Maximum number of iterations for the inverse kinematics solver | 10 | [3, 30] |

## Notes

* Every joint not on the direct path between `base_frame` and `target_frame` is treated as fixed.
* A correctly configured URDF is needed (correct frame names and joint types).
* This controller only support hinge (revolute) joints to be controlled.

## References

* [Pinocchio library](https://github.com/stack-of-tasks/pinocchio)
