# CartesianPoseController

A controller which drives a Cartesian pose of a `target_frame` toward a target pose given relative to a specified `base_frame`.

__NOTE:__ This controller requires a `controller_manager` `update_rate` of at least 500Hz.

## Usage

1. Create an entry in your controllers.yaml and add it to your launch file infrastructure. Set `base_frame` and `target_frame` accordingly. Do not activate on startup.
2. Activate the controller -> It claims the position (and, if `command_velocities` is true, also velocity) interfaces of all joints between `base_frame` and `target_frame`.
3. Publish a target pose -> The end effector should now move towards it, respecting the configured cartesian velocity/acceleration limits.

### Configuration example

```
controller_manager:
  ros__parameters:
    cartesian_pose_controller:
      type: duatic_controllers/CartesianPoseController
      state: inactive

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

Publish target poses relative to `base_frame` on `<topic_prefix>/<target_frame>/<target_topic_suffix>` (`geometry_msgs/msg/PoseStamped`).
Targets with a `frame_id` other than empty or `base_frame`, or with a non-unit quaternion, are ignored.
On activation, the target is set to the current pose, so the robot holds still until the first target arrives.

If `topic_pub_frequency` is greater than zero, the current pose and twist of `target_frame` relative to `base_frame` are published on `<topic_prefix>/<target_frame>/pose` and `<topic_prefix>/<target_frame>/twist`.
The twist is taken at the `target_frame` origin, expressed in `base_frame` axes.

`topic_prefix` defaults to the controller name if left empty. Topic names are relative, i.e. resolved within the controller's namespace, unless `topic_prefix` starts with `/`.

## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `base_frame` | Name of the base frame | `base_link` | - |
| `target_frame` | Name of the target frame to be controlled | - | required for configuration, see [Notes](#notes) |
| `target_filter` | Exponential target filter time constant [s] | 0.1 | [0.0, 10.0] |
| `command_velocities` | If true, also claims and commands the joints' velocity interface | true | - |
| `limits.velocity.linear` | Maximum linear velocity [m/s]. Zero disables linear motion | 1.0 | [0.0, 10.0] |
| `limits.velocity.angular` | Maximum angular velocity [rad/s]. Zero disables angular motion | 6.283 | [0.0, 62.83] |
| `limits.acceleration.linear` | Maximum linear acceleration [m/s^2]. Zero disables linear motion | 2.0 | [0.0, 100.0] |
| `limits.acceleration.angular` | Maximum angular acceleration [rad/s^2]. Zero disables angular motion | 12.566 | [0.0, 628.3] |
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
| `motion_horizon` | Time horizon [s] scaling the IK solver's joint displacement variable. Also bounds how far the internal joint state may run ahead of the measured one: velocity limit * `motion_horizon` per joint | 0.01 | [0.001, 1.0] |
| `ik_damping` | Weight to dampen all joint motions and stabilize the IK solution | 0.01 | [0.000001, 1.0] |
| `ik_meter_to_revolution_error_correlation` | Correlation between linear (one meter) and angular (one revolution) errors in the IK solver | 25.0 | [1.0, 100.0] |
| `ik_precision` | Precision of the inverse kinematics solver | 0.0001 | [0.000001, 0.1] |
| `ik_max_iterations` | Maximum number of iterations for the inverse kinematics solver | 10 | [3, 30] |

## Notes

* `target_frame` has no default. The controller can be loaded without it, but configuration fails if it is empty or not a frame of the robot model; the same holds for `base_frame`.
* Every joint not on the direct path between `base_frame` and `target_frame` is treated as fixed.
* A correctly configured URDF is needed (correct frame names and joint types).
* Only 1-DOF joints (`revolute`, `prismatic`) can be controlled; configuration fails for any other movable joint on the path, including `continuous` joints.

## References

* [Pinocchio library](https://github.com/stack-of-tasks/pinocchio)
* [ProxSuite library](https://github.com/Simple-Robotics/proxsuite)
