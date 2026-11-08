# duatic_control_broker

Hands out groups of joints to one owner at a time and switches their controllers to match.
Programs, behaviour trees and teleoperation claim a group in a mode instead of switching controllers
themselves, so nobody switches a controller under someone else's feet.

## Run

```bash
ros2 run duatic_control_broker control_broker --ros-args --params-file <robot>/control_broker.yaml
```

The parameter file describes one robot and belongs with that robot's bringup, in the format
below.

## Configuration

Each group names its modes and the controllers active in each. A group nobody holds is in its
`default_mode`, which the broker applies on start to every group nobody has claimed yet.

```yaml
control_broker:
  ros__parameters:
    heartbeat_timeout: 1.0      # seconds a lease lives without a heartbeat
    groups: [arm]
    arm:
      default_mode: hold
      modes: [hold, trajectory]
      hold: [freeze_controller]
      trajectory: [joint_trajectory_controller]
```

## Rules

- A claim is all or nothing. Either every group named is switched, or nothing changes.
- A group held by one owner is refused to every other owner until it is released.
- A controller that drives more than one group can only be switched by a claim that names all of them.
- A lease without a heartbeat for `heartbeat_timeout` is released, and its groups return to their
  default mode.
- The broker is not a safety function. Stopping the robot is the safety controller's job.

## Interface

| Name | Type |
|------|------|
| `/control_broker/claim` | `duatic_controller_msgs/srv/ClaimControl` |
| `/control_broker/release` | `duatic_controller_msgs/srv/ReleaseControl` |
| `/control_broker/heartbeat` | `duatic_controller_msgs/msg/ControlHeartbeat` |
| `/control_broker/state` | `duatic_controller_msgs/msg/ControlState`, latched, published on every change |

```bash
ros2 service call /control_broker/claim duatic_controller_msgs/srv/ClaimControl \
  "{owner: program, groups: [arm], modes: [trajectory]}"
ros2 topic pub -r 5 /control_broker/heartbeat duatic_controller_msgs/msg/ControlHeartbeat "{lease_id: lease-1}"
```
