# StatusBroadcaster

A controller which publishes DuaDrive specific information.

## Usage

1. Create an entry in your controllers.yaml and add it to your launch file infrastructure.
You may activate this controller on startup
2. Activate the controller -> You should now see a `/<controller name>/state` topic

## Configuration example

```
dynaarm_status_broadcaster:
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

Subscribe to topic `/<controller name>/state` which is of type:
`duatic_controller_msgs/msg/DriveStateCollection`

For each drive the following information is published:

| Name | Description |
| ------ | ---------- |
| name | drive name |
| id | ethercat bus id |
| joint position | cubrrent joint position in rad|
| joint velocity | current joint velocity in rad|
| joint effort | current joint effort in rad |
| joint position commanded | current commanded joint position as seen by the drive |
| joint velocity commanded | current commanded joint velocity as seen by the drive |
| joint torque commanded | current commanded joint torque as seen by the drive |
| temperature system | system temperature (max of all sensors) in °C |
| temperature phase a | temperature of phase 1 in °C |
| temperature phase b | temperature of phase 2 in °C |
| temperature phase c | temperature of phase 3 in °C |
| bus voltage | measured dc voltage in V |
| current q | measured q current in A |
| current d | measured d current in A |
| current phase a | measured current of phase 1 in A |
| current phase b | measured current of phase 2 in A |
| current phase c | measured current of phase 3 in A |
| brake released | true once motor motion proved the brake pin is out, see [BrakeReleaseController](./BrakeReleaseController.md#checking-the-release) |
| drive state | drive state machine state, one of the `DRIVE_STATE_*` constants of `DriveState` |
| communication timeout | true while the latest reading from the drive is older than its communication timeout |
| error active | true once a drive error froze the hardware system this drive belongs to |
| fault latched | true once this drive was in Error or Fatal state and so froze its hardware system |
| communication timeout latched | true once this drive had a communication timeout and so froze its hardware system |

The three latched fields stay set until the hardware component is reloaded, in practice when the controller manager restarts.
fault latched and communication timeout latched only name drives that failed in the cycle that froze the system, not failures after it.

## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `joints` | List of joint names as described in the URDF the controller shall handle | - | may not be empty |

## Notes
