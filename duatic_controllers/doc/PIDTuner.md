# PIDTuner

A controller which allows to change the PID gains of the PositionVelocityTorque control mode within the DuaDrive actuator.

__NOTE:__ This controller is only intended to be used for testing and setup purposes. There is a high risk of braking the hardware if used wrong. Use at your own risk.


## Usage

1. Create an entry in your controllers.yaml and add it to your launch file infrastructure.
You may activate this controller on startup
2. Activate the controller -> You can now access the pid gains via "ros2 param"
3. Activate the GravityCompensationController and the JTC in order to put the actuators in the necessary control mode
4. Change the pid gains - use the JTC to drive sample trajectories

### Configuration example


```
pid_tuner:
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

See "Usage". After activation the `/pid_tuner` node provides "p_gain, i_gain, d_gain" parameters for each actuator:


```
ros2 param list /pid_tuner
  elbow_flexion/d_gain
  elbow_flexion/i_gain
  elbow_flexion/p_gain
  forearm_rotation/d_gain
  forearm_rotation/i_gain
  forearm_rotation/p_gain
  is_async
  joints
  shoulder_flexion/d_gain
  shoulder_flexion/i_gain
  shoulder_flexion/p_gain
  shoulder_rotation/d_gain
  shoulder_rotation/i_gain
  shoulder_rotation/p_gain
  start_type_description_service
  thread_priority
  update_rate
  use_sim_time
  wrist_flexion/d_gain
  wrist_flexion/i_gain
  wrist_flexion/p_gain
  wrist_rotation/d_gain
  wrist_rotation/i_gain
  wrist_rotation/p_gain
```

You can read the current gains and write them via `ros2 param`.

## Parameters

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `joints` | List of joint names as described in the URDF the controller shall handle | - | may not be empty |

## Notes

* This directly changes the PID gains used within the actuator.
* The gains are not stored permanently
* If choosing too low gains -> The arm may drop, if choosing too high gain -> The arm may oscillate
