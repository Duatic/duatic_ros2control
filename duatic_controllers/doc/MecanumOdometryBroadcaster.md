# MecanumOdometryBroadcaster

A controller which publishes the wheel odometry of a four wheel mecanum base, computed the same way as the
[mecanum_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html)
of ros2_controllers. It only reads the wheel velocity state interfaces and never commands the wheels, so it can stay
active while the `mecanum_drive_controller` is inactive, or run next to it.

## Usage

1. Create an entry in your controllers.yaml and add it to your launch file infrastructure.
You may activate this controller on startup
2. Activate the controller -> You should now see a `/<controller name>/odometry` topic

The odometry is integrated while the controller is active and keeps its pose across deactivation.

## Configuration example

```
mecanum_odometry_broadcaster:
  ros__parameters:
    front_left_wheel_joint_name: "wheel_front_left"
    front_right_wheel_joint_name: "wheel_front_right"
    rear_right_wheel_joint_name: "wheel_back_right"
    rear_left_wheel_joint_name: "wheel_back_left"

    kinematics:
      base_frame_offset: { x: 0.0, y: 0.0, theta: 0.0 }
      wheels_radius: 0.1006
      sum_of_robot_center_projection_on_X_Y_axis: 0.6029

    enable_odom_tf: false
    tf_frame_prefix_enable: false
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.000001]
    twist_covariance_diagonal: [0.01, 0.01, 0.001, 0.001, 0.001, 0.000001]
```

### Interaction

| Topic | Type | Content |
| ----- | ---- | ------- |
| `/<controller name>/odometry` | `nav_msgs/msg/Odometry` | pose in the odometry frame, twist in the base frame |
| `/<controller name>/tf_odometry` | `tf2_msgs/msg/TFMessage` | odometry frame to base frame, only with `enable_odom_tf` |

## Parameters

Apart from the wheel joint names, the parameters have the same names, defaults and meaning as those of the
`mecanum_drive_controller`.

| Name | Description | Default | Bounds |
| ---- | ----------- | ------  | ------ |
| `front_left_wheel_joint_name` | Joint whose `velocity` state interface is the front left wheel speed in rad/s | - | may not be empty |
| `front_right_wheel_joint_name` | Same for the front right wheel | - | may not be empty |
| `rear_right_wheel_joint_name` | Same for the rear right wheel | - | may not be empty |
| `rear_left_wheel_joint_name` | Same for the rear left wheel | - | may not be empty |
| `kinematics.base_frame_offset.{x,y,theta}` | Offset of the base frame from the robot center | 0.0 | - |
| `kinematics.wheels_radius` | Wheel radius in m | - | > 0 |
| `kinematics.sum_of_robot_center_projection_on_X_Y_axis` | lx + ly, the distances from the robot center to the wheels along x and y, in m | 0.0 | - |
| `tf_frame_prefix_enable` | Prefix the frame ids with `tf_frame_prefix` | true | - |
| `tf_frame_prefix` | Frame id prefix, the controller's namespace if empty | "" | - |
| `base_frame_id` | Child frame id of the odometry | base_link | - |
| `odom_frame_id` | Frame id of the odometry | odom | - |
| `enable_odom_tf` | Publish `~/tf_odometry`, can be changed at runtime | true | - |
| `pose_covariance_diagonal` | Diagonal of the pose covariance | 0.1 each | 6 values |
| `twist_covariance_diagonal` | Diagonal of the twist covariance | 0.1 each | 6 values |
