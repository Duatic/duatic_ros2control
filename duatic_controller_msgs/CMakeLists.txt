cmake_minimum_required(VERSION 3.22)
project(duatic_controller_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveState.msg"
  "msg/DriveStateCollection.msg"
  "msg/MeasuredTorques.msg"
  "msg/PIDGains.msg"
  "msg/GravityCompensationControllerStatus.msg"
  "msg/PoseTarget.msg"
  "msg/MultiPoseTarget.msg"
  "msg/ControlGroupState.msg"
  "msg/ControlHeartbeat.msg"
  "msg/ControlState.msg"
  "srv/ClaimControl.srv"
  "srv/ReleaseControl.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
