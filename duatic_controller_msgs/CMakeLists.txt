cmake_minimum_required(VERSION 3.22)
project(duatic_controller_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- Dependencies ---------------------------------------------------------
# Message packages used in the definitions                     (package.xml: <depend>)
set(MSG_DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(dep IN LISTS MSG_DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()

# ---- Targets --------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveState.msg"
  "msg/DriveStateCollection.msg"
  "msg/MeasuredTorques.msg"
  "msg/PIDGains.msg"
  "msg/GravityCompensationControllerStatus.msg"
  "msg/PoseTarget.msg"
  "msg/MultiPoseTarget.msg"
  DEPENDENCIES ${MSG_DEPENDENCIES}
)

# ---- Install & export -----------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)

ament_package()
