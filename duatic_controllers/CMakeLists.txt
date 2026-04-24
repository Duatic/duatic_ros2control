cmake_minimum_required(VERSION 3.22)
project(duatic_controllers)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(duatic_controller_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(pinocchio REQUIRED)
# Generate code from parameter description yaml files
generate_parameter_library(
    safety_scaling_controller_parameters
    src/safety_scaling_controller_parameters.yaml
)
generate_parameter_library(
    duadrive_status_broadcaster_parameters
    src/duadrive_status_broadcaster_parameters.yaml
)
generate_parameter_library(
    force_torque_broadcaster_parameters
    src/force_torque_broadcaster_parameters.yaml
)
generate_parameter_library(
    freedrive_controller_parameters
    src/freedrive_controller_parameters.yaml
)
generate_parameter_library(
    duadrive_pid_tuner_parameters
    src/duadrive_pid_tuner_parameters.yaml
)
generate_parameter_library(
    gravity_compensation_controller_parameters
    src/gravity_compensation_controller_parameters.yaml
)
generate_parameter_library(
    cartesian_pose_controller_parameters
    src/cartesian_pose_controller_parameters.yaml
)



# The actual controller library
add_library(${PROJECT_NAME} SHARED
    src/safety_scaling_controller.cpp
    src/duadrive_status_broadcaster.cpp
    src/force_torque_broadcaster.cpp
    src/freedrive_controller.cpp
    src/duadrive_pid_tuner.cpp
    src/gravity_compensation_controller.cpp
    src/cartesian_pose_controller.cpp
)
# Require C99 and C++20
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_20)
# enable additional warnings
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  -Wshadow
  -Wnon-virtual-dtor
  -Wconversion
  -Wundef
  -Wformat=2
  -Wsign-conversion
  -Wnull-dereference
  -Werror=return-type
)

target_include_directories(${PROJECT_NAME} PRIVATE
    include # Make them private for the moment as it is unlikely that someone wants to use them downstream
)
# Link against the controller parameter targets
target_link_libraries(${PROJECT_NAME}
    safety_scaling_controller_parameters
    duadrive_status_broadcaster_parameters
    force_torque_broadcaster_parameters
    freedrive_controller_parameters
    duadrive_pid_tuner_parameters
    gravity_compensation_controller_parameters
    pinocchio::pinocchio
)
# Use ament_target_dependencies for external deps
ament_target_dependencies(
  ${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  duatic_controller_msgs
  std_srvs
  Eigen3
  eigen3_cmake_module
  pinocchio
)
# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_interface_plugin.xml)

# Install the resulting library
install(
  TARGETS  ${PROJECT_NAME}
  DESTINATION lib/
)


ament_package()
