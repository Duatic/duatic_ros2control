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
find_package(duatic_geometry REQUIRED)
find_package(duatic_geometry_encoder REQUIRED)
find_package(duatic_data_encoding REQUIRED)
find_package(duatic_trajectory REQUIRED)
find_package(duatic_concurrency REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(pinocchio REQUIRED)
find_package(proxsuite REQUIRED)
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
generate_parameter_library(
    freeze_controller_parameters
    src/freeze_controller_parameters.yaml
)
generate_parameter_library(
    brake_release_controller_parameters
    src/brake_release_controller_parameters.yaml
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
    src/freeze_controller.cpp
    src/brake_release_controller.cpp
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
# Pinocchio's own exported target (pinocchioTargets.cmake) declares the Boost MPL/Fusion
# compile definitions it needs (BOOST_MPL_LIMIT_LIST_SIZE, BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS,
# etc.) as INTERFACE_COMPILE_DEFINITIONS on pinocchio::pinocchio. There is no legacy
# PINOCCHIO_CFLAGS_OTHER variable exposing these (pinocchio's CMake config only sets
# PINOCCHIO_USE_* feature flags), and the ament_target_dependencies() legacy path used below
# doesn't propagate a modern target's INTERFACE_COMPILE_DEFINITIONS -- so link the real
# imported target directly to pick them up regardless of which branch below is taken.
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# Link against the controller parameter targets
target_link_libraries(${PROJECT_NAME}
    safety_scaling_controller_parameters
    duadrive_status_broadcaster_parameters
    force_torque_broadcaster_parameters
    freedrive_controller_parameters
    duadrive_pid_tuner_parameters
    gravity_compensation_controller_parameters
    freeze_controller_parameters
    brake_release_controller_parameters
    cartesian_pose_controller_parameters
)

# Support ros distros older than lyrical
if(COMMAND ament_target_dependencies)
    ament_target_dependencies(${PROJECT_NAME}
      controller_interface
      pluginlib
      rclcpp
      rclcpp_lifecycle
      realtime_tools
      duatic_controller_msgs
      duatic_geometry
      duatic_geometry_encoder
      duatic_data_encoding
      duatic_trajectory
      duatic_concurrency
      std_srvs
      Eigen3
      pinocchio
      proxsuite
    )
else()
    target_link_libraries(${PROJECT_NAME}
      controller_interface::controller_interface
      pluginlib::pluginlib
      rclcpp::rclcpp
      rclcpp_lifecycle::rclcpp_lifecycle
      realtime_tools::realtime_tools
      duatic_controller_msgs::duatic_controller_msgs
      duatic_geometry::duatic_geometry
      duatic_geometry_encoder::duatic_geometry_encoder
      duatic_data_encoding::duatic_data_encoding
      duatic_trajectory::duatic_trajectory
      duatic_concurrency::duatic_concurrency
      std_srvs::std_srvs
      Eigen3::Eigen
      pinocchio::pinocchio
      proxsuite::proxsuite
    )
endif()

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface controller_interface_plugin.xml)

# Install the resulting library
install(
  TARGETS
    ${PROJECT_NAME}
    safety_scaling_controller_parameters
    duadrive_status_broadcaster_parameters
    force_torque_broadcaster_parameters
    freedrive_controller_parameters
    duadrive_pid_tuner_parameters
    gravity_compensation_controller_parameters
    cartesian_pose_controller_parameters
    freeze_controller_parameters
    brake_release_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
# Export this as a usable library
ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  duatic_controllers
)
ament_export_targets(
  export_${PROJECT_NAME}
)


ament_package()
