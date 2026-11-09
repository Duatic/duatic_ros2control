cmake_minimum_required(VERSION 3.22)
project(duatic_controllers)

# ---- Dependencies ---------------------------------------------------------
# Targets used in public headers: linked PUBLIC, exported      (package.xml: <depend>)
set(PUBLIC_DEPENDENCY_TARGETS
  controller_interface::controller_interface
  controller_manager_msgs::controller_manager_msgs
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  duatic_controller_msgs::duatic_controller_msgs
  geometry_msgs::geometry_msgs
  std_srvs::std_srvs
  trajectory_msgs::trajectory_msgs
  Eigen3::Eigen
  pinocchio::pinocchio
  proxsuite::proxsuite
)
# Targets used only in sources: linked PRIVATE, not exported   (package.xml: <build_depend> + <exec_depend>)
set(PRIVATE_DEPENDENCY_TARGETS
  pluginlib::pluginlib
  lifecycle_msgs::lifecycle_msgs
  fmt::fmt
)
# Build tools: found, never linked                              (package.xml: <buildtool_depend>)
# generate_parameter_library is a <depend>: its generated targets are linked PUBLIC
set(BUILD_DEPENDENCIES
  generate_parameter_library
)

# Package name = target namespace; list exceptions via find_package() separately
foreach(scope PUBLIC PRIVATE)
  list(TRANSFORM ${scope}_DEPENDENCY_TARGETS REPLACE "::.*$" "" OUTPUT_VARIABLE ${scope}_DEPENDENCIES)
  list(REMOVE_DUPLICATES ${scope}_DEPENDENCIES)
endforeach()

find_package(ament_cmake REQUIRED)
foreach(dep IN LISTS BUILD_DEPENDENCIES PUBLIC_DEPENDENCIES PRIVATE_DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()

# ---- Targets --------------------------------------------------------------
# Generate code from parameter description yaml files
set(PARAMETER_TARGETS
  duadrive_status_broadcaster_parameters
  force_torque_broadcaster_parameters
  freedrive_controller_parameters
  duadrive_pid_tuner_parameters
  gravity_compensation_controller_parameters
  cartesian_pose_controller_parameters
  freeze_controller_parameters
  brake_release_controller_parameters
)
foreach(params IN LISTS PARAMETER_TARGETS)
  generate_parameter_library(${params} src/${params}.yaml)
endforeach()

# The actual controller library
add_library(${PROJECT_NAME} SHARED
  src/duadrive_status_broadcaster.cpp
  src/force_torque_broadcaster.cpp
  src/freedrive_controller.cpp
  src/duadrive_pid_tuner.cpp
  src/gravity_compensation_controller.cpp
  src/cartesian_pose_controller.cpp
  src/freeze_controller.cpp
  src/brake_release_controller.cpp
)

# Require C++20
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
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

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PARAMETER_TARGETS}
    ${PUBLIC_DEPENDENCY_TARGETS}
  PRIVATE
    ${PRIVATE_DEPENDENCY_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface controller_interface_plugin.xml)

# ---- Tests ----------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  ament_add_gtest(test_force_torque_broadcaster
    test/test_force_torque_broadcaster.cpp
  )
  target_link_libraries(test_force_torque_broadcaster
    ${PROJECT_NAME}
  )

  ament_add_ros_isolated_gtest(test_controller_activation
    test/test_controller_activation.cpp
  )
  target_link_libraries(test_controller_activation
    ${PROJECT_NAME}
    pluginlib::pluginlib
    lifecycle_msgs::lifecycle_msgs
  )

  ament_add_ros_isolated_gtest(test_brake_release_controller
    test/test_brake_release_controller.cpp
  )
  target_link_libraries(test_brake_release_controller
    ${PROJECT_NAME}
    lifecycle_msgs::lifecycle_msgs
  )
endif()

# ---- Install & export -----------------------------------------------------
install(
  TARGETS ${PROJECT_NAME} ${PARAMETER_TARGETS}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PUBLIC_DEPENDENCIES})

ament_package()
