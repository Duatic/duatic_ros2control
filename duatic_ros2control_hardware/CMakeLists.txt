cmake_minimum_required(VERSION 3.8)
project(duatic_ros2control_hardware)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(yaml-cpp REQUIRED) # We need to use yaml-cpp from the system otherwise the rsl_drive_sdk will be unhappy

set(PACKAGE_DEPENDENCIES
  rclcpp
  pluginlib
  hardware_interface
  urdf
  ethercat_sdk_master
  rsl_drive_sdk
)

foreach(Dependency IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(duatic_ros2control_hardware src/duadrive_interface.cpp
  src/duadrive_interface_base.cpp
  src/duadrive_interface_mock.cpp
)
add_library(duatic_ros2control_hardware::duatic_ros2control_hardware ALIAS duatic_ros2control_hardware)
target_compile_features(duatic_ros2control_hardware PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_include_directories(duatic_ros2control_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(duatic_ros2control_hardware ${PACKAGE_DEPENDENCIES})
target_link_libraries(duatic_ros2control_hardware ${YAML_CPP_LIBRARIES})
# enable additional warnings
target_compile_options(duatic_ros2control_hardware PRIVATE
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

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(duatic_ros2control_hardware PRIVATE "DUATIC_ROS2CONTROL_HARDWARE_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS duatic_ros2control_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  duatic_ros2control_hardware
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(${PACKAGE_DEPENDENCIES} yaml-cpp)

ament_package()
