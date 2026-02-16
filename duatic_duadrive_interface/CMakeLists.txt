cmake_minimum_required(VERSION 3.8)
project(duatic_duadrive_interface)

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


add_library(duatic_duadrive_interface
  src/duadrive_interface.cpp
  src/duadrive_interface_base.cpp
  src/duadrive_interface_mock.cpp
  src/coupled_kinematics_hardware_interface.cpp
  src/coupled_kinematics_invariant_translator.cpp
)
add_library(duatic_duadrive_interface::duatic_duadrive_interface ALIAS duatic_duadrive_interface)
target_compile_features(duatic_duadrive_interface PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
target_include_directories(duatic_duadrive_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(duatic_duadrive_interface ${PACKAGE_DEPENDENCIES})
target_link_libraries(duatic_duadrive_interface ${YAML_CPP_LIBRARIES})
# enable additional warnings
target_compile_options(duatic_duadrive_interface PRIVATE
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
target_compile_definitions(duatic_duadrive_interface PRIVATE "duatic_duadrive_interface_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS duatic_duadrive_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  duatic_duadrive_interface
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(${PACKAGE_DEPENDENCIES} yaml-cpp)

ament_package()
