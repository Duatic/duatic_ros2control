cmake_minimum_required(VERSION 3.22)
project(duatic_duadrive_interface)

# ---- Dependencies ---------------------------------------------------------
# Targets used in public headers: linked PUBLIC, exported      (package.xml: <depend>)
set(PUBLIC_DEPENDENCY_TARGETS
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  urdf::urdf
  ament_index_cpp::ament_index_cpp
  Eigen3::Eigen
  rsl_drive_sdk::rsl_drive_sdk
  ecat_sdk_master::ecat_sdk_master
)

# Package name = target namespace; list exceptions via find_package() separately
list(TRANSFORM PUBLIC_DEPENDENCY_TARGETS REPLACE "::.*$" "" OUTPUT_VARIABLE PUBLIC_DEPENDENCIES)
list(REMOVE_DUPLICATES PUBLIC_DEPENDENCIES)

find_package(ament_cmake REQUIRED)
foreach(dep IN LISTS PUBLIC_DEPENDENCIES)
  find_package(${dep} REQUIRED)
endforeach()

# ---- Targets --------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/duadrive_interface.cpp
  src/duadrive_interface_base.cpp
  src/duadrive_interface_mock.cpp
  src/coupled_kinematics_hardware_interface.cpp
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
    ${PUBLIC_DEPENDENCY_TARGETS}
)

# ---- Tests ----------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coupled_kinematics_position_limiter
    test/test_coupled_kinematics_position_limiter.cpp
  )
  target_link_libraries(test_coupled_kinematics_position_limiter
    ${PROJECT_NAME}
  )
  ament_add_gtest(test_travel_range
    test/test_travel_range.cpp
  )
  target_link_libraries(test_travel_range
    duatic_duadrive_interface
  )
endif()

# ---- Install & export -----------------------------------------------------
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PUBLIC_DEPENDENCIES})

ament_package()
