cmake_minimum_required(VERSION 3.16)
project(duatic_duadrive_scanner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED) # We need to use yaml-cpp from the system otherwise the rsl_drive_sdk will be unhappy

# add external dependencies
include(./cmake/CPM.cmake)

# Add cxxopts dependency via CPM
cpmaddpackage(
  GITHUB_REPOSITORY jarro2783/cxxopts
  VERSION 3.3.1
  OPTIONS "CXXOPTS_BUILD_EXAMPLES NO" "CXXOPTS_BUILD_TESTS NO" "CXXOPTS_ENABLE_INSTALL YES"
)



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


add_executable(duadrive_scanner src/duadrive_scanner.cpp
src/Scanner.cpp)
target_include_directories(duadrive_scanner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(duadrive_scanner PUBLIC c_std_99 cxx_std_20)  # Require C99 and C++20
ament_target_dependencies(duadrive_scanner ${PACKAGE_DEPENDENCIES})
target_link_libraries(duadrive_scanner ${YAML_CPP_LIBRARIES} cxxopts)

install(TARGETS duadrive_scanner
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)

endif()

ament_package()
