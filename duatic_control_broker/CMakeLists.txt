cmake_minimum_required(VERSION 3.22)
project(duatic_control_broker)

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(duatic_controller_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME}_core STATIC src/broker.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME}_core PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

add_executable(control_broker src/control_broker_node.cpp)
target_compile_options(control_broker PRIVATE -Wall -Wextra -Wpedantic -Wshadow)
target_link_libraries(control_broker
  ${PROJECT_NAME}_core
  rclcpp::rclcpp
  ${controller_manager_msgs_TARGETS}
  ${duatic_controller_msgs_TARGETS}
)

install(TARGETS control_broker DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_broker test/test_broker.cpp)
  target_link_libraries(test_broker ${PROJECT_NAME}_core)
endif()

ament_package()
