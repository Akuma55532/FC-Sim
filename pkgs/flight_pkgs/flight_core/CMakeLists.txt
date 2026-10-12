cmake_minimum_required(VERSION 3.8)
project(flight_core)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(flight_interfaces REQUIRED)
# Protocol-independent action and mission logic.
add_library(flight_logic
  src/core/basic_action_machine.cpp
  src/core/waypoint_mission.cpp)
target_include_directories(flight_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Composition roots explicitly select the MAVROS implementation.
add_executable(flight_actions_node
  src/nodes/flight_actions_node.cpp
  src/backends/mavros/mavros_adapter.cpp
  src/backends/mavros/mavros_backend.cpp)
target_link_libraries(flight_actions_node flight_logic)
ament_target_dependencies(flight_actions_node rclcpp rclcpp_action geometry_msgs std_srvs mavros_msgs flight_interfaces)
install(TARGETS flight_logic flight_actions_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_basic_actions test/test_basic_actions.cpp)
  target_link_libraries(test_basic_actions flight_logic)
  ament_add_gtest(test_waypoint_mission test/test_waypoint_mission.cpp)
  target_link_libraries(test_waypoint_mission flight_logic)
endif()
ament_package()
