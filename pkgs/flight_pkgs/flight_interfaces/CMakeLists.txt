cmake_minimum_required(VERSION 3.8)
project(flight_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlStatus.msg" "msg/ActionResult.msg" "msg/ActionFeedback.msg" "msg/Waypoint.msg"
  "action/Arm.action" "action/Disarm.action" "action/Takeoff.action"
  "action/Land.action" "action/ExecuteWaypoints.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
