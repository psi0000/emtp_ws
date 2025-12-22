cmake_minimum_required(VERSION 3.10)
project(psi_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/RobotPath.msg"
  "msg/SceneGraph.msg"
  "msg/Cancel.msg"
  "msg/FleetState.msg"
  "msg/Location.msg"
  "msg/RobotState.msg"
  "msg/TaskRequest.msg"
  "msg/TaskComplete.msg"
)
set(srv_files
  "srv/AllocationResult.srv"
  "srv/DynamicEvent.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
