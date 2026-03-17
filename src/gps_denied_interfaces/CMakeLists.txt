cmake_minimum_required(VERSION 3.8)
project(gps_denied_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/SystemStatus.msg"
  "msg/NavigationState.msg"
  "msg/ObstacleMap.msg"
  "msg/WaypointList.msg"
  "msg/Waypoint.msg"
  "msg/FailsafeCommand.msg"
  "msg/SubsystemHealth.msg"
  # Services
  "srv/SetMission.srv"
  "srv/GetSystemHealth.srv"
  "srv/EmergencyStop.srv"
  # Actions
  "action/NavigateToWaypoint.action"
  "action/ExecuteMission.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
