cmake_minimum_required(VERSION 3.8)
project(guide_interfaces)

# Tools we need: the base build system, standard messages, and the message code generator
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # PlanRoute's response and SetRoute's request contain a nav_msgs/Path

# Generate Python and C++ code from our message, service and action definitions.
# For .action files the generator automatically adds the action_msgs types it needs
# (goal ids, status); package.xml declares the dependency so it is installed and built first.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RouteProgress.msg"
  "msg/NavGoalStatus.msg"
  "msg/RouteCrossing.msg"
  "msg/MissionState.msg"
  "msg/DestinationCandidate.msg"
  "msg/LocalizationStatus.msg"
  "srv/PlanRoute.srv"
  "srv/SetRoute.srv"
  "srv/ResolveDestination.srv"
  "srv/Geocode.srv"
  "action/NavigateTo.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# Make sure anything using our messages also gets the runtime support
ament_export_dependencies(rosidl_default_runtime)

ament_package()
