cmake_minimum_required(VERSION 3.16)
project(nav2_msgs)

# This package is a source-compatible fallback for development environments
# where Nav2 is not installed.  Do not overlay it on a system that already
# provides nav2_msgs; use the system package there instead.
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
