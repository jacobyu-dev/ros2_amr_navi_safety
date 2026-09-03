cmake_minimum_required(VERSION 3.16)
project(arm_navi_safety_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyStatus.msg"
  "msg/SafetyEvent.msg"
  "msg/MissionStatus.msg"
  "srv/SetFaultMode.srv"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_add_gtest(test_safety_messages test/test_safety_messages.cpp)
  if(TARGET test_safety_messages)
    ament_target_dependencies(test_safety_messages rclcpp)
    rosidl_get_typesupport_target(cpp_typesupport_target
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(test_safety_messages "${cpp_typesupport_target}")
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
