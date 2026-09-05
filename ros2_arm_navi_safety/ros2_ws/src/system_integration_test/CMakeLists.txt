cmake_minimum_required(VERSION 3.16)
project(system_integration_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(arm_navi_safety_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(integration_scenario_logic src/scenario_state_machine.cpp)
target_include_directories(integration_scenario_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(integration_scenario_runner src/scenario_runner.cpp)
target_include_directories(integration_scenario_runner PRIVATE include)
target_link_libraries(integration_scenario_runner integration_scenario_logic)
ament_target_dependencies(integration_scenario_runner
  arm_navi_safety_interfaces geometry_msgs nav_msgs rclcpp ros_gz_interfaces sensor_msgs std_msgs std_srvs)

add_executable(integration_test_navigator src/test_navigate_to_pose_server.cpp)
ament_target_dependencies(integration_test_navigator
  geometry_msgs nav2_msgs nav_msgs rclcpp rclcpp_action)

install(TARGETS integration_scenario_logic integration_scenario_runner integration_test_navigator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch models DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_integration_scenario.sh DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scenario_state_machine test/test_scenario_state_machine.cpp)
  if(TARGET test_scenario_state_machine)
    target_link_libraries(test_scenario_state_machine integration_scenario_logic)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_scenario_runner_node.py TIMEOUT 30)
endif()

ament_export_include_directories(include)
ament_export_libraries(integration_scenario_logic)
ament_package()
