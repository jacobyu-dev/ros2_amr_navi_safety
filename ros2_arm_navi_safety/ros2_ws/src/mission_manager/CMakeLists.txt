cmake_minimum_required(VERSION 3.16)
project(mission_manager)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(mission_logic src/mission.cpp src/mission_state_machine.cpp)
target_include_directories(mission_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mission_logic geometry_msgs)

add_library(mission_manager_node_lib src/mission_manager_node.cpp)
target_include_directories(mission_manager_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mission_manager_node_lib mission_logic)
ament_target_dependencies(mission_manager_node_lib
  arm_navi_safety_interfaces geometry_msgs nav2_msgs rclcpp rclcpp_action std_srvs tf2 tf2_geometry_msgs)

add_executable(mission_manager_node src/main.cpp)
target_link_libraries(mission_manager_node mission_manager_node_lib)
ament_target_dependencies(mission_manager_node rclcpp)

add_library(fake_navigate_to_pose_server_lib src/fake_navigate_to_pose_server.cpp)
target_include_directories(fake_navigate_to_pose_server_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fake_navigate_to_pose_server_lib nav2_msgs rclcpp rclcpp_action std_msgs)

add_executable(fake_navigate_to_pose_server src/fake_navigate_to_pose_main.cpp)
target_link_libraries(fake_navigate_to_pose_server fake_navigate_to_pose_server_lib)
ament_target_dependencies(fake_navigate_to_pose_server rclcpp)

install(TARGETS mission_logic mission_manager_node_lib mission_manager_node
  fake_navigate_to_pose_server_lib fake_navigate_to_pose_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mission test/test_mission.cpp)
  if(TARGET test_mission)
    target_link_libraries(test_mission mission_logic)
  endif()
  ament_add_gtest(test_mission_state_machine test/test_mission_state_machine.cpp)
  if(TARGET test_mission_state_machine)
    target_link_libraries(test_mission_state_machine mission_logic)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_mission_manager_node.py TIMEOUT 120)
  add_launch_test(test/test_safety_mission_integration.py TIMEOUT 150)
endif()

ament_export_include_directories(include)
ament_export_libraries(mission_logic mission_manager_node_lib)
ament_export_dependencies(
  arm_navi_safety_interfaces geometry_msgs nav2_msgs rclcpp rclcpp_action std_srvs std_msgs tf2 tf2_geometry_msgs)
ament_package()
