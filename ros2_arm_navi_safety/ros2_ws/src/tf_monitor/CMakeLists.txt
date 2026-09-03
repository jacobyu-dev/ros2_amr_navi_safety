cmake_minimum_required(VERSION 3.16)
project(tf_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(arm_navi_safety_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(tf_health_checker src/tf_health_checker.cpp)
target_include_directories(tf_health_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(tf_monitor_node src/tf_monitor_node.cpp)
target_link_libraries(tf_monitor_node tf_health_checker)
ament_target_dependencies(tf_monitor_node
  arm_navi_safety_interfaces diagnostic_msgs geometry_msgs rclcpp std_msgs tf2 tf2_msgs tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tf_health_checker test/test_tf_health_checker.cpp)
  target_link_libraries(test_tf_health_checker tf_health_checker)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_tf_monitor_node.py TIMEOUT 60)
endif()

install(TARGETS tf_health_checker tf_monitor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/publish_tf_sequence.py
  scripts/record_synthetic_tf_bag.sh
  DESTINATION lib/${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_libraries(tf_health_checker)
ament_package()
