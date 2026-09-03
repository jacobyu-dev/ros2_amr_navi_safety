cmake_minimum_required(VERSION 3.16)
project(lidar_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(arm_navi_safety_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(lidar_safety_core src/lidar_safety_core.cpp)
target_include_directories(lidar_safety_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(lidar_safety_node src/lidar_safety_node.cpp)
target_link_libraries(lidar_safety_node lidar_safety_core)
ament_target_dependencies(lidar_safety_node arm_navi_safety_interfaces rclcpp sensor_msgs std_msgs)

install(TARGETS lidar_safety_core lidar_safety_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/publish_test_scan.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lidar_safety_core test/test_lidar_safety_core.cpp)
  target_link_libraries(test_lidar_safety_core lidar_safety_core)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_lidar_safety_node.py TIMEOUT 60)
endif()

ament_export_include_directories(include)
ament_export_libraries(lidar_safety_core)
ament_package()
