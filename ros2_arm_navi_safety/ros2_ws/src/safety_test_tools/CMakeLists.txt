cmake_minimum_required(VERSION 3.16)
project(safety_test_tools)

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

add_library(fake_lidar src/fake_lidar.cpp)
target_include_directories(fake_lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(fake_lidar sensor_msgs)

add_executable(fake_lidar_node src/fake_lidar_node.cpp)
target_link_libraries(fake_lidar_node fake_lidar)
ament_target_dependencies(fake_lidar_node arm_navi_safety_interfaces rclcpp sensor_msgs)

install(TARGETS fake_lidar fake_lidar_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/safety_latency_benchmark.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fake_lidar test/test_fake_lidar.cpp)
  if(TARGET test_fake_lidar)
    target_link_libraries(test_fake_lidar fake_lidar)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_fault_injection_pipeline.py TIMEOUT 120)
endif()

ament_export_include_directories(include)
ament_export_libraries(fake_lidar)
ament_export_dependencies(arm_navi_safety_interfaces rclcpp sensor_msgs)
ament_package()
