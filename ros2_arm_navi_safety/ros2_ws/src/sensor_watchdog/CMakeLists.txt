cmake_minimum_required(VERSION 3.16)
project(sensor_watchdog)

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

add_library(watchdog_monitor src/watchdog_monitor.cpp)
target_include_directories(watchdog_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(sensor_watchdog_node src/sensor_watchdog_node.cpp)
target_link_libraries(sensor_watchdog_node watchdog_monitor)
ament_target_dependencies(sensor_watchdog_node arm_navi_safety_interfaces rclcpp sensor_msgs)

install(TARGETS watchdog_monitor sensor_watchdog_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_watchdog_monitor test/test_watchdog_monitor.cpp)
  if(TARGET test_watchdog_monitor)
    target_link_libraries(test_watchdog_monitor watchdog_monitor)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_sensor_watchdog_node.py TIMEOUT 90)
endif()

ament_export_include_directories(include)
ament_export_libraries(watchdog_monitor)
ament_package()
