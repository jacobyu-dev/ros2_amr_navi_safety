cmake_minimum_required(VERSION 3.16)
project(safety_supervisor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(arm_navi_safety_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_library(safety_state_machine src/safety_state_machine.cpp)
target_include_directories(safety_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(safety_latency_monitor src/safety_latency_monitor.cpp)
target_include_directories(safety_latency_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(safety_supervisor_node_lib src/safety_supervisor_node.cpp)
target_include_directories(safety_supervisor_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(safety_supervisor_node_lib safety_latency_monitor safety_state_machine)
ament_target_dependencies(safety_supervisor_node_lib arm_navi_safety_interfaces rclcpp)

add_executable(safety_supervisor_node src/safety_supervisor_main.cpp)
target_link_libraries(safety_supervisor_node safety_supervisor_node_lib)
ament_target_dependencies(safety_supervisor_node rclcpp)

install(TARGETS safety_state_machine safety_latency_monitor safety_supervisor_node_lib safety_supervisor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_safety_state_machine test/test_safety_state_machine.cpp)
  if(TARGET test_safety_state_machine)
    target_link_libraries(test_safety_state_machine safety_state_machine)
  endif()

  ament_add_gtest(test_thread_safety test/test_thread_safety.cpp)
  if(TARGET test_thread_safety)
    target_link_libraries(test_thread_safety safety_supervisor_node_lib safety_state_machine)
    ament_target_dependencies(test_thread_safety rclcpp)
  endif()

  ament_add_gtest(test_safety_latency_monitor test/test_safety_latency_monitor.cpp)
  if(TARGET test_safety_latency_monitor)
    target_link_libraries(test_safety_latency_monitor safety_latency_monitor)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_safety_supervisor_node.py TIMEOUT 90)
  add_launch_test(test/test_safety_pipeline.py TIMEOUT 90)
endif()

ament_export_include_directories(include)
ament_export_libraries(safety_state_machine safety_latency_monitor safety_supervisor_node_lib)
ament_export_dependencies(arm_navi_safety_interfaces rclcpp)
ament_package()
