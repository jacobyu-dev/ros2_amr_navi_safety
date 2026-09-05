cmake_minimum_required(VERSION 3.16)
project(nav2_path_validator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(arm_navi_safety_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(path_metrics src/path_metrics.cpp)
target_include_directories(path_metrics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(nav2_path_validator_node src/path_validator_node.cpp src/main.cpp)
target_include_directories(nav2_path_validator_node PRIVATE include)
target_link_libraries(nav2_path_validator_node path_metrics)
ament_target_dependencies(nav2_path_validator_node
  action_msgs arm_navi_safety_interfaces geometry_msgs nav_msgs rclcpp std_msgs std_srvs
  tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS path_metrics nav2_path_validator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/compare_path_runs.py scripts/record_validation_bag.sh
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_metrics test/test_path_metrics.cpp)
  if(TARGET test_path_metrics)
    target_link_libraries(test_path_metrics path_metrics)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_nav2_path_validator_node.py TIMEOUT 45)
endif()

ament_export_include_directories(include)
ament_export_libraries(path_metrics)
ament_package()
