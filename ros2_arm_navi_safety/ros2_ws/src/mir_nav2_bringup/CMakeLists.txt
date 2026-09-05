cmake_minimum_required(VERSION 3.16)
project(mir_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/check_nav2_prerequisites.sh
  scripts/generate_maps.py
  scripts/initial_pose_publisher.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_generated_maps test/test_generated_maps.py)
  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
endif()

ament_package()
