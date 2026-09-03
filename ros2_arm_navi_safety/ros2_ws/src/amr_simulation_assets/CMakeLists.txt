cmake_minimum_required(VERSION 3.8)
project(amr_simulation_assets)

find_package(ament_cmake REQUIRED)
install(DIRECTORY worlds models actors licenses upstream DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
