cmake_minimum_required(VERSION 3.5)
project(dim_controller)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

install(PROGRAMS
  nodes/position_controller.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
