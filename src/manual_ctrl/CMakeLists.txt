cmake_minimum_required(VERSION 3.5)
project(manual_ctrl)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_package()
