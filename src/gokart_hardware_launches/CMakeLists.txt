cmake_minimum_required(VERSION 3.5)
project(gokart_hardware_launches)

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
