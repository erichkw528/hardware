cmake_minimum_required(VERSION 3.5)
project(gokart_hardware_launches)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# find_package(ament_cmake REQUIRED)
# find_package(zed_wrapper REQUIRED)
# find_package(roar-gokart-urdf REQUIRED)
# find_package(livox_ros2_driver REQUIRED)
# find_package(point_one_gps_driver REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY param DESTINATION share/${PROJECT_NAME})
ament_package()
