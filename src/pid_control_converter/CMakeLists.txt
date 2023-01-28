cmake_minimum_required(VERSION 3.5)
project(pid_control_converter)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_common REQUIRED)
find_package(roar_msgs REQUIRED)
find_package(control_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
  nav2_util
  nav2_core
  roar_msgs
  control_msgs
)

add_executable(pid_control_converter_node src/pid_control_converter_node.cpp)
ament_target_dependencies(pid_control_converter_node
  ${dependencies}
)

target_include_directories(pid_control_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS pid_control_converter_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
