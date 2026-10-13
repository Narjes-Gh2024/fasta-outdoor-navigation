cmake_minimum_required(VERSION 3.8)
project(outdoor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(
  kinematic_controller_node
  src/kinematic_controller.cpp
)
ament_target_dependencies(
  kinematic_controller_node
  rclcpp
  geometry_msgs
  trajectory_msgs
)

add_executable(
  cmd_vel_filter_node
  src/cmd_vel_filter.cpp
)
ament_target_dependencies(
  cmd_vel_filter_node
  rclcpp
  geometry_msgs
)

install(
  TARGETS
  kinematic_controller_node
  cmd_vel_filter_node
  DESTINATION
    lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
