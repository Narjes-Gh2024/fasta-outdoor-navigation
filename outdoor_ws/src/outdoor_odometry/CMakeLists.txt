cmake_minimum_required(VERSION 3.8)
project(outdoor_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

add_executable(frame_converter_node
  src/frame_converter.cpp
)

ament_target_dependencies(frame_converter_node
  rclcpp
  geometry_msgs
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
)

target_compile_features(frame_converter_node PUBLIC cxx_std_17)

add_executable(encoder_odometry_node
  src/encoder_odometry.cpp
)

ament_target_dependencies(encoder_odometry_node
  rclcpp
  geometry_msgs
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
  sensor_msgs
)

target_compile_features(encoder_odometry_node PUBLIC cxx_std_17)


add_executable(odometry_state_machine_node
  src/odometry_state_machine.cpp
)

ament_target_dependencies(odometry_state_machine_node
  rclcpp
  geometry_msgs
  tf2_ros
  nav_msgs
  tf2_geometry_msgs
  sensor_msgs
)

target_compile_features(odometry_state_machine_node PUBLIC cxx_std_17)

install(TARGETS
  frame_converter_node
  encoder_odometry_node
  odometry_state_machine_node
  DESTINATION lib/${PROJECT_NAME}
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
