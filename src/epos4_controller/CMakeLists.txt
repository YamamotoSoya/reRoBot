cmake_minimum_required(VERSION 3.8)
project(epos4_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(canopen_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)  # claude_tire claude_odom claude_sync


add_executable(epos4_controller src/epos4_controller.cpp)
ament_target_dependencies(epos4_controller
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  canopen_interfaces
)

add_executable(epos4_odometry src/epos4_odometry.cpp)
ament_target_dependencies(epos4_odometry
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  message_filters  # claude_tire claude_odom claude_sync
)

install(TARGETS
  epos4_controller
  epos4_odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  urdf
  rviz  # claude
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
