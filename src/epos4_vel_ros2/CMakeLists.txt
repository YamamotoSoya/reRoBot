cmake_minimum_required(VERSION 3.8)
project(epos4_vel_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(canopen_interfaces REQUIRED)

add_executable(epos4_vel src/epos4_vel.cpp)
ament_target_dependencies(epos4_vel
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  canopen_interfaces
)

add_executable(control_vel_2chanel src/control_vel_2chanel.cpp)
ament_target_dependencies(control_vel_2chanel
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  canopen_interfaces
)

install(TARGETS
  epos4_vel
  control_vel_2chanel
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
