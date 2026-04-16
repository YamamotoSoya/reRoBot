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


add_executable(epos4_controller src/epos4_controller.cpp)
ament_target_dependencies(epos4_controller
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  canopen_interfaces
)

install(TARGETS
  epos4_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
