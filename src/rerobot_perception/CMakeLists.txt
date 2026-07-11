# claude: rerobot_perception — 3D 自律移動用ユーティリティ
cmake_minimum_required(VERSION 3.8)
project(rerobot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# claude: pcd_to_gridmap のみ PCL (io/common) を使う。ROS ラッパ (pcl_ros) は不要
find_package(PCL REQUIRED COMPONENTS common io)

add_executable(rfans_ring_converter src/rfans_ring_converter.cpp)
ament_target_dependencies(rfans_ring_converter rclcpp sensor_msgs)

add_executable(pcd_to_gridmap src/pcd_to_gridmap.cpp)
target_link_libraries(pcd_to_gridmap ${PCL_LIBRARIES})
target_include_directories(pcd_to_gridmap PRIVATE ${PCL_INCLUDE_DIRS})

install(TARGETS
  rfans_ring_converter
  pcd_to_gridmap
  DESTINATION lib/${PROJECT_NAME})

ament_package()
