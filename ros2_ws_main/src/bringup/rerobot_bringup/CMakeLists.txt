cmake_minimum_required(VERSION 3.8)
project(rerobot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# claude: 起動結果サマリ (rerobot_bringup.launch.py の最後に起動, 2026-10-01)
# claude: 停止点つき waypoint 走行 (stop: true の点で止まりキーで再開, 2026-10-04)
# claude: bag 撮影中の停止点ティーチング用マーカー (joy ボタン → /teach_marker, 2026-10-09)
# claude: 3D 自己位置推定の z 打ち消し + /initialpose 中継 (loc_lidar3d が起動, 2026-10-10)
install(PROGRAMS
  scripts/bringup_check.py
  scripts/waypoint_runner.py
  scripts/teach_marker.py
  scripts/map_level_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
