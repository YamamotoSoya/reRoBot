cmake_minimum_required(VERSION 3.8)
project(rerobot_bringup)

find_package(ament_cmake REQUIRED)

# claude: behavior_trees = 自作 Nav2 BT (追従型 NavigateThroughPoses, 2026-10-16)
install(DIRECTORY
  launch
  config
  urdf
  rviz
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# claude: 起動結果サマリ (rerobot_bringup.launch.py の最後に起動, 2026-10-01)
# claude: 停止点つき waypoint 走行 (stop: true の点で止まりキーで再開, 2026-10-04)
# claude: bag 撮影中の停止点ティーチング用マーカー (joy ボタン → /teach_marker, 2026-10-09)
# claude: 3D 自己位置推定の z 打ち消し + /initialpose 中継 (loc_lidar3d が起動, 2026-10-10)
# claude: map→odom 見張り番 — AMCL 無音時に最後の TF を出し続ける (loc_amcl が起動, 2026-10-16)
install(PROGRAMS
  scripts/bringup_check.py
  scripts/auto_run_check.py
  scripts/initialpose.py
  scripts/waypoint_runner.py
  scripts/teach_marker.py
  scripts/map_level_bridge.py
  scripts/map_odom_holder.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
