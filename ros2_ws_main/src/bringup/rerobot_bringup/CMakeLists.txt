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
install(PROGRAMS
  scripts/bringup_check.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
