<!-- 自動生成: python3 docs/claude/gen_docs_index.py (2026-10-29)。手で編集しない — 元ファイルのタイトル / ステータス行を直して再生成。 -->

# docs/report — 解決済みバグの事後報告

`/debug-report` skill のテンプレに沿った事後報告。未解決の調査は [docs/issue/INDEX.md](../issue/INDEX.md)、設計文書は [docs/features/INDEX.md](../features/INDEX.md)。

| ファイル | 日付 | タイトル |
|---|---|---|
| [2026-05-26_slam_toolbox_scan_queue_full.md](2026-05-26_slam_toolbox_scan_queue_full.md) | 2026-05-26 | slam_toolbox の `Message Filter dropping message ... queue is full` 調査レポート |
| [2026-06-04_epos4_controller_init_race_dead_wheel.md](2026-06-04_epos4_controller_init_race_dead_wheel.md) | 2026-06-04 | epos4_controller の init 連射レースによる片輪が動かない問題 |
| [2026-06-13_rfans_driver_dataid_0x37_z0_2d.md](2026-06-13_rfans_driver_dataid_0x37_z0_2d.md) | 2026-06-13 | rfans_driver の R-Fans-16(dataID 0x37)で「no matching device model」「点群が z=0 の平面(2D)」になる |
| [2026-06-13_rfans_driver_low_fps_per_point_spin.md](2026-06-13_rfans_driver_low_fps_per_point_spin.md) | 2026-06-13 | rfans_driver の点群更新頻度が遅い(RViz で約 1〜2 Hz) |
| [2026-07-11_colcon_stale_volume_symlink_conflict.md](2026-07-11_colcon_stale_volume_symlink_conflict.md) | 2026-07-11 | colcon build が named volume に残った旧成果物と衝突して realsense2_camera_msgs で失敗する |
| [2026-07-11_dockerfile_libgtsam_unstable_dev_missing.md](2026-07-11_dockerfile_libgtsam_unstable_dev_missing.md) | 2026-07-11 | Dockerfile の apt install が libgtsam-unstable-dev 不在で失敗する (exit 100) |
| [2026-07-11_lio_sam_colcon_not_discovered.md](2026-07-11_lio_sam_colcon_not_discovered.md) | 2026-07-11 | lio_sam が colcon にパッケージとして認識されない (src/external/COLCON_IGNORE 規約の symlink 漏れ) |
| [2026-07-11_lio_sam_ros2_find_package_eigen_fail.md](2026-07-11_lio_sam_ros2_find_package_eigen_fail.md) | 2026-07-11 | lio_sam (ros2 ブランチ) の find_package(Eigen) が EigenConfig.cmake 不在で失敗する |
| [2026-08-01_rfans_driver_libstar_exception_runtime_sigabrt.md](2026-08-01_rfans_driver_libstar_exception_runtime_sigabrt.md) | 2026-08-01 | rfans_driver の起動直後 SIGABRT (libstar.so の例外ランタイム横取り) |
| [2026-09-02_rfans_garbage_frame_glim_abort.md](2026-09-02_rfans_garbage_frame_glim_abort.md) | 2026-09-02 | surestar_rfans_ros2 のゴミフレーム publish による GLIM オフライン SLAM の abort |

PDF 版 (ゼミ資料用、同名 md と同内容): [2026-05-26_slam_toolbox_scan_queue_full.pdf](2026-05-26_slam_toolbox_scan_queue_full.pdf), [2026-06-04_epos4_controller_init_race_dead_wheel.pdf](2026-06-04_epos4_controller_init_race_dead_wheel.pdf)
