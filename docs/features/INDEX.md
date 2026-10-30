<!-- 自動生成: python3 docs/claude/gen_docs_index.py (2026-10-30)。手で編集しない — 元ファイルのタイトル / ステータス行を直して再生成。 -->

# docs/features — 追加機能の設計文書

`/feature-doc` skill のテンプレに沿った設計文書 (目的・設計の勘所・データフロー・使い方)。運用手順は [docs/manual/](../manual/00_index.md)、仕組みの教材は [docs/text/](../text/README.md)。

| ファイル | 日付 | タイトル |
|---|---|---|
| [2026-06-05_motor_free_mode.md](2026-06-05_motor_free_mode.md) | 2026-06-05 | モータ脱力(フリー)モードの追加 |
| [2026-06-13_rfans_driver_ros2_port.md](2026-06-13_rfans_driver_ros2_port.md) | 2026-06-13 | StarROS2 (rfans_driver) の ROS1 → ROS2 移植 |
| [2026-08-17_glim_map_to_nav2.md](2026-08-17_glim_map_to_nav2.md) | 2026-08-17 | GLIM 3D 地図 → Nav2 接続パイプライン (2D 地図変換 + R-Fans /scan 化) |
| [2026-09-14_glim_traj_to_2dmap.md](2026-09-14_glim_traj_to_2dmap.md) | 2026-09-14 | glim_traj_to_2dmap — bag 生点群 × GLIM 最適化軌跡による 2D 地図変換 |
| [2026-09-19_epos4_controller_watchdog_fault_monitor.md](2026-09-19_epos4_controller_watchdog_fault_monitor.md) | 2026-09-19 | epos4_controller の指令ウォッチドッグ・フォルト監視・CAN リンク喪失検知 |
| [2026-09-23_wt901c_comparison_imu.md](2026-09-23_wt901c_comparison_imu.md) | 2026-09-23 | 比較用 IMU WITmotion WT901C-TTL を BNO086 と同時に bag へ記録する手順 |
| [2026-10-02_2dmap_threshold_unknown_keepout.md](2026-10-02_2dmap_threshold_unknown_keepout.md) | 2026-10-02 | 2D 地図のしきい値比較・未観測マーク・keepout 生成 (2026-10-02) |
| [2026-10-04_waypoint_runner_spec.md](2026-10-04_waypoint_runner_spec.md) | 2026-10-04 | waypoint 走行 仕様書 — 打つ・止まる・再開する |
| [2026-10-10_lidar_localization_3d.md](2026-10-10_lidar_localization_3d.md) | 2026-10-10 | 3D 自己位置推定 (lidar_localization_ros2) の導入 — nav 構成 `nav_lidar3d` |
| [2026-10-16_auto_run_joy_mux.md](2026-10-16_auto_run_joy_mux.md) | 2026-10-16 | 自律走行の当日運用フロー: twist_mux 調停 + パッド操作の waypoint_runner + auto_run 統合 |
| [2026-10-16_follow_bt_narrow_corridor.md](2026-10-16_follow_bt_narrow_corridor.md) | 2026-10-16 | 追従型 NavigateThroughPoses BT: 計画失敗でも旧経路を保持して前方ロボットに stop-and-go 追従する |
| [2026-10-16_rfans_yaw_calib.md](2026-10-16_rfans_yaw_calib.md) | 2026-10-16 | R-Fans / urg の yaw 較正ツール (rfans_yaw_calib) と 2026-10-16 の較正結果の反映 |
| [2026-10-30_global_costmap_rolling_window.md](2026-10-30_global_costmap_rolling_window.md) | 2026-10-30 | global costmap の追従窓化 (rolling_window 60 m) と waypoint_runner の窓送り |
| [2026-10-30_nav2_smoothing.md](2026-10-30_nav2_smoothing.md) | 2026-10-30 | Nav2 の平滑化 2 段 (経路 SimpleSmoother + 速度 velocity_smoother) |
