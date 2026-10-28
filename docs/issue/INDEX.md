<!-- 自動生成: python3 docs/claude/gen_docs_index.py (2026-10-28)。手で編集しない — 元ファイルのタイトル / ステータス行を直して再生成。 -->

# docs/issue — 調査記録の一覧

未解決・調査中は `issue/` 直下、解決・前提消失・対象廃止は `issue/closed/` に置く (2026-10-23 から)。
解決したら冒頭のステータス行を更新し `closed/` へ `git mv` → 本索引を再生成。経緯の月別まとめは [docs/monthly/](../monthly/00_index.md)。

## 未解決・調査中 (23 本)

| ファイル | 日付 | ステータス (冒頭行) | タイトル |
|---|---|---|---|
| [2026-07-07_monthly_2026_6_todo_triage.md](2026-07-07_monthly_2026_6_todo_triage.md) | 2026-07-07 | 未解決 (項目ごとの状態は下表。本ファイルは分析、実作業の指示は同名の .prompt.md を参照) | docs/monthly/2026_6 の TODO・問題のトリアージ (原因分析と対処法) |
| [2026-07-07_repository_audit.md](2026-07-07_repository_audit.md) | 2026-07-07 | (ステータス行なし) | reRoBot Issue 一覧 (リポジトリ全体監査) |
| [2026-08-11_joy_spin_epos_shutdown_usb_stall.md](2026-08-11_joy_spin_epos_shutdown_usb_stall.md) | 2026-08-11 | 未解決 (復旧手順は確立済み。根本対策 = EPOS 側ハードニングが未適用、原因フォルトコードも未確認) | joy 操作中の急回転で EPOS 停止 + CANUSB が USB ストールする問題 (3 回目) |
| [2026-08-11_utm30lx_usb_instability.md](2026-08-11_utm30lx_usb_instability.md) | 2026-08-11 | 調査中 (ハード点検待ち — 電源 LED / 12 V 供給 / USB コネクタの確認が次アクション) | UTM-30LX の USB 接続が不安定 → 現在完全に不通の問題 |
| [2026-08-12_glim_horizontal_drift.md](2026-08-12_glim_horizontal_drift.md) | 2026-08-12 | 調査完了・根本原因を特定 / 対策候補 3 (パラメータ調整) を 2026-08-13 に 2 ラウンド実験済み — 塗れ −34.5% の推奨設定 E11 を提示 (`2026-08-13_glim_param_tu… | GLIM (3D+IMU) の水平ドリフト — 原因切り分けと対策候補 |
| [2026-08-13_glim_param_tuning.md](2026-08-13_glim_param_tuning.md) | 2026-08-13 | 実験完了 (2 ラウンド・計 12 run) / 推奨 = E11 (塗れ −34.5%)。repo config は未変更 — 反映はユーザ判断 | GLIM パラメータ掃引実験 — 水平ドリフト対策候補 3 の検証 |
| [2026-08-14_rfans_driver_renewal.md](2026-08-14_rfans_driver_renewal.md) | 2026-08-14 | 統合完了 — bringup/GLIM/scripts を新ドライバへ全面切替済み (実機検証込み)。残は縦角再較正のみ | R-Fans-16 ドライバ刷新 — 旧 StarROS2 の問題総括と新 surestar_rfans_ros2 への移行 |
| [2026-08-15_9goukan_building_wall_doubling.md](2026-08-15_9goukan_building_wall_doubling.md) | 2026-08-15 | 原因特定済み (未対策) — 累積ドリフト + ループ閉合ゼロによる「往路 vs 周回終盤」の再訪ズレ | 9 号館周回 bag — GLIM 地図の建物壁 1〜2 m 二重化の原因調査 |
| [2026-08-16_rfans_mount_angle_glim_z_collapse.md](2026-08-16_rfans_mount_angle_glim_z_collapse.md) | 2026-08-16 | 原因特定済み — 15° はセンサ内部の per-scan 系統誤差 (縦角表が本命) の z 投影が取付傾斜で増幅されて暴騰、45° は「近距離地面偏重による照合縮退」。extrinsic 角度誤差説は因果実験で棄却済… | R-Fans 取付角実験 (45°/15°) — GLIM 地図崩壊の原因調査 |
| [2026-09-02_rfans_scan_motor_dropout.md](2026-09-02_rfans_scan_motor_dropout.md) / [要点版](2026-09-02_rfans_scan_motor_dropout.summary.md) | 2026-09-02 | 調査中 (一次原因未特定) — 2026-09-20 更新: 10 Hz 運転でも発生 (9/19 外周 bag 32 分で 2 回) → 「20 Hz が引き金」説は棄却。GLIM オフライン地図では底の 5 s 欠落… | R-Fans スキャンモータの間欠的な脱落 (~10 s の回転ディップ) |
| [2026-09-10_glim_z_drift_not_vangle.md](2026-09-10_glim_z_drift_not_vangle.md) / [要点版](2026-09-10_glim_z_drift_not_vangle.summary.md) | 2026-09-10 | 調査完了 — critic 2 回査読 (§7) + 計 29 run。z ドリフトは 3 成分に分解できた。(1) 一定沈降 c ≈ −1° (z_end の主因) = LiDAR–IMU extrinsic の pi… | GLIM z ドリフトの再検証 — 「縦角表の系統誤差が主犯」説は棄却 |
| [2026-09-19_epos_stop_link_loss_rpdo_timeout_power.md](2026-09-19_epos_stop_link_loss_rpdo_timeout_power.md) | 2026-09-19 | 調査中・critic 査読済み (09-19 ×2)。事象別の確度は §4 の表。原因は 1 つではなく最低 3 系統が混在 (非常停止 SW の断線 = 確定、PC↔CAN リンク喪失 → 0x8250 = 15:35… | 新電源構成 (harness v2) での EPOS 停止・暴走 — 初のフォルトコード取得 (0x8250 RPDO timeout / 0x3220 低電圧) |
| [2026-09-24_bno086_auto_tare_rotates_motion_outputs.md](2026-09-24_bno086_auto_tare_rotates_motion_outputs.md) | 2026-09-24 | 原因特定済み・対策適用済み・物理向き実測済み (09-24)・accel 再較正済み (09-26) → 較正後 bag で IMU 入力の正しさを確認 (09-27、診断指標 0.91°、BNO≈WT、§2.12)。G… | BNO086 ドライバ既定 `auto_tare: all` が accel/gyro を起動時姿勢の座標に回している — `/imu/data` は物理センサ座標ではない |
| [2026-10-01_amcl_hedge_corridor_scan_reduction.md](2026-10-01_amcl_hedge_corridor_scan_reduction.md) | 2026-10-01 | 調査中 (選び方の効果は未検出。通路の進行方向退化は総当たり照合で直接確認、critic 査読は (1)〜(3) のみ・総当たり結果は未査読) | AMCL が植え込み通路で見失う — スキャン変換の「選び方」を変えた並走比較 |
| [2026-10-02_scan_ground_ghost_rouka_vangle.md](2026-10-02_scan_ground_ghost_rouka_vangle.md) | 2026-10-02 | 調査中 (切り出し・ドライバの計算式は正常と確認。縦角誤差は実在するが値は幅付き。15〜20 m の壁の主因は未特定 — 静的誤差だけでは説明できず、動的ピッチ / 地形が候補。critic 査読済み。§8: 自律走行 … | /scan・/scan_all の「地面の壁」— 縦角・ドライバ・切り出しの切り分け (8 号館廊下 bag) |
| [2026-10-08_glim_gpu_cuda_illegal_address_vram.md](2026-10-08_glim_gpu_cuda_illegal_address_vram.md) | 2026-10-08 | 回避策で解決 / 原因は本命段階 (確定ではない) — `config_global_mapping_gpu.json` の `randomsampling_rate` を上流既定 1.0 から CPU 版と同じ 0.2… | GLIM GPU 版が長尺 bag の終盤で cudaErrorIllegalAddress 落ち — GPU プリセットの点数が重い (回避済み・原因は本命段階) |
| [2026-10-11_loc_eval_2d_corner_loss_vs_3d.md](2026-10-11_loc_eval_2d_corner_loss_vs_3d.md) | 2026-10-11 | 調査中 — 現象は確定 (2 bag × 反復で再現)、原因は傍証どまり (critic 査読 3 回)。 | 2D 自己位置推定 (AMCL / emcl2) が東端の角で見失う — 3D (lidar_localization_ros2) との bag 再生比較と全点スキャン追試 |
| [2026-10-15_keepout_corridor_stall_imu_yaw_jump.md](2026-10-15_keepout_corridor_stall_imu_yaw_jump.md) | 2026-10-15 | 原因特定 (1, 2, 3) + 対策実施済み (10-16、実走未検証)。狭窄通路の停止は「keepout が膨張されず経路が縁を這う」が本命 (critic 維持、交絡 = AMCL の横分散)。 | 10-15 自律走行: keepout で狭めた通路で停止 / AMCL 廊下跳び / IMU yaw 跳び / R-Fans ディップ |
| [2026-10-22_epos_fault_8250_pdo_freeze_lidar3d_run.md](2026-10-22_epos_fault_8250_pdo_freeze_lidar3d_run.md) | 2026-10-22 | 調査中・原因未特定。「何が起きたか」は bag で確定 (§2)。「なぜ PDO が止まったか」は記録不足で層を絞れない (§4)。critic 査読は未実施 (原因を断定していないため) | 10-22 3D 自己位置推定で自律走行中、EPOS が両輪同時に Fault 0x8250 (RPDO timeout) — PDO が両方向とも凍結、監視は検知できず |
| [2026-10-23_repository_review_incremental_cruft.md](2026-10-23_repository_review_incremental_cruft.md) | 2026-10-23 | 一部実施 — docs 構成 (§1.4) と Stop hook (#4) は 2026-10-23 に実施済み。コード・設定側 (#1 #2 #3 #8 #9 #10 #11 #14) と規約浄化 (#6 #7)、i… | リポジトリ全体の見直し (2026-10-23) — 継ぎ足し開発による重複・残骸・不整合 |
| [2026-10-24_start_crowd_sim_localization.md](2026-10-24_start_crowd_sim_localization.md) | 2026-10-24 | 試験結果あり (2026-10-24、2 段階)。第 1 試験 (10〜20 m 環、遮蔽 ≤19 %) は無害に見えたが critic 査読で人垣が薄すぎたと判定 → 第 2 試験 (1.5〜8 m・遮蔽 79 %・i… | スタート地点の人垣 (10〜20 m・前方以外遮蔽) を擬似注入した自己位置推定の再生試験 |
| [2026-10-25_rfans_range_comb_ghost.md](2026-10-25_rfans_range_comb_ghost.md) | 2026-10-25 | 本命 (critic 査読済み、因果は一部確認)。地図で木の幹・柱の周り半径 +0.5〜1 m に出る筒状の点は、R-Fans-16 (0x5C) の生点群に含まれる単発の距離偽点。偽点の距離は隣の点から離散的なオフセッ… | R-Fans 点群の「櫛状」距離偽点 — 木の幹・柱の周りに出る筒の正体 |
| [2026-10-27_rfans_azimuth_step.md](2026-10-27_rfans_azimuth_step.md) | 2026-10-27 | 保留 (原因未特定、2026-10-28 ユーザ判断)。観測は critic 査読済み (§2〜§3 の表現は critic の判定に合わせた)。R-Fans-16 (0x5C) が報告する方位の値が、ときどき「1 列の… | R-Fans の報告方位が 1 列ぶん跳ぶ — 静止ポールの二重像と、回転数で変わる方位 |

## 解決・前提消失・対象廃止 (8 本) — `closed/`

| ファイル | 日付 | ステータス (冒頭行) | タイトル |
|---|---|---|---|
| [2026-07-07_rfans_driver_calc_review.md](closed/2026-07-07_rfans_driver_calc_review.md) | 2026-07-07 | 未解決(静的レビューで検出した問題は未修正。x/y/z の主要経路に実害はなく運用は可能) | rfans_driver (R-Fans-16) 計算式レビュー — 主要経路は正常、堅牢性・副次フィールドに問題 |
| [2026-07-07_wheel_odometry_encoder_scaling_4x.md](closed/2026-07-07_wheel_odometry_encoder_scaling_4x.md) | 2026-07-07 | 解決(2026-07-29 根本修正適用・浮かせ状態で検証済み。残: 接地での 10 m 直進 / 360° 旋回の最終確認) | ホイールオドメトリの回転数が信頼できない問題(エンコーダ分解能スケーリングの4倍ズレ) |
| [2026-08-01_odometry_stop_wobble_zero_stamp_pairing.md](closed/2026-08-01_odometry_stop_wobble_zero_stamp_pairing.md) | 2026-08-01 | 解決 (真因判明: ゲームパッドの充電不足)。ただし §機構 の stamp=0 は実測事実であり、潜在欠陥として残存 | 停止時に scan が左右に揺れる — 真因はゲームパッド電池切れ (stamp=0 は別件の潜在欠陥として記録) |
| [2026-08-15_offline_viewer_zenity_dialog_hang.md](closed/2026-08-15_offline_viewer_zenity_dialog_hang.md) | 2026-08-15 | 解決済み (原因実証 + 修正適用)。修正 = docker-compose glim サービスに `GSK_RENDERER=cairo` 追加 | offline_viewer ファイル選択不能 / open map 0% 停止 — 不可視 zenity ダイアログ待ち |
| [2026-08-30_chassis_rework_followup.md](closed/2026-08-30_chassis_rework_followup.md) | 2026-08-30 | ほぼ完了 (08-31) — 残は ④回転方向の浮かせ確認・⑤実機検証のみ。完了した項目はチェックを付けて消し込む。 | 車体改修 (2026-08-30) フォローアップ — 実測待ちの編集箇所一覧 |
| [2026-09-13_decel_epos_stop_12v_loss.md](closed/2026-09-13_decel_epos_stop_12v_loss.md) | 2026-09-13 | 調査中 — 本命 = 「回生時にバッテリ BMS が充電方向を遮断 → 孤立した母線が跳ね上がり、DSR 50/5 (しきい値 56 V 設定) が受けられず DC-DC (入力上限 36 V) が過電圧」の複合説 (c… | 減速→停止の瞬間に EPOS ×2 停止 + LiDAR 給電 12 V 系が喪失 (DC-DC 死亡疑い) |
| [2026-09-26_rfans_frame_stamp_bundle_quantization.md](closed/2026-09-26_rfans_frame_stamp_bundle_quantization.md) | 2026-09-26 | 解決 (09-27 live 確認済み)。 ドライバ修正実装済み・bag 再生と実機 live の両方で 100 ± 0.2 ms を確認・GLIM 効果測定済み (警告 1,597 → 0、z はノイズ床内)。subm… | R-Fans フレーム stamp が UDP bundle 到着 (41 ms) に量子化され、隣接間隔が 83 / 123 ms で交互する |
| [2026-10-16_amcl_lateral_bias_rfans_yaw_offset.md](closed/2026-10-16_amcl_lateral_bias_rfans_yaw_offset.md) | 2026-10-16 | 解決 (原因特定 → URDF 反映 → 同日実走 0918 で確認: 横ずれ 0.38 → 0.14 m、バイアス −0.32 → +0.02、yaw 2.7 → 0.6°。§8)。 | 10-16 AMCL の自己位置が建物に対して横 0.1〜0.5 m・yaw 数° 泳ぐ: 主因は R-Fans の方位ゼロが車体進行方向から ≈5.6° 回っていること |
