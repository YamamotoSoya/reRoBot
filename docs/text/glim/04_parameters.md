<!-- claude: GLIM 読本 第4章 (2026-08-12)。値は ros2_ws_glim/config/ (GLIM 1.2.2 準拠) の実物。
     解説は上流 config のコメントヘッダ (config_odometry_gpu.json:3-38 等) の訳出 + 第2-3章の機構。 -->

# 第4章 パラメータ大全 — 15 ファイル・全キーの意味と reRoBot の現在値

設定の実体は `ros2_ws_glim/config/` の 15 JSON (コメント可の JSON5 系)。
**上流デフォルトと値が違う箇所には ★** を付ける (glim_env コンテナ内の GLIM 1.2.2
同梱 config との diff による。★が無い表の値はすべて上流デフォルトのまま)。

```
ファイルの棚 (この章の地図)
├── 4.1 config.json .................. 結線表 (★ gpu→cpu ×3)
├── 4.2 config_ros.json .............. ROS I/F (★ topic/frame/TF)
├── 4.3 config_sensors.json ......... 外部パラメータ (★ T_lidar_imu)
├── 4.4 config_preprocess.json ...... 前処理 (無変更)
├── 4.5 config_odometry_cpu.json .... ① LIO オドメトリ (無変更)
├── 4.6 config_odometry_ct.json ..... ① CT-ICP (無変更・比較実験用)
├── 4.7 config_sub_mapping_cpu.json . ② submap 構築 (★ enable_imu — 実質同値)
├── 4.8 config_global_mapping_cpu ... ③ 大域最適化 (★ 同上)
├── 4.9 pose_graph / passthrough .... 代替実装の専用キー
└── 4.10 viewer / logging ........... 表示・ログ
```

## 4.1 config.json — 結線表

| キー | reRoBot | 備考 |
|---|---|---|
| `config_odometry` | ★ `config_odometry_cpu.json` | 上流既定は `_gpu` — **このイメージでは .so が無く動かない**ため cpu 必須 |
| `config_sub_mapping` | ★ `config_sub_mapping_cpu.json` | 同上 |
| `config_global_mapping` | ★ `config_global_mapping_cpu.json` | 同上 |
| 他 (ros/sensors/preprocess/viewer/logging) | 既定ファイル名 | |

⚠️ `config.json:10-14` の「CT-ICP に戻す手順」コメントは IMU レス時代の古い記述を含む
(第 5 章 5.6)。正しい切替レシピは [第7章 7.4](07_checklist.md)。

## 4.2 config_ros.json — ROS I/F

| キー | reRoBot | 意味・指針 |
|---|---|---|
| `imu_topic` | ★ `/imu/data` | BNO086 (上流既定は Ouster 内蔵 IMU の topic) |
| `points_topic` | ★ `/sdk_could` | R-Fans (typo だがドライバ仕様) |
| `image_topic` | `/image` | 未使用 (カメラなし) |
| `imu_qos` / `points_qos` / `image_qos` | `sensor_data` (imu は depth 1000) | 購読 QoS。センサ系は Best Effort の sensor_data プロファイルが定石。imu の depth 1000 は 100 Hz × 10 s 分のバッファ |
| `enable_local_mapping` / `enable_global_mapping` | `true` / `true` | ②③層の有効化。odometry 単体評価なら false にできる |
| `keep_raw_points` | `false` | keyframe に生点群を保持 (dump が肥大。デバッグ用) |
| `imu_time_offset` / `points_time_offset` | `0.0` / `0.0` | 到着 stamp への時刻補正 [s]。**センサ間の系統的時刻ずれの微調整はここ** (検証手順はタイムスタンプ読本 §7.2) |
| `acc_scale` | `0.0` | 加速度スケール補正。0 = 自動検出 (静止時に \|a\|=9.81 になる係数を推定) |
| `ang_scale` | (キー欠落 = 既定 1.0) | 角速度スケール。deg/s 出力の IMU (Hesai 内蔵など) を rad/s に直す係数。BNO086 は rad/s 出力なので 1.0 で正しいが、**キーごと消えているため意図が読めない状態** (第 5 章 5.6 のねじれ③) |
| `imu_frame_id` / `lidar_frame_id` | `""` / `""` | 空 = 受信メッセージの header.frame_id から自動検出 (実測で imu_link / rfans に解決 — dump の meta で確認済み) |
| `base_frame_id` | ★ `"glim_base"` | GLIM が姿勢を出す対象 frame。**車輪オドメトリの base_link と分離**するための reRoBot 独自値 (第 5 章 5.4) |
| `odom_frame_id` / `map_frame_id` | `odom` / `map` | TF の親側 |
| `publish_imu2lidar` | ★ `false` | GLIM 自身による imu→lidar TF 配信を止める。robot_state_publisher と親が二重になり TF ツリーが壊れるため (第 5 章 5.4) |
| `tf_time_offset` | `1e-6` | 配信 TF の stamp を僅かに未来へ (lookup の境界競合回避) |
| `extension_modules` | memory_monitor / standard_viewer / rviz_viewer | 第 2 章 2.4。ヘッドレス化はここを空に |

## 4.3 config_sensors.json — センサ外部パラメータ

| キー | reRoBot | 意味・指針 |
|---|---|---|
| `T_lidar_imu` | ★ `[0.075, 0, -0.085, 0, 0, 0.7071, 0.7071]` | **最重要**。IMU frame の点を LiDAR frame へ移す変換、TUM 形式 `[x y z qx qy qz qw]`。reRoBot は URDF から算出 (検算は第 5 章 5.3)。URDF のセンサ位置・向きを変えたら**必ず再計算** |
| `imu_acc_noise` / `imu_gyro_noise` | `0.05` / `0.02` | プリインテグレーションの雑音 (第 3 章 3.3)。ドリフトが IMU 起因と疑うときの調整点 |
| `imu_int_noise` / `imu_bias_noise` | `0.001` / `1e-5` | 積分雑音 / バイアス漂い速度 |
| `global_shutter_lidar` | `false` | true = 全点同時刻の LiDAR (deskew 不要)。回転式は false |
| `intensity_field` / `ring_field` | `"intensity"` / `""` | PointCloud2 のフィールド名マッピング |
| `autoconf_perpoint_times` | `true` | per-point time の形式 (相対/絶対・スケール) を自動推定 |
| `autoconf_prefer_frame_time` | `false` | 自動推定が怪しいとき frame 時刻 (stamp 一律) を優先するか |
| `perpoint_relative_time` | `true` | time フィールドは header.stamp からの相対秒 (autoconf 無効時に効く明示指定) |
| `perpoint_time_scale` | `1.0` | time フィールド→秒の係数 (ns なら 1e-9) |
| `global_shutter_camera` / `image_size` / `T_lidar_camera` / `intrinsics` / `distortion_model` / `distortion_coeffs` | (上流デフォルト) | カメラ系 — reRoBot 未使用のため実質無効 |

## 4.4 config_preprocess.json — 前処理 (reRoBot 無変更)

| キー | 値 | 意味・指針 |
|---|---|---|
| `distance_near_thresh` / `distance_far_thresh` | `0.5` / `100.0` | 距離帯フィルタ。near は自機筐体の自己反射除去 — **機体が写り込むなら最初に上げる値** |
| `use_random_grid_downsampling` | `true` | ランダムグリッド間引き (false なら通常のボクセル平均) |
| `downsample_resolution` | `1.0` | 間引きグリッド幅 [m]。**odometry に入る点群の密度を決める本丸**。小さくすると精密だが重い |
| `random_downsample_target` / `random_downsample_rate` | `10000` / `0.1` | ランダム間引きの目標点数 / 比率 |
| `enable_outlier_removal` | `false` | 統計的外れ値除去 (k=`outlier_removal_k`=10, `outlier_std_mul_factor`=1.0)。雨・粉塵で有効化を検討 |
| `enable_cropbox_filter` | `false` | 箱型除去 (`crop_bbox_frame`="lidar", min/max ±1 m)。**自機や搭乗者を消す用途はこちら** |
| `k_correspondences` | `10` | GICP の局所共分散推定に使う近傍点数 (第 3 章 3.1) |
| `num_threads` | `2` | 前処理スレッド数 |

## 4.5 config_odometry_cpu.json — ① LIO オドメトリ (現用・reRoBot 無変更)

| キー | 値 | 意味・指針 |
|---|---|---|
| `so_name` | `libodometry_estimation_cpu.so` | LIO 実装の指名 |
| `initialization_mode` | `"LOOSE"` | 初期化方式。LOOSE = 静止不要の緩い初期化 / NAIVE = 静止して重力方向を直接推定 |
| `initialization_window_size` | `3.0` | 初期化に使う時間窓 [s] |
| `init_pose_damping_scale` | `1e10` | 初期姿勢を固定する prior の強さ (大 = 事実上固定) |
| `smoother_lag` | `5.0` | 固定ラグ窓 [s] (第 1 章 1.4)。長い = 精度↑計算↑ |
| `use_isam2_dogleg` | `false` | iSAM2 の更新戦略を Dogleg に (false = Gauss-Newton)。発散するなら true が安定側 |
| `isam2_relinearize_skip` / `isam2_relinearize_thresh` | `1` / `0.1` | 再線形化の頻度 / 変数変動の閾値 |
| `fix_imu_bias` | `false` | true = バイアス推定を止め定数扱い |
| `compute_covs` | `false` | 出力姿勢の共分散を計算 (重い。下流で共分散が要るときだけ) |
| `registration_type` | `"GICP"` | フレーム照合方式 (GICP / VGICP) |
| `max_iterations` | `8` | 最適化の最大反復 |
| `lru_thresh` | `100` | iVox の LRU 保持閾値 (第 3 章 3.2) |
| `target_downsampling_rate` | `0.1` | iVox 地図へ入れる点の割合 |
| `ivox_resolution` / `ivox_min_dist` | `1.0` / `0.1` | iVox ボクセル幅 / ボクセル内最小点間距離。**屋内で精度不足なら resolution を 0.5 に落とすのが定番の一手** |
| `vgicp_resolution` / `vgicp_voxelmap_levels` / `vgicp_voxelmap_scaling_factor` | `0.5` / `1` / `2.0` | registration_type=VGICP のときのボクセル幅 / 多重解像度の段数 / 段間の倍率 |
| `validate_imu` | `true` | IMU データの妥当性チェック (時刻逆行など) |
| `save_imu_rate_trajectory` | `true` | IMU レート (100 Hz) の軌跡も dump に保存 |
| `num_threads` | `2` | スレッド数 |

## 4.6 config_odometry_ct.json — ① CT-ICP (IMU レス・比較実験用、reRoBot 無変更)

| キー | 値 | 意味・指針 |
|---|---|---|
| `so_name` | `libodometry_estimation_ct.so` | CT-ICP 実装 |
| `ivox_resolution` / `ivox_min_points_dist` / `ivox_lru_thresh` | `1.0` / `0.1` / `200` | iVox (キー名が cpu 版と微妙に違う点に注意) |
| `max_correspondence_distance` | `2.0` | 対応付けの最大距離 [m] |
| `location_consistency_inf_scale` | `1e-3` | 「スキャン開始点と終了点は近いはず」拘束の重み |
| `constant_velocity_inf_scale` | `1e3` | 等速仮定の重み。**大きい = 急加減速に弱く、滑らかさ優先** |
| `lm_max_iterations` | `8` | Levenberg-Marquardt 反復数 |
| `smoother_lag` | `1.0` | 固定ラグ窓 (LIO の 5.0 より短い) |
| `use_isam2_dogleg` / `isam2_relinearize_*` / `compute_covs` | (LIO と同義) | |
| `num_threads` | `4` | |

## 4.7 config_sub_mapping_cpu.json — ② submap 構築 (現用)

| キー | 値 | 意味・指針 |
|---|---|---|
| `so_name` | `libsub_mapping.so` | |
| `enable_imu` | ★ `true` | submap 内最適化に IMU ファクタを入れる。**odometry の方式と揃えること** (LIO なら true、CT-ICP なら false — 第 5 章 5.6 の「3 点セット」) |
| `enable_optimization` | `false` | submap 内部の再最適化。false = odometry の推定姿勢をそのまま採用 |
| `max_num_keyframes` | `15` | submap 1 個あたりの keyframe 上限 (= submap の大きさ) |
| `keyframe_update_strategy` | `"OVERLAP"` | keyframe 追加基準。OVERLAP = 既存 keyframe との重なり率 / DISPLACEMENT = 移動量 |
| `keyframe_update_min_points` | `500` | keyframe に足る最小点数 |
| `keyframe_update_interval_rot` / `_trans` | `3.14` / `1.0` | DISPLACEMENT 戦略時の旋回 [rad] / 並進 [m] 間隔 |
| `max_keyframe_overlap` | `0.6` | OVERLAP 戦略: 重なりがこれを下回ったら新 keyframe (小さい = keyframe が疎) |
| `create_between_factors` / `between_registration_type` | `false` / `"GICP"` | 隣接 keyframe 間に古典的相対姿勢ファクタを追加 (堅牢性向上の保険) |
| `registration_error_factor_type` | `"VGICP"` | keyframe 間マッチングコストの方式 |
| `keyframe_randomsampling_rate` | `1.0` | keyframe 点群のサンプリング率 (1.0 = 全点) |
| `keyframe_voxel_resolution` / `keyframe_voxelmap_levels` / `_scaling_factor` | `0.25` / `2` / `2.0` | keyframe 側 VGICP ボクセル (0.25 m 基本 + 0.5 m の 2 段) |
| `submap_downsample_resolution` / `submap_voxel_resolution` | `0.3` / `0.5` | 完成 submap の間引き幅 / ボクセル幅。**最終地図の解像度に直結** |

## 4.8 config_global_mapping_cpu.json — ③ 大域最適化 (現用)

| キー | 値 | 意味・指針 |
|---|---|---|
| `so_name` | `libglobal_mapping.so` | |
| `enable_imu` | ★ `true` | submap 間にも IMU ファクタ (重力整合) を張る。3 点セットの一角 |
| `enable_optimization` | `true` | 大域最適化そのものの有効化 |
| `init_pose_damping_scale` | `1e10` | 最初の submap を固定する強さ |
| `create_between_factors` / `between_registration_type` | `true` / `"GICP"` | 隣接 submap 間の between ファクタ (GPU 版既定は false — CPU 版は保険を厚く) |
| `registration_error_factor_type` | `"VGICP"` | submap 間マッチングコスト |
| `randomsampling_rate` | `0.2` | ファクタ評価に使う点のサンプル率 (GPU 版 1.0 — CPU は 2 割に間引いて速度確保) |
| `submap_voxel_resolution` / `submap_voxelmap_levels` / `_scaling_factor` | `0.5` / `1` / `2.0` | submap 照合のボクセル設定 |
| `max_implicit_loop_distance` | `100.0` | **暗黙的ループ閉じ込み**: 推定位置がこの距離以内の submap ペアを自動照合 (slam_toolbox のような明示検出でなく「近ければ全部張る」方式) |
| `min_implicit_loop_overlap` | `0.2` | 照合ペアに要求する最小重なり率。**下げるとループが増えるが偽拘束リスク増** — 事例A の対策候補③の調整点 |
| `use_isam2_dogleg` / `isam2_relinearize_*` | `false` / `1` / `0.1` | iSAM2 設定 (odometry と同義) |

## 4.9 代替実装の専用キー (reRoBot 未使用)

**libglobal_mapping_pose_graph.so** (`config_global_mapping_pose_graph.json`) — 軽量な
古典ポーズグラフ版。submap を点群でなく姿勢だけで繋ぎ、明示的ループ検出を行う:
`registration_type: "VGICP"` / `min_travel_dist: 50.0` (ループ候補に要求する走行距離) /
`max_neighbor_dist: 5.0` (候補の近さ) / `min_inliear_fraction: 0.5` (照合の
インライア率閾値、上流の typo そのまま) / `odom_factor_stddev: 1e-3` (**オドメトリ
ファクタの標準偏差** — 事例A で言及されたキーだが、これは GLIM 内部のオドメトリ層出力の
重みであって車輪オドメトリ入力ではない) / `loop_factor_stddev: 0.1` /
`loop_factor_robust_width: 1.0` (ロバスト損失幅) / `loop_candidate_buffer_size: 100` /
`loop_candidate_eval_per_thread: 2` / iSAM2 系 / `num_threads: 2`。

**libsub_mapping_passthrough.so** — submap 最適化を省く軽量版:
`keyframe_update_interval_rot: 0.01` / `_trans: 0.1` / `max_num_keyframes: 50` /
`max_num_voxels: -1` / `adaptive_max_num_voxels: 2.5` / `submap_voxel_resolution: 0.5` /
`min_dist_in_voxel: 0.2` / `max_num_points_in_voxel: 100` / `submap_target_num_points: 50000`。

**GPU 版 odometry** (`config_odometry_gpu.json` — ❌ 使用不可) にのみ存在するキー:
距離適応ボクセル `voxel_resolution: 0.25` / `_max: 0.5` / `_dmin: 5.0` / `_dmax: 20.0`
(近距離 0.25 m ↔ 遠距離 0.5 m を距離で内挿) / `voxelmap_levels: 2` /
`full_connection_window_size: 2` (窓内フレームを全結合でマッチング — 激しい運動なら 3〜5
と上流解説) / keyframe 系 (`keyframe_update_strategy: "OVERLAP"`, `max_num_keyframes: 15`,
`keyframe_min_overlap: 0.01` / `_max_overlap: 0.7` / `_delta_trans: 2.0` / `_delta_rot: 0.5` /
`_entropy_thresh: 0.99`)。

## 4.10 viewer / logging

- `config_viewer.json`: standard_viewer / interactive_viewer 共通に
  `viewer_width/height: 2560×1440`, `default_z_range: [-2.0, 4.0]` (高さ色付け範囲),
  `enable_partial_rendering: false` (+budget 1024), `point_shape_circle: true`,
  `point_size: 0.025` (`point_size_metric: true` = メートル単位), `points_alpha` /
  `factors_alpha: 0.5`。**表示のみで推定には無関係** — 「旧い点群が薄くなる」のは
  表示用間引きで、地図が消えているのではない (第 7 章)
- `config_logging.json`: `log_dir: "/tmp"`, `save_logs: true`, `rotate_logs: true`
  (8 MB × 10 世代)

## 4.11 reRoBot 差分の総括

15 ファイル中、上流デフォルトから**値を変えたのは 5 ファイルだけ**。すべてセンサ結線と
TF 整合であり、**推定アルゴリズムのチューニングは一切していない** (= 事例A のドリフトは
「未チューニングの素の GLIM」の挙動である):

| ファイル | 変更点 | 理由 |
|---|---|---|
| config.json | gpu → cpu ×3 | イメージに GPU .so が無い |
| config_ros.json | topics / base_frame_id=glim_base / publish_imu2lidar=false | センサ結線と TF 衝突回避 (第 5 章) |
| config_sensors.json | T_lidar_imu | 実機の取付幾何 (URDF から算出) |
| config_sub_mapping_cpu.json | enable_imu=true (コメントのみ実質) | LIO 構成の明示 (上流既定も true) |
| config_global_mapping_cpu.json | 同上 | 同上 |

チューニングに踏み込むときの入口は、症状別に:

```
チューニングの入口 (症状 → 触る棚)
├── 自機の写り込み ............ 4.4 distance_near_thresh / cropbox
├── 精度不足 (屋内・低速) ..... 4.5 ivox_resolution↓ / 4.4 downsample_resolution↓ (計算増と引換)
├── 処理落ち .................. 4.4 downsample↑ / 4.5 num_threads↑ / smoother_lag↓
├── 旋回で壁が二重 ............ まず時刻と T_lidar_imu を疑う (4.2 offsets / 4.3) — パラメータ以前
├── ループが閉じない .......... 4.8 min_implicit_loop_overlap↓ / max_implicit_loop_distance↑
└── IMU を信じすぎ/なさすぎ .... 4.3 imu_*_noise (上げる = 信じない)
```

→ [第5章 reRoBot での適用](05_rerobot_setup.md)
