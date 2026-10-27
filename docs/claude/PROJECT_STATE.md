<!-- claude: 将来の Claude セッションに全体像を渡すためのメモリファイル。
     大きな状態変化 (機能追加・重要バグの解決・方針変更) があったら必ず更新すること。
     読み順: CLAUDE.md (規約・ビルド) → 本ファイル (現在地) → docs/issue/ (問題詳細)。 -->
# reRoBot プロジェクト状態メモ (Claude 用)

- **最終更新: 2026-10-27** — R-Fans 櫛状距離偽点 (`exp/glim-gpu-imu` で発覚、issue とツールを main に取り込み) の実機検査 (10-26/27 実施、偽点はセンサ本体・設定非依存) と除去フィルタ試作 v4 (ポール奥 85% 減) を `df015f1` でコミット (§5・§10)。10-24: スタート地点の人垣 (10〜20 m・前方以外遮蔽) を bag に擬似注入した自己位置推定の再生試験 (§10 冒頭、`tools/crowd_sim/`。密な近距離人垣で現行 AMCL は 1.5〜2 m 流れ初期誤差を直せない、lidar3d は無影響)。10-23: リポジトリ全体見直し (docs/issue/2026-10-23_repository_review_incremental_cruft.md)・docs 構成変更 (issue/closed・INDEX・docs/README)・月別変更履歴 `docs/monthly/2026_MM.md` 新設。**10-16 以前の更新履歴 (旧 6〜7 行目、39 KB) と §6 タイムライン (113 KB) は同日に削除し、[docs/monthly/](../monthly/00_index.md) に委譲した** — 経緯を追うときはそちらと `git log` を見る。
- 書き手: Claude Code (Fable 5)。次の Claude はまずこれを読めば現在地が分かるようにしてある。

---

## 1. プロジェクトの正体

- **つくばチャレンジ出場用の自律走行ロボット**。ユーザ (YamamotoSoya さん) の研究プロジェクトで、月次のゼミ報告がある (docs/monthly/)。
- 差動二輪。maxon EPOS4 ×2 を CAN (can0, 1 Mbps) 経由 `ros2_canopen` (CiA 402, CSV モード) で駆動。
- ROS 2 Jazzy。開発・実行は **Ubuntu ホスト + Docker コンテナ `rerobot_env`** が正。
  (この Windows チェックアウトは閲覧/文書作業用。symlink が壊れておりビルド不可 — 気にしなくてよい)
- ビルドは必ず `colcon build --symlink-install --executor sequential` (並列は canopen で壊れる)。

## 2. ハードウェア構成 (2026-07 時点)

| 要素 | 内容 | 備考 |
|------|------|------|
| 駆動 | maxon EPOS4 Compact 50/5 ×2, CAN | node_id 1=/motor1, 2=/motor2 |
| 実配線 | **motor1 = 右輪, motor2 = 左輪** | claude_swap。CLAUDE.md の「motor1=left」は古い (監査 Issue 16) |
| シャシ | タイヤ径 **φ0.256 m** (08-30 車体改修、旧 0.15。08-31 実測補正)。tread **0.529 m** (08-31 実測、旧 0.41) — いずれも params 反映済み | ⚠️ センサ搭載位置も改修で変更 — 実測・URDF 反映は未 (§10) |
| 減速比 | 物理 **92.25:1** = `gear_ratio: 92.25` (08-30 減速機交換、旧 5.0) | モータ/エンコーダは同一個体 → bus.yml `scale_pos_from_dev = 2π/1024` 据え置き。タイヤ 1 回転 = 94,464 inc。⚠️ 最高速はモータ 7720 rpm 制約で車体 **~1.12 m/s**。⚠️ 減速機交換で出力軸回転方向が反転している可能性 → invert_left/right の浮かせ確認が未 |
| 2D LiDAR | HOKUYO UTM-30LX (USB, urg_node) | frame `laser`, (0.067, 0, 0.09406) — 08-31 実測。**上下反転取付 → URDF roll=π** (⚠️ 反転軸 roll/pitch で前方 180° 差 — /scan 要確認) |
| 3D LiDAR | Sure-Star R-Fans-16 = **V6K-16G GM 格式** (Ethernet 192.168.0.3, `surestar_rfans_ros2` — 08-14 刷新) | frame `rfans`, PointCloud2 `/rfans_driver/rfans_points` (旧 `/sdk_could` は廃止)。**08-31 車体改修で正立 (flat) 取付に戻した** — xyz 実測 (0, 0, 0.80246)、live プリセット = flat。tilted45/15 プリセットの xyz は換算値 (切替時要再実測) |
| IMU | **自作 BNO086 ボード** (USB CDC, `bno086_imu_driver` → `/imu/data`) を採用予定 (08-02 受領) | RealSense IMU 経路を置き換える本命。製作者 (fTomo-robot) の未公開 repo の先行コピー → 後日 fork + submodule 化予定。**08-10 実機初疎通 OK** (/dev/ttyACM0 開通・auto tare 発火・/imu/data publish 確認)。**取付向きは 08-11 の実走 bag (ekf_test) で最終確定: rpy=(0,0,π/2) = 正立+90°**。証拠 3 系統: 静止 60 s の accel z=+9.815 (Z 上向き)・旋回中の gyro_z/車輪 odom yaw rate=+1.015 で同符号率 100%・裏返し URDF で走った EKF の yaw が車輪 odom の鏡像 (-79° vs +74°)。目視 (矢印) ベースの判定は 2 転 3 転したため、以後の向き検証はデータ照合を正とする。`realsense_imu.launch.py` は完全移行確定まで温存 (⚠️ 同時起動すると /imu/data が衝突) **09-24: 取付は上下逆 (部品面が下) + yaw −93°、URDF rpy=(π,0,−1.625736) に修正済み。旧「正立 +90°」は auto_tare が座標を回していたため見えなかった。accel x バイアス +1.12 m/s² あり (未対処)** |
| 比較用 IMU | **WITmotion WT901C-TTL** (9 軸、USB-TTL = Prolific PL2303GC 067b:23a3 → `/dev/ttyUSB0`、udev 導入後 `/dev/ttyUSB-wt901`) → `witmotion_ros` → `/imu_wit/data` (frame `imu_wit_link`) | 09-23 導入。センサ設定 115200 / 200 Hz / 帯域 98 Hz (不揮発)。BNO086 との同時記録用、URDF 未登録 (取付位置未実測) |
| ゲームパッド | Xbox (joy + teleop_twist_joy, LB=deadman, RB=turbo) | |

## 3. ソフトウェア全体像

```
joy_teleop → /joy_vel (優先 100) ┐
Nav2(RPP)  → /cmd_vel  (優先 10)  ┴─ twist_mux (bringup 常駐, 10-16) ── Twist /robot_speed_cmd  ← teleop_keyboard は直書き (台上専用)
        ▼
epos4_controller ─ IK・CiA402状態管理・100Hz TPDO(0x60FF rpm)
        ▼
ros2_canopen Cia402Driver ×2 (bus.yml: sync 50ms, PDO)
        ▼                    ▲ joint_states (motor rad / rad/s ※velocity は常に0 → odom twist は位置差分算出 08-11)
EPOS4 ×2                     │
        epos4_odometry ── 2topic を ApproximateTime 同期 → /odom + TF + /joint_states
        robot_state_publisher ── URDF (rerobot.urdf — 08-10 に 2d/3d を統合)
        slam_toolbox (mapping) / map_server+amcl+Nav2 (走行)
```

- パッケージ (再編後の ros2_ws_main/src/): `app/epos4_controller` (controller+odometry), `app/epos4_teleop`,
  `bringup/rerobot_bringup` (launch/config/urdf/rviz), `bringup/realsense2_camera_launch`,
  `drivers/` = epos4compact50-5can + StarROS2 + realsense-ros + surestar_rfans_ros2 (すべて submodule 直置き。surestar_rfans_ros2 は 08-14 新設、ROS2 移植まで COLCON_IGNORE 中)。
  ※ `epos4_vel_ros2` (ベンチ用単体テスト) は 07-26 に**削除済み** — 必要なら `v1-monolithic` タグから復元可。
- 詳細な規約 (トピック名・スケーリング・ros__parameters の罠) は CLAUDE.md 参照。
  (監査 Issue 16, 17 の古い記述は 2026-07-26 の CLAUDE.md 更新で解消済み。)

## 4. 何がどこまで動いているか (到達点)

**動く (実機検証済み)**:
- teleop (キーボード/Xbox) での走行。脱力(フリー)モード `f` トグル (docs/features/2026-06-05)。
- 起動時の EPOS4 自動初期化 — init レース修正済みで 10/10 成功 (docs/report/2026-06-04)。
- slam_toolbox での地図生成 (9 号館の map 取得済み → maps/ はホスト側、リポジトリ外)。
- **Nav2 自律走行が成功している** (2026-06-12): 2D Pose Estimate → Nav2 Goal で自律移動。keepout フィルタ込み。
- R-Fans-16: 点群取得 (z=0 問題・低 fps 問題は解決済み — docs/report/2026-06-13 ×2)。
  08-01 に起動直後 SIGABRT (libstar.so の例外ランタイム横取り) を LD_PRELOAD で解決し、実機で
  `/sdk_could` 開通を再確認 (実測 ~6 Hz / 13,035 点/msg @ rps=10。旧記録の 20 Hz との差は要観察 — report 2026-08-01 のフォロアップ TODO)。

**構成はあるが未完/未接続**:
- **リポジトリ再編がほぼ完了** (`feat/workspace-split`, 07-26): 機能別 workspace + コンテナ分割。
  分割基準は「依存の壁」— **Nav2 は main に統合** (apt 同士で衝突しないため)、slam_toolbox は別コンテナ (ユーザ判断)。
  - コンテナ: `rerobot_env` (main: モータ+ドライバ+Nav2+teleop) / `slamtoolbox_env` / `glim_env` (公式イメージ) / `liosam_env` (凍結)。
    全て `network_mode: host` + `ipc: host` + `ROS_DOMAIN_ID=150` (compose の environment が正 — exec bash -c は .bashrc を読まない)。
  - 起動は `scripts/*.sh` (bash 常駐 + docker exec 方式): can_up / build / bringup{2d,3d} / nav2d / slam2d / glim3d / teleop / stop。
  - slam 資産は `rerobot_slamtoolbox` パッケージに分離済み (`ros2 launch rerobot_slamtoolbox slam.launch.py`)。
  - main の build.sh に `CMAKE_EXPORT_COMPILE_COMMANDS=ON` (ホストエディタの参照エラー対策の布石)。
  - ✅ イメージ 4 種ビルド/取得済み (main 6.91GB / slamtoolbox 3.42GB / liosam 5.19GB / glim 公式 3.83GB)。
    colcon ビルド成功: main 9 パッケージ + slamtoolbox 1 パッケージ。旧 volume・旧イメージ (7.95GB) は削除済み。
  - ⚠️ **重いビルドで PC が落ちる** → build.sh は BUILD_JOBS=2 + sequential + nice、イメージビルドは
    `build.sh images` で 1 本ずつ直列。`docker compose build` を引数なしで叩かない (CLAUDE.md 鉄則に明記)。
    colcon はホスト実行禁止 (Humble 混入 + git 汚染) — .gitignore にも build/install/log の保険を追加。
  - **残: 起動検証 (vcan/実機) → main へマージ・空 ros2_ws_nav2 削除**。再編一式はコミット済み (0e8cb39, push 済み)。
  旧構成は `archive/monolithic` ブランチ + タグ `v1-monolithic` (= 2686cd9) に恒久保存済み。
  `feat/claude-optimize` (3D 自律移動 bringup 9 コミット) は**破棄決定** (ユーザ判断)。
- **3D SLAM は GLIM (koide3) を採用する方針** (07-26 決定)。当初は IMU 入手不可のため
  `odometry_estimation_ct` (CT-ICP, IMU レス) で構成 → **08-11 に BNO086 前提の IMU あり LIO 構成へ切替済み**
  (`config_odometry_cpu` + sub/global mapping `enable_imu: true` + `T_lidar_imu` を URDF から算出。実点群+実 IMU での検証は未 —
  bag 録画と LiDAR⇔IMU のタイムスタンプ整合確認が先)。`koide3/glim_ros2:jazzy` 公式イメージあり (CPU 構成は CUDA 不要)。
  **GPU 版 (CUDA / VGICP_GPU) は 10-07 に「構成はあるが未ビルド・未実行」**: profile `glim_gpu` の別コンテナ `glim_gpu_env` として
  CPU 版と分離 (同じ `Dockerfile_glim` を `ARG GLIM_BASE=koide3/glim_ros2:jazzy_cuda12.5` で差し替え、入口 `config.gpu.json` を
  入れ子 bind で `config.json` に被せる)。GPU 用 3 本の config は上流デフォルトのままで CPU 側の調整は入っていない
  (`create_between_factors` CPU true / GPU false、`randomsampling_rate` 0.2 / 1.0 等) → **CPU 版との比較は条件を揃えてから**。
  ⚠️ R-Fans 点群の `timeflag` フィールドは GLIM 非認識 (認識名は `t`/`time`/`time_stamp`/`timestamp`) →
  擬似タイムスタンプにフォールバックし deskew 劣化。GLIM は GTSAM 4.3a0 要求で `ros-jazzy-gtsam` 4.2.0 と競合 → 別コンテナ運用。
- 3D 構成 (`rerobot_bringup_3d.launch.py`): 点群は出るが **/scan が無いので SLAM/Nav2 に繋がらない** (GLIM 導入で解消予定)。
- LIO-SAM: submodule として回収済み (07-11, ros2 branch, T13 解決)。ビルドは通るが **IMU 入手不可のため当面凍結**。
  `realsense_imu.launch.py` は IMU 再入手後の GLIM CPU (LIO) モード格上げ用に温存。
- RViz での R-Fans 点群表示手順が未確立 (triage T12 に手順記載)。
- **R-Fans ドライバ刷新 (08-14): 統合完了 — 現行スタックは surestar_rfans_ros2**。bringup lidar_3d 切替 (LD_PRELOAD 撤去)・点群 topic `/rfans_driver/rfans_points` に統一 (`/sdk_could` 廃止、GLIM config/scripts 追従)・旧 StarROS2 は COLCON_IGNORE (参照専用)。bringup_3d 経由の実機検証済み (10.1 Hz)。残: 縦角再較正 (`vangle_override` へ) と屋外 GLIM 評価。総括: `docs/issue/2026-08-14_rfans_driver_renewal.md`。
- **GLIM 3D 地図→Nav2 接続 (Phase 2, 08-17 実装)**: 地図変換 (`tools/pointcloud_to_2dmap` + `pcl_ply2pcd`) と
  /scan 化 (`rfans_scan.launch.py`) は bag/配信レベルで検証済みだが **AMCL 込みの実走は未**。
  使い方・逆引きは `docs/text/map3d_to_nav2/07_checklist.md`、設計は `docs/features/2026-08-17_glim_map_to_nav2.md`。
  ⚠️ 運用規約 2 つ: 高さ帯 (0.3〜1.5) は変換と rfans_scan の 2 箇所セットで変える /
  3D 運用時は `lidar_2d:=false` (urg_node と /scan 衝突)。本格案 (3D ローカライザ) は **10-10 に実装 (構成はあるが未検証)** — `nav_lidar3d.launch.py map_name:=<名>` (lidar_localization_ros2 v1.3.0 + 橋 `map_level_bridge.py`)。ビルド・橋の z 打ち消し・初期位置中継・合成地図での lifecycle active まで確認 (`docs/features/2026-10-10_lidar_localization_3d.md` §6)。**10-11〜12: 5号館地図 `2026-10-09_1643_5goukan_chal` (2D/3D 同名) で bag 再生評価済み — 3D は独立 bag でも 0.07 m・見失い 0、2D は東端の角で見失う (`tools/loc_eval/README.md`)。実機は未。**
  **08-20 追加: z ドリフト地図は `tools/glim_dump_to_2dmap/` で変換する** (GLIM dump 直読み +
  センサ相対スライス、numpy のみ。5号館 LC 地図で絶対 z 方式の「遠方の壁全滅」を回避。
  読本 map3d_to_nav2 §3.4 / §7.1b)。
  **09-10 追記: 解像度は `-r 0.10` を推奨** (5 cm では GLIM の 0.3 m 間引きに対し画素が細かすぎ、壁が点線・submap 間隔が開く区間で消える。濃さの不均一は GLIM 側 (間引き粒度・submap 間隔・姿勢傾き) 由来で、変換引数では完全には埋まらない — 読本 §3.4 の機構 (1)〜(3) と次の一手)。
  **09-14 追記: 生密度が要るなら `tools/glim_traj_to_2dmap/` (bag 生スキャン × dump の `traj_lidar.txt`、LC 反映済み、30 s/7,000 スキャン)。推奨 `--height_frame ground 0.3..1.5 --range_max 30 --deskew` 点数 4..12。方式間の AMCL 実走優劣は未検証 (読本 §3.5)。**
- **BNO086 IMU** (08-02 統合 → 08-10 実機初疎通 OK): `/dev/ttyACM0` 開通、`/imu/data` publish 確認 (auto tare 発火も確認)。
  bringup への組み込みも完了 (`rerobot_bringup.launch.py imu:=true` / `_imu` 系ラッパ)。搭載位置は実測反映済み (0, 0, 0.64)。
  **取付向きは 08-11 の実走 bag で最終確定・URDF 反映済み**: rpy=(0,0,π/2) (正立+90°)。⚠️ 経緯 (2 転 3 転した):
  08-10 の 240 s データ照合は「正立」→ 08-11 朝にユーザが矢印目視で「裏返し」と修正 → 同日の EKF 実機テスト bag
  (静止 accel z=+9.815 / gyro_z⇔車輪 yaw rate 同符号 +1.015 / EKF yaw 鏡像) で**正立が確定**し裏返しを棄却。
  目視判定は imu_check.rviz の表示バグ (fixed_frame_orientation, 08-11 修正) と混線していた可能性が高い。
  **今後、取付向きの検証は必ず実データ照合で行う** (静止重力 + 旋回ジャイロ vs 車輪 odom の 2 点で一意に決まる)。
  **報告レート (09-13)**: 08-31 の 4 bag で `/imu/data` は 100.0 Hz・間隔中央値 10.03 ms・1〜99 %ile 9.3〜10.6 ms と安定
  (欠落は長 bag で 20 回前後の 1 サンプル遅れ、0124 に 320 ms の穴 1 回)。基板 firmware の実用上限は 200 Hz (README 実測、
  250 以上は SHTP 取得 ~800 pkt/s 飽和で逆に落ちる) → bringup に `imu_rate` 引数 + scripts に env `IMU_RATE` を追加し、
  `glim3d.sh` は LIO deskew 用に 200 固定 (実機 199.9 Hz 確認)。Nav2/EKF 系は 100 のまま (EKF 30 Hz で間引くため差なし)。
  ⚠️ 0124 bag だけ「受信時刻 − header.stamp」が常に約 33 ms (他 3 bag は 0.4 ms) — 原因未調査、時刻オフセット議論では条件が違う bag として扱う。
  → EKF 融合 (`ekf:=true`) + GLIM LIO 構成は 08-11 実装済み。EKF 初回実機テストで /odom twist=0 の実装バグも発見・修正済み (§5)。
- **比較用 IMU WT901C-TTL (09-23)**: ドライバ `witmotion_ros` 単体で実機 200.5 Hz・静置 accel z +9.75・stamp 5 ms 刻みを確認 (`ros2 launch rerobot_bringup wt901_imu.launch.py port:=ttyUSB0`)。**BNO086 と同時起動しての bag 記録は未** (次回実機)。⚠️ SIGINT 終了時に `std::system_error` で abort するが終了時のみ (データには無影響)。udev rule `tools/99-wt901.rules` は未導入 (sudo 要)。コンテナ再作成時は Qt5 が消える → `build.sh images` (main) が必要。
- **BNO086 の物理向き・auto_tare (09-24)**: `auto_tare: off` 適用 (`rerobot_bringup/config/bno086.yaml`)。tare なしの実測で **上下逆 + yaw −93°** (URDF 修正済み、GLIM T_lidar_imu 再計算済み、精度 yaw ±2.5°/roll-pitch ±1° = WT901C 相対)。上記 08-11 の「取付向き最終確定 rpy=(0,0,π/2)」は **tare 後の値に基づく誤り**として撤回 (3 証拠は全て Rz 不変量・tare で正立化)。accel x バイアス +1.12 m/s² は未対処。過去 bag (〜09-23) は tare-all 座標なので新 URDF/T_lidar_imu は適用不可 (`flat_tareall_legacy.json`)。

**品質面の課題**: SLAM が「絶妙にずれる」、低速でハンチング、Nav2 走行が遅い — いずれも
オドメトリ/エンコーダ問題 (§5) が根っこにある可能性が高い。

## 5. 既知の問題 — どこを見ればよいか

**docs/issue/ が問題管理の場所** (1 issue = 1 ファイル、冒頭にステータス、解決したら更新する運用)。

| ファイル | 内容 |
|----------|------|
| `2026-10-25_rfans_range_comb_ghost.md` | **本命 (critic 査読済み)・発生段はセンサ本体が本命・設定依存は未検証**: 木の幹・柱の周り +0.5〜1 m の筒 = R-Fans 生 UDP パケットの段で既にある単発の距離偽点 (約 0.686 m の倍数ずれ、生点群の約 1〜3%)。08-12 旧ドライバ時代から存在。地図ゴミ全体の約 4% のみ。実機検査 (§7: 電源投入直後 / rps 10・20・5・10 を tcpdump) で設定依存を判定予定 |
| `2026-10-15_keepout_corridor_stall_imu_yaw_jump.md` | **原因特定済み・対策実施済み (10-16)・実走未検証**: 10-15 自律走行 6 bag。狭窄通路の永久停止 = Nav2 KeepoutFilter は inflation 後に lethal を直書きして膨張されず、NavFn が縁から 0.35〜0.45 m を通し footprint 半幅 0.325 が乗って RPP collision ahead (交絡 = AMCL σ_y 0.56 m)。`transform_tolerance 8.0` は Nav2 に 8 s 古い補正を返す副作用 (1802 の 2.97 m 跳びが 7.8 s 遅れて 103 abort)。bag 1728 は lifecycle_manager_keepout の get_state 2 s タイムアウトで info server 未 Activate → keepout 無視の経路 (起動レース)。BNO086 yaw 跳びは内部再収束。**10-16 対策: keepout を StaticLayer 化 (⚠ `use_maximum` は costmap 直下で読む Nav2 仕様)、tol 2.0、`map_odom_holder.py` (AMCL 無音時の map→odom 保持)、waypoint_runner 自動再送 1 回、info server 撤去 (§8)** |
| `2026-09-26_rfans_frame_stamp_bundle_quantization.md` | **原因特定済み・修正実装済み (submodule 未コミット)・bag 再生で検証済み・live 未確認**: R-Fans フレーム stamp が UDP 32 個 bundle の到着 (40.9 ms) に量子化され、隣接間隔が 83/123 ms 交互 = 真のスキャン開始から ±20 ms 揺れる。GLIM の `imu_rate stamp does not cover (~17 ms)` 警告・残留時刻オフセット −15 ms と一致。修正 = bundle 先頭 UDP の到着時刻と機器 µs カウンタの offset (32 bundle の最小) で stamp を機器時計に anchor → 再生で 100.1 ± 0.5 ms |
| `2026-09-24_bno086_auto_tare_rotates_motion_outputs.md` | **原因特定済み (09-24、因果実験 ×2 + critic)・対策未適用**: BNO086 の `auto_tare: all` (ドライバ既定) が accel/gyro に起動時姿勢の定数回転 R_tare を掛ける。`/imu/data` は物理センサ座標ではなく、yaw は起動方位依存 (今日 24°)。GLIM の T_lidar_imu が bag ごとにずれる。対策 = `auto_tare: off` → 物理 yaw 実測 → URDF/T_lidar_imu 更新 |
| `2026-07-07_repository_audit.md` | 全体監査の 25 issue 一覧 (サマリ表付き) |
| `2026-07-07_wheel_odometry_encoder_scaling_4x.md` | **解決済み (07-29)**: エンコーダ分解能 4 倍ズレ。実測で原因確定 (EPOS 側 256ppr は正しい / raw tpdo 無スケール / 実車は指令の 1/4 速だった) → bus.yml 2π/1024 + gear_ratio 5.0 の 2 点同時修正を適用、浮かせ検証 OK。**残: 接地での 10 m 直進・360° 旋回の最終確認** |
| `2026-07-07_monthly_2026_6_todo_triage.md` | monthly の TODO 14 件の原因分析 + 「複雑なコード」の経緯と簡素化案 (C1〜C3) |
| `2026-07-07_monthly_2026_6_todo_triage.prompt.md` | ↑を実行する次エージェント用の作業指示書 (A 群=コードのみ, B 群=実機必須) |
| `2026-09-19_epos_stop_link_loss_rpdo_timeout_power.md` | **調査中・critic 査読済み (09-19 ×2) → 09-22 夜に 5 回目 (勝手に停止 + DSR 橙、録画外・コンテナ削除でログ消失、§11。対策 = ROS_LOG_DIR を volume へ / 蓋閉じサスペンド無効化)**: harness v2 で EPOS 停止 ×多数 + 暴走 1 回。**初のフォルトコード**: 15:35 = **0x8250 RPDO timeout** (USB リンク喪失 → PDO 途絶 → EPOS 自衛 quick stop、確定)、午前 = 0x3220 (非常停止 SW 断線、確定・修理済)。14:57 暴走 = 受信側リンク喪失 + パッド瞬断 + controller に指令タイムアウト無し (本命)。15:03・段差頻発 = リンク喪失 / **RSD-60L-24 (60 W) の定電流折り返し** / SW 劣化で未識別。代替 H1〜H5 (PC ジッタ・アダプタ CAN 側・lely wedge・機械振動・RSD) 列挙済み。**次: candump 常時記録 → 0x2200:01 ポーリング段差実験 (H5) → 機械/電気 A/B (H4) → ユーザ確認 (暴走秒数・LB・RViz 凍結)。ソフト対策 = Twist 受信ウォッチドッグ (最優先)・fault 監視 0x1003 ログ・can0 再作成検知・0x60C2 20〜30 ms** |
| `2026-08-11_joy_spin_epos_shutdown_usb_stall.md` | **対策適用済み・再発観察中 → 09-19 に機構を類推 (0x8250)**: 急速度変化で EPOS 停止 + CANUSB が USB ストール (urb -32)。09-19 15:35 の同署名事象でコード 0x8250 を取得 → 同機構と類推 (当時コード無し)。当時の「電源巻き添え」診断は格下げ。復旧手順は確立 (can_up.sh の stale-slcand 検出込み)。**08-30 に EPOS 側ハードニング適用** (0x60C5/0x6085/0x6084=30000 rpm/s・電流 10 A、SDO 裏取り済み)。⚠️ Error History は未確認のまま Clear された可能性 — 8/11 事故の原因コード確定は不能に。**残: NVM 永続確認 (次回電源再投入時) + PC サスペンド無効化 + 再発時は Error History を最優先で読む** |
| `2026-08-11_utm30lx_usb_instability.md` | **調査中**: UTM-30LX が同日 3 回自然断 → 現在 USB バス上に不在。次アクション = 本体電源 LED / 12 V 供給 / コネクタ点検 (ソフトでは復旧不可) |
| `2026-08-12_glim_horizontal_drift.md` | **原因特定済み・根治未達**: GLIM(LIO) の 3D 地図が水平に塗れる。主犯 = 車輪 odom 不使用 × 廊下縮退。同環境撮り直しは効果なし (08-13) |
| `2026-08-13_glim_param_tuning.md` | **実験完了 (2 ラウンド 12 run)**: 上記への対策候補 3 の掃引。推奨 = E11 (ivox 0.5 + downsample 0.5 + sub opt ON + global voxel 0.25 + sampling 0.5、塗れ −34.5%)。**repo config 未反映 (ユーザ判断待ち)**。屋外での評価は未実施 |
| `2026-08-15_offline_viewer_zenity_dialog_hang.md` | **解決済み (08-15)**: offline_viewer の 0% ハング・ファイル選択不能。真因 = zenity 4 (GTK4) の GL 描画が CRD X で失敗しダイアログが不可視のまま回答待ち。修正 = compose glim に `GSK_RENDERER=cairo` (適用・検証済み) |
| `2026-08-16_rfans_mount_angle_glim_z_collapse.md` | **原因特定済み**: 08-15 取付角実験 (45°/15°) の GLIM 崩壊。15° = 縦角系統誤差の z 投影増幅 (+17〜25 m、extrinsic 誤差説は因果実験で棄却)、45° = 近距離地面偏重の照合縮退。**対策の要 = 縦角再較正 (§10-5(f))、それまで傾け取付評価は保留・flat が最小被害** |
| `2026-09-10_glim_z_drift_not_vangle.md` | **09-23 追記 §14: 外周 k=20 評価 — k=20 は外周でも A −31%/rms −54%、roll 歪み = global 段の submap ブロック傾斜 8〜11° (pose-graph global で 0°、global enable_imu=false で 22° → 駆動源は submap 間再登録、IMU 因子は弱いブレーキ。acc_scale 0.985 は欠落時 z 飛びを −2.6 → −1.2 m に半減、A には無効)。** **調査完了・critic 査読済み (09-10)**: 「z ドリフト = 縦角表誤差」を否定検証 → **主犯ではない** (縦角補正で z_end が run 間ばらつき SD 0.9 m 内。≤30% 寄与は追加 run で判定中)。z(s) = 世界固定傾き A (3°→新車体 13°) + 一定沈降 c、**A の発生源は未特定** (候補: 残留時刻オフセット・LiDAR–IMU extrinsic 回転・初期重力整合。屋外の z 観測性欠如は増幅条件)。新車体 bag は加えてモータ脱落窓で −48 m (確定)。08-14/15/16 の「縦角が本命」は冒頭追記で格下げ済み。E11 tuning の屋外評価は OOM (16 GB) で未完。**→ 09-20 (§12): 9/18 2 周 dump で壁抜けを解析 — 周回を結ぶ因子 0 本 (確定)・z_end は指標にしない・壁抜け = 同一周内にじみ (3〜6 m) + 周間 +2.6 m・A が周を跨いで成長 (bias 正帰還仮説)。次: viewer find_overlapping 0.1 → fix_imu_bias run → k=20 の非近接因子数**。**09-20 (2)**: 手動閉合 dump で周間だけ整合・山不変 (§12.7、予測どおり)。外周 bag の 11.5 / 32 m ジャンプ = R-Fans 回転ディップの底 5 s を IMU 単独予測で埋めた結果 (§13、確定)。IMU acc bias x +0.25 ≈ pitch extrinsic 1.5° 未補正の疑い → 静止較正を新車体で再実施 |
| `2026-09-02_rfans_scan_motor_dropout.md` | **調査中 (一次原因未特定) → 09-20: 10 Hz でも発生 (9/19 外周 32 分で 2 回、9/18 5 号館 30 分で 0 回) → 「20 Hz 運転が引き金」説は棄却**。定型の惰性減速 (30k → 146k 点/frame) → 底 5 s → 20 Hz オーバーシュート → 整定 ~7 s。rps watchdog は無関係。GLIM オフライン地図に 11.5 / 32 m ジャンプを作る (GLIM issue §13)。**次: bag/ドライバの「width > 40000」で検知、R-Fans コネクタ直近の電圧ロガー (前倒し)、残る候補 = 経路 (悪路振動)・バッテリ末尾・側枝コネクタ** |
| `2026-08-30_chassis_rework_followup.md` | **未完 (一部消し込み済み)**: 車体改修 (gear 92.25 / tire 0.256 / tread 0.529) の追従編集箇所一覧 — トレッド・センサ位置の実測後に触るファイル:行番号、回転方向確認、検証手順。**§10-0 の実体はこのファイル** |
| `2026-08-14_rfans_driver_renewal.md` | **新ドライバ完成・実機検証済み / 統合待ち**: R-Fans ドライバ刷新の総括 (旧 6 事件の年表・新旧比較表・0x37 謎解き・A/B 床メトリクス)。旧ドライバの fold16 破壊を bag で定量証明 (同一ビームの床が 2 高度)。**縦角表不一致は新旧共通で未解決** → `vangle_override` 受け口実装済み、新ドライバで再較正が次 |
| `2026-09-13_decel_epos_stop_12v_loss.md` | **調査中 → 09-19 に配線 v2 化で前提消失 (再現不能・極性未識別のまま凍結)**。以下は v1 時点の記録: **critic 査読済み (09-13)**: joy turbo を 0.5→2.0 m/s に上げた直後の初停止で EPOS ×2 停止 + LiDAR 給電 12 V 系 (UTM + R-Fans 共通 DC-DC) が喪失し復帰せず (同型 4 回目、初の部品損傷)。本命 = 「回生時に BMS が充電方向を遮断 → 孤立母線が跳ね、DSR 50/5 が **JP1=56 V 設定 (実機確認)** で受けられず DC-DC (36 V 上限) が過電圧」の複合説。極性 (過電圧/瞬断)・EPOS フォルトコード (ros2_canopen は EMCY をログに出さない + 0x1003 揮発)・DC-DC 生死・バッテリ本数 (1 本/2 本直列) が未確認。**次: DC-DC テスタ → バッテリ本数 → 0x2200:01 ポーリング浮かせ実験。対策は本数で分岐 (1 本: JP1 open 27 V / 2 本: DSR 70/30)。実走系は 12 V 復旧まで全停止** |

**優先度の要約**:
1. 🔴 **安全系 (監査 Issue 1〜3)**: /robot_speed_cmd ウォッチドッグなし・Ctrl-C で disable 不達・速度平滑化なし。
   monthly の TODO には無いが**本番前必須**。
2. ~~🔴 エンコーダ 4 倍ズレの根本修正 (B1)~~ → **07-29 適用・浮かせ検証済み** (残: 接地確認)。「Nav2 走行が遅い」は 1/4 速が原因でほぼ確実に説明がつく。ハンチング (T10)・SLAM ずれ (T11) への効果は接地後に再評価。
3. 🟡 LiDAR FOV ±90°→±135° (A1)、slam_toolbox lifecycle 整理 (A2)、脱力モードの競合修正 (A4)。
4. 🟡 ROS_DOMAIN_ID=150 → 101 以下へ (discovery が散発的に失敗しうる)。

## 6. これまでの経緯 (タイムライン)

**本節は 2026-10-23 に [docs/monthly/](../monthly/00_index.md) へ移管した。** 月ごとの `2026_MM.md` に「親 = 大まかな修正・調査テーマ / 子 = 個別の変更 (変更ファイル・コミット・docs へのリンク付き)」のツリー表で全履歴 (2026-03〜) がある。
新しい出来事は本節ではなく当月の `docs/monthly/2026_MM.md` に追記する (手順: `/monthly-changelog` skill。Stop hook が当月コミットの未反映を 7 日で催促する)。
直近の到達点は §4、既知の問題は §5 + [docs/issue/INDEX.md](../issue/INDEX.md)、次の作業は §10 を見る。

## 7. コードを触るときに知らないと踏む罠 (経緯由来の知識)

- **TimerAction の period / 発火後に実行する Node の parameters に `LaunchConfiguration` を直接書かない** (10-16)。その launch が `GroupAction(scoped=True)` で include されると、タイマー発火時にはスコープが閉じていて SubstitutionFailure → 例外が asyncio Task 内で捨てられ、**launch が Ctrl-C で終了しなくなる**。OpaqueFunction で visit 時に `perform()` して素の値で組み立てる (`rerobot_bringup.launch.py` の `_delayed_check` が実例)。

1. **CiA402 遷移サービスは並行発行すると壊れる** (実機で 2 回踏んだ)。init→enable→csv は必ず逐次。
   復帰時に `init` (homing) を呼ぶと復帰不能。サービス戻り値は no-op 遷移で偽陰性 → 成否は SDO
   (0x6041/0x6061) で判定。この 3 つの罠が epos4_controller の「複雑さ」の正体 (triage C1)。
2. **joint_states.velocity は常に 0** (07-29 確定: bus.yml で 0x606C 未マップのため driver のキャッシュが更新されない。SDO read 0x606C を打った直後だけ一度反映される)。回転判定は position 差分で行う。⚠️ これを踏んで `epos4_odometry` の twist が終始 0 だった (velocity 配列の非空チェックだけで採用していた) — 08-11 に位置差分 (d_s/dt) ベースへ修正済み。velocity 配列を「存在するから」と信用しないこと。
2'. **raw `tpdo` トピックは無スケールで素通し** (07-29 candump で実証)。controller の rpm 値がそのまま 0x60FF に届く (EPOS の Velocity Unit が 1 rpm なので単位は一致)。⚠️ 07-29 の根本修正 (gear_ratio 5.0) 以前は実車が指令の 1/4 速だったため、**それ以前に較正された速度系チューニング (teleop スケール・Nav2 速度上限/加速度) は「1/4 速の実車」基準** — 接地検証で要見直し。
3. **「ちょうど 4 倍」のズレを見たらクアドラチャ 4 逓倍の二重解釈を疑う** (encoder issue の教訓)。
4. タイヤを浮かせた状態の挙動 (ハンチング等) は実走行の参考にならない。評価は接地で。
5. bt_navigator の plugin_lib_names は列挙すると二重登録 segfault (Jazzy)。デフォルトに任せる。
6. ROS 2 params の `ros__parameters` はアンダースコア 2 つ。1 つだと起動即死で症状から原因が分からない。
7. **CSV モードにはドライブ側の加減速ランプが無い** (07-31 に実機で踏んだ)。0x60FF へのステップ指令は速度ループが本気で追従し、制動電流/回生スパイクで保護作動や CAN 巻き添え死を起こす。滑らかさの責任は指令側 = epos4_controller のランプ (`max_motor_accel/decel_rpm_per_s`)。これを外す/緩める時は EPOS 側 Max acceleration (0x60C5) が安全網として有限値であることを先に確認。「両モータ同時に SDO timeout」はドライブ単体でなくバス/電源レベルの死のサイン。
7. bus.yml は submodule (`src/external/epos4compact50-5can`) 内 → 変更は submodule 側にコミットし、親で gitlink 更新。
8. **[旧ドライバのみ — 08-14 刷新で現行構成では不要化] rfans_driver (StarROS2) は LD_PRELOAD 必須** (08-01)。プリビルド `libstar.so` が古い C++ 例外ランタイムを export しており、素で起動すると同一ドメインに他ノードがいるだけで無言 SIGABRT する。launch 経由なら `additional_env` で自動適用済み — `ros2 run` で直接起動する時は手動で `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/lib/x86_64-linux-gnu/libgcc_s.so.1` を付ける。launch 前に `./scripts/stop.sh` で残留ノードを掃除する習慣も予防になる。
9. **BNO086 を fork+submodule 化するとき `firmware/COLCON_IGNORE` を fork 側にコミットし忘れると build.sh main が再び壊れる** (08-02)。vendored コピー内へのローカル追加はディレクトリ差し替えで消える。症状は「STM32 HAL/CMSIS がパッケージ誤認識 → colcon 全体停止」で、"前は動いてた" 形で再発する。
10. **`scripts/build.sh` は端末なし (自動化・Claude) から呼ぶと `docker exec -it` が stdin エラーで失敗するのに exit 0 を返す** (08-02 発見)。何もビルドされずに成功に見える。非対話でビルドする時は `-it` を外した `docker exec` で colcon を直接叩く。
11. **中国ベンダのソース (GBK/ISO-8859 エンコード) は grep がバイナリ扱いして黙って 0 件を返す** (08-14 に踏んだ: surestar_rfans_ros2 の ssFrameLib.h で構造体定義が「存在しない」と誤診し PROJECT_STATE に誤記録→撤回)。ベンダファイルを調べる時は先に `file <path>` でエンコード確認、grep には `-a` を付ける。「grep 0 件」は「無い」の証明にならない。
12. **BNO086 の `/imu/data` は既定では物理センサ座標ではない** (09-24 発見)。ドライバ既定 `auto_tare: all` が起動 2 秒後 (+ reset / USB 再接続後) に全モーション出力へ定数回転を掛ける。静止 accel が (0,0,g) に見えても取付が水平とは限らず、傾けテストで yaw が「ずれて見える」のは取付ではなく tare の可能性を先に疑う。gyro_z・accel z 符号・EKF yaw は Rz 不変量なので取付 yaw の検証には使えない。判定法 = 傾けたまま `~/reset` → 傾きが消えれば tare。

13. **rclpy の `StaticTransformBroadcaster` は同じ child_frame_id の 2 回目以降を捨てて最初の値を再送する** (10-10、Jazzy tf2_ros_py 0.36.22 のソースで確認。C++ 版は置き換える)。値を更新する static TF は `/tf_static` (transient_local, depth 1) へ自前で TFMessage を publish する (`map_level_bridge.py`)。
14. **Nav2 obstacle_layer の高さ判定は global_frame 座標の z** で、`min_obstacle_height` 既定 0.0。`/rfans/scan` (target_frame base_link) の点は z=0 ちょうどなので、map から見たロボットの z がわずかでも負だと global costmap から消える (10-10 に global の下限を −0.3 へ。Nav2 ソースの記憶による、実測は未)。

## 8. docs/ ディレクトリの地図

**[docs/README.md](../README.md) に移管 (2026-10-23)。** 各ディレクトリの役割・編集ルール・索引 (issue / report / features の INDEX.md、text/README.md、monthly/00_index.md) はそちらが正。ここに残す規則は 2 つだけ:

- `docs/claude/USER_LEVEL.md` は個人情報のため **git 管理外・コミット禁止** (更新は `/user-level` skill)。
- `docs/monthly/` の拡張子なしファイル (`2026_6` など) と `docs/trial/improvements.md`・`docs/manual/` 本文は **ユーザ執筆領域** — Claude は依頼時のみ編集。Claude が更新するのは `docs/monthly/2026_MM.md`・`00_index.md` と各 INDEX。

## 9. 作業慣例

- Claude が書いたコードには `// claude` 系コメントタグを付ける慣例がある (既存コード参照)。
- clang-format 適用済みのコードベース。フォーマットを合わせる。
- **main 直コミット運用** (2026-07-26 にユーザ指示で方針変更。それ以前の「ブランチ必須」は 07-07 監査時に Claude が書いた慣例で、ユーザの決定ではなかった)。モータが動く検証は「浮かせて確認 → 接地」の順。
- ユーザは日本語話者。ドキュメント・応答は日本語。
- ユーザの技術レベル感: 詳細は `docs/claude/USER_LEVEL.md` (git 管理外の分野別プロファイル) を参照。
  説明は「なぜそうなるか」まで書くと喜ばれる。丸投げ実装より、経緯と理由を残すことが重視される。
- 返信中の発展的コマンド・専門用語には注釈を付ける (`annotate` スキル — 基準と既知用語リストは
  `.claude/skills/annotate/SKILL.md`。「知ってる」と言われた語はリストに追記)。

## 10. 次にやることになっている作業

**🔬 10-26 実施済み: R-Fans 櫛偽点の実機検査 (室内静止)** — C0 (電源投入直後、20 Hz) comb 1.19% (1 s ばらつき 0.019)、C1 (rps 10) 点群で 0.93%。壁の手前・奥に平行な偽の壁が見える。正面 4 m の細いポール 2 本で、ポールの真後ろ +0.69 m・+2.06 m (視線上・同じ高さ・ポールに当たるレーザだけ) に偽点を直接確認 (`poles_side.png`)。C0〜C4 は録り直し済み (`bags/rfans_check/2026-10-26/run2/`): 見かけは rps で 0.72/0.92/1.16% と変わるが、点間隔を揃える (列間引き) と 1.13〜1.20% に収まり、C0≈C2 → **critic 後: 室内 1 シーンで rps・ドライバ書き込みの効果は ±5% 以内 (検出されず)、エコー設定 (シングル/デュアル) は未検証**。次 = デュアルエコー交互録り (偽点の相方エコーが真値なら機構 = エコー選択の誤り)。**10-27 デュアル実施 (critic 査読済み、issue §7.7 (c))**: シングルとデュアル (近い側エコー a) で率・k 内訳とも ±2% 以内で同じ。分離能は約 1.9 m (レーザにより 2.6 m)。k=+3 の約 23% (分離能 1.9 m のレーザ・3〜15 m) では偽点自体が最も近いエコー = 2 反射からの選び間違いは不支持。残りと k=±1/±2 は分離窓内選択を棄却できず。次候補 = 分離能の較正 (ポール + 背後の板、間隔 1.8〜2.3 m) か、エコー選択規則コマンドの有無確認。 **10-27 除去フィルタ試作** `tools/rfans_comb_check/rfans_comb_filter.py` (同レーザ前後の直線予測に乗らず予測+k·0.686 に乗る点、ただし上下レーザが同距離なら残す。オフライン Python、ROS ノード化は未): ポール奥の偽点・植え込み横の三日月形の弧は消える (`bags/rfans_check/2026-10-27/{poles_filter,moving_380,moving_300_v3}.png`)。除去 2〜3% (偽点率 1〜1.5% より多い = 植え込みで過剰の疑い)、本物の誤除去率は未測定 (決まった方位で毎回出る偽点があり、フレーム再現性を正解に使えない)。次 = 植え込み対策 → GLIM 前段に入れて地図比較 → ノード化 (別ブランチ)。**10-27 v4**: 細いポールの奥は左右が奥の壁で水平判定に乗らず 2/3 漏れていた → 「同レーザで支え無し + 上下両方のレーザに k·0.686 手前の本物」を追加 (片側だけだと除去 4.3% に膨らむので両側必須)。今日の場所 (S1.pcap 900 回転) でポール奥 584 → 87 個 (85% 減)、全体除去 2.36 → 2.68%、ポール本体の誤除去 0.4%。図 `bags/rfans_check/2026-10-27/clear/compare_poles.png` (pcap → 点群は `scripts/pcap_points.py`)。追加で消える塊 (5.6 m、手前 3.55 m の物体の真後ろ +3P) は k=+3 偽点と見るが隙間越しの本物は未否定。issue §7.7 記入済み。sudo 不要のキャプチャ `tools/rfans_comb_check/rfans_capture.py` (コンテナ root, AF_PACKET) と画像化 `rfans_comb_image.py` を追加。旧記述: — `docs/issue/2026-10-25_rfans_range_comb_ghost.md` §7。ツール `tools/rfans_comb_check/` はこの PC の rerobot_env で動作確認済み (新習志野 bag 100〜130 s で comb_pct 1.17%)。LiDAR 接続時は `nmcli con up rfans ifname enp0s31f6` が要る (プロファイルが NIC 未紐付け)。bag を読ませるときは `install/setup.bash` の source が必須。結果は §7.7 へ。

**🧪 10-24 完了: スタート地点の人垣試験 (擬似注入 bag、2 段階)** — `docs/issue/2026-10-24_start_crowd_sim_localization.md`。第 1 試験 (10〜20 m 環、353 体、ビーム遮蔽 ≤19 %) は現行 AMCL・lidar3d とも差なし → critic 査読で人垣が薄いと判定 → 第 2 試験 (1.5〜8 m・108 体・3 m 超ビームの 79 % 遮蔽・initialpose 横 0.7 m/yaw 10° ずらし)。**結果: 密な人垣では現行 AMCL は正解 init からでも人垣内で 1.5〜2 m (最大 4.9 m) 流れ、初期誤差は人垣なしなら 30 s で直るが人垣内では直らず抜けた後も 1 m のまま。lidar3d は 5 s で 0.1 m に収束し以後 ≤0.15 m (NDT fitness は人垣で変化 = 入力に届いた上で)。beam skip の害は likelihood_field_prob モデル側 (skip なしでも 9〜17 m 振れ)、高帯 2.0 m 地図は広場を出ると成立しない (5 run)。** 運用上の含意: 人垣が 10 m 以内なら initialpose の目視確認では足りない → 人垣の外へ出てから置く / lidar3d。限界: 静止円柱・地図元 bag。次: 独立 bag (10-17_1346) で再評価、動く人垣。副産物: `replay_one.sh PREWARM=1` (AMCL の composite 地図尤度場構築 30 s。判定 = `/rfans/scan_all` 到着 → busy 後 idle 2 s かつ CPU ≥3 s)、`crowd_inject.py --avoid_map`。

**🧹 10-23 リポジトリ全体見直し (調査済み・修正未着手)** — 継ぎ足し開発の重複・残骸・不整合を全ファイル横断で洗い出した。実行時に効くもの (teleop `invert_*` 逆転 / `bringup2d.sh` の EKF 既定 false / nav 構成の不在 `map_dir` 既定 / `imu_check` が `auto_tare: all` のまま) → 構造的原因 (Stop hook の監視パスが再編前のまま = 一度も発火していない、`__pycache__` 20 本が git 管理下、`package.xml` の旧ドライバ依存) → 規約浄化 (CLAUDE.md の `gear_ratio: 5.0` 等 5 件の自己矛盾、skills の 2 世代前の期待値) → 本ファイルの分割 (210 KB、冒頭 2 行で 39 KB。§6 は新設の `docs/monthly/2026_MM.md` に委譲) の順で直す。一覧と直し順は [docs/issue/2026-10-23_repository_review_incremental_cruft.md](../issue/2026-10-23_repository_review_incremental_cruft.md) §1。同日新設: 月別変更履歴 [docs/monthly/00_index.md](../monthly/00_index.md)、ソフトウェア構成図 [manual 第16章](../manual/16_software_architecture.md)。

**直近 (10-16 (3)): 追従型 BT の箱 1 個実走 (約 30 分、0.4 m/s) → 狭路対策の残り 2 点** — 廊下で waypoint 2 点の短い区間を作り、(1) 正面 2 m に全幅を塞ぐ箱 → 初停止距離 (予測 0.6 m) と 0.3 m までの這い寄りを巻尺 + `/lookahead_collision_arc` + `/rosout` "collision ahead" で確認、(2) 箱を 1 m 前へ → 3 s 以内に再発進するか、(3) 箱を片側に寄せ隙間 0.8〜1.0 m → 経路が隙間を通る (= 追い越し防止は BT の仕事でないことの確認)、(4) Start 位置正面を塞いで区間開始 → 約 20 s で区間失敗 → 自動再送 (10 回) が回るか。bag の planner 行間隔で NavFn 失敗探索の重さも見る。結果次第で `min_lookahead_dist` (車間を伸ばす) か nav2_collision_monitor を検討。**残り 2 点 (未実施)**: (a) keepout の閂 — 各分岐で本線以外の枝に短い黒帯 (分岐から 1〜2 m 入った所。膨張 0.55 で本線幅を削らないように) + 歩道/車道境界を黒で埋める、(b) 停止点の待ち行列 — waypoint_runner に停止点の予備候補 (手前 2 m・4 m) を持たせ `/global_costmap/costmap` で占有なら手前に止まる + NavFn `tolerance` 0.5 → 0.25。設計 `docs/features/2026-10-16_follow_bt_narrow_corridor.md` §6。

**直近 (10-16 (2)): 当日運用フローの台上確認 → イメージ再ビルド** — 浮かせて `auto_run.launch.py` を上げ、(a) LB 介入 → 離して 0.5 s で Nav2 復帰、(b) Start で区間開始、(c) Back → Start で残りから再開、(d) RViz を閉じても走行継続、(e) `ros2 topic info -v /robot_speed_cmd` の publisher が twist_mux 1 つ、を確認。済んだら `./scripts/build.sh images` (main) で twist-mux をイメージに焼く (コンテナ再作成で消えるため)。蓋閉じサスペンド (logind `HandleLidSwitch` 既定) は実績がなければ ignore に (ホスト sudo、ユーザ作業)。詳細 `docs/features/2026-10-16_auto_run_joy_mux.md` §6。
**保留 (10-16 夜、ユーザ判断)**: 初期位置を置くまでの数分間、端末ログが流れて表が読めない (実走ログ 1627 行 = controller_server の local keepout StaticLayer が map を引けない ERROR 513 / rviz2 Message Filter INFO 259 / planner TF 待ち 121 / amcl WARN 79 / **bag_record `/particle_cloud` 型二重 ERROR 72 = nav2.rviz の PoseArray 表示が原因で粒子が記録されていない本物の不具合**)。対策案 = (1) navigation 部品 + runner を map→odom TF 検出後に起動 (待ち受けスクリプト + OnProcessExit)、(2) rviz2 を `--log-level rviz2:=warn`、(3) nav2.rviz の粒子表示を `nav2_rviz_plugins/ParticleCloud` に。feature-doc §6 参照。

**🔴 割込み最優先 (09-19): 通信断で止まらない層の封鎖 + 電源容量の確定 — 済むまで急指令・負荷・段差試験は中止、走行は 0.25 m/s turbo なし** (`docs/issue/2026-09-19_epos_stop_link_loss_rpdo_timeout_power.md` §7〜8, §10): ✅ (1) `epos4_controller` の指令ウォッチドッグ・(2) fault 監視 → 0x1003 即読みログ + `/diagnostics`・can0 再作成検知は **09-19 夕方に実装・実機 3 テスト合格** (feature doc `2026-09-19_epos4_controller_watchdog_fault_monitor.md`)。**残: 実機 `/etc/systemd/system/canusb-up.service` に README の `ExecStartPost` (起動後 4 s で受信ゼロなら slcand やり直し) を反映 (sudo) → 抜き差し 3 回で受信が戻るか検証** (sleep 2 だけでは 17:59 に再発)、candump 常時記録の scripts 組込み (未)、bag に `/diagnostics` を含める → (3) **0x2200:01 10 Hz ポーリングしながら段差・0.5 m/s ステップ** (RSD-60L-24 折り返し H5 の実証。落ちれば電源容量問題に分類変更) → (4) 機械 vs 電気 A/B (車輪浮かせ・固定で電流のみ / ロジックのみでコネクタ揺らし) → (5) ユーザ確認 (14:57 暴走の秒数・LB・RViz 凍結、DB9 の CAN GND 結線、ハブ電源元) → (6) ハード: 絶縁 CAN アダプタ or USB アイソレータ、モータ電源の作り直し (A: バッテリ直結 + DSR 70/30 38〜42 V + Max output current 見直し / B: 300 W 級絶縁 DC-DC)。設定衛生: 0x60C2 20〜30 ms、0x6077 ポーリング停止。**運用: フォルト後は主電源を切らず非常停止のみ → `cansend can0 60x#400310{01..05}00000000` で履歴を読む → 電源断**。`docs/reference/wiring_diagram.mmd` を v2 に更新 (未着手)。

**(09-13 の割込みは v2 化で前提消失 → 上に統合。以下は v1 時点の記録)** ~~**🔴 割込み最優先 (09-13): 電源事故の切り分けと 12 V 系復旧 — 済むまで実走系は全停止**~~ (`docs/issue/2026-09-13_decel_epos_stop_12v_loss.md` §6〜7): (1) DC-DC 単独テスタ (出力 12 V の有無) → (2) バッテリ本数・接続の目視 → (3) 0x2200:01 電源電圧を SDO ポーリングしながら浮かせ低速ブレーキ (DC-DC 切離し・EPOS 過電圧限界を一時 36 V) → (4) **バッテリは 2 本直列で確定 (30.7 V 実測、09-13 夜)。JP1 open (27 V) 追試は DSR 常時 ON 170 W の危険状態 → closed (56 V) に戻す。** 対策 = 入力上限 48 V 以上の DC-DC に交換 + DSR 70/30 を 40〜42 V (現 DC-DC 36 V では窓が 2.4 V で設定不能)。中間策 = LiDAR 系を別電源へ。joy turbo は 1.0 m/s 以下へ戻す (未コミット yaml の 2.0 は物理上限超え)。(5) 再発時にフォルトコードを失わない経路 (`/diagnostics` を bag に / statusword fault 監視で 0x1003 即読み)。

**⚠️ 割込み最優先: 車体改修 (08-30) の追従残タスク** — これを済ませるまで実走系の検証は全て無効
(**編集箇所の詳細一覧 = `docs/issue/2026-08-30_chassis_rework_followup.md`** — ファイル:行番号・プリセット規約・検証手順込み):
0. (a) ~~トレッド幅の実測 → params~~ (✅ 08-31: 0.529 m 実測・反映済み) →
   (b) ~~センサ位置の実測 → URDF 全プリセット + GLIM T_lidar_imu~~ (✅ 08-31 (2): 完了・live=flat 切替済み。
   2D LiDAR は上下反転で roll=π — /scan の前方向き要確認。tilted プリセットの rfans xyz は換算値 = 切替時要再実測) →
   (c) **浮かせ確認**: teleop 前進 → 両輪前転か (減速機交換で回転方向反転の可能性 → invert_left/right)、
   車輪 1 回転手回し → teleop 距離表示 ≈ 0.80 m (π×0.256) で較正確認 →
   (d) 次回 EPOS 電源再投入時に SDO 再読で NVM 永続確認 (0x60C5=30000 等。手順はタイムライン 08-30) →
   (e) 接地で低速走行 → 10 m 直進・360° 旋回の再較正確認 (旧チャシスの接地検証残タスクを新チャシスで実施する形に置換)

**最優先: リポジトリ再編の仕上げ** (計画: `~/.claude-school/plans/imu-glim-swift-hartmanis.md`):
1. ~~ws 分割・slam 分離・Docker 4 分割・scripts・イメージ/colcon ビルド確認・旧資産掃除・コミット (0e8cb39)~~ (✅ 07-26 完了)
2. 起動検証: vcan モード (`/verify`) または実機 (`scripts/can_up.sh` → `nav2d.sh`) で旧構成同等の動作を確認
   → OK なら `feat/workspace-split` を main にマージして再編クローズ。空 `ros2_ws_nav2/` の削除も忘れず
3. GLIM 実データ検証: ~~bag 録画 → glim_rosbag 評価 → CT-ICP 比較~~ (✅ 08-12 完了) →
   水平ドリフト発覚 → 原因特定 (08-12) → ~~パラメータ掃引~~ (✅ 08-13 完了、推奨 E2 を提示)。
   **残: (a) 推奨設定 E11 (ivox 0.5 + downsample 0.5 + far 50 + sub opt ON + global voxel 0.25
   + sampling 0.5) を repo config に反映するかのユーザ判断**
   (適用時は屋外用に far_thresh の扱いに注意 — `2026-08-13_glim_param_tuning.md` の diff 参照)、
   **(b) 特徴の多い環境 (屋外, つくば本番想定) での GLIM 評価** — 廊下は縮退の最悪ケースで
   2D 地図は既にきれいなため、屋外で問題なければ「GLIM=屋外用 / 廊下=2D」運用が成立、
   (c) 根治する場合は車輪 odom 融合 (拡張モジュール/API 実装、中規模タスク)、
   (d) **R-Fans 斜め置きの検証** (08-13 発案、親 issue 対策候補 4) — 16 ビームの横縞が移動で
   埋まり壁面カバー率↑ + 進行方向交差構造を拾って縮退にも効く可能性。
   **→ 08-15 に物理取付を実施 (前傾ピッチ 45° + ヨー 180°。発案のロールでなくピッチ前傾)。
   URDF rpy+xyz (実測 -0.02, 0, 0.75) / GLIM `T_lidar_imu` とも反映・ユーザ実機確認済み。
   取付はプリセット化済み (`rerobot.{tilted45,tilted15,flat}.urdf` 等の 3 種、cp 2 発で切替 — CLAUDE.md 参照)
   (タイムライン 08-15 (2))。
   → **08-16 に屋外周回 bag で評価完了 — 45°/15° とも崩壊で斜め置きは当面不採用**
   (45°=照合縮退で並進喪失、15°=縦角系統誤差の z 暴騰 +17〜25 m。
   `docs/issue/2026-08-16_rfans_mount_angle_glim_z_collapse.md`)。
   縦角再較正 (下記 5(f)) 完了後に 15° 系のみ再評価の価値あり**
4. ~~EKF 融合の実機検証~~ (✅ 08-11 完了: 再テストで filtered 正常動作を確認 — 静止 305 s ドリフト 14 mm/-1.1°、
   直進 10.08 m (実測 10 m)、360° 旋回残差 +3.9° (生 odom は 0.1° と車輪較正が優秀)。filtered の旋回残差は
   磁北基準 yaw の影響の可能性 → 気になる場合の調整候補: imu0 の yaw 融合を切って yaw rate のみ /
   `use_game_rotation_vector: true` (磁気非依存))。
   **Nav2 の odom 入力は /odometry/filtered へ切替済み (08-11)**: nav2_params.yaml (bt_navigator +
   controller_server の odom_topic) + nav2d.sh が `IMU=true EKF=true` で bringup する構成に変更
   (bringup2d.sh に IMU/EKF env 変数追加、既定 false で従来互換)。**残: この構成での Nav2 実走行確認**。
   BNO086 repo が公開されたら fork + submodule 化 (⚠️ §7-9: COLCON_IGNORE を fork に含める)
5. **R-Fans ドライバ刷新** (08-14 開始、タイムライン 08-14 (2)): (a) ~~実機 UDP の device ID バイト確認~~
   (✅ 08-14 実測確定: **gmReservedA = 0x5C = V6K-16G** — 実機 3 パケットとも payload[-2]=0x5C。
   新ドライバは無修正で正しいテーブルを選ぶ。**旧 SDK の「dataID 0x37」の正体は隣の payload[-1]=0x37
   (gmReservedB) を機種 ID として誤報告していたもの** — 0x37→0x57 remap ハックは SDK の読み違い補正だったと確定) →
   (b) ~~ROS2 移植~~ (✅ 08-14) → (c) ~~time リネーム + stamp 巻き戻し + mirrorid 修正~~ (✅ 08-14 (4)) →
   (d) ~~A/B 比較~~ (✅ 08-14 (4): 旧の fold16 破壊を定量証明・新で解消。総括は
   `docs/issue/2026-08-14_rfans_driver_renewal.md`) → (e) ~~bringup 統合~~ (✅ 08-14 (5): topic は `/rfans_driver/rfans_points` に統一・StarROS2 は COLCON_IGNORE) →
   **(f) 縦角再較正** (新ドライバ出力でその場旋回較正をやり直し → `vangle_override` パラメータへ。
   旧較正値は fold16 汚染の疑いで直接使わない。
   **⚠️ 08-16 に最優先へ格上げ: 壁二重化 (z ドリフト→ループ不発) と取付角実験の崩壊の両方の根で、
   これを済ませるまで傾け取付の屋外評価は原理的に無意味**。
   **→ 08-17: 較正環境が無いため保留 (ユーザ指示)。環境が整い次第の後日タスク**。
   **→ 09-10: z ドリフトの主犯ではない → 優先度下げ** — 縦角を補正して GLIM を再実行しても z_end は
   run 間ばらつき (SD 0.9 m) 内 (`docs/issue/2026-09-10_glim_z_drift_not_vangle.md`、critic 査読 §7)。≤30% の
   副次寄与は追加 run で判定中。z ドリフトの正体は「世界固定傾き A + 一定沈降 c」で A の発生源は未特定
   (時刻オフセット / extrinsic 回転 / 初期整合が候補)、新車体ではモータ脱落が加算。
   **→ 09-16: A の機構特定 → 対策は GLIM 前処理 `k_correspondences` 10 → 20** (issue §11。IMU・地面・反復数は無関係、植栽/壁の共分散退化。odom 段 A −55〜65%、残り 1.9〜2.5° の heading 依存成分は未解明 — CW 周回 / 第 3 直進開始 bag で識別)。
   **→ 09-20: 2 周 bag で閉合因子 0 本を確認 (§12)** — implicit loop は隣接でも overlap 0.24 の感度で再訪問を捕まえない。z_end は指標にせず z_max・同一地点 Δz で評価する。次の安い順: viewer find_overlapping 0.1 (数分) → `fix_imu_bias: true` 1 run (A の周跨ぎ成長の識別) → k=20 run の非近接因子数。2 周 bag から 2D 地図を作るなら当面 1 周目 submap のみ。
   **→ 09-20 (2): 手動閉合は周間ずれの対策にしかならない (山・壁抜け不変) = A 本体を縛る必要 (k=20 + fix_imu_bias + 新車体 extrinsic pitch 再較正 (acc bias x +0.25 m/s² の正体確認) + acc_scale 0.983)。外周 bag の飛びは R-Fans 回転ディップ × IMU 単独予測 — 人工 5 s 欠落 bag で 1 run して「IMU 状態の質」か「劣化フレーム」かを識別してから対策 (車輪 odom 因子 / 欠落 >2 s で再初期化 / ドライバガード強化) を選ぶ。**
   **→ 09-20 (3): IMU なし CT-ICP は現構成で破綻 (選択肢外)。fix_imu_bias で H3 棄却 → 1.5 倍成長は「状態の持ち越し」か「経路差」— 2 周目開始で切った bag の 1 run が次の最安識別。IMU 側の残作業は extrinsic pitch 再較正 + acc_scale 0.983 (5 s 欠落ジャンプの縮小用) のみで、坡そのものは LiDAR 登録側 (k=20 → 残り半分の機構) を追う。**
   **→ 09-21: 照合方式 (GICP/VGICP) では坡の形は変わらず k で決まる → GPU 版も k=20 が前提 (本命)。GPU 実走 (MX150、ドライバ復旧後) は H1〜H3 (同時最適化・surface validation・0.25 m target) の確認としてなら価値あり。source/target どちらの共分散かは patch 実験で識別。**
   **→ 09-21 (2): k=20 を live に反映済み (ユーザ)。残るのは (1) odom 段の山 5.9 m/周 (機構未特定: A1/A2/heading 符号)、(2) global 段の +3 m/周 上振れ (新、pose_graph 構成で切り分け)、(3) 閉合ゼロによる周間オフセット (手動閉合 or 1 周目のみ)。**
   縦角較正は「地図精度の詰め」として残す) →
   (g) メーカー問い合わせ (任意、連絡先は issue doc §7)
6. **GLIM 3D 地図→Nav2 接続 (Phase 2)** (08-17 実装済み — タイムライン 08-17):
   (a) ~~地図変換 (pointcloud_to_2dmap) + /scan 化 (rfans_scan.launch.py)~~ (✅ bag/配信検証まで) →
   (a2) ~~屋外 z ドリフト地図の変換~~ (✅ 08-20: `glim_dump_to_2dmap` のセンサ相対スライスで解決 —
   5号館 LC 地図で実証、タイムライン 08-20) →
   (a3) ~~濃さ不均一の原因調査~~ (✅ 09-10: 原因は GLIM 間引き × submap 間隔 × 姿勢傾き、`-r 0.10` を推奨。**残候補: glim_dump_to_2dmap に観測 submap 数モード追加 / GLIM `submap_downsample_resolution` 0.3→0.1 で 5号館 bag を再 dump して比較 / glim_ext flat_earther 試験** — 読本 §3.4 末尾) →
   (a4) ~~生スキャン × traj 再投影~~ (✅ 09-14: `glim_traj_to_2dmap` 実装・比較済み → submap 数依存の薄化は解消。**残: bag 再生 + AMCL で R (ground+deskew) vs B (submap r=0.10) の収束比較** — `ros2 bag play --clock` + `rfans_scan.launch.py` + map_server/amcl のみ、`/amcl_pose` と `traj_lidar.txt` の xy 差で判定。(a3) の残候補は (a4) で不要化: 観測 submap 数モード・GLIM 再 dump は取り下げ、flat_earther は z ドーム側の課題として継続) →
   (b) **AMCL 込みの実走検証** (実機再開時。手順は `docs/text/map3d_to_nav2/07_checklist.md` §7.2) →
   (c) keepout マスク作成 (未知領域なし制約の運用面の補い) →
   (d) ~~本格案 = 3D ローカライザ実装~~ (✅ 10-10 実装: lidar_localization_ros2 + 橋、`docs/features/2026-10-10_lidar_localization_3d.md`) →
   (d2) **地図作成 (ユーザ)**: 同じ dump から 2D + `--export_pcd` の 3D + traj_lidar.txt を `map_name` 同名で配置 (§4.1) →
   (d3) ~~bag 再生で本体検証 → A/B (ndt_resolution 2/5)~~ (✅ 10-11〜12: `tools/loc_eval/` で 2 bag 評価、ndt 2.0 採用。IMU ON/OFF は未) →
   (d4) **実機で nav_lidar3d** (CPU 負荷・TF 鮮度・EPOS RPDO 途絶との兼ね合い・初期姿勢誤差耐性を見る) →
   (d5) 2D の原因追試 (安い順): ~~帯一致地図 / 100 m レンジ / 全点~~ (✅ 10-12: 帯・レンジは無効、全点は角を通過するが選点法か未分離、alpha 0.02 が主レバー) → 選点法の分離 (scan_all から 0.2° 扇内最近点のスキャンを a002 と既定 alpha で) / 既定 alpha の全点を 2〜3 本反復 / 2D 共通の横バイアス +0.5〜1.0 m の由来 / alpha 0.02 を 1643 でも / emcl2 を alpha≠0 の設定で / 3D の摂動試験。実機 2D は nav_amcl_scanall + alpha 0.02 + max_beams 300〜500 が候補。随伴者の帯の根治は glim_traj_to_2dmap 側のフィルタ (後日)。
   ⚠️ main イメージ再ビルド (`./scripts/build.sh images`) で libpcl-dev を恒久化するまで、コンテナ作り直しでビルド不能

**✅ IMU 前提整備 (09-24→27 完了)**: auto_tare off / URDF 実測 / accel 再較正 / 診断指標 0.91° / BNO≈WT / R-Fans stamp live 確認。✅ acc_scale 0.9705 → 0 (自動 1.0) を live 反映 (09-27)。残作業: submodule `surestar_rfans_ros2` コミット。

**GLIM GPU 版の立ち上げ (10-07〜、構成のみ済み)**: `./scripts/build.sh glim_gpu` で CUDA 版イメージを建てる (`jazzy_cuda12.5` を 4.4 GB pull、展開後 10 GB 前後 — ディスク残量に注意) → `docker compose --profile glim_gpu up -d glim_gpu` → `ls /root/ros2_ws/install/glim/lib | grep gpu` で `libodometry_estimation_gpu.so` を確認 → `glim_rosbag` を `config_path:=/glim_config`・`dump_path:=…_dump/gpu` で実行。⚠️ 再ビルド時に確認すべき 2 点: Dockerfile_glim が後から入れている apt (rviz2 / PCL / VirtualGL / mesa-utils) が CUDA 土台で通るか、`.bashrc` の `/root/ros2_ws/install/setup.bash` のパスが同じか。⚠️ GPU 用 config 3 本は上流デフォルトで CPU 側の調整が入っていないため、**CPU 版との比較をするなら条件を揃えてから** (09-21 の「GPU に替えても k を上げない限り同じ坂」の検証に使う場合は特に)。⚠️ CPU 版コンテナと同時起動しない。

**🔴 次の本題: LiDAR 側の坂 L (09-27〜)** — 5号館 1 周で A ≈ 4°@+70 (BNO・WT で一致、IMU 入力正)。IMU が正しい状態で GLIM の LiDAR 側パラメータ (k_correspondences・前処理ダウンサンプル・登録解像度・global 段) を同一 bag (`2026-09-27_1651_5goukan`) で A/B し、A と診断指標で評価する。09-10 issue §11/§12/§14 の候補を順に。(旧: (b) GLIM 側吸収は棄却、tare 時代 bag は legacy 設定のまま A 解析に使える)

**比較用 IMU (09-23)**: 次回実機で (1) `sudo cp tools/99-wt901.rules /etc/udev/rules.d/` → (2) bringup `imu:=true imu_rate:=200` + `wt901_imu.launch.py` 同時起動 → (3) 両 IMU 30 s 静置込みで bag (`bags/<場所>/2d3d_imu_wit/`、topic に `/imu_wit/data` `/imu_wit/mag` 追加) → (4) 静置 |g| (BNO086 +3% 問題の裏取り)・gyro_z 重ね描き・GLIM `imu_topic` 差替え run。手順 `docs/features/2026-09-23_wt901c_comparison_imu.md` §5〜7。

従来のトリアージ指示書 `docs/issue/2026-07-07_monthly_2026_6_todo_triage.prompt.md` は再編完了後に再開 (優先順・完了条件つき)。
要約: A1 LiDAR FOV → A2 slam lifecycle → A3 rviz エラー確認 → A4 脱力モード修正 →
~~B1 エンコーダ根本修正~~ (✅07-29 適用済み。残タスク: **接地検証** — 低速から 10 m 直進 + 360° 旋回、teleop/Nav2 の速度チューニング見直し) → B2 ハンチング → B3 オドメトリ/SLAM 精度 → B4 R-Fans RViz → B5 LIO-SAM 回収。
加えて監査 Issue 1〜3 (安全系) を本番前に必ず。ros2_control 移行 (C1 案 3) はユーザ合意が出たら本命。
