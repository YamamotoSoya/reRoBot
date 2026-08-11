<!-- claude: 将来の Claude セッションに全体像を渡すためのメモリファイル。
     大きな状態変化 (機能追加・重要バグの解決・方針変更) があったら必ず更新すること。
     読み順: CLAUDE.md (規約・ビルド) → 本ファイル (現在地) → docs/issue/ (問題詳細)。 -->
# reRoBot プロジェクト状態メモ (Claude 用)

- **最終更新: 2026-08-11** (**(1) 車輪 odom + IMU の EKF 融合を実装** (`robot_localization`、`ekf.yaml` + `ekf:=true` launch 引数 + epos4_odometry に共分散対角追加)、**(2) GLIM を IMU あり LIO 構成へ切替** (config_odometry_cpu + enable_imu:true ×2 + T_lidar_imu)。**(3) EKF 初回実機テストの bag 解析で 3 件発見・修正** — teleop 51 m 表示は params 未指定起動 (車輪 odom 自体は 10 m 実測に対し 10.18 m と正確)・/odom twist が常に 0 だった実装バグ (velocity 常0 問題の踏み抜き) を位置差分に修正・**IMU 取付向きは実走データで正立 rpy=(0,0,π/2) に最終確定** (朝の目視判定「裏返し」を棄却、タイムライン 08-11 (2) 参照)。**(4) EKF 再テスト成功** (filtered 正常動作: 直進 10.08 m/実測 10 m、360° 残差 +3.9°) → **Nav2 の odom 入力を /odometry/filtered へ切替** (nav2_params.yaml ×2 箇所 + nav2d.sh を IMU+EKF 標準に)。**次: EKF 構成での Nav2 実走行確認**。前回 08-10: bringup 統合再編: URDF 1 本化 `rerobot.urdf` (laser + rfans + imu_link 常設 — 実機に両 LiDAR 併設のため)、params 1 本化 `params.yaml` (params_2d/3d の epos4 重複を廃止)、launch は実体 `rerobot_bringup.launch.py` (boolean 引数 lidar_2d/lidar_3d/imu) + 構成別ラッパ 5 本 (`_2d`/`_3d` = IMU なし scripts 互換, `_2d_imu`/`_3d_imu`/`_2d3d_imu` = IMU 込み新設)。コンテナ内ビルド + 全 launch の評価検証済み。センサ位置はユーザ実測を反映済み: laser (0.07, 0, 0.215, rpy=0 — ⚠️ 旧 roll=π の逆さ補正を撤廃、実機 /scan で要確認)・rfans (-0.075, 0, 0.725)・imu (0, 0, 0.64, rpy=(π,0,π/2) = 裏返し+90° 取付の軸対応 imu_X→base_Y, imu_Y→base_X, imu_Z→-base_Z)。GLIM config は 08-01 にコミット済み (config_odometry_ct + enable_imu:false ×2) で、StarROS2 の per-point time フィールド化 (timeflag→time) も完了 — GLIM 実点群での起動検証が次)
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
| シャシ | tread 0.41 m, タイヤ径 0.15 m | |
| 減速比 | 物理 5:1 = `gear_ratio: 5.0` (07-29 根本修正済み) | エンコーダは 256 pulses × 4 逓倍 = 1024 inc/モータ回転、タイヤ 1 回転 = 5120 inc。bus.yml `scale_pos_from_dev = 2π/1024` |
| 2D LiDAR | HOKUYO UTM-30LX (USB, urg_node) | frame `laser`, base_link から z+0.714 |
| 3D LiDAR | Sure-Star R-Fans-16 (Ethernet 192.168.0.3, rfans_driver) | frame `rfans`, PointCloud2 `/sdk_could` (typo だが仕様) |
| IMU | **自作 BNO086 ボード** (USB CDC, `bno086_imu_driver` → `/imu/data`) を採用予定 (08-02 受領) | RealSense IMU 経路を置き換える本命。製作者 (fTomo-robot) の未公開 repo の先行コピー → 後日 fork + submodule 化予定。**08-10 実機初疎通 OK** (/dev/ttyACM0 開通・auto tare 発火・/imu/data publish 確認)。**取付向きは 08-11 の実走 bag (ekf_test) で最終確定: rpy=(0,0,π/2) = 正立+90°**。証拠 3 系統: 静止 60 s の accel z=+9.815 (Z 上向き)・旋回中の gyro_z/車輪 odom yaw rate=+1.015 で同符号率 100%・裏返し URDF で走った EKF の yaw が車輪 odom の鏡像 (-79° vs +74°)。目視 (矢印) ベースの判定は 2 転 3 転したため、以後の向き検証はデータ照合を正とする。`realsense_imu.launch.py` は完全移行確定まで温存 (⚠️ 同時起動すると /imu/data が衝突) |
| ゲームパッド | Xbox (joy + teleop_twist_joy, LB=deadman, RB=turbo) | |

## 3. ソフトウェア全体像

```
teleop_keyboard / joy_teleop / Nav2(RPP) ── Twist /robot_speed_cmd
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
  `drivers/` = epos4compact50-5can + StarROS2 + realsense-ros (すべて submodule 直置き)。
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
  bag 録画と LiDAR⇔IMU のタイムスタンプ整合確認が先)。`koide3/glim_ros2:jazzy` 公式イメージあり (CUDA 不要)。
  ⚠️ R-Fans 点群の `timeflag` フィールドは GLIM 非認識 (認識名は `t`/`time`/`time_stamp`/`timestamp`) →
  擬似タイムスタンプにフォールバックし deskew 劣化。GLIM は GTSAM 4.3a0 要求で `ros-jazzy-gtsam` 4.2.0 と競合 → 別コンテナ運用。
- 3D 構成 (`rerobot_bringup_3d.launch.py`): 点群は出るが **/scan が無いので SLAM/Nav2 に繋がらない** (GLIM 導入で解消予定)。
- LIO-SAM: submodule として回収済み (07-11, ros2 branch, T13 解決)。ビルドは通るが **IMU 入手不可のため当面凍結**。
  `realsense_imu.launch.py` は IMU 再入手後の GLIM CPU (LIO) モード格上げ用に温存。
- RViz での R-Fans 点群表示手順が未確立 (triage T12 に手順記載)。
- **BNO086 IMU** (08-02 統合 → 08-10 実機初疎通 OK): `/dev/ttyACM0` 開通、`/imu/data` publish 確認 (auto tare 発火も確認)。
  bringup への組み込みも完了 (`rerobot_bringup.launch.py imu:=true` / `_imu` 系ラッパ)。搭載位置は実測反映済み (0, 0, 0.64)。
  **取付向きは 08-11 の実走 bag で最終確定・URDF 反映済み**: rpy=(0,0,π/2) (正立+90°)。⚠️ 経緯 (2 転 3 転した):
  08-10 の 240 s データ照合は「正立」→ 08-11 朝にユーザが矢印目視で「裏返し」と修正 → 同日の EKF 実機テスト bag
  (静止 accel z=+9.815 / gyro_z⇔車輪 yaw rate 同符号 +1.015 / EKF yaw 鏡像) で**正立が確定**し裏返しを棄却。
  目視判定は imu_check.rviz の表示バグ (fixed_frame_orientation, 08-11 修正) と混線していた可能性が高い。
  **今後、取付向きの検証は必ず実データ照合で行う** (静止重力 + 旋回ジャイロ vs 車輪 odom の 2 点で一意に決まる)。
  → EKF 融合 (`ekf:=true`) + GLIM LIO 構成は 08-11 実装済み。EKF 初回実機テストで /odom twist=0 の実装バグも発見・修正済み (§5)。

**品質面の課題**: SLAM が「絶妙にずれる」、低速でハンチング、Nav2 走行が遅い — いずれも
オドメトリ/エンコーダ問題 (§5) が根っこにある可能性が高い。

## 5. 既知の問題 — どこを見ればよいか

**docs/issue/ が問題管理の場所** (1 issue = 1 ファイル、冒頭にステータス、解決したら更新する運用)。

| ファイル | 内容 |
|----------|------|
| `2026-07-07_repository_audit.md` | 全体監査の 25 issue 一覧 (サマリ表付き) |
| `2026-07-07_wheel_odometry_encoder_scaling_4x.md` | **解決済み (07-29)**: エンコーダ分解能 4 倍ズレ。実測で原因確定 (EPOS 側 256ppr は正しい / raw tpdo 無スケール / 実車は指令の 1/4 速だった) → bus.yml 2π/1024 + gear_ratio 5.0 の 2 点同時修正を適用、浮かせ検証 OK。**残: 接地での 10 m 直進・360° 旋回の最終確認** |
| `2026-07-07_monthly_2026_6_todo_triage.md` | monthly の TODO 14 件の原因分析 + 「複雑なコード」の経緯と簡素化案 (C1〜C3) |
| `2026-07-07_monthly_2026_6_todo_triage.prompt.md` | ↑を実行する次エージェント用の作業指示書 (A 群=コードのみ, B 群=実機必須) |
| `2026-08-11_joy_spin_epos_shutdown_usb_stall.md` | **未解決 (3 回目)**: 急速度変化で EPOS 停止 + CANUSB が USB ストール (urb -32)。復旧手順は確立 (can_up.sh の stale-slcand 検出込み)。**残: EPOS Error History 確認 + EPOS 側ハードニング (0x60C5 有限化・電流制限・Save All) + PC サスペンド無効化** |
| `2026-08-11_utm30lx_usb_instability.md` | **調査中**: UTM-30LX が同日 3 回自然断 → 現在 USB バス上に不在。次アクション = 本体電源 LED / 12 V 供給 / コネクタ点検 (ソフトでは復旧不可) |

**優先度の要約**:
1. 🔴 **安全系 (監査 Issue 1〜3)**: /robot_speed_cmd ウォッチドッグなし・Ctrl-C で disable 不達・速度平滑化なし。
   monthly の TODO には無いが**本番前必須**。
2. ~~🔴 エンコーダ 4 倍ズレの根本修正 (B1)~~ → **07-29 適用・浮かせ検証済み** (残: 接地確認)。「Nav2 走行が遅い」は 1/4 速が原因でほぼ確実に説明がつく。ハンチング (T10)・SLAM ずれ (T11) への効果は接地後に再評価。
3. 🟡 LiDAR FOV ±90°→±135° (A1)、slam_toolbox lifecycle 整理 (A2)、脱力モードの競合修正 (A4)。
4. 🟡 ROS_DOMAIN_ID=150 → 101 以下へ (discovery が散発的に失敗しうる)。

## 6. これまでの経緯 (タイムライン要約)

| 時期 | 出来事 |
|------|--------|
| 2026-05 末 | slam_toolbox の scan queue full 問題を解決 (report 05-26)。scan_queue_size=10 等は意図的な設定 |
| 06-04 | 片輪が動かない問題の根因特定 = init サービス連射レース → 逐次化+SDO検証+リトライで修正 (report) |
| 06-05 | 脱力モード追加。実装は 3 転 (重装備→最小化→逐次化を再導入) — 経緯は features 文書と triage C1 |
| 06-07 | motor1/2 と左右の対応を swap 修正 (実配線: motor1=右)。gear_ratio 1.25 に (=対症療法と後日判明) |
| 06-09〜12 | 9 号館で SLAM 地図取得 → Nav2 自律走行成功。keepout 追加。bt_navigator plugin 二重登録 segfault の教訓 |
| 06-13 | R-Fans-16 の z=0 問題 (dataID 0x37 remap) と低 fps 問題を解決 (report ×2)。StarROS2 を ROS2 移植 (features) |
| 06 末 | 2D/3D bringup 分離 (params_2d/3d, urdf 2d/3d)。RViz を nav2.rviz/slam.rviz に分割。joy teleop 追加 |
| 07-07 | Claude によるリポジトリ全体監査 (25 issue) + monthly TODO トリアージ。docs/issue/ 運用開始 |
| 07-11〜12 | LIO-SAM (ros2) + realsense-ros を submodule 追加、`realsense_imu.launch.py` 作成 (T13 回収)。project skills (.claude/skills) と .mcp.json 導入 |
| 07-26 | CLAUDE.md 全面更新 (Issue 16/17 の古い記述修正、LIO-SAM/RealSense/skills/docs 運用を反映)。.gitmodules の LIO-SAM branch 設定を修正 (末尾スラッシュ付き孤立セクションに `branch = ros2` が置かれ `update --remote` が master を取る状態だった)。コード変更後に本ファイルの更新を促す Stop hook (.claude/hooks/check-project-state.sh) を導入。`annotate` スキル追加 (返信中の発展用語に ※n + 📘 注釈ブロック、既知用語リストで自己調整)。`user-level` スキル + `docs/claude/USER_LEVEL.md` (git 管理外) 導入 — monthly/knowledge/既知リストからユーザ知識レベルを推定し annotate・knowledge-check の較正元にする。git 運用を **main 直コミット**に方針変更 (ユーザ指示) |
| 07-26 (2) | IMU 入手不可が判明 → **3D SLAM に GLIM 採用を決定** (要件調査で IMU レス CT-ICP を確認)。機能別 workspace + コンテナ分割の再編開始 (`feat/workspace-split`)。旧構成を `archive/monolithic` + タグ `v1-monolithic` にアーカイブ (ユーザ自身が git 操作を実施)。`feat/claude-optimize` は破棄決定 |
| 07-26 (3) | 再編の ws 分割 + app/bringup/drivers グループ化をコミット (1e7f11c)。`epos4_vel_ros2` を削除 (553068d, 役割は epos4_controller に置換済み)。`epos4_teleop` は脱力モード入口 + 距離計測ツールとして存続と判断 |
| 07-26 (4) | **Nav2 は main 統合・slam_toolbox は別コンテナ**の分割基準を確定 (「機能でなく依存の壁で分ける」)。slam 資産を `rerobot_slamtoolbox` に分離。Docker 4 分割 (main/slamtoolbox/glim/liosam) + scripts/ 8 本 + CLAUDE.md/skills 追従を Claude が実施。cv_bridge の「LIO-SAM 用」コメントが誤り (realsense2_camera が要求) と判明し main に残置、realsense の欠落依存 (image_transport 等) も補完 |
| 07-29 | **エンコーダ 4 倍ズレの実測確定セッション** (実機・タイヤ浮かせ)。SDO read で両ノード 0x3010:01=256/0x3000:05=1024 (EPOS 側は正しい)、手回し 1 回転 = 7.86 rad ≈ 5120 inc (EPOS Studio 側実測とも一致)、candump で raw tpdo 無スケール素通し (0.2 m/s 指令 → 0x60FF=-31) を確認。**速度相殺説は否定 — 実車は指令の 1/4 速で走っていた**。根本修正は ROS 側 2 点同時に簡約 (issue doc 更新済み)。副産物: joint_states.velocity=0 の機構解明、再編後 bringup2d の実機起動成功 |
| 07-29 (2) | **根本修正を適用・浮かせ検証 OK** (issue 解決)。bus.yml `scale_pos_from_dev: 2π/1024` / `scale_pos_to_dev: 162.97` + `gear_ratio: 5.0` ×5 箇所を同時変更、3 パッケージ再ビルド。検証: 0.2 m/s 指令 → 0x60FF=-127・0x606C≈-127 rpm・/odom 変位 2.15 m (≈指令×実効時間) で 3 系統整合。CLAUDE.md の規約 2 箇所も更新 (2π/1024、raw tpdo 無スケールの明記)。余波調査で未使用の pos 側 bus.yml も同修正、古い「3 点同時」指示 (triage/prompt/audit) を解決済みに更新。**未コミット** (bus.yml ×2 は submodule 側コミット + 親 gitlink 更新が必要)。接地検証 (10 m 直進・360° 旋回・低速から) が残 |
| 07-30 | glim コンテナを `docker/Dockerfile_glim` 化 (公式 `koide3/glim_ros2:jazzy` ベースの薄い層: rviz2 + 対話 bashrc のみ追加、GLIM 本体はビルドしない)。他 3 コンテナと compose/build.sh の構成を統一。ビルド・起動・glim_ros 実行体の疎通は検証済み。**`ros2_ws_glim/config/` の設定 JSON 作成と glim_rosnode 起動検証は未着手** (ユーザ指示で次回に持ち越し) |
| 08-02 | **自作 BNO086 IMU ボード受領・main ws に統合** (RealSense IMU 置き換えの本命、未公開 repo の zip 先行コピー → 後日 fork+submodule 化)。colcon 干渉解消 (`firmware/COLCON_IGNORE` + Mac ビルド成果物 `build/` 削除 — 放置すると STM32 HAL がパッケージ誤認識され build.sh main が壊れる)、Dockerfile_main に `python3-serial` 追加 + main イメージ再ビルド、URDF 2d/3d に `imu_link` (暫定 z=0.714, rfans と同位置)。ws ビルド 10 パッケージ + tf_static (base_link→imu_link, z=0.714) 検証済み。実機ボード接続は未 |
| 08-10 | **bringup 統合再編** (実機に 2D/3D LiDAR 併設のため)。URDF を `rerobot.urdf` 1 本化 (laser + rfans + imu_link 常設、旧 rerobot_2d/3d.urdf 削除)、params を `params.yaml` 1 本化 (epos4 セクションの 2 ファイル重複を廃止、旧 params_2d/3d.yaml 削除)、launch を実体 `rerobot_bringup.launch.py` (boolean 引数 `lidar_2d`/`lidar_3d`/`imu` + 接続系引数透過) + ラッパ 5 本に再編 — `_2d`/`_3d` は IMU なしで scripts/*.sh 互換維持、`_2d_imu`/`_3d_imu`/`_2d3d_imu` を新設 (IMU は bno086.launch.py を include, port 引数 `imu_port`)。CLAUDE.md / params-sync skill を追従更新 (params-sync は 2 ファイル検査に)。コンテナ内で colcon ビルド + 全 6 launch の generate_launch_description() 評価 OK、install 内の旧ファイル残骸も掃除。同日中にユーザが実測 → URDF 反映済み: laser (0.07, 0, 0.215)・rfans (-0.075, 0, 0.725)・imu (0, 0, 0.64)。IMU の向きはユーザ実測の軸対応 (imu_X→base_Y, imu_Y→base_X, imu_Z→-base_Z) から rpy=(π, 0, π/2) を導出・数値検証して適用。⚠️ 2 点要実機確認: (1) laser の旧 roll=π (逆さ補正) が撤廃された — 逆さのままなら /scan が左右鏡像になる、(2) IMU 軸対応が mount_yaw_deg=180 補正後の出力軸か (チップ印字読みなら yaw が π ずれる。静止時 accel z≈-9.8・左旋回で gyro z<0 で判定)。同日さらに **BNO086 実機初疎通に成功** (/dev/ttyACM0, /imu/data publish, auto tare 確認 — 08-02 からの残タスク解消)。ただし静止時 **accel z=+9.81 → Z 軸は上向き**でユーザ申告の「裏返し」と矛盾 → 向き確定用に `imu_check.launch.py` + `rviz/imu_check.rviz` を新設 (driver + robot_state_publisher + RViz 重力矢印。CAN 非依存)、`ros-jazzy-rviz-imu-plugin` を稼働コンテナに導入 + Dockerfile_main に追記 (イメージ再ビルドは次回まとめて)。モータ系の実機起動検証は未 |
| 08-11 | **IMU 取付向き最終確定 + EKF 融合実装 + GLIM LIO 切替**。(1) `imu_check.rviz` の表示バグ発見・修正 — `fixed_frame_orientation: true` だと rviz_imu_plugin は imu_link の TF を捨てて生値を base_link 軸に描くため、加速度矢印が URDF 検証にならなかった (Imu 表示自前の axes も msg orientation 描画で TF 軸とは別物 → 紛らわしいので無効化)。その時点ではユーザ目視で rpy=(π,0,π/2) (裏返し) と判断し URDF に反映 (⚠️ のち同日の実走 bag で覆る — 08-11 (2) 行)。(2) **車輪 odom + IMU の EKF 融合** — `epos4_odometry` に pose/twist 共分散対角パラメータ追加 (claude_ekf タグ)、`config/ekf.yaml` 新規 (odom0=vx,vy,vyaw / imu0=yaw(relative),vyaw / two_d_mode)、`rerobot_bringup.launch.py` に `ekf:=true` 引数 (ekf_node 起動 + epos4_odometry の publish_tf 自動 false で TF 二重配信防止)。(3) **GLIM を IMU あり LIO 構成へ切替** — config.json→`config_odometry_cpu`、sub/global mapping `enable_imu:true`、`T_lidar_imu=[0.075,0,-0.085, √2/2,√2/2,0,0]` (URDF から算出・数値検証済み)、config_ros.json の IMU レスハック撤回 (imu_frame_id 自動検出へ。publish_imu2lidar は TF 親二重化防止のため false 維持)。**残: 実機検証** — EKF は静止/直進/旋回比較、GLIM は bag 録画 + LiDAR⇔IMU 時刻整合確認 + glim_rosbag 評価。Nav2 の odom 入力切替も未 |
| 08-11 (2) | **EKF 初回実機テスト → bag 解析で 3 件発見・全て修正**。ユーザが静止 + 10 m 直進を実施 (360° 旋回は未実施)、bag は `/workspace/log/ekf_test`。(a) **teleop 距離が 51 m 表示** (実測 10 m) — bag の /odom は 10.18 m / 旋回 +0.5° と正確で車輪較正は無罪。51≈10×gear(5.0) から teleop が params 未指定起動 (gear=1.0 既定) と特定 (teleop.sh 経由なら 5.0 が読まれることも実測確認)。対策: teleop 起動時にパラメータ値を INFO 表示 + gear=1.0 なら WARN。(b) **/odometry/filtered の位置が終始 (0,0)** — /odom の twist が常に 0 のせい。原因は epos4_odometry が joint_states.velocity (0x606C 未 PDO マップで常に 0 — bag で実証、従来「疑い」だったもの) を配列非空というだけで優先使用していたこと。位置差分 (d_s/dt) ベースに修正。(c) **EKF の yaw が車輪 odom の鏡像 (-79° vs +74°)** — IMU 裏返し URDF が原因。bag の静止 accel z=+9.815 と gyro_z⇔車輪 yaw rate 同符号 (中央値 +1.015, 100%) で**正立 rpy=(0,0,π/2) が確定**、URDF と GLIM の T_lidar_imu (q=(0,0,√2/2,√2/2) に再計算) を修正。**残: 修正後の EKF 再テスト (静止/直進/360° 旋回 — C は初回未実施)** |
| 08-11 (3) | **EKF 再テスト成功 → Nav2 を EKF オドメトリへ切替**。再テスト bag (`/workspace/log/ekf_test2`): filtered の位置が正常に動き (path 22.37 m で odom と一致、yaw rate 同符号率 98%)、静止 305 s ドリフト 14 mm/-1.1°・直進 10.08 m (odom 10.19 m, 実測 10 m)・360° 旋回残差 filtered +3.9° / odom 0.1° (車輪較正が優秀。filtered の残差は磁北基準 yaw の外乱の可能性 — 調整候補は §10-4)。IMU 向きの疑問には「URDF は基板シルクではなく **/imu/data のデータ座標系**を表す」と整理 (物理は上下逆でもドライバ/FW 補正後のデータは Z 上向き — ユーザ了解)。**Nav2 切替**: nav2_params.yaml の bt_navigator + controller_server に `odom_topic: /odometry/filtered`、`nav2d.sh` は IMU+EKF 標準に (bringup2d.sh へ IMU/EKF env 変数追加、既定 false で slam2d.sh 等は従来互換)。EKF なしへ戻す手順は nav2_params.yaml のコメントに記載。残: EKF 構成での Nav2 実走行確認 |
| 08-11 (4) | **joy 操作中に急回転 → EPOS 停止 (07-31 と同型、3 回目)**。kernel log 16:50:14 に CANUSB (1-2.3, ttyUSB0) の USB ストール `urb -32` — モータ電流スパイク→USB/CAN 巻き添えの既知シグネチャ。UTM-30LX (Hokuyo 15d1, ttyACM) は**人手によらず自然断** (ユーザ確認: 誰も触っていない。同日 14:50 切断・16:43 二重列挙と接続不安定の前兆があり、16:53:07 の切断が 3 回目 — コネクタ緩み or 電源系を疑う)。同時刻にハブ 1-2.4 へ「同一シリアルの CANUSB」が出現したが、これは物理 2 台目ではなく**ストール事故に伴う幽霊列挙の疑い** (ioctl 無応答の wedged デバイス。実体 1 台はユーザ確認済み)。17:09 の xHC リセット (ノート PC のサスペンド復帰) で実体側 (1-2.3) は ttyUSB2 として復活・/dev/ttyCANUSB も追従、can0 は DOWN に。さらに boot 時の slcand が死んだ旧 ttyUSB0 を掴んだまま can0 が見かけ上 UP になる罠を確認 → **can_up.sh に stale-slcand 検出 (slcand の実 fd と /dev/ttyCANUSB の実体照合) を追加**。udev ルール (99-hokuyo-devices.rules: UTM 15d1→ttyUSB-utm-30lx / CANUSB→ttyCANUSB+canusb-up.service) は整備済みと判明 — launch の古い「手動 ln -sf」コメントを訂正。⚠️ 同型 CANUSB 2 台が同時接続だと /dev/ttyCANUSB がどちらを指すか不定 (シリアル同一で udev 判別不能) — **運用は 1 台のみ接続**。⚠️ EPOS 側ハードニング (0x60C5 有限化・電流制限・Save All、07-31 提案) が未適用なら 3 回目の今回で適用を強く推奨。Error History (0x3210/0x2310/0x81FD) の確認も未 |
| 08-01 | **3D 点群不通の解決** (report 08-01)。`rfans_driver` が起動 ~90 ms で無言 SIGABRT。gdb で `libstar.so` (プリビルド blob) が `__cxa_throw` 等の古い例外ランタイムを export → FastDDS のポート衝突例外 (正常系) の unwind を横取りして abort と特定。発火条件は「同一ドメインに先住ノード」= コンテナ内の残留 RViz 等 (だから従来のまっさら起動では潜伏)。bringup_3d + StarROS2 の両 launch に `additional_env: LD_PRELOAD=libstdc++:libgcc_s` を追加して解消。tcpdump で LiDAR パケット到着 (192.168.0.3:2014, 1206 B) も確認し全経路開通。**未コミット** (StarROS2 submodule → 親 gitlink) |
| 07-31 | **全モータ同時停止 ×2 の診断と対策**。ユーザが EPOS Studio でゲイン硬化 (キビキビ化) 後、joy 走行中に「動いて止まる + PDO 送信不能連発 + EPOS 赤ランプ」。1 回目は CANUSB の USB stall (`urb -32`) と同時刻に両モータ SDO timeout、2 回目は**最高速→ゼロ指令の瞬間に USB 無傷のまま**同症状 — 個別フォルトなら CAN 応答は残るので、ランプ無しステップ指令 → 制動電流/回生スパイク → 電源/CAN 巻き添えの signature と診断 (旧「ふにゃんふにゃん」ゲインが実質ランプとして働き隠れていた)。対策: **epos4_controller に slew rate limiter 実装** (`max_motor_accel/decel_rpm_per_s`=2000 rpm/s ≈ 3.1 m/s²、params_2d/3d に追加)。スクリーンショット (~/Pictures/epos4 setting) から EPOS 側 Max acceleration=0xFFFFFFFF (無制限)・Max output current=15 A (上限) と判明 → 0x60C5 有限化 (>2000 rpm/s) + 電流 8〜10 A + Save All Parameters を提案。EPOS Error History での確定 (0x3210 過電圧 / 0x2310 過電流 / 0x81FD bus-off) と浮かせ→接地検証が残 |

## 7. コードを触るときに知らないと踏む罠 (経緯由来の知識)

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
8. **rfans_driver は LD_PRELOAD 必須** (08-01)。プリビルド `libstar.so` が古い C++ 例外ランタイムを export しており、素で起動すると同一ドメインに他ノードがいるだけで無言 SIGABRT する。launch 経由なら `additional_env` で自動適用済み — `ros2 run` で直接起動する時は手動で `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/lib/x86_64-linux-gnu/libgcc_s.so.1` を付ける。launch 前に `./scripts/stop.sh` で残留ノードを掃除する習慣も予防になる。
9. **BNO086 を fork+submodule 化するとき `firmware/COLCON_IGNORE` を fork 側にコミットし忘れると build.sh main が再び壊れる** (08-02)。vendored コピー内へのローカル追加はディレクトリ差し替えで消える。症状は「STM32 HAL/CMSIS がパッケージ誤認識 → colcon 全体停止」で、"前は動いてた" 形で再発する。
10. **`scripts/build.sh` は端末なし (自動化・Claude) から呼ぶと `docker exec -it` が stdin エラーで失敗するのに exit 0 を返す** (08-02 発見)。何もビルドされずに成功に見える。非対話でビルドする時は `-it` を外した `docker exec` で colcon を直接叩く。

## 8. docs/ ディレクトリの地図

| 場所 | 役割 | 編集ルール |
|------|------|-----------|
| `docs/claude/` | 本ファイル (Claude 用の状態メモ) | 状態が変わったら更新 |
| `docs/claude/USER_LEVEL.md` | ユーザ知識レベルプロファイル (annotate/knowledge-check の較正元) | ⚠️ 個人情報のため **git 管理外・コミット禁止**。更新は `/user-level` skill |
| `docs/issue/` | 未解決問題の調査記録 + 作業指示書 | 1 issue = 1 ファイル。解決したらステータス更新 |
| `docs/report/` | 解決済みバグの事後報告 (debug-report スキルのテンプレ準拠) | 大きなデバッグ完了時に追加 |
| `docs/features/` | 追加機能の設計文書 | 機能追加時に追加。※free_mode の §8 に古い記述あり (§9-11 が最終形) |
| `docs/monthly/` | ユーザの月次ゼミ報告 | **Claude は編集禁止** (読み取り専用の入力) |

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

**最優先: リポジトリ再編の仕上げ** (計画: `~/.claude-school/plans/imu-glim-swift-hartmanis.md`):
1. ~~ws 分割・slam 分離・Docker 4 分割・scripts・イメージ/colcon ビルド確認・旧資産掃除・コミット (0e8cb39)~~ (✅ 07-26 完了)
2. 起動検証: vcan モード (`/verify`) または実機 (`scripts/can_up.sh` → `nav2d.sh`) で旧構成同等の動作を確認
   → OK なら `feat/workspace-split` を main にマージして再編クローズ。空 `ros2_ws_nav2/` の削除も忘れず
3. GLIM 実データ検証 (08-11 に LIO 構成へ切替済み): `_3d_imu` 構成で bag 録画 (rfans 点群 + /imu/data) →
   **LiDAR⇔IMU の header.stamp が同一時間軸か確認** (rfans は GPS 時刻の扱いに注意、ずれは `imu_time_offset` で微調整)
   → `glim_rosbag` オフライン評価 → CT-ICP 構成と品質比較 → OK なら `glim3d.sh` オンライン運用へ
4. ~~EKF 融合の実機検証~~ (✅ 08-11 完了: 再テストで filtered 正常動作を確認 — 静止 305 s ドリフト 14 mm/-1.1°、
   直進 10.08 m (実測 10 m)、360° 旋回残差 +3.9° (生 odom は 0.1° と車輪較正が優秀)。filtered の旋回残差は
   磁北基準 yaw の影響の可能性 → 気になる場合の調整候補: imu0 の yaw 融合を切って yaw rate のみ /
   `use_game_rotation_vector: true` (磁気非依存))。
   **Nav2 の odom 入力は /odometry/filtered へ切替済み (08-11)**: nav2_params.yaml (bt_navigator +
   controller_server の odom_topic) + nav2d.sh が `IMU=true EKF=true` で bringup する構成に変更
   (bringup2d.sh に IMU/EKF env 変数追加、既定 false で従来互換)。**残: この構成での Nav2 実走行確認**。
   BNO086 repo が公開されたら fork + submodule 化 (⚠️ §7-9: COLCON_IGNORE を fork に含める)

従来のトリアージ指示書 `docs/issue/2026-07-07_monthly_2026_6_todo_triage.prompt.md` は再編完了後に再開 (優先順・完了条件つき)。
要約: A1 LiDAR FOV → A2 slam lifecycle → A3 rviz エラー確認 → A4 脱力モード修正 →
~~B1 エンコーダ根本修正~~ (✅07-29 適用済み。残タスク: **接地検証** — 低速から 10 m 直進 + 360° 旋回、teleop/Nav2 の速度チューニング見直し) → B2 ハンチング → B3 オドメトリ/SLAM 精度 → B4 R-Fans RViz → B5 LIO-SAM 回収。
加えて監査 Issue 1〜3 (安全系) を本番前に必ず。ros2_control 移行 (C1 案 3) はユーザ合意が出たら本命。
