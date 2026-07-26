<!-- claude: 将来の Claude セッションに全体像を渡すためのメモリファイル。
     大きな状態変化 (機能追加・重要バグの解決・方針変更) があったら必ず更新すること。
     読み順: CLAUDE.md (規約・ビルド) → 本ファイル (現在地) → docs/issue/ (問題詳細)。 -->
# reRoBot プロジェクト状態メモ (Claude 用)

- **最終更新: 2026-07-26** (GLIM 採用方針の決定 + リポジトリ再編 `feat/workspace-split` 進行中まで反映。ros2_ws_main/src の app/bringup/drivers グループ化まで完了。⚠️ 再編完了まで docker compose は起動不可の過渡状態)
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
| 減速比 | **物理は 5:1** だが params は `gear_ratio: 1.25` | エンコーダ分解能 4 倍ズレの対症療法 (下記 §5 の最重要問題) |
| 2D LiDAR | HOKUYO UTM-30LX (USB, urg_node) | frame `laser`, base_link から z+0.714 |
| 3D LiDAR | Sure-Star R-Fans-16 (Ethernet 192.168.0.3, rfans_driver) | frame `rfans`, PointCloud2 `/sdk_could` (typo だが仕様) |
| IMU | RealSense (D435i) を 6 軸 IMU として使用予定 | ⚠️ **07-26 時点で IMU 実機が入手不可** → LiDAR-only の GLIM を先行導入する方針。`realsense_imu.launch.py` → madgwick → `/imu/data` は IMU 再入手後の LIO 系入力用に温存 |
| ゲームパッド | Xbox (joy + teleop_twist_joy, LB=deadman, RB=turbo) | |

## 3. ソフトウェア全体像

```
teleop_keyboard / joy_teleop / Nav2(RPP) ── Twist /robot_speed_cmd
        ▼
epos4_controller ─ IK・CiA402状態管理・100Hz TPDO(0x60FF rpm)
        ▼
ros2_canopen Cia402Driver ×2 (bus.yml: sync 50ms, PDO)
        ▼                    ▲ joint_states (motor rad / rad/s ※velocity は常に0の疑い)
EPOS4 ×2                     │
        epos4_odometry ── 2topic を ApproximateTime 同期 → /odom + TF + /joint_states
        robot_state_publisher ── URDF (rerobot_2d/3d.urdf)
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
- R-Fans-16: 20 Hz で点群取得 (z=0 問題・低 fps 問題は解決済み — docs/report/2026-06-13 ×2)。

**構成はあるが未完/未接続**:
- **リポジトリ再編が進行中** (`feat/workspace-split`, 07-26〜): 機能別 workspace 分割
  (`ros2_ws_main` / `ros2_ws_slamtoolbox` / `ros2_ws_nav2` / `ros2_ws_glim` / `ros2_ws_liosam`)。
  `src/` からの移動は完了 (submodule は各 ws の src/ 直下に直接配置、symlink 撤廃)。
  **rerobot_bringup の slam/nav2 分離・Dockerfile 分割・compose 書き換えが未了 → この間 docker compose は起動不可**。
  旧構成は `archive/monolithic` ブランチ + タグ `v1-monolithic` (= 2686cd9) に恒久保存済み。
  計画詳細: `~/.claude-school/plans/imu-glim-swift-hartmanis.md` (ユーザがディレクトリ再編を自分で実施中、
  Docker 再編以降は Claude が引き継ぐ)。`feat/claude-optimize` (3D 自律移動 bringup 9 コミット) は**破棄決定** (ユーザ判断)。
- **3D SLAM は GLIM (koide3) を採用する方針** (07-26 決定、IMU 入手不可のため)。要件調査済み:
  IMU レスは `odometry_estimation_ct` (CT-ICP) で公式サポート、`koide3/glim_ros2:jazzy` 公式イメージあり (CUDA 不要)。
  ⚠️ R-Fans 点群の `timeflag` フィールドは GLIM 非認識 (認識名は `t`/`time`/`time_stamp`/`timestamp`) →
  擬似タイムスタンプにフォールバックし deskew 劣化。GLIM は GTSAM 4.3a0 要求で `ros-jazzy-gtsam` 4.2.0 と競合 → 別コンテナ運用。
- 3D 構成 (`rerobot_bringup_3d.launch.py`): 点群は出るが **/scan が無いので SLAM/Nav2 に繋がらない** (GLIM 導入で解消予定)。
- LIO-SAM: submodule として回収済み (07-11, ros2 branch, T13 解決)。ビルドは通るが **IMU 入手不可のため当面凍結**。
  `realsense_imu.launch.py` は IMU 再入手後の GLIM CPU (LIO) モード格上げ用に温存。
- RViz での R-Fans 点群表示手順が未確立 (triage T12 に手順記載)。

**品質面の課題**: SLAM が「絶妙にずれる」、低速でハンチング、Nav2 走行が遅い — いずれも
オドメトリ/エンコーダ問題 (§5) が根っこにある可能性が高い。

## 5. 既知の問題 — どこを見ればよいか

**docs/issue/ が問題管理の場所** (1 issue = 1 ファイル、冒頭にステータス、解決したら更新する運用)。

| ファイル | 内容 |
|----------|------|
| `2026-07-07_repository_audit.md` | 全体監査の 25 issue 一覧 (サマリ表付き) |
| `2026-07-07_wheel_odometry_encoder_scaling_4x.md` | ★最重要技術問題: エンコーダ分解能 4 倍ズレ。gear_ratio 1.25 は対症療法。修正は 3 点同時 (EPOS Studio 0x3010:01 / bus.yml scale / gear_ratio→5.0) が必須 |
| `2026-07-07_monthly_2026_6_todo_triage.md` | monthly の TODO 14 件の原因分析 + 「複雑なコード」の経緯と簡素化案 (C1〜C3) |
| `2026-07-07_monthly_2026_6_todo_triage.prompt.md` | ↑を実行する次エージェント用の作業指示書 (A 群=コードのみ, B 群=実機必須) |

**優先度の要約**:
1. 🔴 **安全系 (監査 Issue 1〜3)**: /robot_speed_cmd ウォッチドッグなし・Ctrl-C で disable 不達・速度平滑化なし。
   monthly の TODO には無いが**本番前必須**。
2. 🔴 **エンコーダ 4 倍ズレの根本修正** (B1): ハンチング (T10) と SLAM ずれ (T11) の根っこの疑いも濃い。
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

## 7. コードを触るときに知らないと踏む罠 (経緯由来の知識)

1. **CiA402 遷移サービスは並行発行すると壊れる** (実機で 2 回踏んだ)。init→enable→csv は必ず逐次。
   復帰時に `init` (homing) を呼ぶと復帰不能。サービス戻り値は no-op 遷移で偽陰性 → 成否は SDO
   (0x6041/0x6061) で判定。この 3 つの罠が epos4_controller の「複雑さ」の正体 (triage C1)。
2. **joint_states.velocity は常に 0 の疑い** (bus.yml で 0x606C 未マップ)。回転判定は position 差分で行う。
3. **「ちょうど 4 倍」のズレを見たらクアドラチャ 4 逓倍の二重解釈を疑う** (encoder issue の教訓)。
4. タイヤを浮かせた状態の挙動 (ハンチング等) は実走行の参考にならない。評価は接地で。
5. bt_navigator の plugin_lib_names は列挙すると二重登録 segfault (Jazzy)。デフォルトに任せる。
6. ROS 2 params の `ros__parameters` はアンダースコア 2 つ。1 つだと起動即死で症状から原因が分からない。
7. bus.yml は submodule (`src/external/epos4compact50-5can`) 内 → 変更は submodule 側にコミットし、親で gitlink 更新。

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

**最優先: リポジトリ再編の完遂** (計画: `~/.claude-school/plans/imu-glim-swift-hartmanis.md`):
1. ~~ros2_ws_main/src の app/bringup/drivers グループ化~~ (✅ 07-26 完了) + rerobot_bringup から slam/nav2 パッケージ分離 (未了、ユーザ実施中)
2. Dockerfile 4 分割 + docker-compose 書き換え (profiles + `ipc: host`) + scripts/ 作成 (Claude 担当)
3. CLAUDE.md・.claude/skills のパス参照を新構成に全面更新
4. GLIM 導入: `ros2_ws_glim/config/` (CT-ICP, `enable_imu: false` ×2) → bag 録画 → `glim_rosbag` オフライン評価
   → 必要なら StarROS2 に per-point `time` フィールド追加

従来のトリアージ指示書 `docs/issue/2026-07-07_monthly_2026_6_todo_triage.prompt.md` は再編完了後に再開 (優先順・完了条件つき)。
要約: A1 LiDAR FOV → A2 slam lifecycle → A3 rviz エラー確認 → A4 脱力モード修正 →
B1 エンコーダ根本修正 (⚠️3 点同時変更) → B2 ハンチング → B3 オドメトリ/SLAM 精度 → B4 R-Fans RViz → B5 LIO-SAM 回収。
加えて監査 Issue 1〜3 (安全系) を本番前に必ず。ros2_control 移行 (C1 案 3) はユーザ合意が出たら本命。
