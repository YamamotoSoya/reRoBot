<!-- claude: reRoBot 運用手引き (詳細版 README) のテンプレ (2026-09-21 作成)。
     docs/text/ の書籍形式 (00_index + 章分割) に合わせた骨格のみ。本文はユーザが執筆する。 -->

# reRoBot 運用手引き — 動かし方の詳細版

<!-- TODO: この文書は誰のために・何を・どこまで書くか (README との役割分担)。 -->

## 本書の構成

<pre>
reRoBot 運用手引き
├── <a href="01_overview.md">第1章 全体像</a> ...................... 01_overview.md
├── <a href="02_setup.md">第2章 初回セットアップ</a> ............ 02_setup.md
├── <a href="03_parameters.md">第3章 基本パラメータ</a> .............. 03_parameters.md
├── <a href="04_startup_teleop.md">第4章 起動と手動操作</a> .............. 04_startup_teleop.md
├── <a href="05_bag_recording.md">第5章 bag記録</a> ..................... 05_bag_recording.md
├── <a href="06_slam_toolbox.md">第6章 SLAM_toolbox</a> ................ 06_slam_toolbox.md
├── <a href="07_glim.md">第7章 GLIM</a> ........................ 07_glim.md
├── <a href="08_lio_sam.md">第8章 LIO_SAM</a> ..................... 08_lio_sam.md
├── <a href="09_map2d_compression.md">第9章 map2d圧縮</a> ................... 09_map2d_compression.md
├── <a href="10_nav2.md">第10章 Nav2</a> ....................... 10_nav2.md
├── <a href="11_amcl.md">第11章 amcl</a> ....................... 11_amcl.md
├── <a href="12_challenge_day.md">第12章 つくチャレ当日の実行手順</a> ... 12_challenge_day.md
└── <a href="13_troubleshooting.md">第13章 トラブルシューティング</a> ..... 13_troubleshooting.md
</pre>

## 読み方

<!-- TODO -->

## 凡例

| 記法 | 意味 |
|---|---|
| `$ cmd` | ホストで実行するコマンド |
| `# cmd` | コンテナ内 (`docker exec -it <container> bash` 後) で実行するコマンド |
| `path/to/file:123` | リポジトリ内の実コード位置 (行番号は YYYY-MM-DD 時点) |
| ✅ / ⚠️ / ❌ | 推奨 / 注意つきで可 / やってはいけない |

## 関連資料

- [README](../../README.md) — 最短セットアップ手順・ハードウェア udev 設定
- [CLAUDE.md](../../CLAUDE.md) — 開発規約・ビルド規則・アーキテクチャの要約
- `docs/claude/PROJECT_STATE.md` — 現在地・既知の問題・タイムライン
- `docs/text/` — テーマ別解説書 (仕組みの理解はこちら)
- `docs/issue/` / `docs/report/` — 問題の調査記録・事後報告

---

→ [第1章 全体像](01_overview.md)

---

## 構成メモ (案)

<!-- claude: テンプレ作成時 (2026-09-21) に考えた節構成案を、2026-09-22 のユーザ目次 (13 章) に振り直したもの。採用・不採用は自由。書き終わった章から消してよい。 -->

**読み方の入口 (案)**: 初めて触る人 = 1 → 2 → 3 → 4 章を順に / 動かし方だけ = 3 章 + 4 章 / 当日 = 12 章 + 13 章。

**第1章 全体像**
- ハードウェア構成 — 樹形図で車体 → 駆動系 (モータ×2、EPOS4×2、USB-CAN) / センサ (2D LiDAR、3D LiDAR、IMU) / 計算機 / 電源。配線図は `docs/reference/wiring_diagram.mmd` も参照可
- ソフトウェア構成 — コンテナ × workspace × 役割 × いつ使うか の表
- データの流れ — 指令系 (teleop/Nav2 → controller → CAN → モータ) とセンサ系 (LiDAR/IMU/encoder → odometry/EKF → SLAM/Nav2) の 2 本を図で (mermaid は `graph TD` / `flowchart LR` の基本記法のみ)
- ディレクトリ構成 — scripts/ docker/ ros2_ws_*/ docs/ tools/

**第2章 初回セットアップ**
- 前提 (OS / Docker / ディスク / ネットワーク)
- clone と submodule (--recursive、忘れたときの復旧)
- Docker イメージとコンテナ (1 本ずつビルド、profile の意味、入り方)
- ビルド (scripts/build.sh、なぜコンテナ内・なぜ直列か、所要時間)
- ホスト側デバイス設定 — 詳細は README に置き、ここは「デバイス | 固定名/IP | 確認コマンド」の表
- セットアップ完了チェックリスト

**第3章 基本パラメータ**
- 車体パラメータ (`config/params.yaml`: tread_width / tire_diam / gear_ratio / invert_left・right) — 意味・現在値・変えたら何が狂うか。teleop 側の複製と同期が必要 (`/params-sync`)
- センサ接続パラメータ (serial_port / device_ip / rps / imu_port / imu_rate) — launch 引数で上書きできるもの
- URDF のセンサ取付位置 (laser / rfans / imu_link) と rfans 取付プリセット (tilted45 / tilted15 / flat) — 切替時は GLIM の config も一緒に
- EKF (`config/ekf.yaml`) の融合設定、Nav2 (`config/nav2_params.yaml`) の速度上限など「よく触る値」の一覧表 (パラメータ | ファイル | 意味 | 現在値)

**第4章 起動と手動操作**
- 電源投入の順序と、間違えたときに起きること
- CAN バスの確認 (半死状態の見分け方・復旧)
- bringup の選択フロー (樹形図: 手動走行 / 2D 地図 / 自律走行 / 3D SLAM 評価 → 構成)。スクリプト経由と手動 launch の両方
- 正常起動の確認 (確認項目 | コマンド | 正常時 の表)
- 安全上の原則 (浮かせて確認 → 接地、非常停止手段)
- キーボード teleop (起動、キー割り当て表、速度スケール) / ゲームパッド teleop (起動、ボタン割り当て、ペアリング)
- 脱力モード (何が起きるか、いつ使うか、戻し方)、走行距離表示の読み方
- 停止の順序 (ROS プロセス → コンテナ → 電源)

**第5章 bag記録**
- 記録 (用途 | トピックセット | 目安サイズ の表、保存先)
- 再生 (use_sim_time、レート、実機ノードとの TF 衝突)
- ログの置き場所 (scripts のログ、launch のログ、/workspace/log)

**第6章 SLAM_toolbox**
- 起動 / 走り方のコツ (速度、ループを閉じる、RViz で見るもの)
- 保存 (.pgm .yaml .posegraph と置き場所)、地図の品質確認 (OK / NG の判断基準)

**第7章 GLIM**
- 起動 / 3D 特有の注意 (回転ディップ、IMU 静止開始)
- オフライン処理 (bag からの再処理、メモリ上限)、dump の保存

**第8章 LIO_SAM**
- 位置づけ (IMU 再入手まで凍結中) と、再開時に必要なもの

**第9章 map2d圧縮**
- GLIM 3D 地図 → 2D 占有格子 (tools/README.md: pointcloud_to_2dmap / glim_traj_to_2dmap)
- 地図の配置と命名、keepout マスクの作り方

**第10章 Nav2**
- 起動 (前提: 地図あり、IMU+EKF 構成)
- ゴール指定 (単一 / ウェイポイント、経路が出ないときの確認)
- 調整ポイント (症状 | 触る場所 | 方向 の表)
- 走行中の監視と介入 (停止・手動切替)、走行後 (ログ・bag の回収)

**第11章 amcl**
- 初期位置合わせ (2D Pose Estimate、粒子の収束の見方)
- 触ることが多いパラメータと、ずれたときの立て直し

**第12章 つくチャレ当日の実行手順**
- 前日チェックリスト (充電、地図、設定ファイル、bag 容量)
- 当日の時系列手順 (電源 → CAN → bringup → Nav2 → 初期位置 → 走行 → 停止・回収)
- 役割分担、非常停止の合図

**第13章 トラブルシューティング**
- 症状からの逆引き表 (症状 | まず疑う | 確認コマンド | 処置 | 詳細リンク)。候補行: モータが動かない / can0 が無い・受信ゼロ / /scan が出ない / 点群が出ない / IMU が出ない / TF が繋がらない / topic list に見えるが echo できない / Nav2 が経路を出さない / ビルドが落ちる
- 症状別の手順 (電源 → CAN → CiA402 状態 → controller の順で切り分け、など)。経緯は docs/issue/ docs/report/ にリンクし、ここは手順だけ

**章に割り当てていない案 (リファレンス系)** — 必要なら 13 章の後ろか各章末に
- コマンド早見表 / スクリプト一覧 (scripts/*.sh) / launch 一覧 / トピック一覧 / TF ツリー (map → odom → base_link → laser / rfans / imu_link) / 主要パラメータ表 / 用語集
