<!-- claude: 次の Claude エージェントへの作業指示書。
     分析・根拠は同名の 2026-07-07_monthly_2026_6_todo_triage.md を先に読むこと。
     この指示書は Ubuntu 実機 (Docker コンテナ rerobot_env) での作業を前提とする。 -->
# 作業指示: monthly/2026_6 TODO の消化 (次エージェント用)

## 前提・制約 (必読)

- **先に読む**: [2026-07-07_monthly_2026_6_todo_triage.md](2026-07-07_monthly_2026_6_todo_triage.md) (各タスクの原因分析)、
  [2026-07-07_repository_audit.md](2026-07-07_repository_audit.md)、
  [2026-07-07_wheel_odometry_encoder_scaling_4x.md](2026-07-07_wheel_odometry_encoder_scaling_4x.md)、CLAUDE.md。
- 環境: Ubuntu ホスト + Docker コンテナ `rerobot_env` (ROS 2 Jazzy)。ビルドは必ず
  `colcon build --symlink-install --executor sequential` (並列ビルド禁止 — canopen スタックの既知問題)。
- **タスクごとにブランチを切る** (例: `fix/lidar-fov`)。main へ直接コミットしない。
- `docs/monthly/` は編集禁止 (ユーザの月報)。作業結果は各 issue ファイルのステータス更新 + 必要なら docs/report/ に報告書。
- **タスク完了時は該当 issue ファイルのステータスを「解決済み (日付, コミット)」に更新すること。**
- モータが動くタスクは必ず「タイヤを浮かせた状態で最初の動作確認 → 接地して本検証」の順。緊急停止 (電源断) をすぐ実行できる状態で行う。

## タスク一覧 (推奨順)

実機なしでできる順 + 依存関係順に並べてある。A 群はコード/設定のみ、B 群は実機必須。

---

### A1. LiDAR FOV の修正 (T5 / 監査 Issue 11)

1. `src/rerobot_bringup/launch/rerobot_bringup_2d.launch.py` の `angle_min/angle_max` を `-2.3562 / 2.3562` (±135°) に広げる。
2. 実機で /scan を RViz 表示し、機体パーツの映り込み角度帯を特定 (LaserScan の Decay Time を 3s 程度に)。
3. 映り込みがあればその分だけ絞った値に調整し、launch のコメントに「なぜこの値か」を明記する。
4. 完了条件: SLAM を短時間走らせ、壁の直線が従来より安定して出ること。映り込みが障害物として map に残らないこと。

### A2. slam_toolbox の lifecycle 起動整理 (T4 + T7)

1. まず調査 (T7): コンテナ内で
   ```bash
   apt show ros-jazzy-slam-toolbox | grep Version
   ros2 launch rerobot_bringup slam.launch.py &   # 起動して
   ros2 param get /slam_toolbox autostart
   ros2 lifecycle get /slam_toolbox
   ```
   autostart パラメータが実装済みか、GitHub の slam_toolbox (jazzy ブランチ、該当バージョンのタグ) も確認。
2. 結果に応じて `slam.launch.py` を修正 (lifecycleEvent チェーンの仕組みと置き換え方針は triage の
   [C2](2026-07-07_monthly_2026_6_todo_triage.md#c2) 参照):
   - autostart が機能する → EmitEvent/OnStateTransition チェーンを削除し `autostart: true` に一本化。
   - 機能しない → `nav2_lifecycle_manager` 方式に置き換え:
     `node_names: ["slam_toolbox"]`, `autostart: true`, **`bond_timeout: 0.0` を必ず設定** (slam_toolbox は bond 非対応。忘れると activate 後に kill される)。
3. 調査結果 (バージョン・原因) を triage ファイルの T7 に追記してステータス更新。
4. 完了条件: `ros2 lifecycle get /slam_toolbox` が起動後自動で `active` になり、/map が出ること。

### A3. "Empty topic name" の再発確認 (T6)

1. `slam.launch.py` / `nav2.launch.py` をそれぞれ起動し、RViz 起動端末のログに `Error subscribing: Empty topic name` が出ないことを確認するだけ。
2. 出た場合: RViz の Displays パネルでトピック欄が空のディスプレイを特定し、該当 .rviz を修正。
3. 完了条件: 両 launch で当該エラーなし → T6 を解決済みに更新。

### A4. 脱力モードのコードレビュー反映 (T2 / 監査 Issue 6, 24)

**着手前に triage の [C1](2026-07-07_monthly_2026_6_todo_triage.md#c1) を読むこと** (このコードが複雑な経緯と、
実機で踏んだ罠 3 つ: 並行発行 NG / 復帰時 init NG / 戻り値は偽陰性あり。修正時にこれらを再導入しないこと)。

実装は 2 案からユーザに確認して選ぶ:
- **案 A (最小)**: `std::atomic<bool> reenable_running_` で実行中の再トグルを WARN + 拒否 (join で executor をブロックしない)。
  init 完了フラグ (`init_done_`) で init 未完了時の free_mode 要求を拒否。
- **案 B (推奨・C1 案 1)**: 常駐ワーカー 1 本 + ジョブキューの「遷移シーケンサ」に統合し、
  init と脱力復帰を同じ仕組みに載せる。Issue 6 と 24 が構造的に消え、スレッドとコード重複が減る。

検証 (どちらの案でも): ベンチ (タイヤ浮かせ) で f 連打 / 起動直後の f / 通常のトグル、の 3 ケースで両輪が正しく脱力・復帰すること。
完了条件: 上記 3 ケースで TPDO 送信が止まらない (`ros2 topic hz /motor1/cia402_device_1/tpdo` が 100Hz を維持)。

---

### B1. エンコーダスケーリングの根本修正 (T9 → T10 の前提) ★実機 + EPOS Studio 必須

**⚠️ 3 点同時変更が必須。単体適用すると 4 倍速で走る。手順の詳細と根拠は encoder_scaling_4x issue の「根本修正の手順」を厳守。**

1. 事前検証 (変更前に必ず):
   ```bash
   ros2 service call /motor1/cia402_device_1/sdo_read canopen_interfaces/srv/CORead "{index: 0x3010, subindex: 1}"
   ```
   + モータ軸手動 1 回転で 0x6064 の増分を記録 (1024 なら仮説確定)。
   + candump で 0x60FF に届く実値を確認 (tpdo トピックのスケーリング有無の確定)。
2. タイヤを浮かせた状態で 3 点同時変更: EPOS Studio 0x3010:01 → 実パルス数 / bus.yml の scale_pos_from_dev・scale_pos_to_dev / gear_ratio → 5.0 (5 箇所: params_2d ×2, params_3d ×2, epos4_teleop ×1)。
3. 検証: teleop 直進 10 m をメジャー実測と突き合わせ / その場 360° 旋回で /odom yaw が 2π / 指令 0.5 m/s の実速度をストップウォッチ確認。
4. bus.yml は submodule 内なので `src/external/epos4compact50-5can` 側にコミット → 親リポジトリで gitlink 更新。
5. EPOS Studio の設定一式をエクスポートして `docs/` 配下に保存 (T14 と同時にやる)。

### B2. ハンチング検証 (T10) — B1 の後に実施

1. B1 完了後、接地状態でハンチングが残るか確認 (triage T10 の仮説 1: 分解能誤設定によるループゲイン 4 倍が主因なら B1 で消えるはず)。
2. 残る場合: EPOS Studio Data Recorder で velocity demand vs actual を記録 → 接地 or ローラー台で Auto Tuning 再実行。
3. 併せて `epos4_controller.cpp` の `static_cast<int>` を `std::lround` に修正 (監査 Issue 7、低速量子化の除去)。
4. 完了条件: 0.1〜0.4 m/s の定速指令で速度振動が目視・Data Recorder 上で消えること。

### B3. オドメトリ/SLAM 精度改善 (T11) — B1, A1 の後に実施

1. 360° 旋回テストで yaw のずれを測り、必要なら `tread_width` を較正 (ずれ率ぶん補正)。
2. 監査 Issue 4 の検証: 走行中 `ros2 topic echo /odom --field twist` — 全ゼロなら bus.yml TPDO に 0x606C を追加 (B1 で bus.yml を触るとき同時にやると効率的)。
3. `/odom` に共分散を設定 (監査 Issue 12。将来の robot_localization 導入準備)。
4. 完了条件: 同じ 9 号館ルートで SLAM を再実行し、「存在しない空間」が出ないこと。

### B4. R-Fans の RViz 表示 (T12)

1. `ros2 topic hz /sdk_could` で流量確認 → RViz: Fixed Frame を `rfans` (単体時) または `base_link` (bringup_3d 時) にして Add → By topic → /sdk_could。
2. 表示できたら 3D 用 `rviz/rfans.rviz` を作成し、`rerobot_bringup_3d.launch.py` から起動できるようにする (2D の方針に合わせ、別 launch でも可)。
3. 完了条件: 点群が z 方向に広がって表示される (z=0 ペタンコ問題は解決済みのはず — 再発したら docs/report/2026-06-13 を参照)。

### B5. LIO-SAM のコミット漏れ回収 (T13)

1. 実機 PC のワークツリーで `git status` / `git diff` を確認。LIO-SAM の submodule 追加・Dockerfile 変更が未コミットで残っていれば整理してコミット。
2. 無ければ `git submodule add <LIO-SAM の fork or 本家> src/external/LIO-SAM` からやり直し。依存 (gtsam) を Dockerfile に追加。
3. **着手前にユーザへ確認すること**: LIO-SAM は IMU 必須。IMU 搭載計画が未定なら、このタスクは保留にして先に進む。

---

## 触らないでよいもの (確認済み・解決済み)

- T1 (右輪のみ動く) は解決済み — 実機で起動ログの `ready (Operation Enabled, CSV)` を確認するだけでよい。
- T3 (map 更新間隔) は回答済み — 作業不要。
- T8 (実機移行で動かない) は T1 の修正で解消している見込み — B 群の作業中に再発したときだけログを保存して調査。

## 参考: 「複雑で理解できない」箇所について

- nav2.launch.py / nav2_params.yaml は **作業不要** — 複雑さは Nav2 自体の構成要素の多さ由来で、
  ノード役割表と読み順を triage の [C3](2026-07-07_monthly_2026_6_todo_triage.md#c3) にまとめた。
  簡素化 (nav2_bringup include) は透明性が下がるため推奨しない。
- epos4_controller の抜本簡素化 (ros2_control + diff_drive_controller 移行、C1 案 3) は
  監査 Issue 1 のウォッチドッグまで標準装備になる本命だが大改修。**ユーザの合意なしに着手しないこと。**

## このリストに含めていないが本番前に必須のもの

監査 Issue 1〜3 (速度指令ウォッチドッグ / 停止保証 / 速度平滑化) は monthly の TODO には無いが、
つくばチャレンジ本番前に必ず対応すること。詳細は [2026-07-07_repository_audit.md](2026-07-07_repository_audit.md)。
