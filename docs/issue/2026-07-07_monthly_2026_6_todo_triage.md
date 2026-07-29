<!-- claude: docs/issue — 未解決問題の調査記録。解決したらステータスを更新すること。-->
# docs/monthly/2026_6 の TODO・問題のトリアージ (原因分析と対処法)

- **ステータス: 未解決** (項目ごとの状態は下表。本ファイルは分析、実作業の指示は同名の [.prompt.md](2026-07-07_monthly_2026_6_todo_triage.prompt.md) を参照)
- 日付: 2026-07-07
- 入力: `docs/monthly/2026_6` に記載された TODO / 抱えている問題
- 分析環境: **Windows チェックアウト上での静的解析のみ**。実機・Docker・ROS 2 が使えないため、
  動作検証が必要な項目は「デバッグ手順(実機/Ubuntu)」と「予想される原因」を記載する。
- 関連: [2026-07-07_repository_audit.md](2026-07-07_repository_audit.md) (リポジトリ全体監査、以下「監査 Issue N」と参照)、
  [2026-07-07_wheel_odometry_encoder_scaling_4x.md](2026-07-07_wheel_odometry_encoder_scaling_4x.md)

## サマリ

| # | monthly の記載 | 状態 |
|---|----------------|------|
| [T1](#t1) | teleop_keyboard で wheel_right しか動かない | 解決済み (要実機再確認) |
| [T2](#t2) | 脱力モードほしい / コードが複雑になり要確認 | 実装済み・レビュー未 |
| [T3](#t3) | slam の map 更新を何 m ごとにやるのか確認 | 解決 (本ファイルで回答) |
| [T4](#t4) | lifecycleEvent の理解 / slam_toolbox 起動を lifecycleEvent 不使用に修正 | 未解決 |
| [T5](#t5) | LiDAR の範囲修正 (多分ミスってる) | 未解決 |
| [T6](#t6) | laser_scan が "Error subscribing: Empty topic name" になる | おそらく解決済み (要確認) |
| [T7](#t7) | slam_toolbox の autostart: true が効かない原因特定 | 未解決 (調査手順あり) |
| [T8](#t8) | epos4 が実機移行した際に動かない (確率的) | おそらく解決済み (要実機確認) |
| [T9](#t9) | 車輪回転数が現実と Rviz で不一致 / gear_ratio 1.25 | **解決済み (2026-07-29)** 残: 接地確認 |
| [T10](#t10) | モータのハンチング | 未解決 (有力仮説あり) |
| [T11](#t11) | SLAM が絶妙にずれる (ホイールオドメトリ不足?) | 未解決 |
| [T12](#t12) | R-Fans: TF は OK だが RViz に表示できない | 未解決 (手順を記載) |
| [T13](#t13) | LIO-SAM を external 下に追加したはず | 未解決 (リポジトリに存在しない) |
| [T14](#t14) | epos4 の設定見直し (電流制限等) | 未解決 (EPOS Studio 作業) |

さらに monthly に「コードが複雑で理解できない」と書かれた 3 箇所について、
経緯・原因・簡素化の選択肢を [C1](#c1) (epos4_controller のスレッド群)、[C2](#c2) (lifecycleEvent)、
[C3](#c3) (nav2 launch/params) にまとめた。

---

<a id="t1"></a>
## T1: teleop_keyboard で wheel_right しか動かない

**状態: 解決済み (要実機再確認)**

- **原因 (確定済み)**: epos4 初期化サービス (init→enable→cyclic_velocity_mode) を応答を待たずに連射したことによるモード指令の未到達。詳細は `docs/report/2026-06-04_epos4_controller_init_race_dead_wheel.md`。
- **現行コードの状態**: `epos4_controller.cpp` は修正済み。init はワーカースレッド (`run_init_sequence`) で逐次実行し、SDO で statusword(0x6041)/mode(0x6061) を読み戻して検証、失敗時は recover を挟んで最大 5 回リトライする (`init_motor`)。
- **実機での確認方法**: 起動ログに `motorX ready (Operation Enabled, CSV) on attempt N` が両モータ分出ること。`FAILED to reach CSV` が出たら再発。

<a id="t2"></a>
## T2: 脱力モード — コードが複雑になったため要確認

**状態: 実装済み・レビュー未**

- **現行コードの状態**: `/robot_free_mode` (Bool) トグルとして実装済み (`epos4_controller.cpp:440-476`、teleop の `f` キー)。機能仕様は `docs/features/2026-06-05_motor_free_mode.md`。
- **静的レビューで見つけた問題 2 件** (動作を妨げるものではないが直すべき):
  1. **executor ブロック** (監査 Issue 6): 復帰 (OFF) 時、前回の `reenable_thread_` が走行中だと executor スレッドで `join()` がブロックし、TPDO 送信とサービス応答処理が最悪 30 秒近く止まる。ON→OFF 高速トグルで disable(async) と enable シーケンス(別スレッド) が競合するパターンもある。
  2. **init 中のトグル競合** (監査 Issue 24): 起動 ~5 秒 + リトライ中に `f` を押すと init シーケンスと enable/disable が並行してサービスを叩き合う。
- **対処法**: 復帰シーケンスに「実行中フラグ」を持たせ再トグルを拒否/キューイング。init 完了フラグを見て完了前の free_mode 要求は遅延させる。
- **「なぜこんなに複雑なのか」については [C1](#c1) を参照** (経緯と簡素化の選択肢)。

<a id="t3"></a>
## T3: slam の map 更新を何 m ごとにやるのか

**状態: 解決 (設定から回答確定)**

`src/rerobot_bringup/config/slam_toolbox.yaml` より:

- **map (占有格子) の再描画は距離ではなく時間ベース**: `map_update_interval: 5.0` → **5 秒ごと**。
- **ポーズグラフへのノード追加 (= 地図に反映されるスキャンの採用) が距離ベース**:
  `minimum_travel_distance: 0.5` (**0.5 m ごと**) または `minimum_travel_heading: 0.5` (**0.5 rad ごと**) のどちらか早い方。
- つまり「0.5 m / 0.5 rad 動くごとにスキャンが取り込まれ、見た目の /map は 5 秒ごとに更新される」が答え。
- 補足: `minimum_time_interval: 0.5` (スキャン処理の最小間隔 0.5 s)、`throttle_scans: 1` (間引きなし)。

<a id="t4"></a>
## T4: slam_toolbox 起動を lifecycleEvent 不使用に修正したい

**状態: 未解決**

- **現状**: `slam.launch.py` は launch の `EmitEvent`/`OnStateTransition` チェーンで CONFIGURE→ACTIVATE を発火している (slam_toolbox 公式 online_async_launch.py と同じパターン)。動作はするが理解しづらい、というのが TODO の動機。
- **対処法 (推奨)**: nav2.launch.py で既に使っている **`nav2_lifecycle_manager`** に管理させる方式へ置き換える。
  `lifecycle_manager` ノードを `node_names: ["slam_toolbox"]`, `autostart: true` で起動するだけで CONFIGURE→ACTIVATE を代行してくれる。
  - **注意点**: lifecycle_manager は既定で bond (ハートビート) 接続を要求するが、slam_toolbox は bond 非対応。`bond_timeout: 0.0` を設定して bond を無効化する必要がある。これを忘れると activate 後に「Have not received a response from bond」で kill される。
- **代替案**: T7 の調査で autostart が使える (or 使えるバージョンに上げられる) と判明すれば、EmitEvent チェーンを消して `autostart: true` に一本化するのが最小変更。

<a id="t5"></a>
## T5: LiDAR の範囲修正 (「多分ミスってる」)

**状態: 未解決 — 「ミスってる」の直感はおそらく正しい** (= 監査 Issue 11)

- **原因 (コードから確定)**: `rerobot_bringup_2d.launch.py:82-83` で `angle_min: -1.5708, angle_max: 1.5708` (±90° = 180°)。
  **UTM-30LX の実力は 270° (±135° = ±2.3562 rad)** なので、90° ぶん捨てている。SLAM/AMCL のマッチング品質 (特に横方向の壁) に直結する。
- **対処法**: 筐体・支柱の映り込みが無い範囲を実測し、可能な限り ±2.3562 に近づける。映り込みがある場合のみその角度だけ絞る。
- **実機でのデバッグ手順**:
  1. 一時的に `angle_min:=-2.3562 angle_max:=2.3562` で起動し、RViz で /scan を表示。
  2. 機体パーツが映る角度帯を特定 (LaserScan の Decay Time を数秒にすると見やすい)。
  3. その角度を除いた最大範囲を launch に反映。

<a id="t6"></a>
## T6: "Error subscribing: Empty topic name" (場合によって発生)

**状態: おそらく解決済み (要確認)**

- **予想される原因**: これは **RViz2 のディスプレイのトピック欄が空のときに出るエラー** (rclcpp は空文字トピックの subscribe で例外を投げ、RViz がこのメッセージを表示する)。「場合によって」= 旧・共通 `rerobot.rviz` 時代に、LaserScan 等のディスプレイでトピック未設定のものが残っていたと推測。
- **現行コードの状態**: rviz 設定は `slam.rviz` / `nav2.rviz` に分割済みで、**両ファイルを grep した限り空トピックのディスプレイは存在しない** (`Value: ""` なし)。よって再発しない見込み。
- **実機での確認方法**: `slam.launch.py` / `nav2.launch.py` をそれぞれ起動し、RViz のコンソール (起動端末) に同エラーが出ないことを確認。出た場合はどのディスプレイか RViz の Displays パネルでトピック欄が空のものを探して埋める。

<a id="t7"></a>
## T7: slam_toolbox の autostart: true が効かない原因

**状態: 未解決 (Windows では検証不可 — 調査手順を記載)**

- **予想される原因 (可能性順)**:
  1. **配布バイナリのバージョンが autostart 対応前**。slam_toolbox の lifecycle 化と autostart パラメータ対応は比較的新しく、Jazzy の apt バイナリ (2.8.x 系) のマイナーバージョンによっては autostart パラメータを読まない。
  2. autostart の実装が「configure のみ」で activate まで進まないバージョンだった。
  3. パラメータは効いているが、起動タイミングの問題 (TF/scan 未着) で activate 直後に inactive へ戻っていた。
- **実機/Ubuntu でのデバッグ手順**:
  1. `apt show ros-jazzy-slam-toolbox | grep Version` でバージョン確認。
  2. GitHub の slam_toolbox リポジトリ (jazzy ブランチ) で `autostart` を grep し、そのバージョンに実装が入っているか確認。
  3. `ros2 param get /slam_toolbox autostart` で実行時にパラメータが載っているか確認。
  4. `ros2 lifecycle get /slam_toolbox` で起動直後の状態遷移を観察 (unconfigured のままなら 1/2、inactive 止まりなら activate 側の問題)。
- **結論の使い道**: 対応版なら T4 は「autostart に一本化」で解決。非対応なら T4 の lifecycle_manager 方式を採用。

<a id="t8"></a>
## T8: epos4 が実機移行した際に動かない (確率的、build のバグ?)

**状態: おそらく解決済み (要実機確認)**

- **予想される原因 (コードと report から)**: 「確率で動かない」の主因は T1 と同じ **init サービスのレース** だった可能性が高い (症状の性質 = タイミング依存・確率的、が一致)。現行コードの逐次化 + SDO 検証 + リトライで解消しているはず。
- **残っている確率的要因 (念のため)**:
  1. bringup launch の 5 秒 TimerAction より device_manager の起動が遅いケース → ただし現行 controller は `wait_for_service(20s)` で待つため実害は出にくい。
  2. `colcon build` を `--executor sequential` なしで実行した場合の canopen スタックのビルド不整合 (CLAUDE.md 記載の既知問題)。「build の際のバグかも」という記憶はこれかもしれない。
  3. CAN インターフェース (can0) の up 忘れ / bitrate 不一致。
- **実機での確認方法**: 電源投入→bringup を 10 回程度繰り返し、毎回ログに両モータの `ready (Operation Enabled, CSV)` が出ること。出なかった回はログ全文を保存して `FAILED to reach CSV` / `not available` のどちらで落ちたかを記録する。

<a id="t9"></a>
## T9: 車輪回転数が現実と Rviz で不一致 (gear_ratio 1.25 で対症)

**状態: 解決済み (2026-07-29 根本修正適用・浮かせ検証 OK。残: 接地での最終確認)** — EPOS 側は正しく (0x3010:01=256 実測)、bus.yml `2π/1024` + `gear_ratio: 5.0` の 2 点同時修正で解決。副次発見: 修正前は**実車が指令速度の 1/4 で走っていた** (T10 ハンチング・「Nav2 が遅い」の再評価材料)。詳細は下記 issue の「実測記録 2026-07-29」。

- 詳細分析は既存 issue に完結している → [2026-07-07_wheel_odometry_encoder_scaling_4x.md](2026-07-07_wheel_odometry_encoder_scaling_4x.md)
- 要点: monthly の推測「EPOS Studio でギヤ比 4:1 に誤設定」は**否定** (ズレ係数が 4 ちょうどで 5/4=1.25 と合わない)。
  真因は `bus.yml` の `scale_pos_from_dev = 2π/4096` が実機エンコーダ (1024 inc/rev = 256 パルス × 4 逓倍) と 4 倍食い違っていること。
- monthly の TODO「epos studio の設定を見る」はそのまま有効: **0x3010:01 (エンコーダパルス数) の実値確認**が根本修正の第一歩 (同 issue の「未検証事項」参照)。

<a id="t10"></a>
## T10: モータのハンチング

**状態: 未解決 (Windows では検証不可 — 有力仮説あり)**

- **予想される原因 (可能性順)**:
  1. **★T9 のエンコーダ分解能誤設定が速度ループを狂わせている (有力・新仮説)**。
     EPOS4 内蔵の速度制御ループはエンコーダ分解能設定 (0x3010:01) を前提にゲインを解釈する。
     設定が実機の 4 倍なら、ドライブは実速度を 1/4 に誤認し、**実効ループゲインが 4 倍**になる。
     ゲイン過大はハンチング (振動的挙動) の古典的原因であり、T9 と同一根源で説明がつく。
     → **T9 の根本修正を先にやり、ハンチングが消えるか確認するのが最短経路。**
  2. **無負荷 (タイヤ浮かせ) での挙動は参考にならない**: monthly の「タイヤを浮かせた状態だとバチ止め」はその通りで、
     慣性・摩擦が実走行と桁違いのため、浮かせた状態でのチューニング/評価は無意味。接地状態で評価すること。
  3. EPOS Studio の Regulation Tuning (速度レギュレータ) が未実施 or 無負荷で実施された。
- **monthly で試して効果がなかったこと (記録)**: sync_period 50ms→10ms + 0x60C2 50ms→10ms → 変化なし (現行 bus.yml は 50ms のまま)。
  0x6060=3 (PVM) デフォルト → 後で CSV(9) 上書き → 影響なし。この 2 つは容疑から除外してよい。
- **実機でのデバッグ手順**:
  1. まず T9 根本修正 (エンコーダ分解能 → bus.yml → gear_ratio=5.0 の 3 点セット) を実施。
  2. 接地 + 実負荷で EPOS Studio の Data Recorder を使い、velocity demand vs actual を記録。ハンチングの周波数と振幅を定量化。
  3. 残るなら EPOS Studio の Auto Tuning を接地状態 (またはローラー台) で再実行。
  4. ソフト側の微修正として監査 Issue 7 (`static_cast<int>` → `std::lround`) も適用 (低速の ±1rpm 量子化を除去)。

<a id="t11"></a>
## T11: SLAM が絶妙にずれる (ホイールオドメトリだけでは不十分?)

**状態: 未解決 (複合要因 — 改善順序を提示)**

「9 号館 map で存在しない空間を作る」「改善点: オドメトリ弱い」も同根とみなす。寄与が大きい順に:

1. **T9 のスケーリング問題**: 現在は gear_ratio 1.25 で数値上補正されているが、根本修正と実測検証 (直進 10 m、その場 360° 旋回) が先。
   特に **回転 (yaw) の精度は tread_width (0.41 m) の正確さに直結** — 360° 旋回で /odom の yaw が 2π に戻るか検証し、ずれたら tread_width を較正する。
2. **LiDAR FOV が ±90° に絞られている (T5)**: スキャンマッチングの拘束が減り、廊下等でずれやすくなる。T5 の修正で改善が見込める。
3. **IMU 非搭載・未融合**: robot_localization は Docker に入っているが未配線 (監査 Issue 12: /odom の共分散も全ゼロで、融合するなら先に設定必須)。
   ホイールオドメトリの yaw ドリフトは IMU 融合が定石。中期課題。
4. slam_toolbox のループ閉じ込みパラメータは現状ほぼデフォルト — 上記 1〜3 を直してから触る (先にチューニングすると原因を隠す)。

<a id="t12"></a>
## T12: R-Fans — TF は OK だが RViz に表示できない (設定わからない)

**状態: 未解決 (Windows では検証不可 — 手順と予想原因を記載)**

- **予想される原因 (可能性順)**:
  1. **Fixed Frame の不一致**。monthly のメモ時点ではドライバの frame_id デフォルトが `world` だったが、
     現在は `params_3d.yaml` で **`rfans`** に変更済み。RViz の Fixed Frame を `world` のままにしていると
     TF が引けず PointCloud2 ディスプレイが `Frame [world] does not exist` 系で表示されない。
     → bringup_3d と同時起動なら Fixed Frame = `odom` か `base_link`、ドライバ単体なら **`rfans`** にする。
  2. **トピック名**: 出力は `/sdk_could` (typo だが仕様)。ディスプレイのトピックを `/sdk_could` に設定しているか。
  3. QoS は `rclcpp::QoS(10000)` = Reliable/Volatile (rfans_driver.cpp:135-136) なので RViz デフォルトと互換 — QoS 起因ではない。
- **実機での確認手順**:
  1. `ros2 topic hz /sdk_could` で流量確認 (これが出ていればドライバは正常)。
  2. `ros2 run tf2_tools view_frames` で `rfans` フレームの接続確認 (bringup_3d 起動時のみ base_link に繋がる)。
  3. RViz: Fixed Frame を上記どおり設定 → Add → By topic → /sdk_could/PointCloud2。
- **恒久対処**: 3D 用の rviz 設定ファイル (例: `rviz/rfans.rviz`) を作って手順を固定化する。
  なお **3D 構成では /scan が無いため slam/nav2 はそのままでは動かない** (監査 Issue 10) 点も忘れないこと。

<a id="t13"></a>
## T13: LIO-SAM を external 下に追加 (したはず)

**状態: 未解決 — リポジトリに存在しない (コミット漏れの疑い)**

- **確認した事実**: `.gitmodules` に LIO-SAM のエントリなし。`src/external/` は COLCON_IGNORE / StarROS2 / epos4compact50-5can のみ。
  Dockerfile にも LIO-SAM の依存 (gtsam 等) が無い。
- **予想される原因**: 実機 PC のワークツリーでは追加したが、`git submodule add` の結果 (.gitmodules と gitlink) を
  コミット/プッシュしていない。「依存関係を DockerFile に追加」もローカル変更のままの可能性。
- **確認方法 (実機 PC で)**: `git status` と `git diff` で .gitmodules / Dockerfile / src/external の未コミット変更を確認。
  あればコミット、なければ改めて `git submodule add` からやり直し。
- **補足**: LIO-SAM を使うなら IMU が必須 (9 軸推奨)。T11-3 (IMU 導入) との整合を先に決めること。

<a id="t14"></a>
## T14: epos4 の設定見直し (電流制限等)

**状態: 未解決 (EPOS Studio 作業 — リポジトリ外)**

- コード側から言えること: bus.yml の SDO 初期化列 (0x6081 profile velocity=1000, 0x6083 profile acceleration=2000 等) は
  プロファイルモード用で、**CSV 運転時の電流/トルク制限には効かない**。電流制限は EPOS4 本体設定 (EPOS Studio) の管轄。
- T9/T10 の作業で EPOS Studio を開くとき、以下も同時に記録しておくと後で効く:
  モータ定格電流・ピーク電流設定、Thermal time constant、velocity regulator のゲイン、エンコーダ設定 (0x3010:01)。
  **設定一式をエクスポートして `docs/` 配下に保存しておくこと** (実機 NVM にしか無い状態はリスク)。

---

# 補足: 「コードが複雑で理解できない」箇所の経緯・原因・簡素化の選択肢

monthly に「複雑」「理解できていない」と書かれた 3 箇所について、なぜそうなったか (経緯は
docs/report・docs/features に記録あり) と、より簡単に書けるならその方法をメモする。

<a id="c1"></a>
## C1: epos4_controller のスレッド + future + SDO 検証の増殖 (T2 の背景)

### 経緯 (詳細: `docs/report/2026-06-04_...init_race_dead_wheel.md`, `docs/features/2026-06-05_motor_free_mode.md`)

1. **init レース修正 (6/4)**: コンストラクタからサービスを「撃ちっぱなし」にすると片輪が死ぬ問題を、
   ワーカースレッド (`init_thread_`) + 同期呼び出し (`call_trigger_sync`) + SDO 検証 + リトライで修正。
2. **脱力モード (6/5)**: 初版はこの重装備を丸ごと再利用 → §9 で「復帰時に init(homing) を呼ぶと復帰不能」と判明し除去
   → §10 で「過剰」として非同期投げっぱなしに最小化 → §11 で「並行発行だと mode 設定がレースして左輪が遅れる」と再発し、
   **enable→csv だけ短命スレッド (`reenable_thread_`) で逐次化**に落ち着いた。さらに「no-op 遷移でサービスが
   success=false を返す偽陰性」が見つかり、成否判定を SDO 読み (`motor_ready`) に変更。
3. つまり現在の形は行き当たりばったりではなく、**「並行だと壊れる」「init を呼ぶと壊れる」「戻り値は信用できない」を
   1 つずつ実機で踏んで到達した形**。複雑さの大半は消せない要件を反映している。
   - 注意: features 文書の §8 には「復帰は init_motor() を再利用」と古い記述が残っている (§9 以降で否定)。
     読むときは §9→§10→§11 が最終形。

### 複雑さの本質的原因 (2 つの掛け算)

- **(a) rclcpp の制約**: シングルスレッド executor では、コールバックやコンストラクタの中でサービス応答を
  同期待ちするとデッドロックする。→ 「順番に待つ」処理は毎回スレッドを自作するしかなかった。
  これが `init_thread_` と `reenable_thread_` の 2 本に増殖した直接原因。
- **(b) CiA402 の性質**: ①遷移は逐次でないと取りこぼす、②サービス戻り値は no-op 遷移で偽陰性になる、
  ③本当の状態は SDO (0x6041/0x6061) でしか分からない。→ 検証コードは削れない。

### より簡単に書く方法

- **案 1 (小改修・推奨): 遷移シーケンサを 1 本に統合。**
  「Trigger サービス列を順に実行し、最後に motor_ready で検証する」処理は init と復帰で同型なので、
  **常駐ワーカースレッド 1 本 + ジョブキュー** (`std::deque<std::vector<Step>>` + condition_variable) に集約する。
  init も脱力復帰も「ジョブを積むだけ」になり、`join()` 由来の executor ブロック (監査 Issue 6) と
  init/free_mode の並行競合 (監査 Issue 24) が**構造的に消滅**する。スレッドは減り、コードの重複も消える。
- **案 2 (中改修・非推奨): MultiThreadedExecutor + Reentrant callback group。**
  コールバック内で future を待てるようになりスレッド自作は不要になるが、全共有状態 (`m1_value_` 等) の
  排他を自分で管理する必要が生じ、**今より理解が難しくなる**可能性が高い。
- **案 3 (大改修・将来の本命): ros2_control への移行。**
  ros2_canopen には ros2_control 統合 (`canopen_ros2_control/cia402_system`) があり、
  `diff_drive_controller` と組み合わせると **epos4_controller (IK・状態管理) と epos4_odometry (オドメトリ) の
  自作コードがほぼ全部標準スタックに置き換わる**。CiA402 遷移は controller_manager のライフサイクルが面倒を見て、
  cmd_vel タイムアウト (監査 Issue 1 のウォッチドッグ!) や odom 共分散 (Issue 12) も diff_drive_controller が標準装備。
  学習コストと移行リスクは大きいが、「複雑さの根源 (自前の CiA402 管理) を捨てる」唯一の選択肢。
  Dockerfile に `ros-jazzy-ros2-control` は導入済み。

<a id="c2"></a>
## C2: slam.launch.py の lifecycleEvent チェーン (T4 の背景)

### 経緯 (slam.launch.py / slam_toolbox.yaml のコメントに記録)

Jazzy の slam_toolbox は managed (lifecycle) node で、起動直後は `unconfigured` のまま待機する。
`autostart: true` を yaml に入れても効かなかった (T7) ため、slam_toolbox 公式 launch
(online_async_launch.py) と同じ **EmitEvent/OnStateTransition チェーン**を移植した。
つまり複雑なのは公式のワークアラウンドをそのまま持ってきたから。

### 仕組みの要約 (理解用 — これだけ知れば読める)

- lifecycle node は `unconfigured → inactive → active` の 3 段で起動する。
- `EmitEvent(ChangeState(..., TRANSITION_CONFIGURE))` = launch から「configure しろ」というコマンドを 1 発送る。
- `RegisterEventHandler(OnStateTransition(start_state="configuring", goal_state="inactive", ...))` =
  「configure が完了して inactive に達したのを見届けたら、次 (activate) を送る」というイベント連鎖。
- 要するに **「configure → (完了を待って) → activate」を launch の言葉で書いただけ**。

### より簡単に書く方法

T4 に記載のとおり **`nav2_lifecycle_manager` に置き換えるのが最も読みやすい**
(`node_names: ["slam_toolbox"]`, `autostart: true`, `bond_timeout: 0.0` — nav2.launch.py で既に
2 回使っているパターンなので、リポジトリ内の書き方が 1 種類に統一される利点もある)。
T7 の調査で autostart が使えると分かれば、チェーン削除 + `autostart: true` がさらに短い。

<a id="c3"></a>
## C3: nav2.launch.py / nav2_params.yaml (「claude 任せで理解できていない」の背景)

### 経緯

- nav2 公式の `nav2_bringup` (全部入り launch) を使わず、必要な 8 ノードだけを列挙する**最小構成を手書き**した。
  その分ファイルは長いが、「何が起動しているか」が全て見える構成になっている。
- `bt_navigator` の `plugin_lib_names` を明記したら内部デフォルトとの二重登録で segfault した教訓 (monthly 記載) は
  `nav2_params.yaml` の bt_navigator セクションのコメントに反映済み。

### 複雑さの本質的原因

**Nav2 自体が「約 8 個の lifecycle サーバの連邦」であり、これはどう書いても消えない複雑さ。**
launch が長いのはコードが悪いのではなく、構成要素が多いから。各ノードの役割は 1 行ずつ:

| ノード | 役割 |
|--------|------|
| map_server | 保存済み地図を /map に配信 |
| amcl | /scan と /map を照合して map→odom TF を出す (自己位置推定) |
| filter_mask_server | keepout マスク画像を配信 (map_server の別インスタンス) |
| costmap_filter_info_server | マスク値→コストの変換則を配信 |
| planner_server | 地図全体での経路計画 (/plan) |
| controller_server | 経路追従の速度指令生成 (+内部に local_costmap) |
| behavior_server | 復帰行動 (旋回・後退・待機) |
| bt_navigator | 上記を Behavior Tree で統括する司令塔 |
| lifecycle_manager ×2 | 上記をまとめて configure→activate する世話役 |

### より簡単に書く方法 (トレードオフあり)

- **`nav2_bringup` の `localization_launch.py` + `navigation_launch.py` を include する**と自作 launch は
  数十行に減る。ただし (1) 内部で何が起きるかがブラックボックス化する、(2) keepout filter は結局
  params 側に同じ記述が必要、(3) `/cmd_vel → /robot_speed_cmd` の remap を include 越しに効かせる工夫が要る。
- **推奨は現状維持 + この表で理解すること**。「デバッグに移る前に理解すべし」という monthly の TODO に対しては、
  読み順 = 上の表の順 (データの流れ順) で launch とコメントを読むのが最短。手書き最小構成は
  デバッグ時に切り分けやすい (ノードを 1 個ずつ外せる) という実利もある。
