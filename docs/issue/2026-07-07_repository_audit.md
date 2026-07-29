# reRoBot Issue 一覧 (リポジトリ全体監査)

リポジトリ全体監査 (2026-07-07, Claude Code) で見つかった潜在的問題の一覧。
各 issue の冒頭に **状態** (`未解決` / `解決済み` / `未解決(要実機検証)` / `未解決(設計判断)`) を記載する。
解決したら状態を `解決済み (日付, コミット)` に書き換えること。

深掘りが必要な問題は本ディレクトリの慣例どおり 1 issue = 1 ファイルで別途起票する。
既存の個別 issue: [2026-07-07_wheel_odometry_encoder_scaling_4x.md](2026-07-07_wheel_odometry_encoder_scaling_4x.md)
(エンコーダ分解能の 4 倍ズレ — **2026-07-29 解決済み**: bus.yml 2π/1024 + gear_ratio 5.0 の 2 点同時修正を適用。以下の本文中の「gear_ratio 1.25」記述は監査時点 (2026-07-07) のスナップショット)。

## サマリ

| # | 状態 | 重要度 | 概要 |
|---|------|--------|------|
| [1](#issue-1) | 未解決 | 🔴 高 | `/robot_speed_cmd` にウォッチドッグがなく、指令元が死ぬと暴走が止まらない |
| [2](#issue-2) | 未解決(要実機検証) | 🔴 高 | Ctrl-C 時に disable がドライブに届かない + heartbeat consumer 未設定 |
| [3](#issue-3) | 未解決 | 🔴 高 | Nav2 → controller 直結で速度平滑化・衝突監視なし |
| [4](#issue-4) | 未解決(要実機検証) | 🟡 中 | `/odom` の twist が常時 0 の疑い (bus.yml の 0x606C 未マッピング) |
| [5](#issue-5) | 未解決 | 🟡 中 | teleop の invert_left/right が controller/odometry と不一致 |
| [6](#issue-6) | 未解決 | 🟡 中 | 脱力モード復帰時の join で executor が長時間ブロックしうる |
| [7](#issue-7) | 未解決 | 🟡 中 | 目標 rpm の `static_cast<int>` 切り捨てによるデッドバンドと左右非対称 |
| [8](#issue-8) | 未解決 | 🟡 中 | odometry の同期が片系停止で警告なしに止まる |
| [9](#issue-9) | 未解決 | 🟡 中 | `ROS_DOMAIN_ID=150` が Linux の安全範囲外 (discovery が散発的に失敗しうる) |
| [10](#issue-10) | 未解決(設計判断) | 🟡 中 | 3D 構成では SLAM / Nav2 の入力 (/scan) が存在しない |
| [11](#issue-11) | 未解決(意図確認) | 🟡 中 | urg_node の FOV を ±90° に絞っている (UTM-30LX は 270°) |
| [12](#issue-12) | 未解決 | 🟡 中 | `/odom` の共分散が全ゼロ (robot_localization 導入時に破綻) |
| [13](#issue-13) | 未解決 | 🟡 中 | launch に respawn 指定がなく、ノード死亡が無検知・無復帰 |
| [14](#issue-14) | 未解決 | 🟠 低 | nav2.rviz の「局所経路」表示 (/local_plan) は RPP では常に空 |
| [15](#issue-15) | 未解決 | 🟠 低 | keyboard teleop が idle でも 20 Hz で 0 を publish し続け、Nav2 と競合する |
| [16](#issue-16) | 未解決 | 🟠 低 | CLAUDE.md の「motor1 = left wheel」が実配線と逆 / コード内ログも逆ラベル |
| [17](#issue-17) | 未解決 | 🟠 低 | CLAUDE.md のアーキテクチャ記述 (/robot_encoder_states) が現状と不一致 |
| [18](#issue-18) | 未解決 | 🟠 低 | README のシンボリックリンク復旧コマンドのパスが誤り + typo |
| [19](#issue-19) | 未解決 | 🟠 低 | nav2 の map_dir 既定値が日付固定 / nav2_params.yaml のコメントが launch と不一致 |
| [20](#issue-20) | 未解決 | ⚪ 軽微 | epos4_vel_test: cin fail で無限ループ / メニュー 8 で即座にモータが動く |
| [21](#issue-21) | 未解決 | ⚪ 軽微 | epos4_controller のデッドコード (未使用 publisher / client) |
| [22](#issue-22) | 未解決 | ⚪ 軽微 | epos4_controller/package.xml の maintainer が maxon サンプルの残骸 |
| [23](#issue-23) | 未解決 | ⚪ 軽微 | bus_config launch の `slave_eds_path` が存在しないファイルを指す (未使用) |
| [24](#issue-24) | 未解決 | ⚪ 軽微 | 起動 5 秒以内の脱力トグルが init シーケンスと競合 |
| [25](#issue-25) | 未解決 | ⚪ 軽微 | Dockerfile の不要パッケージ (nav2-minimal-tb*, 未配線の robot_localization) |

---

## 🔴 安全性 (実走行でのリスク)

<a id="issue-1"></a>
### Issue 1: `/robot_speed_cmd` にウォッチドッグがない

**状態: 未解決** | 重要度: 高 (最優先)

`src/epos4_controller/src/epos4_controller.cpp:221-233` の 100 Hz タイマは、最後に受けた指令値を無条件に再送し続ける。指令元 (teleop / Nav2) がクラッシュ・SIGKILL・無線切断で消えると、ロボットは最後の速度で走り続ける。

- joy_teleop の deadman (LB) も joy_node 自体が死ねば無力。
- keyboard teleop のデストラクタでの stop 送信 (`teleop_keyboard.cpp:87-92`) は異常終了では実行されない。

**対処案**: controller 側に「最後の Twist 受信から N ms (例: 500 ms) 経過したら目標を 0 にする」タイムアウトを実装する。つくばチャレンジの安全要件としてもほぼ必須。

<a id="issue-2"></a>
### Issue 2: Ctrl-C 時に disable がドライブに届かない + heartbeat consumer 未設定

**状態: 未解決(要実機検証)** | 重要度: 高

`epos4_controller.cpp:208-219` の `shutdown_node()` はデストラクタから呼ばれるが、その時点では SIGINT ハンドラが context を shutdown 済みのため `wait_for_service(1s)` が false を返し、disable は送信されない。

さらに `bus.yml` は `heartbeat_producer` (スレーブ→マスタ) のみで、スレーブ側の heartbeat consumer / life guarding が未設定。マスタ側プロセスが消えても EPOS4 は CSV の最終目標速度を保持し続ける可能性がある。

**検証項目**: bringup launch を `kill -9` したとき車輪が止まるか (ros2_canopen の shutdown 処理が NMT 停止してくれるかに依存)。

**対処案**: (a) SIGINT を自前でハンドリングして shutdown 前に disable を同期送信する、(b) bus.yml に heartbeat consumer を設定してマスタ喪失時にドライブ側で停止させる。

<a id="issue-3"></a>
### Issue 3: Nav2 → controller 直結で速度平滑化・衝突監視なし

**状態: 未解決** | 重要度: 高

`nav2.launch.py` は RPP の出力を `/robot_speed_cmd` に直結している。`nav2_velocity_smoother` も `collision_monitor` も無い。CSV モードは加速度制限なしで目標に即時追従する (bus.yml の 0x6083 profile acceleration はプロファイルモードにしか効かない) ため、リカバリ行動の急反転などがそのまま機体に出る。

**対処案**: nav2_velocity_smoother を controller_server と `/robot_speed_cmd` の間に挟む。余裕があれば collision_monitor も。

---

## 🟡 バグ / 動作不整合の可能性

<a id="issue-4"></a>
### Issue 4: `/odom` の twist が常時 0 の疑い

**状態: 未解決(要実機検証)** | 重要度: 中

`src/external/.../bus_config_cia402_epos4_vel/bus.yml:43` で TPDO の `0x606C (velocity actual value)` がコメントアウトされている。ドライバがこれを PDO で受けていない場合 `joint_states.velocity` は常に 0 になり、`epos4_odometry.cpp:137-145` は velocity 配列が「空でなければ」velocity 経路を使うため、`/odom` の twist が恒常的に 0 になる (pose は position 差分なので正常)。

RPP の `use_velocity_scaled_lookahead_dist: true` はこの twist を参照するため、lookahead が常に最小値に張り付く。

**検証コマンド**: 走行中に `ros2 topic echo /odom --field twist`

**対処案**: bus.yml の TPDO に 0x606C を追加する、または odometry 側で velocity が全 0 のときは Δposition/dt へフォールバックする。

**関連**: 同じ bus.yml のスケーリング係数に別の根本問題あり →
[2026-07-07_wheel_odometry_encoder_scaling_4x.md](2026-07-07_wheel_odometry_encoder_scaling_4x.md)。
bus.yml を修正するときは両方まとめて対応するのが効率的。

<a id="issue-5"></a>
### Issue 5: teleop の invert_left/right が他ノードと不一致

**状態: 未解決** | 重要度: 中

`src/epos4_teleop/config/params.yaml:5-6` は `invert_left/right: false`、controller/odometry (`params_2d.yaml` / `params_3d.yaml`) は `true`。CLAUDE.md 自身が「必ず同期させる」と定めているパラメータ。このままだと前進時の距離表示が負に出るはず (表示のみの実害)。

commit `8ca003c` で gear_ratio は 1.25 に同期されたが invert は取り残された模様。

**対処案**: teleop の invert を `true` に揃え、実機で前進時に距離が正で増えることを確認。

<a id="issue-6"></a>
### Issue 6: 脱力モード復帰時の join で executor がブロックしうる

**状態: 未解決** | 重要度: 中

`epos4_controller.cpp:451-453`: `f` を OFF にしたとき前回の `reenable_thread_` がまだ走っていると、executor スレッド上で `join()` がブロックする。その間 100 Hz の TPDO 送信もサービス応答処理も止まり、サービス不応答時は最悪 30 秒近く固まる。また ON→OFF を高速トグルすると async の disable と別スレッドの enable シーケンスが競合し、`free_mode_ = true` なのにドライブが enable のまま終わるパターンがある (指令は 0 のままなので暴走はしない)。

**対処案**: 復帰シーケンスの実行中フラグを持ち、実行中の再トグルは拒否 or キューイングする。join は detach + 世代カウンタ等でノンブロッキング化。

<a id="issue-7"></a>
### Issue 7: 目標 rpm の `static_cast<int>` 切り捨て

**状態: 未解決** | 重要度: 中

`epos4_controller.cpp:226,232`: ゼロ方向への切り捨てなので ±1 rpm 未満はデッドバンドになり、左右で符号が異なるとき丸め方向が非対称になり低速時にわずかな直進偏りが出る。

**対処案**: `std::lround()` に置き換える。

<a id="issue-8"></a>
### Issue 8: odometry の同期が片系停止で警告なしに止まる

**状態: 未解決** | 重要度: 中

`epos4_odometry.cpp` の ApproximateTime 同期は、片方のモータの joint_states が途絶えると `/odom` も `/joint_states` (→ TF) も警告なしに止まる。Nav2 側では「TF timeout」としてしか見えず切り分けが困難。

**対処案**: 一定時間 (例: 500 ms) ペアが成立しなければ WARN を出すタイマを追加。

<a id="issue-9"></a>
### Issue 9: `ROS_DOMAIN_ID=150` が Linux の安全範囲外

**状態: 未解決** | 重要度: 中

`Dockerfile:51` で `ROS_DOMAIN_ID=150` を設定しているが、Linux で安全に使えるのは 0〜101 (と 215〜232)。ドメイン 150 の DDS ポート (7400 + 250×150 = 44900 付近) は Linux のエフェメラルポート範囲 (32768〜60999) に入るため、他プロセスが先にポートを掴んでいるとノードの discovery が散発的に失敗する。「たまにトピックが見えない」という再現困難な症状の原因になりうる。

**対処案**: 101 以下 (例: 42) に変更。

<a id="issue-10"></a>
### Issue 10: 3D 構成では SLAM / Nav2 の入力が存在しない

**状態: 未解決(設計判断)** | 重要度: 中

`rerobot_bringup_3d.launch.py` は `/sdk_could` (PointCloud2) しか出さず、`slam_toolbox.yaml` / `nav2_params.yaml` は `/scan` (LaserScan) 前提。`pointcloud_to_laserscan` 等の変換ノードがどこにもないため、3D 構成のままでは slam.launch.py も nav2.launch.py もエラーすら出さず scan 待ちで止まる。

**対処案**: 3D で SLAM するなら (a) pointcloud_to_laserscan を追加して既存 2D スタックを流用、(b) 3D 対応 SLAM (例: LIO 系) を導入、のどちらかを決める。

<a id="issue-11"></a>
### Issue 11: urg_node の FOV を ±90° に絞っている

**状態: 未解決(意図確認)** | 重要度: 中

`rerobot_bringup_2d.launch.py:82-83` で `angle_min/max: ±1.5708`。UTM-30LX は本来 270° の FOV があり、半分近く捨てている。筐体の映り込みマスクとして意図的なら妥当だが、コメントに理由がなく、SLAM/AMCL のマッチング品質 (特に廊下・交差点) に直結する。

**対処案**: 意図的ならコメントに理由を明記。そうでなければ実際の映り込み角度を測って必要最小限に広げる。

<a id="issue-12"></a>
### Issue 12: `/odom` の共分散が全ゼロ

**状態: 未解決** | 重要度: 中

`epos4_odometry.cpp` は pose/twist の covariance を一切設定していない (全ゼロ = 「完全に正確」の意味)。AMCL は TF しか見ないので今は無害だが、Dockerfile に入れてある robot_localization (EKF) を配線すると、ゼロ共分散はフィルタを壊すか車輪オドメトリを盲信させる。

**対処案**: 対角に現実的な値 (例: x/y 0.01, yaw 0.05, 未使用軸は大きな値 1e6) を設定。

<a id="issue-13"></a>
### Issue 13: launch に respawn 指定がなく、ノード死亡が無検知・無復帰

**状態: 未解決** | 重要度: 中

launch 全体で `respawn` 指定がゼロ。屋外走行中に urg_node (USB 抜け等で割と死ぬ) や nav2 サーバが落ちても、再起動されず警告も出ず、SLAM/ローカリゼーションが静かに止まる。

**対処案**: 最低限 urg_node に `respawn=True, respawn_delay=2.0` を。nav2 は lifecycle_manager の bond 監視が機能しているか確認。

---

## 🟠 低 (紛らわしさ・運用上の注意)

<a id="issue-14"></a>
### Issue 14: nav2.rviz の「局所経路」表示は RPP では常に空

**状態: 未解決** | 重要度: 低

`nav2.rviz` は `/local_plan` を表示しているが、これは DWB 用のトピック。RegulatedPurePursuit は `/lookahead_point` (PointStamped) と `/lookahead_arc` を出すため、この表示は永遠に空のまま。デバッグ時に「経路が出ていない」と誤解しやすい。

**対処案**: `/local_plan` 表示を削除し、`/lookahead_point` / `/lookahead_arc` の表示に差し替える。

<a id="issue-15"></a>
### Issue 15: keyboard teleop が idle でも 20 Hz で 0 を publish し続ける

**状態: 未解決** | 重要度: 低

`teleop_keyboard.cpp:110-120` は入力がなくても 20 Hz で Twist を publish し続けるため、Nav2 走行中に立ち上げると `/robot_speed_cmd` を取り合い機体がスタッターする。cmd mux (twist_mux) も無い。

**対処案**: 運用ルールとして「Nav2 走行中は keyboard teleop を立てない」を徹底するか、twist_mux を導入して優先度制御する。

<a id="issue-16"></a>
### Issue 16: CLAUDE.md の「motor1 = left wheel」が実配線と逆

**状態: 未解決** | 重要度: 低 (ただし事故のもと)

`claude_swap` (`epos4_controller.cpp:271-275`) 以降、実配線は motor1 = 右輪 / motor2 = 左輪。CLAUDE.md の Key conventions は旧配線のまま。さらに init ログも `"motor1(left)"` / `"motor2(right)"` (`epos4_controller.cpp:427,430`) と逆ラベルで、現場でのデバッグを誤誘導する。URDF の joint 名 `m1_wheel` (+y=左) に motor2 のデータが入る点も同根 (表示は正しいが名前が実体と逆)。

**対処案**: CLAUDE.md の記述とログ文字列を実配線に合わせて修正。

<a id="issue-17"></a>
### Issue 17: CLAUDE.md のアーキテクチャ記述が現状と不一致

**状態: 未解決** | 重要度: 低

「odometry は `/robot_encoder_states` を購読 (現在未配信)」「bringup launch が robot_state_publisher の `/joint_states` を `/robot_encoder_states` に remap」という記述は古い。現在は odometry が message_filters で per-motor topic を直接同期し、自ら `/joint_states` を publish しており、launch に remap は無い。

**対処案**: CLAUDE.md のアーキテクチャ節と epos4_odometry の説明を現状に合わせて書き直す。

<a id="issue-18"></a>
### Issue 18: README のシンボリックリンク復旧コマンドのパスが誤り

**状態: 未解決** | 重要度: 低

`README.md`: `ln -s external/maxon_epos4_ros2_repo/maxon_epos4_ros2 .` → 正しくは `external/epos4compact50-5can/maxon_epos4_ros2` (CLAUDE.md 側は正しい)。ほか typo: `docker copose down`、`Hardwere`。

**対処案**: README を修正。

<a id="issue-19"></a>
### Issue 19: nav2 の map_dir 既定値が日付固定 / params コメント不一致

**状態: 未解決** | 重要度: 低

`nav2.launch.py` の map_dir 既定値 `/workspace/maps/slam_toolbox/2026_6_9__22:00` は特定計測日のローカル地図に依存。また `nav2_params.yaml` 内コメントの「`map` 引数で上書き」は現 launch の引数名 `map_dir` と不一致。

**対処案**: コメントを `map_dir` に修正。既定地図は「最新」を指すシンボリックリンク等にすると差し替えが楽。

---

## ⚪ 軽微

<a id="issue-20"></a>
### Issue 20: epos4_vel_test の入力処理

**状態: 未解決** | 重要度: 軽微

- `epos4_vel_test.cpp:215-217`: `std::cin >> menu_i` が EOF や非数値入力で fail 状態になると、エラーメッセージを吐き続ける無限ビジーループに入る。
- メニュー 8 (cyclic velocity mode) は選択した瞬間サイン波でモータが勝手に動き出す (`sine_wave_time_ = 3.0`) が、メニュー表記にその旨がない。ベンチで軸を接続したまま押すと危険。

**対処案**: cin fail 時に `cin.clear()` + `ignore()`。メニュー 8 の表記に「モータが動き出す」旨を追記。

<a id="issue-21"></a>
### Issue 21: epos4_controller のデッドコード

**状態: 未解決** | 重要度: 軽微

- `m1_publisher_` / `m2_publisher_` (Float64): 宣言のみで未生成。
- `halt` / `shutdown` / `velocity_mode` / `target` クライアント: 生成のみで未使用。
- コメントアウトされた `/robot_encoder_states` fan-in コード (CLAUDE.md では「将来の意図された修正」とされているので、残すなら TODO と明記)。

**対処案**: 使わないものは削除、将来使うものは TODO コメントを付ける。

<a id="issue-22"></a>
### Issue 22: epos4_controller/package.xml のメタデータ残骸

**状態: 未解決** | 重要度: 軽微

maintainer が `MAKA <support@maxongroup.com>` (maxon のサンプルコードの残り)。

**対処案**: 自分の名前・メールに書き換え。

<a id="issue-23"></a>
### Issue 23: bus_config launch の死にコード

**状態: 未解決** | 重要度: 軽微 (submodule 側)

`bus_config_cia402_epos4_vel.launch.py` の `slave_eds_path` は存在しない `maxon_epos4.eds` を指すが、変数自体が未使用なので実害なし。実際に使われる `maxon_epos4_0x1018.eds` と `master.dcf` は `cogen_dcf` でビルド時に正しく生成・参照される (確認済み)。

**対処案**: submodule 側を触る機会があれば削除。

<a id="issue-24"></a>
### Issue 24: 起動直後の脱力トグルが init シーケンスと競合

**状態: 未解決** | 重要度: 軽微

`init_thread_` 実行中 (起動 〜5 秒 + リトライ時間) に `f` で脱力 ON/OFF すると、init シーケンスと enable/disable が並行してサービスを叩き合う。起動直後に f を押さなければ実害なし。

**対処案**: init 完了フラグを見て、完了前の free_mode 要求は拒否 or 遅延させる。

<a id="issue-25"></a>
### Issue 25: Dockerfile の不要パッケージ

**状態: 未解決** | 重要度: 軽微

- `ros-jazzy-nav2-minimal-tb*`: TB3 シミュレータ用で実機に不要。イメージサイズの無駄。
- `ros-jazzy-robot-localization`: インストール済みだが EKF はどこにも配線されていない (Issue 12 参照)。使う予定がないなら削除、あるなら早めに配線。

**対処案**: 方針を決めて Dockerfile を整理。

---

## 監査で問題なしを確認できた点 (参考)

- `params_3d.yaml` の rfans_driver セクションのキーは StarROS2 側の `declare_parameter` と全項目一致。
- bus.yml (vel 版) の RPDO は 0x60FF を正しくマッピング (pos 版との取り違えなし)。
- スケール係数 (2π/4096, 2π/60 等) と controller / odometry / teleop の単位変換は**リポジトリ内では**相互に整合。
  ただし実機エンコーダとの物理整合は別問題で、`gear_ratio: 1.25` が 4 倍ズレの対症療法になっている
  (→ [2026-07-07_wheel_odometry_encoder_scaling_4x.md](2026-07-07_wheel_odometry_encoder_scaling_4x.md))。
- IK (`v = x ∓ yaw·tread/2`) と odometry の積分 (mid-step heading)、claude_swap 後の左右整合は正しい。
- `src/external/COLCON_IGNORE` により symlink 経由との二重ビルドは起きない。
- maps/ は .gitignore 済みで巨大ファイルの混入なし。
