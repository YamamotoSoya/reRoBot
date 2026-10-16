<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# 自律走行の当日運用フロー: twist_mux 調停 + パッド操作の waypoint_runner + auto_run 統合

- 日付: 2026-10-16
- 対象パッケージ: `rerobot_bringup` (`ros2_ws_main/src/bringup/rerobot_bringup`)、`docker/Dockerfile_main`
- 対象 ROS: ROS 2 Jazzy (reRoBot コンテナ `rerobot_env`)。twist_mux 4.5.0 (apt `ros-jazzy-twist-mux`)
- 関連文書: 起動 1 本化の経緯 [docs/issue/2026-10-15_keepout_corridor_stall_imu_yaw_jump.md](../issue/2026-10-15_keepout_corridor_stall_imu_yaw_jump.md)、停止点つき走行 `scripts/waypoint_runner.py` 冒頭コメント (2026-10-04)、teleop の導入 `config/joy_teleop.yaml`

## 1. 目的・概要

10-14 / 10-15 の自律走行で `auto_run.launch.py` (2026-10-15) を使ってみて、当日の運用に合わない点が 4 つ出た:
(a) joy_teleop が別端末、(b) waypoint_runner がキーボード入力前提で `-it` の端末が要る、(c) joy と Nav2 が同じ
`/robot_speed_cmd` に直接書いていて調停が無い、(d) initialpose が「起動 60 s 後」固定。

ユーザが想定する当日の流れは「待機エリアで起動 → joy でスタート付近へ → AMCL の初期位置を**目視**で合わせる →
合わせたまま joy でスタート列で待機 → スタート位置で簡単な操作で joy を切り waypoint_runner を走らせる」。
これを「ロボット側は最初から全部起動しておき、人の操作はゲームパッドだけで完結させる」構成で実現した。

実装した機能:
- **twist_mux の常駐** (`rerobot_bringup.launch.py`): joy (`/joy_vel`, 優先 100) と Nav2 (`/cmd_vel`, 優先 10) を調停し `/robot_speed_cmd` へ 1 本で出す。LB を離せば 0.5 s で Nav2 に切り替わる (= 「joy を切る」操作は不要)。走行中に LB を握ればいつでも止められる。
- **waypoint_runner のパッド入力** (`class Input`): Start = 開始/再開、Back = 一時停止 (LB 同時押し要求 `start_requires_enable` は任意、既定 off)。キーボードは端末があるときだけ併用。launch 配下 (端末なし) で起動できる。
- **auto_run への統合**: joy_teleop (0 s) と waypoint_runner (30 s) を同居させた。起動後に人が打つコマンドはゼロ。bag に `/joy` `/joy_vel` `/cmd_vel` を追加。

スコープ外 (意図的に未対応):
- 初期自己位置の自動送信 (course.yaml の start_pose をボタンで送る案): ユーザ判断で**目視 (RViz 2D Pose Estimate)** に確定。`initial_pose` 引数は再起動時の保険として残す。
- 画面なしでの状態通知 (スピーカで beep 等): 別件。蓋を開けて RViz を見る運用なので当面不要。
- twist_mux の lock (非常停止で全入力遮断): 必要になったら priority 255 の lock を 1 本足すだけなので今回は入れない。
- ロボット PC の蓋閉じサスペンド対策 (logind `HandleLidSwitch`): システム設定なので Claude は変更していない (§6)。

## 2. 設計の勘所

### 2.1 速度指令の調停は twist_mux (優先度 + timeout) に任せ、bringup に常駐させる

実測: 旧配線では joy_teleop と Nav2 (controller_server / behavior_server) が両方 `/robot_speed_cmd` の publisher。
両方が出ている間は epos4_controller に 2 系統が交互に届き、100 Hz タイマーが最後に受けた値を使う (どちらが勝つかはタイミング次第)。
実測 (台上、2026-10-16、ROS_DOMAIN_ID 151 で疑似入力): Nav2 相当 0.3 のみ → 出力 0.3 / joy 相当 0.1 を重ねる → 0.1 /
joy 停止 → 約 0.5 s 後に 0.3 へ復帰。`/robot_speed_cmd` の publisher は twist_mux 1 つ、型 `geometry_msgs/msg/Twist`。

判断: twist_mux は「timeout 内に受信した入力のうち最高優先度の 1 本を素通し (混ぜない)」なので、
- LB を離す = teleop_twist_joy が 0 を 1 発出して無音 → 0.5 s 後に Nav2 側へ自動で落ちる。専用の切替操作が要らない。
- 走行中に LB を握る = スティック中立でも `/joy_vel` に 0 が流れ Nav2 を遮る → 人がいつでも止められ、離せば Nav2 が続きを走る (ゴールは生きたまま)。
- 両方無音なら何も出さない → epos4_controller の既存保護 `cmd_timeout_s` (0.5 s) が目標を 0 へランプ。

判断: bringup に**常駐・boolean なし**で置く。false にできると「joy が `/joy_vel` に出しているのに誰も受けない」事故が起きるため。
Nav2 の remap は外して既定名 `/cmd_vel` のまま (legacy `nav2.launch.py` も `parts/navigation.launch.py` を include するので同時に直る)。
epos4_teleop (キーボード) は台上デバッグ専用なので従来どおり `/robot_speed_cmd` に直接出す (mux と同時には使わない)。

制約: `use_stamped` は twist_mux の版で既定が違う (rolling は true = TwistStamped)。epos4_controller は素の Twist なので
`config/twist_mux.yaml` で `false` を明示する。出力トピック名は `cmd_vel_out` 固定で launch で remap。

却下した代替案:
- 各ノードが `/robot_speed_cmd` に直接書く (現状維持) + 「開始時に joy_teleop を kill」: 走行中の介入手段が無く、kill/再起動の操作が結局端末を要する。
- Nav2 の `collision_monitor` / `velocity_smoother` 経由で調停: どちらも 1 入力 1 出力で優先度調停の機能は無い。
- waypoint_runner が joy を見て Nav2 のゴールを cancel/再送する「ソフト調停」: LB を握るたびにゴールが切れ、再送の 1〜2 s の遅れと経路再計画が毎回入る。mux なら Nav2 は介入に気付かない。

### 2.2 waypoint_runner は「入力の口」だけ差し替え、走行ロジックは触らない

判断: `Keyboard` の外側に `Input` を置き、キーボード (stdin が tty のときだけ) と `/joy` のボタン立ち上がりを**同じ 1 文字**
(`g` = 再開キー, `p` = 一時停止) に正規化してキューに積む。`kb.get()` を呼ぶ既存 5 か所 (`spin_until` / `wait_key` / `wait_seconds` / `run_segment`) は無変更。
区間分割・NavigateThroughPoses・自動再送 (10-16 朝に追加) の挙動が変わらないことが、本番直前の変更として最も重要。

判断: 立ち上がり検出 (joy_node は autorepeat 20 Hz で同じ状態を再送する) は `teach_marker.py` と同じ方式。
`button_debounce` 1.0 s 以内の連続イベントは捨てる (スタート列で 2 度押しになっても 1 回扱い)。

判断 (ユーザ決定 10-16): Start は**単独**で開始・再開 (`start_requires_enable: false`)。Claude は誤爆防止に LB 同時押し必須を提案したが、操作の単純さを優先して却下。オプションとして残す (true にすると LB を握りながら Start でないと開始せず、LB を離してから 0.5 s は Nav2 の最初の指令が遅れる)。

判断: ボタン番号は `config/joy_teleop.yaml` の `waypoint_runner` 節に集約 (teleop の `enable_button` と同じファイルで整合を見られる)。
argparse は `allow_abbrev=False` — launch が付ける `--ros-args` / `-r` が長い option の前方一致に吸われないため。
`print` は `flush=True` — launch 配下は stdout がブロックバッファで、ログが出てこない。

実測 (2026-10-16、`on_joy` に合成 Joy を直接与える単体試験): Start 単独 → イベントなし + 警告ログ / LB+Start 立ち上がり (当時の既定 true) → `g` /
押しっぱなし再送 → なし / Back 立ち上がり → `p` / 端末なし → `kb is None`。`--params-file` で `start_button: 2` に変えると describe が button 2 になる (yaml 経路の確認)。

### 2.3 初期自己位置は目視、RViz は常時起動のまま

判断 (ユーザ決定): initialpose は RViz の 2D Pose Estimate で、`/rfans/scan_all` と地図の重なりを見ながら置く。ロボット PC (T480s) の蓋を開けて直接見る。
自己位置が決まった後に RViz を閉じるか付けっぱなしにするかは当日判断 — `parts/rviz_nav.launch.py` に `on_exit=Shutdown` は無いので、閉じても auto_run の他ノードは落ちない。
起動から数分経ってから置くので、AMCL が尤度場を作っている最初の ≈13 s に initialpose を受けて bond が切れる問題 (10-11) は踏まない。

却下した代替案:
- course.yaml に `start_pose` を持たせボタンで `/initialpose` を送る: 置き直しができず、スタート列の実位置とずれる。ユーザが目視を選択。
- 別 PC で RViz (DDS を WiFi 越え) / リモートデスクトップ: 手持ち機材と WiFi 品質の都合で今回は見送り。将来の選択肢として docs/trial/improvements.md の Parsec と同列。

### 2.4 「何が起動したか」と「runner は今どこか」を流れないところに出す (10-16 夕、ユーザ指摘)

実測 (ユーザ): auto_run で全部を 1 本にしたら、bringup_check の表 (15 s) も waypoint_runner の待機/走行の行も他ノードのログに流されて、
どのセンサが落ちたか・Nav2/joy/runner が揃ったか・runner が今どの区間かが追えなくなった。

判断: 2 つの出口を足す。
- **`scripts/auto_run_check.py`** — auto_run の最後 (55 s、全部品が出揃った後) に 1 回だけ走り、全部品を 1 枚の表にする。
  bringup の行は `bringup_check.py` の判定部 (`collect_rows()` に切り出し) を同じディレクトリから import して再利用 (二重実装しない)。
  追加の行: twist_mux (publisher が twist_mux 1 つだけか)、パッド (ノード + `/dev/input/js0`)、Nav2 lifecycle 7 ノードの `get_state`、
  `/map` と `/keepout_filter_mask` の受信 (transient_local)、map_odom_holder、map→odom TF (初期位置前は ▲ で「未」= 正常)、
  waypoint_runner (ノード + 最新状態)、bt_navigator の action サーバ、rviz2、rosbag2_recorder。何度でも `ros2 run rerobot_bringup auto_run_check.py` で再確認できる。
  ⚠ action サーバの有無はサービス名一覧では判定できない (runner の action **クライアント** でも同じ名前が見える、10-16 実測) →
  `get_action_server_names_and_types_by_node(bt_navigator)` で判定。
- **runner の状態発行** — `set_status()` が `/waypoint_runner/status` (std_msgs/String) と `/waypoint_runner/marker` (車体上 1.6 m の
  TEXT_VIEW_FACING、nav2.rviz に表示を追加) を transient_local で出す。待機 (Start で再開) / 走行中 (残り点数) / 自動再送待ち / 失敗 / 到着 / 完走 / Ctrl-C。
  RViz を見ていれば端末を遡らなくても分かる。端末なら `ros2 topic echo /waypoint_runner/status`。
  サーバ待ちのログは 1 s ごと → 10 s に 1 回に (auto_run では Nav2 より 10 s 早く起動するので毎秒出ていた)。

却下した代替案:
- runner を別ウィンドウ (xterm) で起動: コンテナに端末エミュレータが無く、イメージにまた依存を足すことになる。tty が付くのでキーボードも効く利点はあり、
  将来 `prefix="xterm -e"` で足せる。
- runner が定期的に状態を再表示: ログの流量を増やすだけで、流れる問題は解決しない。

### 2.5 Ctrl-C で launch が終了しなかった原因 = scoped include 内の TimerAction が引数を後から引いていた (10-16 夜、ユーザ報告)

実測: 全部品が揃った auto_run に Ctrl-C 相当 (プロセスグループへ SIGINT) → 子プロセスは 10 s 以内に全部終了するのに
`ros2 launch` 本体だけが残り、端末に戻らない。launch 自身の stderr に起動直後から
`Task exception was never retrieved … SubstitutionFailure: launch configuration 'check_delay' does not exist` (TimerAction._wait_to_fire_event)。
最小再現 (`DeclareLaunchArgument` + `TimerAction(period=LaunchConfiguration(...))` を `GroupAction(scoped=True)` で include) で同じ例外と同じハングを再現、
単体起動では起きない (因果確定)。

機構: `GroupAction(scoped=True)` は include を visit し終えた直後に launch configuration のスコープを閉じる。TimerAction の coroutine は
その後 (次のイベントループ周回) で period の substitution を評価するので、include 元で宣言した引数が見えない。例外は asyncio の Task 内で
捨てられ、launch のシャットダウン処理が完了しなくなる (launch 側の挙動。ROS 2 Jazzy launch)。副作用として **auto_run 配下では 15 s の
bringup_check が一度も起動していなかった** (ユーザの「どのセンサが起動したか分からない」の一因)。

対策: `rerobot_bringup.launch.py` の bringup_check 起動を OpaqueFunction の中で組み立て、`check_delay` と各フラグを visit 時点で Python の素の値に
解決してから `TimerAction(period=float)` + `Node(parameters=[dict])` を作る (発火時に context を参照しない)。
修正後の全体試験: SIGINT から 3 s で全プロセス終了、bringup_check が 15 s に表を出す、bag は mcap + metadata.yaml を閉じて終了。

試験の落とし穴 (自分用): 非対話 bash から `cmd &` で起動したプロセスは SIGINT が SIG_IGN で継承される (Python は継承した SIG_IGN を既定に戻さない)。
最初の再現で「runner が SIGINT を無視する」「launch が ctrl-c を認識しない」と見えたのはこの artifact。SIGINT を SIG_DFL に戻す exec ラッパを噛ませて再試験した。
`| tee` は原因ではない (`tee -i` でも同じハング、修正後は素の tee で正常終了)。`ros2 bag record` は launch 配下では stdin 非 tty でキーボード制御が自動無効、SIGINT で正常終了。

教訓 (CLAUDE.md にも転記): **TimerAction の period や、タイマー発火後に実行される Node の parameters に `LaunchConfiguration` を直接書かない**。
その launch が scoped include されると壊れる。OpaqueFunction で先に perform する。

## 3. データフロー

```
auto_run.launch.py map_dir:=...        (待機エリアで 1 本起動。以後、端末は触らない)
├─  0 s rerobot_bringup.launch.py rfans_scan_all:=true
│      ├─ twist_mux (常駐、config/twist_mux.yaml)
│      │     /joy_vel  geometry_msgs/msg/Twist  優先 100  timeout 0.5 s ─┐
│      │     /cmd_vel  geometry_msgs/msg/Twist  優先  10  timeout 0.5 s ─┴→ cmd_vel_out → remap → /robot_speed_cmd
│      └─ epos4_controller  ← /robot_speed_cmd (publisher は twist_mux だけ。cmd_timeout_s 0.5 で無音時 0 へ)
├─  0 s joy_teleop.launch.py                (joy:=false で省略)
│      ├─ joy_node (/dev/input/js0)  → /joy  sensor_msgs/msg/Joy (autorepeat 20 Hz)
│      ├─ teleop_twist_joy_node      /joy → /joy_vel  (LB=enable 4, RB=turbo 5。/cmd_vel を /joy_vel に remap)
│      └─ teach_marker               /joy button 3 → /teach_marker
├─ 20 s nav_amcl_scanall.launch.py          (map_server / AMCL + map_odom_holder / Nav2 → /cmd_vel。remap なし)
├─ 30 s waypoint_runner.py <map_dir>/waypoints/course.yaml --ros-args --params-file joy_teleop.yaml
│      ├─ /joy → class Input: Start(7) → "g" (開始/再開), Back(6) → "p" (一時停止)
│      ├─ ~/status std_msgs/String, ~/marker visualization_msgs/Marker (transient_local。RViz 車体上テキスト / auto_run_check が読む)
│      └─ navigate_through_poses (nav2_msgs/action/NavigateThroughPoses) を区間ごとに送る
├─ 35 s RViz nav2.rviz                      (2D Pose Estimate → /initialpose。閉じても他は落ちない)
├─ 40 s ros2 bag record (RECORD_TOPICS に /joy /joy_vel /cmd_vel を追加 — 誰の指令が通ったかを後から追える)
├─ 55 s auto_run_check.py  (全部品を 1 枚の表に。bringup 行は bringup_check を import して再利用)
└─ 60 s initialpose.py (initial_pose:="x,y,yaw" を渡したときだけ。目視運用では使わない)

時間軸 (スタート位置):  LB 押下中 ─ joy_vel 20 Hz ─ LB 離す ─ 0 を 1 発 ─ 無音 0.5 s ─ Start ─→ 以後 /cmd_vel (Nav2)
                        出力 = joy の値                     0                       runner が goal 送信 → Nav2 の値
```

## 4. 使い方

```bash
# rerobot_env 内。待機エリアで 1 本だけ起動する
ros2 launch rerobot_bringup auto_run.launch.py map_dir:=/workspace/maps/2d/glim/2026-10-09_1643_5goukan_chal \
  2>&1 | tee /workspace/log/auto_$(TZ=Asia/Tokyo date +%F_%H%M).log
#   joy:=false  … パッドなし (waypoint_runner は端末から別途)   rviz:=false / record:=false … 従来どおり

# 端末から waypoint_runner を動かす場合 (joy ボタンの割り当ても読む。キーボードも有効)
ros2 run rerobot_bringup waypoint_runner.py /workspace/maps/2d/glim/<map>/waypoints/course.yaml \
  --ros-args --params-file $(ros2 pkg prefix rerobot_bringup)/share/rerobot_bringup/config/joy_teleop.yaml

# twist_mux 単体 (台上の配線確認)
ros2 run twist_mux twist_mux --ros-args --params-file $(ros2 pkg prefix rerobot_bringup)/share/rerobot_bringup/config/twist_mux.yaml \
  -r /cmd_vel_out:=/robot_speed_cmd
ros2 topic info -v /robot_speed_cmd     # Publisher count: 1 (twist_mux), Type: geometry_msgs/msg/Twist

# 全部品の起動結果を再確認 (auto_run は 55 s に自動で 1 回出す)
ros2 run rerobot_bringup auto_run_check.py            # 引数なし = auto_run の構成。rviz を閉じた後は -p rviz:=false
# runner の現在状態 (RViz の車体上テキストと同じ)
ros2 topic echo /waypoint_runner/status
```

当日の操作 (パッド):

| 場面 | 操作 | 何が起きるか |
|---|---|---|
| 待機エリア → スタート付近 | LB を握ってスティック | joy が最優先で通る。Nav2 はゴールを持たず無音 |
| スタート付近 | 蓋を開け RViz の 2D Pose Estimate | AMCL が収束。RViz は閉じても残してもよい |
| スタート列へ | LB を握ってスティック | AMCL はスキャンで追従し続ける |
| スタート位置 | **Start** | runner が区間 1 を送る。LB を離していれば Nav2 の指令がそのまま通る (握ったままなら離して 0.5 s 後) |
| 停止点 (stop: true) | Start | 次の区間へ |
| 走行中に危ない | **LB を握る** | スティック中立なら停止、そのまま手で動かすことも可。離せば Nav2 が続きを走る |
| 走行中に一時停止 | Back | ゴールをキャンセルして待機。Start で残りの点から再開 |
| 全部止める | 端末で Ctrl-C 1 回 | runner が走行中ゴールをキャンセル、bag も正常終了 |

パラメータ (`config/joy_teleop.yaml` の `waypoint_runner` 節):

| 項目 | 値 | 説明 |
|---|---|---|
| `start_button` | 7 (Start) | 区間の開始 / 停止点・一時停止からの再開 (= キーの g) |
| `pause_button` | 6 (Back) | 走行中のゴールをキャンセルして待機 (= キーの p) |
| `start_requires_enable` | false | true にすると LB を握りながら Start でないと開始しない (誤爆防止オプション、ユーザ判断で off) |
| `enable_button` | 4 (LB) | teleop_twist_joy_node の `enable_button` と同じ値にする |
| `button_debounce` | 1.0 s | この間隔未満の連続ボタンは 1 回扱い |

ボタン番号 (Linux の Xbox 配置、実機は joy_node が "Generic X-Box pad" として認識): 0 A / 1 B / 2 X / 3 Y (teach_marker 使用中) / 4 LB / 5 RB / 6 Back / 7 Start

## 5. 変更ファイル一覧

- `docker/Dockerfile_main` — `ros-jazzy-twist-mux` を追加 (**イメージ再ビルドが必要**。稼働中コンテナには 2026-10-16 に apt で直接入れたが、コンテナを作り直すと消える)
- `ros2_ws_main/src/bringup/rerobot_bringup/package.xml` — `exec_depend` twist_mux
- `ros2_ws_main/src/bringup/rerobot_bringup/config/twist_mux.yaml` — 新規。入力 2 本・`use_stamped: false`
- `ros2_ws_main/src/bringup/rerobot_bringup/launch/rerobot_bringup.launch.py` — twist_mux ノードを常駐追加 (`cmd_vel_out` → `/robot_speed_cmd`)、構成バナーに 1 行
- `ros2_ws_main/src/bringup/rerobot_bringup/launch/joy_teleop.launch.py` — teleop の出力先を `/robot_speed_cmd` → `/joy_vel`
- `ros2_ws_main/src/bringup/rerobot_bringup/launch/parts/navigation.launch.py` — controller_server / behavior_server の `/cmd_vel → /robot_speed_cmd` remap を削除
- `ros2_ws_main/src/bringup/rerobot_bringup/config/nav2_params.yaml` — 冒頭の配線コメントを mux 経由に更新 (値は無変更)
- `ros2_ws_main/src/bringup/rerobot_bringup/scripts/waypoint_runner.py` — `class Input` 追加、tty 必須を撤廃、`allow_abbrev=False`、起動時 print に `flush=True`
- `ros2_ws_main/src/bringup/rerobot_bringup/config/joy_teleop.yaml` — `waypoint_runner` 節 (ボタン割り当て) を追加
- `ros2_ws_main/src/bringup/rerobot_bringup/launch/auto_run.launch.py` — joy_teleop include (0 s, `joy` 引数)、waypoint_runner Node (30 s、course.yaml があるときだけ)、auto_run_check Node (55 s)、バナーをボタン案内 + 確認コマンドに、RECORD_TOPICS に `/joy` `/joy_vel` `/cmd_vel`
- `ros2_ws_main/src/bringup/rerobot_bringup/scripts/auto_run_check.py` — 新規。全部品の起動結果表 (§2.4)
- `ros2_ws_main/src/bringup/rerobot_bringup/scripts/bringup_check.py` — 判定部を `collect_rows()` に分離、`node_name` / `parameter_overrides` を受ける (ロジック無変更)
- `ros2_ws_main/src/bringup/rerobot_bringup/rviz/nav2.rviz` — Marker 表示 `RunnerStatus` (/waypoint_runner/marker) を追加
- `ros2_ws_main/src/bringup/rerobot_bringup/CMakeLists.txt` — auto_run_check.py を install

## 6. 既知の制限

- **保留 (10-16 夜、ユーザ判断「いったん保留」)**: 初期位置を置くまでの間、端末ログが流れて 15 s / 55 s の表が読めない。実走ログ (`auto_2026-10-16_2031.log`、1627 行) の内訳:
  controller_server 513 行 (local costmap の keepout StaticLayer が map フレームを lookupTransform できない ERROR。今朝の StaticLayer 化で新規) /
  rviz2 259 行 (Message Filter dropping、INFO、map が無いので正常) / planner_server 121 行 (base_link→map TF 待ち INFO) / amcl 79 行 (initial pose を置けの WARN、正常) /
  **bag_record 72 行 (`/particle_cloud` に型が 2 つ → 記録拒否。nav2.rviz の「Amcl Particles」が `rviz_default_plugins/PoseArray` で購読していて AMCL の `nav2_msgs/ParticleCloud` と衝突。粒子が bag に残っていない本物の不具合)**。
  対策案 (未実施): (1) `parts/navigation.launch.py` + waypoint_runner の起動を map→odom TF 検出後にする (`wait_for_tf.py map odom` を ExecuteProcess で回し `OnProcessExit` で include。auto_run_check は 55 s 表で Nav2 行を「initialpose 待ち」にし、Nav2 起動 20 s 後にもう 1 枚出す)。
  (2) `parts/rviz_nav.launch.py` の rviz2 に `--ros-args --log-level rviz2:=warn`。(3) nav2.rviz の Amcl Particles を `nav2_rviz_plugins/ParticleCloud` 型に。
  副作用: (1) で初期位置を置いてから Start が効くまで Nav2 activate 分 10〜20 s 遅れる。
- Ctrl-C ハング (§2.5) は修正済み・CAN なし構成の全体試験で確認。実機でも同じ経路なので再発しない見込み。
- **実機 (CAN + Nav2) での end-to-end は未確認** (2026-10-16 は can0 なし)。台上で確認したのは twist_mux の切替 (疑似入力)・`Input` の単体試験・joy_teleop が js0 を開くこと・waypoint_runner が launch 配下で起動することまで。次の台上 (浮かせ) で確認する項目: LB 介入 → 離して Nav2 復帰 / Start で区間開始 / Back → Start で再開 / RViz を閉じても走行継続。
- **Docker イメージ未再ビルド**。`./scripts/build.sh images` (または main だけ `docker compose build main`) を走らせるまで、コンテナ再作成で twist_mux が消える。
- **ロボット PC の蓋閉じ**: logind `HandleLidSwitch` が既定 (suspend) のまま、seat0 に GNOME セッション無し・inhibitor 無し (2026-10-16 確認)。10-14 に蓋を閉じて走れていた実績が無ければ `HandleLidSwitch=ignore` + `HandleLidSwitchExternalPower=ignore` を `/etc/systemd/logind.conf` に書き `systemctl kill -s HUP systemd-logind`。Claude は変更していない。
- waypoint_runner は `navigate_through_poses` サーバ待ちの間 (`wait_for_server` ループ) は spin しないので、その間はパラメータサービスに応答せず joy イベントも溜まらない。nav 起動後は問題ない。
- `button_debounce` は全ボタン共通 (Start の 1 s 以内に Back を押すと Back が捨てられる)。必要ならボタン別にする。
- 画面なしで「区間を受理したか」を知る手段は RViz のテキストと `/waypoint_runner/status` のみ (音は無い)。蓋を開けて RViz を見る運用が前提。
- auto_run_check の bringup 行は auto_run の構成 (全センサ + scan_all + EKF) を決め打ち。`lidar_2d:=false` 等で bringup を減らした場合は該当行が NG に見える。
- 介入で長く止めても Nav2 は abort しない (`movement_time_allowance` 3600 s、10-15 設定) が、RPP の経路追従は介入前の経路を保持するので、手で大きく動かした後は経路が引き直されるまで一瞬迷う可能性がある (未検証)。
