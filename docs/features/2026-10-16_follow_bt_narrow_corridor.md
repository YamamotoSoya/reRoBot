<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# 追従型 NavigateThroughPoses BT: 計画失敗でも旧経路を保持して前方ロボットに stop-and-go 追従する

- 日付: 2026-10-16
- 対象パッケージ: `rerobot_bringup` (`ros2_ws_main/src/bringup/rerobot_bringup`)
- 対象 ROS: ROS 2 Jazzy (reRoBot コンテナ `rerobot_env`)。Nav2 1.3.13、BehaviorTree.CPP 4.10.0
- 関連文書: keepout 縁での RPP 永久停止と `failure_tolerance: -1` の経緯 [docs/issue/2026-10-15_keepout_corridor_stall_imu_yaw_jump.md](../issue/2026-10-15_keepout_corridor_stall_imu_yaw_jump.md)、当日運用フロー [2026-10-16_auto_run_joy_mux.md](2026-10-16_auto_run_joy_mux.md)、停止点つき走行 `scripts/waypoint_runner.py` 冒頭コメント
- ステータス: **設計実装済み・台上で BT 機構のみ確認。衝突停止 / 再発進 / 本物の計画失敗 (NavFn NO_VALID_PATH) は未検証** (critic 10-16 判定: 傍証のみ。主張を §2.1 の範囲に狭めれば本命)

## 1. 目的・概要

つくばチャレンジの狭い通路で前方のロボットが遅い・止まっているとき、Nav2 既定の BT
(`navigate_through_poses_w_replanning_and_recovery.xml`) は 3 s ごとの再計画で (a) 地図に迂回路があれば戻る方向の
別経路を採用し、(b) なければ計画失敗 → global costmap 全消去 → Spin 90° / Wait / BackUp のリカバリ → 6 回で ABORTED
となる。(b) の「完全停止」は、NavFn が目標まで一本につながる経路しか返さないため「前方ロボットが区間 (waypoint 間隔
20〜60 m) を抜け切るまで動けない」ことを意味する。

本機能はこの (b) を「旧経路を保持して FollowPath を続け、RPP の衝突先読みが前方ロボットの手前で速度 0 にし、前が進めば
進む」の stop-and-go 追従に変える。(a) の迂回防止は本 BT の守備範囲外で、keepout に「閂」を引いて迂回路を地図から
消す作業 (未実施) が担う。

実装した機能:
- `behavior_trees/navigate_through_poses_follow.xml` — 計画失敗時に旧経路を保持する自作 BT。リカバリは costmap クリア + Wait のみ (Spin / BackUp を削除)。連続失敗 20 回 (≈60 s) で旧経路を諦め既定と同じ失敗経路に落ちる。
- `navigation.launch.py` が `bt_navigator` の `default_nav_through_poses_bt_xml` でこの XML を指す (waypoint_runner の NavigateThroughPoses にだけ効く。RViz 単発ゴール = NavigateToPose は既定 BT のまま)。
- RPP `max_allowed_time_to_collision_up_to_carrot: 1.0 → 2.0` (走行中の停止距離 0.4 → 0.6 m。§2.2)。
- `waypoint_runner.py --auto-retry` 既定 1 → 10 (区間開始時に塞がれていると約 15 s で区間失敗になるため、再送を続けて「前が抜けるまで待つ」を実現。§2.4)。

スコープ外 (意図的に未対応):
- **迂回路を消す keepout の閂** (各分岐で本線以外の枝に短い黒帯)。地図画像の編集作業で、本 BT の相方。これが無いと「戻るルート」「停止ロボットの横を車道側へ回り込む」は防げない (§2.1)。
- **停止点の待ち行列** (停止点が占有されていたら手前の予備候補に止まる waypoint_runner 側ロジック)。NavFn の `tolerance: 0.5` が横の空きセルへ目標をずらす問題への対策で、別機能として実装する。
- **局所回避制御器 (DWB / MPPI) への変更**: 狭い通路でロボットの横をかすめて抜ける挙動は接触リスクが大きいと判断。RPP は経路から外れないので「NavFn が通せる幅があるときだけ抜ける」になる。
- **nav2_route**: Jazzy 1.3.13 にバイナリあり。コースを経路グラフにすれば迂回は構造的に消えるが、waypoint_runner の区間送信との組み直しが重く今期は見送り。
- **nav2_collision_monitor** による速度に依らない距離ガード (§2.2 の代替案)。

## 2. 設計の勘所

### 2.1 効く範囲は「計画が失敗する場面」だけ (critic で主張を狭めた)

判断: 本 BT が挙動を変えるのは ComputePathThroughPoses が失敗したときだけ。NavFn は cost < 253 のセルを通行可とし
内接半径 0.325 m で膨張するので、前方ロボットと壁の間に **約 0.7 m 以上の隙間があれば計画は成功**し、その経路
(追い越し・迂回) を 3 s 以内に採用する。地図上に迂回路がある区間も同じで、塞がれた瞬間に迂回路が「成功経路」になる。
10-14 / 10-15 に観察した「戻るルートへの迂回」は planner 成功ケースであり、本 BT では防げない。

したがって本 BT の主張は「**costmap 上の隙間が車体幅 0.65 m 未満で、地図に迂回路が無い区間**で、既定 BT の完全停止を
stop-and-go 追従に変える」に限る。迂回・回り込みの防止は keepout の閂 (スコープ外) と、必要なら global costmap の
`footprint_padding` (planner が要求する隙間を広げ、怪しい隙間では計画失敗 = 追従側に倒す。local は RPP の実寸判定に
使うので触らない) の担当。

却下した代替案:
- 計画失敗 = 完全停止で待つ (既定 BT): 区間の通路が全部空くまで動けない。前が「遅いが動いている」ときに差が出る。
- 既定 BT の「失敗時に global costmap を消して再試行」: 前方ロボットも消えて貫通経路が「成功」として返るだけで、結果は追従と同じ。消さずに旧経路を使う方が素直で、ゴースト消去の副作用も無い。

### 2.2 車間は RPP の衝突掃引が決め、先読み点で打ち切られる

実測 (ソース `collision_checker.cpp isCollisionImminent`、1.3.13): 掃引は `i × projection_time <
max_allowed_time_to_collision_up_to_carrot` のループだが、投影点が `carrot_dist` (先読み距離) を超えた時点で break する。
先読み距離は `use_velocity_scaled_lookahead_dist: true` のため **実測速度 × 1.5 s を [0.3, 0.9] m にクランプ**。

| 状態 | 掃引距離 = min(2.0 s × 指令速度, 先読み距離) | 旧設定 (1.0 s) |
|---|---|---|
| 走行中 0.4 m/s | min(0.8, 0.6) = **0.6 m** | 0.4 m |
| 停止中 (実測速度 0) | min(0.8, 0.3) = **0.3 m** | 0.3 m |

判断: 2.0 s への変更で「止まる距離」は 0.4 → 0.6 m に伸びたが「止まった後に再発進を許す距離」は 0.3 m のまま。
前方停止物には 0.6 m で初停止 → 掃引が縮んで再発進 → 速度が乗って再停止、を繰り返し **前端から 0.3 m まで段階的に寄る**。
10-15 bag の停止 9 時刻で lethal が前端から 0.14〜0.28 m にあった観測と整合する。実装当初の「車間 0.8 m」は誤りで
critic が訂正した。車間を本当に伸ばすなら `min_lookahead_dist` を上げる (経路の内回りと引き換え) か、
nav2_collision_monitor の stop ポリゴンで速度に依らない距離ガードを別に置く。どちらも箱 1 個の実走 (§6) の結果を見てから。

### 2.3 BT 実装: 計画結果を別キーで受け、成功時だけ採用する

実測 (ソース): `ComputePathThroughPosesAction::on_aborted` は空 Path を自分の `path` ポートに書く。既定 XML のまま
失敗を飲み込むと FollowPath が空経路で INVALID_PATH になる。`SetBlackboard` (BT.CPP 4.10) は `value="{src}"` で
エントリ間の Any コピー (型継承) ができる。`PipelineSequence` は RateController 配下が RUNNING / SUCCESS の間
FollowPath を tick し続ける。`bt_action_server loadBehaviorTree` は同一 XML なら tree を再生成せず blackboard が
航行間で残る。

判断:
- 計画は `path="{new_path}"` で受け、成功時だけ `SetBlackboard output_key="path" value="{new_path}"` で採用。
- 根を (Reactive でない) `Sequence` にし `Script "have_path := false; plan_fail := 0"` を航行ごとに 1 回流す。前回航行の `{path}` を誤用しない。
- 失敗側は `Fallback` の第 2 子 `Sequence(Script "plan_fail := plan_fail + 1", ScriptCondition "have_path == true && plan_fail < 20")`。一度も成功していなければ既定どおり失敗を上へ返す。
- `RemovePassedGoals` は `Sequence` 内なので 3 s に 1 回しか動かない (既定は ReactiveSequence で毎 tick)。feedback の残り点数が最大 3 s 遅れるだけ。
- FollowPath 直下の `RecoveryNode(1) + ClearLocalCostmap` は `WouldAControllerRecoveryHelp` の対象コードが限られ実質死にコード (既定と同じ、害なし)。

### 2.4 連続失敗 20 回で旧経路を諦める (AMCL 跳び対策) と runner の再送増

判断: AMCL が 0.3〜1 m 跳ぶと現在位置が膨張帯に入り NavFn は失敗し続ける。旧経路は跳んだ分ずれて壁を貫くので、
保持し続けると RPP が壁へ向かって local costmap の判定で止まり、`failure_tolerance: -1` + `movement_time_allowance: 3600`
で **静かに永久 RUNNING** になる (critic H4)。既定 BT なら ABORT → waypoint_runner が跳び後の位置から再計画する。
そこで連続 20 回 (RateController 3 s × 20 ≈ 60 s) で旧経路を諦め、既定と同じ失敗経路 (リカバリ Clear / Wait → 6 回で
ABORTED → runner の自動再送) に落とす。

その帰結として「前が 60 s 以上止まっている」場合も ABORT → 再送になる。また「区間開始時に前方ロボットで塞がれている」
(スタート待機列で最も起きやすい) は have_path=false で約 15 s で区間失敗になる (critic H3)。どちらも
waypoint_runner の `--auto-retry` を 1 → 10 に増やして「再送で待ち続ける」(10 回 × 約 20 s ≈ 3〜4 分) で吸収する。
本当に通れない場所なら随伴者が Back で止める。

### 2.5 Spin / BackUp を外した — 実害はほぼ無く、理由は 10-15 の controller 設定

判断: 狭い通路で隣にロボットがいる状態の 90° 旋回・0.3 m 後退は接触リスクそのものなので外した。ただし
`failure_tolerance: -1` の下では FollowPath は衝突停止で失敗しないため、外側リカバリは「塞がれて停止」では既定 BT でも
到達しない。自力復帰を失ったのは 10-15 の controller 設定であり、本 BT の削除が効くのは区間開始時の計画失敗の局面だけ。

## 3. データフロー

```
waypoint_runner.py ──NavigateThroughPoses (goals)──▶ bt_navigator (navigate_through_poses_follow.xml)
                                                        │
   RateController 0.333 Hz ─ Fallback ┬ Sequence: RemovePassedGoals → ComputePathThroughPoses(path={new_path})
                                      │             → SetBlackboard path←{new_path} → have_path:=true, plan_fail:=0
                                      └ Sequence: plan_fail+=1 → ScriptCondition(have_path && plan_fail<20)   ← 旧経路維持
   FollowPath(path={path}) ──▶ controller_server (RPP) ──/cmd_vel (Twist)──▶ twist_mux ──▶ /robot_speed_cmd
        │ RPP: 掃引 min(2.0 s×v, carrot) に lethal → "collision ahead" → 速度 0 (failure_tolerance -1 で失敗にしない)
   失敗経路 (have_path=false または plan_fail≥20): RecoveryNode(6) RoundRobin = ClearCostmaps / Wait 5 s → ABORTED
        └──▶ waypoint_runner: --retry-wait 5 s 後に残りの点を再送 (--auto-retry 10 回まで) → 以後 Start 待ち
```

## 4. 使い方

本番は `auto_run.launch.py` 経由で自動的に効く (navigation.launch.py が XML を指す。runner は既定引数)。手動:

```bash
# rerobot_env 内。nav 構成を上げれば bt_navigator はこの XML を読む
ros2 launch rerobot_bringup nav_amcl_scanall.launch.py map_dir:=/workspace/maps/2d/glim/<地図>
ros2 run rerobot_bringup waypoint_runner.py /workspace/maps/2d/glim/<地図>/waypoints/course.yaml   # --auto-retry 既定 10
# 既定 BT に戻す (比較用): navigation.launch.py の default_nav_through_poses_bt_xml を外すか、以下で上書き
ros2 launch rerobot_bringup nav_amcl_scanall.launch.py map_dir:=... \
  params_file:=...   # ※ bt_navigator のパラメータは launch 内の dict で与えているので XML 切替は launch を編集する
```

台上スモークテスト (2026-10-16 に 2 回実施、静的 TF + 地図 `2026-10-09_1643_5goukan_chal`、センサなし):
map_keepout + navigation を上げ waypoint0 → waypoint1 の goal を送り `/cmd_vel 0.4 m/s` を確認 → `/planner_selector` に
存在しない planner 名を流して計画を連続失敗させる → 20 s 後も bt_navigator active・goal RUNNING・recoveries 0・`/cmd_vel`
継続。3 回目は失敗を 75 s 続け、**20 回目の失敗 (最初の失敗から 57.5 s) で Goal failed → ABORTED、`/cmd_vel` 停止**を確認
(上限が効く。テストの失敗コードは INVALID_PLANNER 301 でリカバリ対象外のため即 ABORT。実機の NO_VALID_PATH では
Clear / Wait のリカバリ 6 回 ≈ 15 s を経てから ABORT になる)。スクリプトは session scratchpad の `bt_smoke.sh` (リポジトリ外)。

| 項目 | 値 | 説明 |
|---|---|---|
| `default_nav_through_poses_bt_xml` | `share/rerobot_bringup/behavior_trees/navigate_through_poses_follow.xml` | navigation.launch.py が設定 |
| `RateController hz` | 0.333 | 再計画周期 3 s (既定と同じ) |
| `plan_fail` 上限 | 20 回 ≈ 60 s | 旧経路を諦める連続失敗回数 |
| `max_allowed_time_to_collision_up_to_carrot` | 2.0 s | 掃引は先読み距離で頭打ち (§2.2) |
| `waypoint_runner --auto-retry` | 10 (旧 1) | 区間失敗の自動再送回数 |

## 5. 変更ファイル一覧

- `ros2_ws_main/src/bringup/rerobot_bringup/behavior_trees/navigate_through_poses_follow.xml` — 新規。追従型 BT (設計は冒頭コメント)
- `ros2_ws_main/src/bringup/rerobot_bringup/CMakeLists.txt` — `behavior_trees` を install
- `ros2_ws_main/src/bringup/rerobot_bringup/launch/parts/navigation.launch.py` — bt_navigator に `default_nav_through_poses_bt_xml`
- `ros2_ws_main/src/bringup/rerobot_bringup/config/nav2_params.yaml` — FollowPath `use_collision_detection: true` (明示) / `max_allowed_time_to_collision_up_to_carrot: 2.0` + 掃引の注記
- `ros2_ws_main/src/bringup/rerobot_bringup/scripts/waypoint_runner.py` — `--auto-retry` 既定 1 → 10
- `CLAUDE.md` — rerobot_bringup 資産一覧に behavior_trees を追加

## 6. 既知の制限

- **実走未検証**。台上で確認したのは BT 機構 (失敗を飲み込み FollowPath が続く) だけ。衝突停止・stop-and-go 再発進・本物の NavFn 失敗 (探索時間・CPU)・初回失敗分岐・runner の p → 再送・preempt は未検証。
- **critic が推奨する次の 1 実験 = 箱 1 個の実走 (約 30 分、0.4 m/s)**: 廊下で waypoint 2 点の短い区間を作り (1) 正面 2 m に全幅を塞ぐ箱 → 初停止距離 (予測 0.6 m) と這い寄り (0.3 m) を巻尺 + `/lookahead_collision_arc` + `/rosout` "collision ahead" で確認、(2) 箱を 1 m 前へ → 3 s 以内に再発進するか、(3) 箱を片側に寄せ隙間 0.8〜1.0 m → 経路が隙間を通るか (§2.1 の確認)、(4) Start 位置正面を塞いで区間開始 → 約 20 s で区間失敗 → 自動再送が回るか。NavFn 失敗時間は `/rosout` の planner 行の間隔 (3 s 超なら探索が重い) で副次的に判定。
- **停止と追従が外から区別できない**: 衝突停止中も `/waypoint_runner/status` は「走行中」。`distance_remaining` が 60 s 変わらなければ「停止 N s」と出す改良が安価 (未実施)。
- **走行中に RViz から Nav Through Poses を送らない**: preempt は goals だけ差し替え `{path}` / have_path を触らないので、計画が成功するまで旧 goals への旧経路を走る。
- **ゴースト障害物**: 旧経路保持 + 永久待機の組合せでは local costmap のゴースト lethal (地面の壁、`scan-ground-ghost`) が前方 0.3〜0.6 m に出ると人が来るまで止まる (既定 BT も同じ)。計画失敗時の global clear を外したので global 側のゴースト耐性は観測層の raytrace のみ。60 s 上限で ABORT → 再送に落ちるのが唯一の自動復帰。
- **追従中の planner 負荷**: 失敗探索は A* の早期終了が効かず上限サイクルまで回る可能性 (critic H5)。3 s 周期で 1 コア級なら 10-11 の TF 失効再発の懸念。実走 bag の planner 行間隔で確認。
- **cost 規制速度との相互作用**: 1 m 通路では中心コストで速度が 0.33 m/s 程度に落ち、掃引 (= 停止距離) もそれに比例して縮む。「車間」は距離ではなく時間 (しかも先読みで頭打ち)。
