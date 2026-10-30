<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# Nav2 の平滑化 2 段 (経路 SimpleSmoother + 速度 velocity_smoother)

- 日付: 2026-10-30
- 対象パッケージ: `rerobot_bringup` (`ros2_ws_main/src/bringup/rerobot_bringup`、launch / config / behavior_trees / scripts のみ。C++ 変更なし)
- 対象 ROS: ROS 2 Jazzy、Nav2 1.3.13 (reRoBot コンテナ `rerobot_env`)
- 関連文書: [2026-10-16_follow_bt_narrow_corridor.md](2026-10-16_follow_bt_narrow_corridor.md) (SmoothPath を挿した追従 BT)、[2026-10-16_auto_run_joy_mux.md](2026-10-16_auto_run_joy_mux.md) (twist_mux の配線)、[2026-10-30_global_costmap_rolling_window.md](2026-10-30_global_costmap_rolling_window.md) (smoother_server が読む global costmap)

## 1. 目的・概要

経路追従の動きがカクつく (ユーザ観察、10-30)。調べると平滑化の層がどこにも無かった:
NavFn は 0.05 m 格子の 8 近傍探索なので経路は 0°/45° の折れ線、RPP はその折れ目で曲率減速を発動し、
出た速度段差は twist_mux → `epos4_controller` を素通りする (slew 制限 15000 rpm/s は実質無制限)。
Nav2 標準の 2 部品を追加して、経路の形 (空間) と速度の変化率 (時間) を別々に滑らかにする。

実装した機能:
- `smoother_server` (SimpleSmoother) を navigation 部品に追加し、追従 BT の計画成功枝で `SmoothPath` を呼ぶ
- `velocity_smoother` を controller_server / behavior_server と twist_mux の間に挟み、加減速上限をかける
- `auto_run_check.py` の起動表に 2 ノードを追加、`auto_run.launch.py` の bag に `/cmd_vel_nav` (平滑化前) を追加

スコープ外 (意図的に未対応):
- planner の変更 (Smac Hybrid-A* / Theta*): 経路の折れを根本から無くせるが CPU 負荷が増える。planner_server が
  1 コアを占有して AMCL の TF が失効した前歴 (10-11) があるので見送り
- RViz 単発ゴール (NavigateToPose) への経路平滑化: Nav2 既定 BT を使っているので SmoothPath は入らない。速度平滑化は効く
- nav2_collision_monitor: 公式 launch では velocity_smoother の後段にあるが、RPP の衝突先読みで代替している現状を維持
- カクつきの原因の切り分け (経路形状 / 再計画の差し替え / 衝突先読み stop-and-go): 未実施。bag の `/cmd_vel_nav` と `/cmd_vel` と `/plan` で判別する

## 2. 設計の勘所

### 2.1 速度平滑化を全指令の通り道に置き、joy は通さない

判断: controller_server と behavior_server の出力を `/cmd_vel_nav` に逃がし、`/cmd_vel` は velocity_smoother だけが出す
(Nav2 公式 `navigation_launch.py` と同じ remap)。behavior_server も通さないと Wait / BackUp の指令だけ平滑化を素通りする。
joy (`/joy_vel`) は twist_mux に直接入るので対象外 — 人が握っている間は即応を優先する。

実測 (台上 10-30): `/cmd_vel_nav` の 0 → 0.4 m/s の段差が `/cmd_vel` では 0.025 m/s 刻み (20 Hz × 0.5 m/s²) のランプになった。
`ros2 topic info -v /cmd_vel` の publisher は velocity_smoother 1 つ。

パラメータの根拠:
- `max_accel` 前進 0.5 m/s² (0 → 0.4 を 0.8 s)。角 3.2 rad/s² は RPP の `max_angular_accel` と同じ
- `max_decel` は −2.5 / −3.2 と大きいまま: 緩めると RPP の "collision ahead" (速度 0) が車体に届くのが遅れる。減速側は平滑化しない
- `feedback: OPEN_LOOP`: 直前の自分の出力を現在速度とみなす。CLOSED_LOOP は `/odometry/filtered` の遅れを拾うので採用しない
- `velocity_timeout: 1.0` > twist_mux の timeout 0.5 s: 入力が止まったら先に twist_mux が joy / 無音へ切り替える

却下した代替案:
- `epos4_controller` の slew 制限 (`max_motor_accel_rpm_per_s`) を下げる: モータ単位の制限なので直進と旋回を別に制御できず、joy にも効いてしまう

### 2.2 経路平滑化は追従 BT の「計画成功枝」だけ、失敗しても新経路を捨てない

判断: `SmoothPath` を `ComputePathThroughPoses` と `SetBlackboard path:={new_path}` の間に入れ、`ForceSuccess` で包む。
smoother が失敗・タイムアウトしても出力ポート `smoothed_path` は書かれないので、未平滑の `{new_path}` がそのまま採用される。
包まないと Sequence が失敗して「旧経路保持」の枝に逃げ、出来たばかりの新経路を捨てる。

`max_smoothing_duration: 0.5` s: SimpleSmoother は数 ms で終わるが、`RateController 0.333 Hz` の計画周期に収まる上限を置く。
衝突チェック (`check_for_collisions`) は既定 false のまま — SimpleSmoother 自体が反復中に costmap の致死セルを避ける。

却下した代替案:
- ConstrainedSmoother: 曲率制約つきで品質は上だが Ceres 依存で重い。SimpleSmoother で足りなければ検討

### 2.3 smoother_server は global costmap の追従窓を読む

smoother_server は既定で `global_costmap/costmap_raw` と `global_costmap/published_footprint` を購読する。
同日の追従窓化 (rolling_window 60 m) でも経路は窓内に収まる前提なので変更なし。

## 3. データフロー

```
planner_server ──/plan── bt_navigator ──[SmoothPath]──> smoother_server (SimpleSmoother)
                                            │ smoothed path ({new_path} → {path})
                                            ▼
controller_server (RPP) ──┐
behavior_server (Wait…) ──┴── /cmd_vel_nav (geometry_msgs/Twist) ──> velocity_smoother ──> /cmd_vel
                                                                                             │ 優先 10
joy_teleop ──────────────────────────────────────────────────────────── /joy_vel ──> twist_mux ──> /robot_speed_cmd
                                                                                             優先 100
```

lifecycle_manager_navigation の管理対象: controller_server, smoother_server, planner_server, behavior_server, bt_navigator, velocity_smoother。

## 4. 使い方

通常運用は変更なし (`auto_run.launch.py` / `nav_*.launch.py` が navigation 部品を include する)。

台上確認 (10-30 に実施したもの。map と静的 TF を与えて navigation 部品だけを起動):

```bash
# rerobot_env 内 (install/setup.bash を source 済み)
for n in controller_server smoother_server planner_server behavior_server bt_navigator velocity_smoother; do ros2 lifecycle get /$n; done
ros2 topic info -v /cmd_vel          # publisher が velocity_smoother 1 つ
ros2 topic echo /cmd_vel_nav --field linear.x --csv   # 平滑化前
ros2 topic echo /cmd_vel --field linear.x --csv       # 平滑化後 (0.025 刻みのランプ)
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [{header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}]}"
# smoother_server のログに "Received a path to smooth." が計画ごとに出る
```

| パラメータ (`nav2_params.yaml`) | 値 | 説明 |
|---|---|---|
| `velocity_smoother.max_velocity` | [0.4, 0, 1.0] | RPP の desired_linear_vel / rotate_to_heading_angular_vel と揃える |
| `velocity_smoother.max_accel` | [0.5, 0, 3.2] | 発進のランプ。カクつきが残るなら 0.3 まで下げる |
| `velocity_smoother.max_decel` | [−2.5, 0, −3.2] | 衝突停止を遅らせないため大きいまま |
| `smoother_server.simple_smoother.w_smooth` | 0.3 | 上げるほど折れが消えるが壁に寄る |
| `smoother_server.simple_smoother.w_data` | 0.2 | 元経路への忠実度 |

## 5. 変更ファイル一覧

- `launch/parts/navigation.launch.py` — controller/behavior の `cmd_vel` → `cmd_vel_nav` remap、smoother_server / velocity_smoother 追加、lifecycle 登録
- `config/nav2_params.yaml` — `velocity_smoother:` / `smoother_server:` 節を追加 (controller_server 節の直後)
- `behavior_trees/navigate_through_poses_follow.xml` — 計画成功枝に `ForceSuccess{SmoothPath}` (変更点 4)
- `scripts/auto_run_check.py` — 起動表に smoother_server / velocity_smoother
- `launch/auto_run.launch.py` — bag 記録に `/cmd_vel_nav`

## 6. 既知の制限

- **実走未検証** (台上 = 静的 TF + 地図のみ)。走行での発進・旋回の体感と、`/cmd_vel_nav` vs `/cmd_vel` の bag 比較が次
- カクつきの原因が「再計画で経路が差し替わり先読み点が跳ぶ」なら、この 2 段では消えない (計画周期 3 s と NavFn の経路のばらつきの問題)
- SimpleSmoother は衝突チェックに失敗すると元経路を返す (平滑化なし)。狭路では効かないことがある
- velocity_smoother の `min_velocity` 前進側 −0.4: BackUp 行動用。RPP は後退しない
- RViz 単発ゴールには経路平滑化が効かない (§1 スコープ外)
