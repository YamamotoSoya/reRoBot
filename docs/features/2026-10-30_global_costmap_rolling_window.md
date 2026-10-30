<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# global costmap の追従窓化 (rolling_window 60 m) と waypoint_runner の窓送り

- 日付: 2026-10-30
- 対象パッケージ: `rerobot_bringup` (`ros2_ws_main/src/bringup/rerobot_bringup` — `config/nav2_params.yaml`, `scripts/waypoint_runner.py`)
- 対象 ROS: ROS 2 Jazzy / Nav2 1.3.13 (reRoBot コンテナ `rerobot_env`)
- 関連文書: 台上試験の記録 `bags/exp/2026-10-30_rolling_window/README.md`、追従 BT `2026-10-16_follow_bt_narrow_corridor.md`、auto_run `2026-10-16_auto_run_joy_mux.md`

## 1. 目的・概要

global planner (NavFn) の計画が重い問題への対策。つくば地図 (composite 9984×4736 = 47M セル @0.1 m) では
global costmap の層更新 (1 Hz) と NavFn の探索が地図全面を対象にしており、ロボット PC では計画 1 回に 6〜9 s かかっていた
(10-17 bag)。10-12 の軽量化 (解像度 0.05→0.1、A*、rfans のみ、raytrace 10 m) の後も主因は地図サイズそのものだった。

対策は「global costmap を自機中心 60 m 四方の追従窓にする」。これだけなら yaml 3 行だが、Nav2 の ComputePathThroughPoses は
渡された点を**全部**計画するため、窓の外に 1 点でもあると `GOAL_OUTSIDE_MAP` (304) で区間が始まらない。waypoint_runner は
停止点で区切った区間 (つくばは 11 点・281 m) を 1 本で送っていたので、runner 側を「自機から半径 R 以内の点だけ送り、
進んだら差し替える」窓送りに変えた。

実装した機能:
- `nav2_params.yaml` global_costmap: `rolling_window: true` / `width: 60` / `height: 60` / `track_unknown_space: false` (§2.2)
- `waypoint_runner.py`: 窓送り (`--send-radius` 20 m、`--resend-left` 2)、中間点の自動挿入 (`--max-spacing` 10 m)、自機位置は TF map→base_link
- `waypoint_runner.py`: 例外終了時も走行中ゴールを必ずキャンセル (SIGINT が rclpy の pybind 内で TypeError 化する件、§2.4)

スコープ外 (意図的に未対応):
- BT 側でゴール列を窓内に切り詰めるノード: Nav2 に無く C++ プラグイン自作になる。Jazzy の bt_navigator は `plugin_lib_names` を列挙すると二重登録で segfault する (CLAUDE.md) ので、runner 側 (Python) で済ませた
- Smac planner への切替・`max_planning_time`: 範囲でなく時間で縛る別方式。planner が変わると RPP との組み合わせが未検証になる
- 停止点の待ち行列・keepout の閂 (10-16 の残件) は本件と独立

## 2. 設計の勘所

### 2.1 窓送り = 「伸ばせるときだけ差し替える」

実測: bt_navigator (`NavigateThroughPosesNavigator::onPreempt`) は、走行中に同じ BT を指す goal (behavior_tree 空 = 既定 BT) が
来ると木を作り直さず blackboard の `{goals}` だけ入れ替える (`initializeGoalPoses`。`{path}` は触らない)。台上では差し替え要求から
10 ms で `Begin navigating ... through 6 poses`。FollowPath は旧経路のまま走り続け、次の RateController tick (3 s) で新経路に乗る。
旧 goal は ABORTED で返るが runner は見ない。

判断: 差し替えの条件は「1 s ごとに自機位置で窓を選び直し、**今送っている最後の点より先へ伸ばせる** かつ 残りが `--resend-left` 以下」。
初版の「残りが 2 点になったら」だけだと、次の点がまだ半径外のときに 1 点だけの窓を送ってそこで goal 成功 → 停止 していた
(台上 runner_test.log)。伸ばせないときは待てば自機が進んで半径内に入る。

- 窓 R = 20 m < 窓半幅 30 m: 差し替えまでに進む分 (3 s × 0.5 m/s) と costmap 更新 1 Hz の中心ずれの余裕
- 窓の終点に着いてしまった (差し替えが間に合わなかった) ときは Nav2 が goal 成功で止めるので、続きを送り直す (一瞬止まるだけ)
- `--send-radius 0` で従来の区間一括送り (rolling_window: false に戻すとき)

却下した代替案:
- feedback の `number_of_poses_remaining` だけで差し替え: 上記の 1 点窓問題
- 区間の全点を送って窓を 230 m にする: 地図全面と変わらず軽くならない

### 2.2 `track_unknown_space: false` — Nav2 1.3.13 の rolling StaticLayer の制約

実測 (台上、修正前): rolling + `use_maximum: true` + `track_unknown_space: true` で global costmap が **全面 unknown (lethal 0)**。
地図も keepout マスクも一度も書かれず、NavFn は unknown を直進する経路を返した。

機構 (確定。根拠はソースと上流修正、台上試験はそれに整合): `nav2_costmap_2d/plugins/static_layer.cpp` 479 行 (jazzy と 1.3.13 で同一) の
isRolling 分岐は、非 rolling の `updateWithMax` を使わずセルごとに `master.setCost(i, j, max(getCost(mx, my), master.getCost(i, j)))` を書く。
master は毎周期 `resetMap` で既定値に戻り (`layered_costmap.cpp`)、`track_unknown_space: true` だと既定値 = NO_INFORMATION (255) > LETHAL (254)
で常に master が勝つ。非 rolling の `updateWithMax` は NO_INFORMATION を特別扱いするので 10-16 の試験 (非 rolling) では見えなかった。
local_costmap は `track_unknown_space` 未指定 (Costmap2DROS 既定 false) なので同じ keepout_static でも成立していた。
**上流 main では同じ箇所が「updateWithMax と同じ規則」に修正済み** (既知のバグ、1.3.13 未反映)。
識別実験 (critic 指摘の交絡を切る): `plugins: ["static_layer"]` のみ・rolling・track true で `use_maximum` true → lethal 0、false (上書き) → lethal 68845
(= 窓内の地図 lethal)。max と既定値 255 の組み合わせだけが残る。

判断: global も `track_unknown_space: false` (既定値 FREE=0) にする → 窓内の 地図 lethal ∪ keepout lethal 300644 セルが全て lethal で出た。
副作用 = 地図の未知セルが unknown でなく free になる (NavFn `allow_unknown` の「ほぼ致死 253 で通過可」が効かない)。試験窓 (wp0 中心 60 m) では
「地図未知 ∧ keepout 非 lethal」が 0 セルで、全面 B の区間 1 全点の経路も true/false で同一 (239.2 m) だったが、コース全体では走行帯内に
未知の斑点 69463 セル (約 695 m²、最大 1 m² の小片 20656 個) があり、これらの扱いが変わる。区間 2〜3・他地図は未評価。
未知の意味を捨てる最も安い選択であることは意識しておく。

却下・保留した代替案:
- `use_maximum: false` (上書き): 後の StaticLayer (keepout) の白が地図の壁を消す (10-16 の問題そのもの)
- (a) 上流修正を static_layer.cpp にバックポートして `track_unknown_space: true` を維持: Nav2 のローカルビルドが要る (保留)
- (b) keepout を地図 pgm に焼き込んで StaticLayer 1 枚 + `use_maximum: false`: 未知の意味を保てるが地図生成 (make_maps.sh) の変更 (保留)

### 2.3 中間点の自動挿入

つくば course.yaml の隣接間隔は `20 14 12 10 14 72 91 12 29 6 25 33` m で、72/91 m は窓 20 m に次の 1 点すら入らない。
runner が読み込み時に `--max-spacing` (10 m) を超える区間を等分して中間点を挿む (名前 `<前の点>+k`、向きは線分方向、stop ではない)。
`--start N` は元ファイルの index で受け、挿入後に同名の点を探し直す。地図リポジトリ (maps/2d) の course.yaml は触らない。

### 2.4 例外終了時のゴールキャンセル

台上で SIGINT (timeout -s INT) が rclpy の feedback 変換 (pybind) 中に届き、KeyboardInterrupt が `TypeError: Unable to convert
function return value` に包まれて `except KeyboardInterrupt` を素通り → ゴールを残したまま終了 → 次の runner 起動時に bt_navigator
が古い goal を preempt する形で発覚。本番で Ctrl-C しても走り続ける経路なので、main() を `BaseException` で受けて先にキャンセルし、
Ctrl-C 由来 (`__cause__`/`__context__` も見る) でなければ再送出する。

## 3. データフロー

```
course.yaml ──load_course──▶ densify(max_spacing 10 m) ──split_segments(stop)──▶ 区間 [i0..i1]
                                                                                    │
  TF map→base_link ──▶ select_window(idx, cursor): 自機から send_radius 以内の先頭の点列 (sub)
                                                                                    │
  NavigateThroughPoses goal (sub) ──▶ bt_navigator ──▶ ComputePathThroughPoses (窓内のみ) ──▶ FollowPath → /cmd_vel
        ▲                                   │ feedback number_of_poses_remaining
        │ 1 s ごと: 自機位置で窓を選び直し、sub[-1] より先へ伸ばせる & 残り ≤ resend_left なら新 goal を送る
        └── (同 BT なので onPreempt が {goals} だけ入替、木は継続)

  global_costmap (planner_server 内): rolling_window 60×60 m @0.1 m、plugins static_layer(/map) + keepout_static(/keepout_filter_mask)
                                     + obstacle_layer(/rfans/scan) + inflation、use_maximum: true、track_unknown_space: false
```

## 4. 使い方

```bash
# 本番は auto_run.launch.py がそのまま起動する (runner の既定値で窓送り)。単体:
ros2 run rerobot_bringup waypoint_runner.py <map_dir>/waypoints/course.yaml [--send-radius 20] [--resend-left 2] [--max-spacing 10]
# 台上試験 (隔離ドメイン 151、実機なし。bags/exp/2026-10-30_rolling_window/README.md)
/workspace/bags/exp/2026-10-30_rolling_window/run_headless.sh "" A        # map + keepout + navigation + 静的 TF
python3 plan_test.py "47.4,-6.33;50.34,-15.96;53.28,-25.59" --repeat 3    # 計画時間
python3 costmap_check.py                                                   # 地図 ∪ keepout の lethal 合成検査
```

| 項目 | 値 | 説明 |
|---|---|---|
| global_costmap `rolling_window` / `width` / `height` | true / 60 / 60 | 自機中心 60 m 窓。戻すときは false (width/height は無視) |
| global_costmap `track_unknown_space` | false | §2.2。true だと rolling で地図が書かれない |
| `--send-radius` | 20 m | 1 本の goal に入れる点は自機からこの距離以内。0 = 区間全点 (rolling_window: false 用) |
| `--resend-left` | 2 | 送った点の残りがこれ以下で、かつ伸ばせるときに次窓へ差し替え |
| `--max-spacing` | 10 m | 隣接点の間隔がこれを超えたら中間点を挿む。0 = 挿まない |

計画時間 (このノート PC、同じ地図・同じゴール、planning_time): 全面 0.78 s (3 点 20 m) / 2.78 s (8 点 239 m) → 窓 0.002〜0.004 s。
ロボット PC では全面で 6〜9 s だった (costmap 更新と競合)。

## 5. 変更ファイル一覧

- `ros2_ws_main/src/bringup/rerobot_bringup/config/nav2_params.yaml` — global_costmap に rolling_window/width/height、track_unknown_space true→false (注記つき)
- `ros2_ws_main/src/bringup/rerobot_bringup/scripts/waypoint_runner.py` — densify()、select_window()/robot_xy() (tf2_ros)、run_segment の窓送り、spin_until の until、main() の例外時キャンセル、CLI 3 引数
- `bags/exp/2026-10-30_rolling_window/` — 台上試験 (run_headless.sh / plan_test.py / costmap_check.py / nav2_params_full.yaml / ログ / README.md)
- 再ビルド不要 (yaml・scripts とも symlink install)

## 6. 既知の制限

- **実走未検証**: 差し替え時に RPP が旧経路のまま途切れず走るか (台上は静止)。窓の縁 (R 20 m / 半幅 30 m) で costmap 更新 1 Hz の中心ずれが計画失敗を起こさないか
- 窓の外を通る迂回路は見つからない (60 m を超えて回り込む経路は計画できない)。歩道コースは窓内で足りる想定
- 未知セルが free になるので、keepout が覆っていない未知領域があればそこを通る経路が出うる (今の地図はコース外を keepout が覆う)
- 差し替えで bt_navigator の `number_recoveries` と start_time がリセットされる (リカバリ回数の上限が窓ごとになる)
- 通過判定 (RemovePassedGoals 0.7 m) で取りこぼした点が窓の後ろに残ると、その点が窓外に出た時点で GOAL_OUTSIDE_MAP → 旧経路保持 20 回 → 区間失敗 → 自動再送 (再送は残り点数から組むので取りこぼし点は捨てられる)
- `--start N` を中間点 (`waypointK+j`) には指定できない (元ファイルの index のみ)
