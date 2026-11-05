<!-- claude: 経路計画・経路追従読本 第7章 (2026-11-06) -->

# 第7章 実務 — 逆引きと再生試験

この章は繰り返し参照する道具箱である。
症状から原因の候補を引く表、設定を変えるときのチェックリスト、bag で原因を切り分ける再生試験の手順を置く。

## 7.1 症状からの逆引き

| 症状 | まず見るもの | 候補の原因 | 章 |
|---|---|---|---|
| 物体に触れていないのに止まり続ける | rosout の「collision ahead」、`/lookahead_collision_arc` | 弧の掃引で後ろの角が物体を踏む | 4.2.3、事例 A |
| 止まり続け、旋回も後退もしない | local costmap で footprint の縁 | 現在姿勢の縁が既に lethal 上 | 4.2.3、事例 E |
| 隙間に入ってから出られない | global の 253 帯の幅、経路 | 点の planner が 1 セルの帯を縫った | 3.3、事例 B |
| 「no valid path found」が続く | 道幅、地図と live のずれ | 向き付き planner には通れない幅、地図ずれ | 2.6、事例 C |
| 「Start occupied」 | 自機の姿勢の global コスト | 始点の四角が既に当たっている | 3.5 |
| 全ての計画が 0.00 s で失敗 | 起動時の ERROR | Smac の膨張半径 < 外接半径 | 3.5、7.2 |
| 角を大回りする、急に曲がらない | 経路の最小回転半径 | Hybrid-A* には旋回が無い、R と刻みの下限 | 3.5、事例 C |
| 角を内側に攻めて接触 | 楽観設定の有無、物体が LiDAR に写るか | 楽観 footprint・細い地図・センサの死角 | 4.6、事例 D |
| 近くの人を避けず、前で止まって待つ | 再計画の周期、global の障害物の範囲 | RPP は避けない。迂回は 3 s ごとの再計画だけ | 1.3 |
| 「Goal ... outside bounds」 | global の窓の大きさと waypoint の距離 | ゴールが追従窓の外 | 3.5、7.2 |
| 止まったのか人が動かしたのか分からない | `/joy_vel`、`/cmd_vel_nav` | twist_mux の優先 (joy > Nav2) | 1.4 |
| 指令は出ているのに車輪が動かない | `/robot_speed_cmd` と車輪位置 | 経路の問題ではない (EPOS / CAN 側) | 事例 C の末尾 |

## 7.2 設定変更のチェックリスト

Smac (Hybrid / Lattice) を使うとき:

- [ ] global の `inflation_radius` ≥ 外接半径 (footprint_padding 込み)。本機は 0.62 m 以上。満たさないと黙って全失敗する
- [ ] Hybrid の `minimum_turning_radius` と `angle_quantization_bins` の組が格子の下限を満たす
      (0.1 m 格子: 16 分割 → 0.26 m、32 → 0.51 m、64 → 1.02 m。下回ると無警告で引き上げられる)
- [ ] コース YAML の waypoint の向きが実際の進行方向と合っている (Smac はゴールの向きを満たしに行く)

footprint を触るとき:

- [ ] local の footprint は実寸のまま (縮めると実衝突を見逃す)
- [ ] 前端を実体より小さくしていないか (内接半径が縮み、点の planner が狭い帯に線を引く。第5章 5.2)
- [ ] global だけ縮めるなら、LiDAR に写りにくい物がコースにあるか確認する (事例 D)

global costmap の窓と格子を触るとき:

- [ ] 窓を縮めたら waypoint_runner の `--send-radius` と `--warn-spacing` も縮める
- [ ] コースの点間隔が窓の半径以下か (窓の外のゴールは即失敗)
- [ ] セル数 = (窓 ÷ 格子)²。200 m / 0.05 m は 16M セルで重すぎる

controller (RPP) を触るとき:

- [ ] `min_lookahead_dist` を下げると停止中の曲率が上がる (事例 A)。上げると前方停止距離と角の内回りが増える
- [ ] `max_lookahead_dist` ≥ `min_lookahead_dist`
- [ ] `rotate_to_heading_min_angle` を変えると、角での「止まって回る」動きの出方が変わる

## 7.3 診断コマンド

コンテナ (`rerobot_env`) 内のシェルで実行する。

```bash
ros2 param get /planner_server GridBased.plugin
```

```bash
ros2 param get /global_costmap/global_costmap footprint
```

```bash
ros2 param get /controller_server FollowPath.min_lookahead_dist
```

起動ログで Smac が動いているかは次の 1 行で分かる。

```
Created global planner plugin GridBased of type nav2_smac_planner::SmacPlannerHybrid
```

bag の rosout から停止の種類を数える道具は `bags/tools/2026-11-02_person_stop/rosout_stops.py` (controller の collision と planner の失敗の時系列)。
5 s ごとの集計は `bags/tools/2026-11-02_person_stop/timeline.py`、停止の瞬間の local costmap を画像にするのは `render_stop.py`。

## 7.4 bag 再生試験 — planner 単体

「この時刻、この姿勢で、別の planner 設定なら経路が出るか」を試す。
実機を使わず、本物の planner_server を bag の地図と点で動かす。

```
replay_plan/run_plan.sh の流れ
├── planner_server だけを起動 (試したい設定の yaml)
├── 再生用クライアントが bag の /map・keepout・tf_static を latched で配信
├── planner を configure、地図の読み込みを待つ
├── bag の /tf と LiDAR を --clock で再生
└── 指定時刻に、記録された経路の終点 (または GOAL_DIST m 先) をゴールに ComputePathToPose を投げる
```

道具: `bags/tools/2026-11-02_person_stop/replay_plan/`

| ファイル | 役割 |
|---|---|
| `run_plan.sh <params> <tag> <bag> <再生開始秒> <時刻...>` | 1 条件を回す |
| `plan_replay.py` | ゴールを投げて経路と計画時間を保存 (`result_<tag>.json`) |
| `eval_paths.py <bag> <result.json>` | 出だしの横ずれ・lethal までの距離・RPP 近似の閉ループ模擬 |
| `gcost_at.py <bag> <時刻...>` | その時刻の global costmap で自機の中心コストと縁のコスト |

注意:

- planner の再生は **global しか読まない**。local の設定 (格子・footprint) はこの試験では効かない。
- 再生用の yaml は `nav2_params.yaml` から planner_server と global_costmap の節を抜き出し、`use_sim_time: true` を足して作る。
- 窓を小さくした試験では、ゴールを `GOAL_DIST` で近くに置く (窓の外は即失敗)。
- 後始末の `pkill -f` はパターンに `[]` を入れる (`planner_[s]erver`)。入れないと docker exec 自身にマッチして自分を落とす。

## 7.5 bag 再生試験 — controller 単体

「この経路を渡したら、RPP は動き出すか」を試す。

```
replay/run_ab.sh の流れ
├── controller_server だけを起動 (local costmap も中で作られる)
├── bag の /tf・LiDAR・/odometry/filtered を再生
└── 指定時刻に FollowPath を投げ、/cmd_vel_nav と rosout の collision を記録
```

道具: `bags/tools/2026-11-02_person_stop/replay/`

| ファイル | 役割 |
|---|---|
| `run_ab.sh <params> <tag> <再生開始秒> <経路を取る秒> <投げる秒> <終了秒>` | 1 条件を回す |
| `follow_replay.py` | 記録された /plan、または `PATH_JSON` + `PATH_KEY` で planner 再生の結果を渡す |

制約: ロボットは bag の位置に固定なので、「動き出す指令を出すか」までしか分からない。
動いた後の振る舞いは閉ループ模擬 (`closed_loop_sim.py`、costmap 固定) で補う。どちらも近似なので、最終確認は実走である。

## 7.6 実走で記録しておくもの

`auto_run.launch.py` の記録トピックには、11-04 から次が入っている。

| トピック | 何が分かるか |
|---|---|
| `/lookahead_collision_arc` | RPP が衝突を調べた弧。止まった理由を再現計算なしで読める |
| `/is_rotating_to_heading` | RPP がその場旋回モードだったか |
| `/plan`、`/lookahead_point` | planner の経路と先読み点 |
| `/local_costmap/*`、`/global_costmap/*` | 2 つの地図 (global は 0.2 Hz 配信) |
| `/joy_vel`、`/cmd_vel_nav`、`/robot_speed_cmd` | 誰の指令が通ったか |

実走のメモには「どこで・何秒くらい・人が動かしたか」を残すと、bag の時刻と突き合わせやすい (11-04 はパッドのボタンでチェックポイントを残した)。

← [目次に戻る](00_index.md)
