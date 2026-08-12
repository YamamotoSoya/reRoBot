<!-- claude: slam_toolbox 読本 第4章 (2026-08-12)。上流デフォルトは apt 版 2.8.5 の
     mapper_params_online_async.yaml + バイナリ抽出の隠しパラメータによる。 -->

# 第4章 パラメータ大全 — 全パラメータの意味と reRoBot の現在値

reRoBot の実設定は `ros2_ws_slamtoolbox/src/rerobot_slamtoolbox/config/slam_toolbox.yaml`
(69 行)。上流デフォルトは apt 版 2.8.5 の `mapper_params_online_async.yaml`
(コンテナ内 `/opt/ros/jazzy/share/slam_toolbox/config/`)。本章は全パラメータを
第 3 章の機構別に棚に分け、**上流デフォルトと違う値には ★** を付ける。

```
パラメータの棚 (この章の地図)
├── 4.1 solver 系 ............... 触らない棚 (Ceres の解法選択)
├── 4.2 ROS I/F 系 .............. frame / topic / モード
├── 4.3 スキャン入口系 .......... queue・時間ゲート・TF 待ち  ← 事故多発地帯
├── 4.4 地図生成系 .............. 占有格子のレンダリング
├── 4.5 ノード採用・逐次マッチング系
├── 4.6 相関探索空間系 .......... (未指定 = 上流デフォルト)
├── 4.7 ペナルティ系 ............ (未指定 = 上流デフォルト)
├── 4.8 ループ閉じ込み系
├── 4.9 隠しパラメータ .......... yaml に載らないがコードに実在
└── 4.10 モード別追加 ........... localization / lifelong 専用
```

## 4.1 solver 系 (触らない棚)

| パラメータ | 上流 = reRoBot | 意味 |
|---|---|---|
| `solver_plugin` | `solver_plugins::CeresSolver` | バックエンドソルバ。実質 Ceres 一択 |
| `ceres_linear_solver` | `SPARSE_NORMAL_CHOLESKY` | 線形化方程式の解法 (疎グラフの定石) |
| `ceres_preconditioner` | `SCHUR_JACOBI` | 反復解法用前処理 (直接法では実質未使用) |
| `ceres_trust_strategy` | `LEVENBERG_MARQUARDT` | 非線形反復の信頼領域戦略 |
| `ceres_dogleg_type` | `TRADITIONAL_DOGLEG` | DOGLEG 選択時のみ有効 |
| `ceres_loss_function` | `None` | ロバスト損失。偽ループ対策に `HuberLoss` の選択肢 (→ 第3章 3.4) |

## 4.2 ROS I/F 系

| パラメータ | 上流 | reRoBot | 意味・指針 |
|---|---|---|---|
| `odom_frame` | `odom` | `odom` | オドメトリ座標系名 |
| `map_frame` | `map` | `map` | 地図座標系名 (TF map→odom の親) |
| `base_frame` | `base_footprint` | ★ `base_link` | ロボット基準 frame。**URDF に存在する frame を指すこと**。reRoBot の URDF には base_footprint が無いため必ず上書き (`slam_toolbox.yaml:2-3`「本件の本丸」)。間違えると TF 解決が永遠に失敗し scan が全 drop |
| `scan_topic` | `/scan` | `/scan` | 入力トピック |
| `mode` | `mapping` | `mapping` | `mapping` / `localization`。localization は保存済みグラフ上での位置推定 (ノード追加はするが刈り続ける) |
| `use_map_saver` | `true` | `true` | 旧 map_saver 互換サービスの有効化 |
| `map_file_name` / `map_start_pose` / `map_start_at_dock` | (コメントアウト) | 未指定 | 起動時に .posegraph を読み「続きから作図」する 3 点セット。pose 指定 or ドック位置のどちらかで初期位置を与える |
| `debug_logging` | `false` | `false` | 詳細ログ |
| `use_lifecycle_manager` | `false` (yaml 非掲載) | 未指定 | bond 連携 (→ 4.9) |

## 4.3 スキャン入口系 — 事故多発地帯

第 2 章 2.4 の MessageFilter と第 3 章 3.2 の時間ゲートに対応する。

| パラメータ | 上流 | reRoBot | 意味・指針 |
|---|---|---|---|
| `scan_queue_size` | **1** (yaml 非掲載) | ★ `10` | MessageFilter のキュー長。**scan レート > TF レートの構成では既定 1 は必ず溢れる**。目安は `ceil(TF周期/scan周期)` + オドメトリ瞬断の余裕。reRoBot は 40 Hz/20 Hz で計算上 2、余裕を見て 10 (= 250 ms 分) (`slam_toolbox.yaml:38-43`、事例A) |
| `transform_timeout` | `0.2` | ★ `0.5` | TF 到着を待つ最大秒。TF 配信周期 + 起動直後の buffer ウォームアップ分。短すぎると起動直後に drop の嵐、長すぎると障害時の発覚が遅れるだけ |
| `minimum_time_interval` | `0.5` | `0.5` | スキャン処理の最小間隔 (時間ゲート)。距離ゲートより先に律速し得る (>0.4 m/s で。→ 第3章 3.2) |
| `throttle_scans` | `1` | `1` | N 枚に 1 枚だけ見る間引き。1 = 間引きなし。CPU が苦しいときの最終手段 |
| `tf_buffer_duration` | `30.0` | `30.0` | TF buffer の保持秒数。bag 再生の一時停止などで過去の TF を引く余地 |

## 4.4 地図生成系 (占有格子レンダリング)

| パラメータ | 上流 | reRoBot | 意味・指針 |
|---|---|---|---|
| `map_update_interval` | `5.0` | ★ `2.0` | /map 再生成・配信の周期 [s]。**距離ではなく時間** (→ 第1章 1.2)。小さいほど RViz の見えは機敏だが、再生成コストは地図面積に比例して増える — 広域で重くなったら戻す (`slam_toolbox.yaml:31-33`) |
| `resolution` | `0.05` | `0.05` | セルサイズ [m]。Nav2 の costmap と揃えるのが無難 |
| `min_laser_range` | `0.0` | `0.0` | レンダリングに使う最短距離。ロボット筐体の自己反射が写るなら上げる |
| `max_laser_range` | `20.0` | ★ `30.0` | レンダリングに使う最長距離。UTM-30LX の実力 30 m に合わせた。センサ定格より大きくしても意味はない |
| `transform_publish_period` | `0.02` | `0.02` | TF map→odom の配信周期 [s] (= 50 Hz)。0 で配信停止 |
| `restamp_tf` | `false` (yaml 掲載) | 未指定 | map→odom TF の stamp を「計算時点」でなく「配信時点」に付け直す。基本 false のまま |
| `stack_size_to_use` | `40000000` | `40000000` | 大きな地図の serialize に必要なスタックサイズ [byte] (40 MB) |
| `enable_interactive_mode` | `true` | `true` | RViz からノードを手動移動・手動ループ閉じできるモード。グラフ保持コストが増えるので、確定運用では false も選択肢 |

## 4.5 ノード採用・逐次マッチング系

| パラメータ | 上流 | reRoBot | 意味・指針 |
|---|---|---|---|
| `use_scan_matching` | `true` | `true` | false にするとオドメトリだけでノードを置く (マッチング無効。デバッグ用: 事例C の開ループ積分と同じ状態になる) |
| `use_scan_barycenter` | `true` | `true` | ノード間距離の判定にスキャン点群の重心を使う (Karto 由来) |
| `minimum_travel_distance` | `0.5` | ★ `0.2` | ノード追加の距離ゲート [m]。旋回ゲートと **OR** |
| `minimum_travel_heading` | `0.5` | ★ `0.25` | ノード追加の旋回ゲート [rad] (≈14°)。低速機で地図更新の体感を上げるため半減 (git 82edfef, 2026-08-01) |
| `check_min_dist_and_heading_precisely` | `false` (yaml 掲載) | 未指定 | ゲート判定を厳密計算にする。通常不要 |
| `scan_buffer_size` | `10` | `10` | 逐次マッチングの参照に使う直近スキャン数 (= 参照地図の厚み)。増やすと安定するが計算増 |
| `scan_buffer_maximum_scan_distance` | `10.0` | `10.0` | 参照バッファに残すスキャンの最大距離 [m] — 遠くまで届いたスキャンだけで参照地図が薄くなるのを防ぐ |
| `link_match_minimum_response_fine` | `0.1` | `0.1` | 逐次エッジを張るための最小 response。上げすぎるとエッジが張れずグラフが千切れる |
| `link_scan_maximum_distance` | `1.5` | `1.5` | 逐次エッジを張る最大ノード間距離 [m] |

## 4.6 相関探索空間系 — reRoBot は**全て未指定** (上流デフォルト)

第 3 章 3.1 の探索窓。未指定なのは「オドメトリと FOV を直すまで触らない」方針 (事例B)。

| パラメータ | 上流デフォルト | 意味・指針 |
|---|---|---|
| `correlation_search_space_dimension` | `0.5` | 逐次マッチングの並進探索窓 [m]。**オドメトリの 1 ノード区間あたり誤差がこれを超えると真値が窓外**になり必ず誤マッチ。オドメトリが荒い機体では拡大 (計算量は面積で増加) |
| `correlation_search_space_resolution` | `0.01` | 探索格子の刻み [m] |
| `correlation_search_space_smear_deviation` | `0.1` | 参照地図のガウスぼかし幅 [m]。大きい = 応答が滑らか (収束しやすいが甘い)、小さい = 尖る (精密だが取り逃しやすい) |

## 4.7 ペナルティ系 — reRoBot は**全て未指定** (上流デフォルト)

第 3 章 3.1 の「prior からの逸脱減点」。**オドメトリへの信頼度のダイヤル**である。

| パラメータ | 上流デフォルト | 意味・指針 |
|---|---|---|
| `distance_variance_penalty` | `0.5` | 並進逸脱の分散想定。**小さいほどオドメトリを信じる**。オドメトリが正確な機体は下げると縮退環境で強くなる |
| `angle_variance_penalty` | `1.0` | 回転逸脱の分散想定 (同上) |
| `fine_search_angle_offset` | `0.00349` | 精密角度探索の範囲 [rad] (±0.2°) |
| `coarse_search_angle_offset` | `0.349` | 粗角度探索の範囲 [rad] (±20°)。旋回中の取り逃しがあるなら拡大 |
| `coarse_angle_resolution` | `0.0349` | 粗角度探索の刻み [rad] (2°) |
| `minimum_angle_penalty` | `0.9` | 角度減点の下限 (response が 0 に潰れないため) |
| `minimum_distance_penalty` | `0.5` | 並進減点の下限 |
| `use_response_expansion` | `true` | 全候補が低 response のとき角度窓を自動拡大する救済 |
| `min_pass_through` | `2` | 占有格子: セルを free と数える最小通過レイ数 (→ 第1章 1.5) |
| `occupancy_threshold` | `0.1` | 占有格子: occupied と判定する反射率閾値 |

## 4.8 ループ閉じ込み系

第 3 章 3.3 の多段検査。reRoBot はすべて上流デフォルト値のまま明示記載している。

| パラメータ | 上流 = reRoBot | 意味・指針 |
|---|---|---|
| `do_loop_closing` | `true` | ループ閉じ込みの有効化。切ると純粋オドメトリ+逐次マッチングになる (切り分け用) |
| `loop_search_maximum_distance` | `3.0` | 候補を探す半径 [m] (推定位置基準)。**周回のドリフトがこれを超えると候補にすら挙がらない** — 大きい建物を周回するなら拡大が最初の一手 |
| `loop_match_minimum_chain_size` | `10` | 候補が連続ノード列 (チェーン) として持つべき最小長。下げると閉じやすいが偽ループも増える |
| `loop_match_maximum_variance_coarse` | `3.0` | 粗マッチ解の最大分散 — 「自信のない解」を弾く |
| `loop_match_minimum_response_coarse` | `0.35` | 粗マッチの最小一致度 |
| `loop_match_minimum_response_fine` | `0.45` | 精マッチの最小一致度 |
| `loop_search_space_dimension` | `8.0` | ループ用探索窓 [m] (逐次用 0.5 m より広い — ドリフトを跨いで照合するため) |
| `loop_search_space_resolution` | `0.05` | 同・刻み [m] |
| `loop_search_space_smear_deviation` | `0.03` | 同・ぼかし幅 [m] |

## 4.9 隠しパラメータ — yaml に載らないがコードに実在

上流のどのサンプル yaml にも書かれていないが、バイナリに宣言が実在するもの
(2.8.5 の `libtoolbox_common.so` から抽出):

| パラメータ | 既定 | 意味 |
|---|---|---|
| `scan_queue_size` | `1` | → 4.3。**yaml 非掲載なのに全ロボットで要調整**という最悪の組み合わせ。事例A の核心 |
| `use_lifecycle_manager` | `false` | nav2_lifecycle_manager 管理下に置くとき true (bond 連携)。⚠️ slam_toolbox は bond 非対応のため manager 側に `bond_timeout: 0.0` が必須 — 忘れると activate 直後に「Have not received a response from bond」で kill される (`docs/issue/2026-07-07_monthly_2026_6_todo_triage.md` T4) |
| `position_covariance_scale` / `yaw_covariance_scale` | `1.0` | `/pose` 出力の共分散スケール。EKF 等に `/pose` を食わせるときの信頼度調整用 |
| `paused_new_measurements` / `paused_processing` | `false` | 一時停止フラグ (サービス `pause_new_measurements` の実体) |

## 4.10 モード別追加パラメータ

reRoBot 未使用だが、モードを変えるときに現れる:

```
モード別の差分 (上流サンプル yaml 比較)
├── localization (mapper_params_localization.yaml)
│   ├── scan_buffer_size: 3 / loop_match_minimum_chain_size: 3 (軽量化方向)
│   ├── 購読トピック initialpose (RViz の 2D Pose Estimate で初期位置を与える)
│   └── ⚠️ 上流サンプルは mode: mapping のまま — コピーして使うと localization にならない罠
└── lifelong (mapper_params_lifelong.yaml) — 実験的
    ├── lifelong_search_use_tree: false      ... 候補探索に木構造を使う
    ├── lifelong_minimum_score: 0.1          ... ノード維持の最低スコア
    ├── lifelong_iou_match: 0.85             ... スキャン重なり (IoU) の一致判定
    ├── lifelong_node_removal_score: 0.04    ... これを下回るノードを削除
    ├── lifelong_overlap_score_scale: 0.06 / lifelong_constraint_multiplier: 0.08
    └── lifelong_nearby_penalty: 0.001 / lifelong_candidates_scale: 0.03
```

## 4.11 reRoBot 差分の総括

上流デフォルトから変えたのは **6 個だけ**。全て理由つき:

| パラメータ | 上流 → reRoBot | 理由 | いつ |
|---|---|---|---|
| `base_frame` | `base_footprint` → `base_link` | URDF に base_footprint が無い | 初版 (05-05) |
| `max_laser_range` | 20.0 → 30.0 | UTM-30LX の定格 | 初版 (05-05) |
| `scan_queue_size` | 1 → 10 | 40 Hz scan × 20 Hz TF で必然的に溢れる (事例A) | 05-28 |
| `transform_timeout` | 0.2 → 0.5 | TF 周期 50 ms + 起動直後の余裕 (事例A) | 05-28 |
| `map_update_interval` | 5.0 → 2.0 | 地図更新の体感改善 | 08-01 |
| `minimum_travel_distance` / `_heading` | 0.5/0.5 → 0.2/0.25 | 低速機でノードが疎すぎた | 08-01 |

**意図的に触っていない領域** = 相関探索空間 (4.6)・ペナルティ (4.7)・ループ閉じ込み (4.8)。
「オドメトリ精度 (tread_width)・LiDAR FOV (±90° 制限) を直す前にここを触ると原因を隠す」
という切り分け順序が理由 (事例B)。チューニングに手を出す前に必ず
[第6章 事例B](06_case_studies.md) を読むこと。

→ [第5章 reRoBot での適用](05_rerobot_setup.md)
