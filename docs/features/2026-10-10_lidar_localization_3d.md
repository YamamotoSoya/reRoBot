<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# 3D 自己位置推定 (lidar_localization_ros2) の導入 — nav 構成 `nav_lidar3d`

- 日付: 2026-10-10
- 対象パッケージ:
  - `lidar_localization_ros2` (`ros2_ws_main/src/localization/lidar_localization_ros2`, **submodule、タグ v1.3.0 固定**、無改変)
  - `ndt_omp_ros2` (`ros2_ws_main/src/localization/ndt_omp_ros2`, **submodule、humble ブランチ** — 上流 Jazzy CI と同じ組み合わせ)
  - `rerobot_bringup` (`ros2_ws_main/src/bringup/rerobot_bringup`) — 部品・nav 構成・橋ノード・パラメータ
- 対象 ROS: ROS 2 Jazzy (main コンテナ `rerobot_env`)
- 関連文書:
  - 読本 [第5章 3D ローカライザへの道](../text/map3d_to_nav2/05_localization_3d.md) / [第8章 手法カタログ](../text/map3d_to_nav2/08_method_catalog.md) (本機能はその「系統 III — 定位だけ 3D のまま残す」の実装)
  - [2026-08-17 GLIM 3D 地図 → Nav2 接続](2026-08-17_glim_map_to_nav2.md) (§2.5 が本機能の旧設計メモ)
  - [2026-10-02 2D 地図の閾値・unknown・keepout](2026-10-02_2dmap_threshold_unknown_keepout.md) (対になる 2D 地図の作り方)

## 1. 目的・概要

AMCL / emcl2 は、3D 点群を高さ帯 1 本の 2D スキャン (/rfans/scan) に潰してから自己位置を解く。屋外 (つくば) では次の 3 つの弱点がある。

- 開けた場所で断面が痩せる
- 地図と実スキャンで高さ帯が食い違う
- 植生の変化に弱い

本機能は、自己位置推定だけを 3D のまま解く。R-Fans の点群を GLIM の 3D 点群地図へ NDT で直接照合し、その結果を Nav2 にとっての `map→odom` として供給する。Nav2 側 (経路計画・costmap・keepout) は 2D 地図のまま変えない。つくば上位チームの定番である「定位 3D + 計画 2D」の分離型にあたる。

一番の難所は、3D の推定を 2D の Nav2 に渡す継ぎ目である。3D 地図上のロボットには高さ z があり、坂や GLIM の z ドリフトで数 m 変わる。これを Nav2 にそのまま渡すと、global costmap が障害物を捨ててしまう。そこで、3D 地図の座標系 `map_3d` と Nav2 の平らな座標系 `map` を分け、その間で z だけを打ち消す橋ノード `map_level_bridge.py` を挟んだ (§2.1、本文書の中心)。

実装した機能:
- 自己位置推定部品 `parts/localization/loc_lidar3d.launch.py` (仮置きを置換): lidar_localization_ros2 本体 + lifecycle 起動 + 橋ノード
- 橋ノード `scripts/map_level_bridge.py`: TF `map→map_3d` で z を打ち消す + `/initialpose` (map) を `/initialpose_3d` (map_3d、z は GLIM 軌跡から) に中継する
- nav 構成 `nav/nav_lidar3d.launch.py` (仮置きを置換): `map_name` 1 つで 2D 地図と 3D 地図を対にして起動する
- 本機用の差分パラメータ `config/localization/lidar_localization.yaml` (上流プリセット `nav2_ndt_urban.yaml` の上に重ねる)
- global costmap の `min_obstacle_height: -0.3` (§2.2。AMCL 構成にも効く潜在不具合の修正)
- main イメージの依存追加 (`libpcl-dev`, `ros-jazzy-pcl-conversions`)

スコープ外 (意図的に未対応):
- **3D 地図・対になる 2D 地図の作成**: ユーザが後で作る (手順は §4.1)。そのため、地図を使った動作検証 (bag 再生・実機) は未実施
- **IMU の利用** (`use_imu_preintegration`): 既定 OFF。配線が通ってから ON / OFF で A/B する (つくば 2456 Abudori は安価 IMU の横移動バイアスで無効化した)
- **見失い時の自動復帰**: `/reinitialization_requested` の監視と再初期化の自動化 (上流の recovery supervisor 等) は未接続。当面は人が 2D Pose Estimate し直す
- **パラメータのチューニング** (ndt_resolution 2.0 / 5.0、地図の間引き、local_map_radius)。地図ができてから行う
- **RViz 表示の追加** (`/initial_map`, `/pcl_pose`, `/path`): nav2.rviz は未変更
- **Beluga (粒子型) との比較**: 2 番手候補として記録のみ (§2.4)
- **main イメージの再ビルド**: Dockerfile は更新済み。いまのコンテナには同じパッケージを apt で手入れしてある (コンテナを作り直すと消えるので、次のイメージ再ビルドで恒久化される)

## 2. 設計の勘所

> 査読: §2.1〜2.5 は反証専任の critic subagent の査読 (2026-10-10) を受けて修正済み。初版にあった 2 つの実装不具合は、critic の指摘で見つかった。
> (1) rclpy の static broadcaster が更新を無視する。(2) `use_odom` が seed に使われていない。
> 棄却できなかった代替案は各節の「却下した代替案」に残してある。

### 2.1 3D の世界 → 2D の世界: `map_3d` と `map` を分け、z だけを打ち消す (最重要)

#### 2.1.1 何が問題か — 3D の推定には「高さ」が付いてくる

lidar_localization_ros2 は、3D 地図の上でロボットの 6 自由度 (x, y, z, roll, pitch, yaw) を解く。Nav2 が欲しいのは、平面上の 3 自由度 (x, y, yaw) の `map→odom` だけである。

```
3D 推定の 6 自由度 → Nav2 に渡すときの扱い
├── x, y, yaw ...... そのまま渡したい (これが自己位置推定の本体)
├── roll, pitch .... 渡すと map 上で base_link が傾く → 消したい
│     └── 上流の level_map_to_odom: true で消える (§2.1.2)
└── z .............. 渡すと map 上で base_link が浮く / 沈む → 消したい
      └── 上流には消す手段が無い → 橋ノードで消す (§2.1.3)
```

z が「3D 地図上のロボットの高さ」としてどれくらいになるかの目安:

- 実測 (GLIM 軌跡 `bags/glim/2026-10-09_1643_5goukan_chal_dump/g1_imu_strong_filtered/traj_lidar.txt` の先頭): 出発点の LiDAR z = +0.06 m。URDF の LiDAR 取付高 0.794 m を引くと、**出発点の base_link は z ≈ −0.73 m**。GLIM の world 原点がセンサ付近にあるため、平地でも 0 にならない
- これに、坂 (つくばの公園区間) や GLIM の z ドリフト (09-10 issue で 1 周あたり数 m) が上乗せされる

#### 2.1.2 上流の `level_map_to_odom` がやること・やらないこと

上流 `include/lidar_localization/pose_publish_policy.hpp` の `composeMapToOdomTransform(..., level=true)` (ソースで確認):

```
入力:  T_map3d_base  = 照合結果 (6 自由度)
       T_odom_base   = EKF の odom→base_link (two_d_mode: true なので z = roll = pitch = 0)
出力:  T_map3d_odom の回転    = T_map3d_base * T_odom_base⁻¹ の yaw だけ   (roll, pitch → 0)
       T_map3d_odom の並進    = p(map3d_base) − R_yaw · p(odom_base)
                                                └ z 成分は 0 なので、並進 z = p(map3d_base).z
```

つまり **`map_3d→odom` の z は、3D 地図上の base_link の高さそのもの**になり、消えずに残る。

#### 2.1.3 z が残ると何が壊れるか — global costmap の高さフィルタ

Nav2 で `map` 座標の z を使う部品を洗い出した (critic 査読で確認した範囲):

| 部品 | `map` 座標の z を使うか | z が残ったときの影響 |
|---|---|---|
| **global costmap / obstacle_layer** | **使う** — 観測点を global_frame (`map`) に変換してから `min_obstacle_height ≤ z ≤ max_obstacle_height` で採否を決める (Nav2 ObservationBuffer の挙動。Nav2 ソースの記憶による、本機では未実測) | **障害物が捨てられる** (下で詳述) |
| static_layer / keepout_filter / navfn / RPP / goal checker | 使わない (x, y のみ) | なし |
| local costmap | 使わない (global_frame は `odom`) | なし |
| RViz | 表示だけ | 2D 地図とロボットが上下にずれて見える |

捨てられる仕組み:

- `/rfans/scan` は `target_frame: base_link` で作っているので、点の z は base_link 基準で **ちょうど 0** にある。`/urg_front/scan` は laser の高さ +0.094 m にある
- `map` から見た base_link の z を z_b とすると、点の `map` 座標の z は「0 + z_b」(rfans) と「0.094 + z_b」(urg) になる
- 本機の global costmap は `min_obstacle_height` を書いておらず既定 0.0、`max_obstacle_height` は 2.0 だった
  - z_b = −0.73 (出発点) → rfans の点は −0.73 < 0.0 で**全部捨てられ**、urg も −0.64 で全部捨てられる
  - z_b = +2.5 (坂の上 / ドリフト) → rfans は 2.5 > 2.0 で全部捨てられる
- 補足: AMCL 構成で問題が出なかったのは、map→odom の z が「ちょうど 0.0」で、rfans の点が境界 (≥ 0.0) にぴったり乗っていたからにすぎない (critic 指摘)。→ §2.2 で下限を下げた

#### 2.1.4 解決 — 座標系を 2 枚にして、間で z を打ち消す

```
TF の木 (nav_lidar3d 起動時)

map            ← Nav2・2D 地図・RViz の基準 (平らな世界、ロボットの z ≈ 0)
 │  TF map→map_3d = 並進 (0, 0, −z)・回転なし            [map_level_bridge.py、/tf_static]
 ▼
map_3d         ← 3D 地図 (map.pcd) の座標系。ロボットの高さ z が乗っている
 │  TF map_3d→odom (yaw のみの回転 + 並進 x, y, z)       [lidar_localization、/tf]
 ▼
odom
 │  TF odom→base_link (2D)                               [EKF (robot_localization)]
 ▼
base_link ─┬─ rfans / laser / imu_link                   [robot_state_publisher (URDF)]
```

- `map` から見た odom: `T_map_odom = T(0,0,−z) · T_map3d_odom` → 並進 z = z − z = **0**、x, y, yaw はそのまま
- どの frame も親は 1 つだけ。AMCL / emcl2 は include しないので、`map→odom` を出すのは実質この 2 段だけ
- `map` と `map_3d` の x, y, yaw が一致するのは、2D 地図と 3D 地図を**同じ GLIM dump から同じ変換 (T_world_origin) で**作るから (§2.6)

#### 2.1.5 橋ノード `map_level_bridge.py` の中身

```
map_level_bridge の仕事
├── ① z の打ち消し (20 Hz タイマ)
│     ├── TF map_3d→odom の最新を引き、並進 z を読む
│     ├── 前回出した値から 1 cm (z_update_threshold) 以上変わっていたら
│     │     TF map→map_3d = (0, 0, −z) を /tf_static に出し直す
│     ├── 起動直後 (推定がまだ無い間) も、仮の z で map→map_3d を出しておく
│     │     → map が TF の木から孤立しない (RViz・Nav2 が map を引けなくならない)
│     │     仮の z = GLIM 軌跡の始点 (= 地図を撮り始めた場所) の base_link 高さ。軌跡が無ければ initial_z
│     └── 初期位置を置き直した直後は、それより古い stamp の map_3d→odom を無視する
│           (旧トラックの z で戻されないように)
└── ② /initialpose の中継
      ├── RViz の 2D Pose Estimate は「map 座標・z = 0」で /initialpose を出す
      ├── 本体は frame_id が global_frame_id (map_3d) と違う初期位置を**捨てる**
      │     (上流 src/component_subscribers.cpp initialPoseReceived の admission 判定)
      ├── frame_id を map_3d に書き換え、z に「その場所の 3D 地図上の高さ」を入れて /initialpose_3d に出す
      │     ├── 第 1 候補: traj_lidar.txt (GLIM 軌跡、map_3d 座標の LiDAR 姿勢) で水平最寄りの点の
      │     │     LiDAR z − LiDAR 取付高 (TF base_link→rfans から取得)
      │     │     → 坂の上・下で置き直しても、その場所の高さが入る
      │     └── 第 2 候補 (軌跡が無い / 最寄り点が 5 m より遠い): 現在の打ち消し量 (直近の推定 or initial_z)
      └── 同時に ① の打ち消し量もその z に合わせる (推定が出る前から map 上の z ≈ 0 にする)
```

なぜ /tf_static で出すか (判断):

- z は坂に沿ってゆっくりとしか変わらず、時刻つきで補間する必要がない
- static は受信側 (tf2 BufferCore の static キャッシュ) で単一値として上書きされ、どの時刻で引いても最新値が返る。過去の時刻で引いても z の誤差は閾値 (1 cm) 以内で無害
- ⚠️ ただし「static にしたから時刻の外挿エラーが消える」わけではない。外挿は鎖の中の動的 TF (`map_3d→odom`) で決まる (critic 指摘)。照合が棄却された区間も、上流が凍結した `map_3d→odom` を再 stamp して出し続ける (`component_pose_publisher.cpp` の frozen 再送) ので、TF は途切れない

⚠️ 実装上の落とし穴 (実測・ソースで確認):

- rclpy の `StaticTransformBroadcaster.sendTransform` は、同じ `child_frame_id` の **2 回目以降の値を捨てて、最初の値を再送する** (コンテナ内 `/opt/ros/jazzy/lib/python3.12/site-packages/tf2_ros/static_transform_broadcaster.py`、tf2_ros_py 0.36.22。C++ 版は置き換える)
- 初版はこれを使っていたため、**z が initial_z のまま固定**されていた (critic 指摘)
- → `/tf_static` (transient_local, depth 1) へ自前の publisher で `TFMessage` を毎回作り直して出す方式に変えた

閾値 1 cm と、§2.2 の下限 −0.3 m の関係:

- 打ち消した後の残差 (map から見たロボットの z) は ±1 cm の範囲で正負に揺れる
- 下限 0.0 のままだと、負の側に振れた瞬間に rfans の点が消える (初版の閾値 2 cm でも同じ問題があった — critic 指摘)
- 下限を −0.3 m に下げたので、残差の符号は問題にならない

却下した代替案:

- **橋を使わず、global costmap の高さ範囲を広げるだけ** (`min/max_obstacle_height` を ±10 m 等にして `global_frame_id: map` のまま運用): **棄却できていない有力な代替案** (critic)。入力は 2 本とも高さ切り出し済みの LaserScan なので、範囲を広げても余計な点は入らない。不採用にした理由は次の 3 つ (判断):
  - (a) RViz で 2D 地図とロボットが数 m 上下にずれ、運用中の目視確認がしにくい
  - (b) 結局 `/initialpose` の z を地図の高さにする処理は要る (RViz は z = 0 で出す)
  - (c) map の z = 0 という前提を Nav2 の他の部品や将来の追加 (collision_monitor 等) に持ち込まなくて済む
  - 橋が不調なときの切り戻し先として有効
- **上流を改造して z も 0 にする**: submodule 無改変の方針に反する。上流への PR は可能
- **Mode A (map→base_link を直接出す) + 別名 frame で受ける**: base_link に親が 2 つでき (odom と map_3d)、TF の木が壊れる
- **動的 TF (/tf) で map→map_3d を出す**: stamp を map_3d→odom と揃える管理が要り、外挿・補間の失敗点が増える。z がゆっくりしか変わらない以上、利点が無い

### 2.2 global costmap の `min_obstacle_height` を −0.3 m に下げた

- 判断: §2.1.3 のとおり、`/rfans/scan` の点は base_link 基準で z = 0 ちょうどにあり、既定の下限 0.0 と境界で接していた
  - 3D 構成では、橋の残差 (±1 cm) で負に振れたときに点が消える
  - AMCL / emcl2 構成でも、浮動小数の誤差次第で消えうる潜在不具合だった
- 下限 −0.3 m の根拠: 入力は `rfans_scan.launch.py` (地上 0.3〜1.5 m 帯) と urg の 2D スキャンで、高さはすでに切り出してある。下限を下げても地面などの余計な点は入らない。−0.3 は「残差 + 多少の z 誤差」を吸収する余裕で、精密な値ではない
- 影響範囲: `nav2_params.yaml` を共有する nav 構成すべて (nav_amcl / nav_amcl_scanall / nav_emcl2 / nav_lidar3d) の global costmap。local costmap (odom フレーム、z = 0 ちょうど) は変更していない
- 却下した代替案: nav_lidar3d 専用の nav2_params を作る — ファイルの重複が増える割に、他構成でも直すべき不具合だったので共通側で直した

### 2.3 照合の初期値 (seed) は EKF の TF から取る (`use_odom_tf_prediction: true`)

上流の seed の優先順 (`include/lidar_localization/registration_seed_policy.hpp`、ソースで確認):

```
seed の優先順 (上ほど優先、条件を満たした最初の 1 つが使われる)
├── 1. odom_tf ............ use_odom_tf_prediction: true かつ、照合が 1 度通って map→odom の凍結値がある
├── 2. imu ................ use_imu_preintegration: true かつ準備完了
├── 3. gtsam .............. use_gtsam_smoother
├── 4. twist_ekf .......... use_twist_ekf
├── 5. twist_prediction ... use_twist_prediction (プリセットは true、/twist 入力)
├── 6. previous_delta ..... predict_pose_from_previous_delta (プリセットは true) = 前回の動きで等速外挿
└── 7. current_pose ....... 最後の姿勢そのまま
```

- 初版は `use_odom: true` (`/odom` の twist 積分) にしていた。しかし use_odom はこの優先順に入っておらず、**6 の等速外挿に負けて実質使われていなかった** (critic 指摘、ソースで確認)
- 判断: 最優先の 1 を使う。seed = 「最後に照合が通ったときの `map_3d→odom`」×「いまの EKF の `odom→base_link`」で、AMCL がオドメトリで粒子を動かすのと同じ考え方になる。照合が痩せる区間 (つくばの公園中央 — レポート 2504 / 2352 等の主要な破綻要因) で、車輪 + IMU の EKF が姿勢を持ちこたえる
- 起動直後 (照合がまだ 1 度も通っていない) は 6・7 にフォールバックし、初期位置 (/initialpose_3d) から始まる
- 注意 (critic): `level_map_to_odom` を使うので、seed の roll / pitch は毎回 0 になる。GLIM 地図に局所的な傾きがある区間 (09-23 に submap ブロック傾斜 8〜11° の事例) では、NDT が毎回傾きを解き直すことになる。bag 再生で確かめる
- `use_twist_prediction` は false にして、seed の入口を 1 本に絞った (挙動を追いやすくするため)

### 2.4 方式の選定 — lidar_localization_ros2 (追跡型 NDT)

調査の経緯 (2026-10-10。Web 調査 + つくばチャレンジ 2023〜25 レポート索引):

| 候補 | 判定 | 理由 |
|---|---|---|
| **lidar_localization_ros2** | **採用** | Jazzy が主対象 (上流 CI に Jazzy job あり)、最終 push 10-09 で活発。R-Fans の点ごとの `"time"` (float32 相対秒) を読める (`include/lidar_localization/point_field_read.hpp`)。map→odom モードあり。PCD / PLY を直接読める。周囲だけの切り出し (`enable_local_map_crop`)。見失いの申告 (`/reinitialization_requested`) あり。つくばでは完走 2564 KUAMS、ほぼ同構成で安定 2582 SICKONICA |
| Beluga `NdtAmclNode3D` | 2 番手 | 粒子型 (SE3) で誤収束に強く、Jazzy の apt あり・企業保守。ただし 3D ノードは新しく、つくばでの実例が無い。GLIM 地図を HDF5 形式の NDT 地図に変換する手順は未確認 |
| hdl_localization | 不採用 | つくばでの完走実績は多い (2551 Kerberos 等) が、本家は ROS 1 |
| mcl_3dl | 不採用 | ROS 1 のみ。つくばでは完走 1 件に対し破綻が多い (2504 公園中央 等) |
| Autoware ndt_scan_matcher | 不採用 | つくばでの実績は最多だが、Autoware のフレーム / メッセージ規約ごと持ち込むコストが大きい |
| MOLA (LO localization / PF) | 不採用 | 地図が独自 `.mm` 形式で、GLIM 地図をそのまま使えない |
| GLIM 本体 | 対象外 | localization モードが無い (koide3/glim issue #158 が open) |

- 弱点 (判断): 追跡型なので、誤収束から自力では戻れない。つくばで追跡型を完走させたチームは、破綻の検知・棄却・復帰を自前で足していた (2431/2589 AMSL はマハラノビス距離のゲート、2419 石川研は移動量差で検知、2327 イエスマンは見失い判定 + 乱数で復帰)。本実装では上流内蔵のスコア棄却と `/reinitialization_requested` に頼り、自動復帰はスコープ外 (§1)
- リスク (判断): 保守者は実質 1 人で、変化が非常に速い (1 日に複数 PR)。**v1.3.0 に固定**する。上流の検証範囲は「Humble + 公開 bag 1 本 (75 m)、正解の初期姿勢を与えた条件」で、実機・長時間・自動初期化までは作者自身も保証していない

### 2.5 lifecycle は上流付属の `start_lifecycle_node.py` で起動する

- 実測 (ソース): v1.3.0 は `use_bond_{false}` (`lidar_localization_component.hpp`) をパラメータから有効にする経路が無く、nav2 の bond (生存監視) を張らない
- 判断: 上流付属の `start_lifecycle_node.py` を採用した。理由は 2 つ
  - bond を張らないこと
  - configure (PCD ロード込み) の待ち時間を長く取れること (`--timeout`、本機は `startup_timeout` 既定 120 s。大きい地図のロードに備える)
- 却下はしていない代替案: nav2 `lifecycle_manager` も `bond_timeout: 0.0` にすれば bond なしで動かせる (critic 指摘)。こちらにすると RViz の Navigation 2 パネルが `lifecycle_manager_localization` の状態を表示できる。ただし configure の待ち時間の扱いは Jazzy で未確認。いまの構成ではパネルの localization 表示は unknown になる (走行には影響しない)

### 2.6 2D 地図と 3D 地図は同じ dump から作り、`map_name` 1 つで対にする

- 実測 (critic がソースで確認): `tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py` は、2D 地図 (pgm) も `--export_pcd` の PCD も、同じ `w = pts·Rᵀ + t` (submap の T_world_origin) から作る。画素の割当と origin は、W・H が偶数なら map_server の規約と一致する (auto は 64 の倍数なので偶数)
- → 同じ dump・同じ実行から出せば、`map` と `map_3d` の x, y, yaw は一致する
- 判断: 2 枚の対応を人の注意だけに頼らないよう、`nav_lidar3d` は `map_name` 1 つから `maps/2d/glim/<map_name>/` と `maps/3d/<map_name>/` の両方のパスを組み立てる
- 注意: 同じ dump 由来であることは、ディレクトリ名でしか保証されない。W・H を奇数で手指定すると半画素 (2.5 cm @ 0.05 m) ずれる

### 2.7 パラメータの初期値 (根拠と A/B 候補)

| パラメータ | 値 | 根拠 | A/B 候補 |
|---|---|---|---|
| `ndt_resolution` | 2.0 (プリセット 1.0) | 16 ラインは点が疎で、マスが細かいと 1 マスあたりの点が足りない | 5.0 (2456 Abudori・2441 成蹊が 16 ライン相当で採用) |
| `use_imu_preintegration` | false | 配線確認が先。安価 IMU で悪化した前例 (2456) | true |
| `imu_preintegration_use_base_frame_transform` | true | BNO086 は上下逆 + yaw −93° 取付。false だと IMU 値をセンサ座標のまま使う | (IMU ON 時は必須) |
| `point_timestamp_unit` | seconds | R-Fans の `"time"` は float32 の相対秒 | auto |
| `scan_period` / `scan_min_range` | 0.1 / 1.0 | R-Fans 10 Hz / 車体自身の点を除く (プリセットと同値、明示のため) | — |
| その他 | プリセット `nav2_ndt_urban.yaml` | `registration_method: NDT_OMP`, `score_threshold: 6.0`, `local_map_radius: 150.0` 等 | 地図ができてから |

## 3. データフロー

```
[R-Fans driver] ─ /rfans_driver/rfans_points (sensor_msgs/PointCloud2, frame rfans, 10 Hz, 点ごと "time") ─┐
[BNO086]        ─ /imu/data (sensor_msgs/Imu, frame imu_link, 200 Hz) ─── (use_imu_preintegration 時のみ) ┤
[EKF]           ─ TF odom→base_link ────────────────────────────── (seed: use_odom_tf_prediction) ──────┤
[RViz 2D Pose Estimate] ─ /initialpose (PoseWithCovarianceStamped, frame map, z=0)                      │
        │                                                                                               │
        ▼                                                                                               ▼
[map_level_bridge] ─ /initialpose_3d (frame map_3d, z=軌跡の高さ) ─▶ [lidar_localization] ◀─ map.pcd (map_path)
        ▲   ▲                                                         │ (lifecycle: start_lifecycle_node.py)
        │   └ traj_lidar.txt (traj_path)                              │
        │                                                             ├─ TF map_3d→odom (/tf, level 済み)
        └──────── TF map_3d→odom の z を読む ◀─────────────────────────┤
        │                                                             ├─ /pcl_pose (PoseWithCovarianceStamped, map_3d)
        └─ TF map→map_3d = (0,0,−z) (/tf_static)                      ├─ /path, /initial_map
                │                                                     ├─ /alignment_status (照合の健全性)
                ▼                                                     └─ /reinitialization_requested (見失い申告)
[Nav2: map_server (/map 2D, frame map) + keepout + planner/controller + global costmap (map) / local costmap (odom)]
```

remap (loc_lidar3d.launch.py): `/cloud→/rfans_driver/rfans_points`、`/odom→/odometry/filtered` (use_odom: false なので既定構成では未使用)、`/imu→/imu/data`、`/initialpose→/initialpose_3d`、`/map→/lidar_localization/map_cloud_in`。最後の remap の理由: 本体は点群地図を topic で受ける入口 `/map` を PointCloud2 型で購読している (use_pcd_map: true では未使用)。Nav2 の `/map` (OccupancyGrid) と同名・別型になるため逃がした (統合試験で発見)。上流付属の launch は include しない。付属 launch は LiDAR / IMU の static TF を自前で出すため、robot_state_publisher と二重配信になる。

## 4. 使い方

### 4.1 地図の準備 (未実行 — ユーザが後で作る)

コマンドの正本は **運用手引き [第9章 §9.7](../manual/09_map2d_compression.md)** (2026-10-10 に一本化。ここに同じコマンドを複製しない)。要点:

- 同じ dump から 2D (dump 版 `raw_dump/<帯>/`・traj 版 `raw_traj/<帯>/`・`nav2/`・`keep_out/`) と 3D (`maps/3d/<NAME>/map.pcd`、glim_dump_to_2dmap の `--export_pcd`) を作り、`traj_lidar.txt` を 3D 地図の隣にコピーする
- 高さは**両ツールとも base_link 基準・同じ帯** (2026-10-02 の決定、第9章 §9.6.1)。経路計画用 `/map` の帯は bringup の `scan_min_height` / `scan_max_height` と揃える
- 期待する配置: `maps/2d/glim/<map_name>/{raw_dump/<帯>/, raw_traj/<帯>/, nav2/map.yaml, keep_out/keep_out.yaml}` と `maps/3d/<map_name>/{map.pcd, traj_lidar.txt}`
- PCD の大きさ: 08-31 5号館 dump で 406 万点 (≈ 49 MB)。本体は周囲だけ切り出して照合するので動く見込みだが、ロード時間・メモリは最初の bag 再生で確認する
- 代替: `offline_viewer <dump> --export_path .../map.ply` でも可 (本体は .ply を直接読む)。その場合は `map_path:=.../map.ply` を指定する
- keepout が無ければ `use_keepout:=false`

### 4.2 起動 (main コンテナ内)

```bash
# robot 側 (既定の全部入りで可: R-Fans 点群・EKF・IMU)
ros2 launch rerobot_bringup rerobot_bringup.launch.py
# nav 側
ros2 launch rerobot_bringup nav_lidar3d.launch.py map_name:=<map_name>
# → RViz (Fixed Frame: map) の 2D Pose Estimate で初期位置を与える
#   (bridge が z を軌跡から入れて /initialpose_3d に中継し、ログに「← 軌跡の最寄り点」と出る)
```

主な引数:

| 引数 | 既定 | 説明 |
|---|---|---|
| `map_name` | (空) | 2D / 3D 共通の地図名。空だと 3D 地図が見つからず起動時に止まる |
| `map_path` | `/workspace/maps/3d/<map_name>/map.pcd` | 3D 地図を直接指定 |
| `traj_path` | `/workspace/maps/3d/<map_name>/traj_lidar.txt` | 初期位置の z を引く GLIM 軌跡。無ければ initial_z |
| `initial_z` | 0.0 | 軌跡が無いときのスタート地点の base_link の z [m] (map_3d 座標)。⚠️ GLIM 地図では出発点でも ≈ −0.7〜−0.8 m になる |
| `map_dir` / `map_yaml` / `keepout_yaml` / `use_keepout` | nav_amcl と同じ | 2D 地図側 |

### 4.3 監視 (bag 再生・実機で見るもの)

```bash
ros2 topic echo /alignment_status --once          # 照合の健全性
ros2 topic echo /reinitialization_requested       # 見失いの申告
ros2 run tf2_ros tf2_echo map base_link           # z ≈ 0 なら橋が効いている
ros2 run tf2_ros tf2_echo map_3d odom             # 3D 地図上の高さ (z) が見える
```

## 5. 変更ファイル一覧

- `.gitmodules` — submodule 2 本を追加
- `ros2_ws_main/src/localization/lidar_localization_ros2` — **新規 submodule** (rsasaki0109, v1.3.0 = 3878a7ad)
- `ros2_ws_main/src/localization/ndt_omp_ros2` — **新規 submodule** (rsasaki0109, humble ブランチ 63bf15b)
- `docker/Dockerfile_main` — `libpcl-dev`, `ros-jazzy-pcl-conversions` を追加
- `ros2_ws_main/src/bringup/rerobot_bringup/config/localization/lidar_localization.yaml` — **新規**。本機の差分パラメータ
- `ros2_ws_main/src/bringup/rerobot_bringup/scripts/map_level_bridge.py` — **新規**。z の打ち消し + /initialpose の中継
- `ros2_ws_main/src/bringup/rerobot_bringup/launch/parts/localization/loc_lidar3d.launch.py` — 仮置き (Shutdown) を実装に置換
- `ros2_ws_main/src/bringup/rerobot_bringup/launch/nav/nav_lidar3d.launch.py` — 仮置きを実装に置換 (`map_name` / `map_path` / `traj_path` / `initial_z`)
- `ros2_ws_main/src/bringup/rerobot_bringup/config/nav2_params.yaml` — global costmap の `urg_front` / `rfans` に `min_obstacle_height: -0.3`
- `ros2_ws_main/src/bringup/rerobot_bringup/CMakeLists.txt` — `map_level_bridge.py` をインストール
- `ros2_ws_main/src/bringup/rerobot_bringup/package.xml` — `lidar_localization_ros2`, `tf2_ros` を exec_depend に追加

## 6. 既知の制限

### 6.1 検証済み (2026-10-10、main コンテナ、ROS_DOMAIN_ID=77 で隔離、実データ・実地図なし)

| 試験 | 方法 | 結果 |
|---|---|---|
| ビルド | `ndt_omp_ros2` + `lidar_localization_ros2` を `--executor sequential`・make -j2・Release で | ✅ 通過 (約 30 分、PCL の非推奨警告のみ) |
| 橋: 起動時の仮 z | 実 GLIM 軌跡 (`2026-10-09_1643_5goukan_chal_dump/g1_imu_strong_filtered/traj_lidar.txt`) + 偽の TF base_link→rfans | ✅ map→map_3d = +0.732 (軌跡始点の base 高 −0.73 を打ち消し) |
| 橋: z の打ち消し | 偽の map_3d→odom (/tf, 10 Hz) を z = −0.8 → +2.5 と切り替えて `tf2_echo map odom` | ✅ どちらも z = 0.000 (/tf_static の出し直しが効く — rclpy broadcaster の不具合の回避を確認) |
| 橋: 初期位置の中継 | 軌跡 1000 点目の xy で /initialpose (map) を出す | ✅ /initialpose_3d (frame map_3d, z = −1.44 = 軌跡 z −0.644 − 取付高 0.794) |
| 地図なし起動 | `nav_lidar3d.launch.py` を map_name 無しで | ✅ 「3D 地図が見つからない …」を出して Shutdown (lifecycle_manager の終了時例外は巻き添えのノイズ) |
| 部品の統合 | 合成 PCD (20 m 箱) で `loc_lidar3d.launch.py` | ✅ lifecycle active、パラメータ 13 項目が意図どおり、remap 有効、中継した初期位置を本体が受理 (`published map_3d -> odom TF from initial pose`) |

### 6.2 未検証・制限

- **地図・実データを使った検証は未実施** (地図が未作成のため)。検証の予定順:
  1. bag 再生で本体だけ確認する: `/pcl_pose` と traj_lidar.txt の xy 差、`/alignment_status`、R-Fans の回転ディップ区間で飛ばないか、CPU 負荷 (つくば 2572 は計算資源不足で断念した)
  2. 橋 + Nav2: `tf2_echo map base_link` の z ≈ 0、global costmap に /rfans/scan の障害物が載るか (§2.1.3 と §2.2 の効き方の実測)
  3. A/B: `ndt_resolution` 2.0 / 5.0、IMU の ON / OFF
  4. 実機: 静止 → 直進 → 旋回 (浮かせ → 接地) → Nav2 で Goal
- **Nav2 の高さフィルタが map 座標の z で効くこと**は Nav2 ソースの記憶による (本機の /opt/ros/jazzy には header しか無い)。§6 の 2 の実測で確認する
- **初期位置の z は軌跡の最寄り点の高さ**であり、地図を撮ったときに通っていない場所 (最寄り点が 5 m より遠い) では、直近の推定 / initial_z にフォールバックする。z が大きく違う場所に置くと、NDT が z を引き寄せきれない可能性がある
- **3D 地図の傾き**: GLIM 地図に坂 L / A の歪みが残っていると、6DoF 照合が傾いた地図に合わせにいく。TF からは level で消えるが、照合の質そのものには影響する。歪みの少ない地図 (5号館) で配線を通してから、つくばの地図に進む
- **自動復帰なし**: 見失ったら `/reinitialization_requested` が立つだけ。人が 2D Pose Estimate し直す
- **RViz の Navigation 2 パネル**: localization の状態は unknown 表示になる (nav2 lifecycle_manager を使っていないため。§2.5)
- **コンテナの手入れ**: いまの `rerobot_env` には `libpcl-dev` 等を apt で入れてある。イメージを作り直すまでは、コンテナを作り直すと消える (`./scripts/build.sh images` で恒久化)
