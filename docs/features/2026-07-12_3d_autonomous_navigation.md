<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# 3D 自律移動 (LIO-SAM mapping + NDT localization + Nav2)

- 日付: 2026-07-12
- 対象パッケージ: `rerobot_perception` (新規)、`rerobot_bringup`、submodule: `src/external/LIO-SAM` (ros2 branch)・`src/external/lidar_localization_ros2`・`src/external/ndt_omp_ros2` (humble branch)
- 対象 ROS: ROS 2 Jazzy (検証は使い捨てコンテナ、リソース上限付き。実機・実地図は未検証)
- 関連文書: [2026-07-12_robustness_debuggability.md](2026-07-12_robustness_debuggability.md) (同ブランチの堅牢性改善)、[2026-06-13_rfans_driver_ros2_port.md](2026-06-13_rfans_driver_ros2_port.md) (R-Fans ドライバ移植)

## 1. 目的・概要

PROJECT_STATE で「設計判断待ち」だった 3D 構成のギャップ (/scan が無く SLAM/Nav2 に
繋がらない) を埋め、R-Fans-16 での自律移動を成立させる。ユーザ決定は
「mapping は LIO-SAM、3D localization は LIO-SAM 非対応なので外部リポジトリを
submodule 追加」。全体は 3 段構成:

1. **mapping**: LIO-SAM (R-Fans 点群 + RealSense IMU) → 3D PCD 地図
2. **localization**: lidar_localization_ros2 (NDT スキャンマッチング) → map→odom TF
3. **planning**: Nav2 (2D と同一サーバ群)。global は PCD を投影した 2D 地図、
   obstacle は /sdk_could を高さスライスした /scan

実装した機能:
- `rfans_ring_converter` (rerobot_perception): /sdk_could → /points。laserid/timeflag を LIO-SAM 要求の ring/time に変換
- `pcd_to_gridmap` (rerobot_perception): PCD 地図 → Nav2 用 2D 占有格子 (pgm+yaml) 投影 CLI
- `slam_3d.launch.py`: realsense_imu + converter + LIO-SAM 4 ノード + 静的 map→odom + RViz
- `nav2_3d.launch.py`: nav2.launch.py の amcl を lidar_localization_ros2 (map→odom モード) に差し替え
- `lio_sam_params.yaml` / `lidar_localization.yaml`: reRoBot 用パラメータ
- bringup_3d に pointcloud_to_laserscan (/scan 生成) と `odom_tf` / `publish_scan` 引数を追加
- Dockerfile に ros-jazzy-pointcloud-to-laserscan 追記

スコープ外 (意図的に未対応):
- **LIO-SAM への GPS 融合**: GPS 非搭載のため。パラメータは既定 (無効) のまま
- **3D costmap (voxel/STVL)**: Nav2 の障害物入力は /scan (2D スライス) で開始。段差・
  低い障害物が問題になったら検討
- **lidar_localization の IMU/twist 予測**: まず TF ベースの単純構成で開始 (use_imu /
  use_twist_prediction とも false)。NDT が遅れて破綻する場合に有効化を検討
- **slam_toolbox + /scan での 2D SLAM (3D LiDAR 利用)**: pointcloud_to_laserscan が
  /scan を出すので実は可能だが、本命は LIO-SAM のため文書化のみ (フォールバック手段)

## 2. 設計の勘所

### 2.1 ring/time は変換ノードで与える (LIO-SAM の角度逆算に頼らない)

実測: R-Fans の PointCloud2 フィールドは x,y,z,intensity,**laserid**(int32),
**timeflag**(float32, 絶対秒),hangle,pulseWidth,range,rol,mirrorid
(`StarROS2/include/rfans_driver/point_types.h` + `rfans_driver.cpp` の
initCloud.fields で確認)。LIO-SAM が要求するのは **ring**(uint16) と
**time**(float32, スキャン先頭からの相対秒) で、名前も型も異なる。

判断: LIO-SAM の velodyne モードは ring 欠落時に垂直角から
`rowIdn = (verticalAngle + (N_SCAN-1)) / 2.0` で逆算するが、これは縦角等間隔
(VLP-16 の 2° 刻み) 仮定であり、R-Fans-16 GM の不等間隔縦角
(memory: reference_rfans16_dataid_0x37 の GM-16 縦角表) では誤分類する。
実機の laserid をそのまま ring に使う変換ノード (`rfans_ring_converter`) が正。
timeflag は絶対秒なので `timeflag - min(timeflag)` で相対化し deskew を有効にする。

なお LIO-SAM の ring チェックは「"ring" フィールドがあれば ringFlag=1 で実値使用、
無ければ velodyne のみ逆算 (ringFlag=2)」なので、変換後の /points では実値が使われる
(imageProjection.cpp:279-301 で確認)。

却下した代替案:
- LIO-SAM を R-Fans 形式対応にパッチ: submodule のフォーク保守が増える。10 行の
  独立ノードで済む変換を本体改造でやる理由がない
- rfans_driver 側に ring/time フィールドを追加: SDK 由来の点構造体 (pack(1)) と
  フィールド定義が密結合で、driver 改造は影響範囲が広い

### 2.2 localization は lidar_localization_ros2 (submodule) — LIO-SAM は mapping 専用

実測: LIO-SAM (ros2 branch) に保存地図に対する localization モードは無い
(README・ソース確認)。地図保存は `/lio_sam/save_map` (lio_sam/srv/SaveMap) で
GlobalMap.pcd 等を書き出す方式。

判断: rsasaki0109/lidar_localization_ros2 を採用。選定理由:
(1) Jazzy を CI で検証済み (v1.1.0, 2026-06 リリースで活発)、
(2) **enable_map_odom_tf: true で map→odom を出す Nav2 互換モード**があり、
    epos4_odometry の odom→base_link TF と正しく分業できる、
(3) /initialpose 入力 = RViz "2D Pose Estimate" で amcl と同じ操作感、
(4) 依存が ndt_omp_ros2 (同作者, humble branch 指定) だけで submodule 2 個で済む。

却下した代替案:
- hdl_localization (koide3): ROS 1 のみ。ROS 2 の公式ポートが無い
- Autoware の ndt_scan_matcher: スタック全体の引き込みが必要で過大
- LIO-SAM localization フォーク: メンテ状況がまちまちで Jazzy 動作実績が不明

### 2.3 TF odom→base_link の所有権をモードで切り替える (odom_tf 引数)

実測: LIO-SAM の imuPreintegration は odometryFrame→baselinkFrame の TF を IMU
レートで broadcast する (imuPreintegration.cpp:128-146)。epos4_odometry も同じ
TF を出すため、mapping 中に両方生きていると TF ツリーが二重親で壊れる。

判断: bringup_common に `odom_tf` launch 引数 (既定 true) を追加し、epos4_odometry
の `publish_tf` パラメータへ後勝ち注入。mapping 時のみ `odom_tf:=false` で起動する
運用にした。/odom トピック自体は出続けるので走行ログは残る。
自律走行時は従来どおり epos4_odometry が odom→base_link、lidar_localization が
map→odom を出す (amcl と同じ分業)。

補足: LIO-SAM mapOptimization は odom→"lidar_link" (ハードコード名) も出すが、
本機の LiDAR frame は rfans なので孤立フレームになるだけで無害 (パッチ不要)。

### 2.4 Nav2 の障害物入力は /scan (pointcloud_to_laserscan の高さスライス)

判断: Nav2 の costmap 構成 (nav2_params.yaml) は 2D と共通のまま、/sdk_could を
pointcloud_to_laserscan で高さ 0.1〜1.0 m (base_link 基準) にスライスして /scan を
作る。`target_frame: base_link` にしたのは、センサフレーム (z=0.714 取付) のまま
だと高さ指定が直感に反するため。範囲は range_min 0.5 (車体・マスト反射除去) /
range_max 40 / 360°。QoS はドライバ→変換が reliable、変換→slam_toolbox・costmap
が best-effort (sensor QoS) で整合する。

却下した代替案:
- costmap の obstacle layer に PointCloud2 を直接入れる: voxel layer の調整項目が
  増える。まず 2D と同じ構成で実績を作る (§1 スコープ外)
- nav2_params.yaml を 3D 用に分岐: /scan 経由なら分岐不要。ファイル 2 重化を回避

### 2.5 global 地図は PCD を pcd_to_gridmap でオフライン投影

判断: Nav2 の planner は 2D 占有格子が必要。LIO-SAM の GlobalMap.pcd を
「障害物帯 (0.2〜1.5 m) に min-points 個以上 → 占有 / 地面帯 (-0.3〜0.1 m) に
点あり → 自由 / その他 → 未知」の規則で投影する CLI を用意した。レイキャストに
よる厳密な自由空間推定はせず、**生成物は GIMP で手直しする下絵**という位置付け
(keepout マスクと同じ運用)。z 基準は LIO-SAM map 原点 (≒開始時の姿勢) なので、
実 PCD を CloudCompare 等で確認して z 帯を指定し直すこと。

### 2.6 lifecycle は launch 側 CONFIGURE→ACTIVATE (slam.launch.py の前例踏襲)

実測: lidar_localization_node は LifecycleNode だが main() で自己 configure/activate
しない (外部遷移待ち)。

判断: nav2 lifecycle_manager は bond 接続前提のため非 nav2 ノードの管理には使わず、
slam.launch.py で実績のある EmitEvent CONFIGURE → OnStateTransition → ACTIVATE
パターンを踏襲した。リポジトリ内で lifecycle の扱いが 1 パターンに揃う。

## 3. データフロー

```
[mapping 時]  bringup_3d (odom_tf:=false) + slam_3d.launch.py

rfans_driver ──/sdk_could (PointCloud2: laserid/timeflag)──▶ rfans_ring_converter
                                                                  │ /points (ring/time)
realsense2_camera ──/camera/camera/imu──▶ imu_filter_madgwick     │
                                                │ /imu/data       │
                                                ▼                 ▼
                          LIO-SAM (imuPreintegration / imageProjection /
                                   featureExtraction / mapOptimization)
                                │ TF odom→base_link (imuPreintegration)
                                │ TF map→odom は静的恒等 (launch)
                                ▼
                     /lio_sam/save_map (lio_sam/srv/SaveMap) → GlobalMap.pcd
                                │ (オフライン)
                                ▼
                     pcd_to_gridmap → nav2/my_map.{pgm,yaml}

[自律走行時]  bringup_3d (既定) + nav2_3d.launch.py

/sdk_could ──▶ pointcloud_to_laserscan ──/scan (LaserScan, base_link z 0.1〜1.0m)──▶ costmap 障害物層
/sdk_could ──▶ lidar_localization (NDT, GlobalMap.pcd) ── TF map→odom
                    ▲ /initialpose (RViz 2D Pose Estimate)
                    │ /localization/pose_with_covariance (旧 /pcl_pose)
epos4_odometry ── TF odom→base_link + /odom
map_server (my_map.yaml) + keepout ──▶ Nav2 サーバ群 ──/robot_speed_cmd──▶ epos4_controller
```

## 4. 使い方

```bash
# --- mapping ---
ros2 launch rerobot_bringup rerobot_bringup_3d.launch.py odom_tf:=false
ros2 launch rerobot_bringup slam_3d.launch.py
# 走行後 (destination は $HOME からの相対。先頭/末尾 "/" 必須):
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2, destination: /maps/lio_sam/2026_07_12/}"
# 2D 投影 (z 帯は PCD を見て調整):
ros2 run rerobot_perception pcd_to_gridmap ~/maps/lio_sam/2026_07_12/GlobalMap.pcd \
    ~/maps/lio_sam/2026_07_12/nav2/my_map --obstacle-zmin 0.2 --obstacle-zmax 1.5

# --- 自律走行 ---
ros2 launch rerobot_bringup rerobot_bringup_3d.launch.py
ros2 launch rerobot_bringup nav2_3d.launch.py map_dir:=$HOME/maps/lio_sam/2026_07_12
# RViz "2D Pose Estimate" → NDT 収束 → "Nav2 Goal"
```

| 項目 | 値 | 説明 |
| --- | --- | --- |
| `odom_tf` (bringup) | true | epos4_odometry の odom→base_link TF。**mapping 時は false 必須** |
| `publish_scan` (bringup_3d) | true | pointcloud_to_laserscan の起動有無 |
| `map_dir` (nav2_3d) | — | `lio_sam/GlobalMap.pcd`・`nav2/my_map.yaml`・`keep_out/` の親 |
| pointcloud_to_laserscan 高さ帯 | 0.1〜1.0 m | base_link 基準 (params_3d.yaml) |
| NDT (lidar_localization.yaml) | resolution 1.0 / score 6.0 | 屋外都市部向け初期値。要実地調整 |

## 5. 変更ファイル一覧

- `src/rerobot_perception/` — 新規パッケージ (rfans_ring_converter / pcd_to_gridmap)
- `src/rerobot_bringup/launch/slam_3d.launch.py` — 新規 (LIO-SAM mapping)
- `src/rerobot_bringup/launch/nav2_3d.launch.py` — 新規 (NDT localization + Nav2)
- `src/rerobot_bringup/config/lio_sam_params.yaml` — 新規 (LIO-SAM 用)
- `src/rerobot_bringup/config/lidar_localization.yaml` — 新規 (NDT 用)
- `src/rerobot_bringup/launch/rerobot_bringup_3d.launch.py` — pointcloud_to_laserscan + odom_tf/publish_scan 引数
- `src/rerobot_bringup/launch/rerobot_bringup_common.launch.py` — odom_tf 引数 → epos4_odometry publish_tf
- `src/rerobot_bringup/config/params_3d.yaml` — pointcloud_to_laserscan セクション
- `Dockerfile` — ros-jazzy-pointcloud-to-laserscan 追加
- `.gitmodules` / `src/external/{lidar_localization_ros2,ndt_omp_ros2}` + `src/` symlink — submodule 追加
- `CLAUDE.md` — 3D 自律移動セクション・パッケージ説明・TF 所有権の罠を追記

## 6. 既知の制限

- **実機・実地図で全段未検証**。コンテナ検証済みなのは: ビルド (rerobot_perception /
  lio_sam)、rfans_ring_converter の変換正当性 (合成点群で ring/time/無効点除去を確認)、
  pointcloud_to_laserscan の /scan 生成、pcd_to_gridmap の投影、launch --print、
  LIO-SAM ノードの起動。実機で最初に確認すべきは (1) IMU extrinsics (lio_sam_params の
  仮値恒等行列 — 違うと即発散)、(2) Horizon_SCAN 2000 の妥当性、(3) NDT の初期収束
- **IMU ノイズパラメータは D435i の一般流通値** (allan variance 未計測)
- lidar_localization_ros2 / ndt_omp_ros2 の Jazzy ビルドは submodule 追加後に要確認
  (upstream は Jazzy CI ありだが本環境では未実施)
- timeflag が use_gps:=false 時に UTC 内部時刻であることは確認済みだが、スキャン境界を
  跨ぐ値の折り返しは未確認 (converter は min 基準の相対化なので通常は問題にならない)
- pcd_to_gridmap は自由空間をレイキャストせず地面点の有無で判定する。密な下草などで
  地面点が欠ける場所は未知セルになるため、投影後の手直し (GIMP) が前提
