<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# GLIM 3D 地図 → Nav2 接続パイプライン (2D 地図変換 + R-Fans /scan 化)

- 日付: 2026-08-17
- 対象パッケージ: `tools/pointcloud_to_2dmap` (submodule, koide3 製既製ツール)、
  `rerobot_bringup` (`ros2_ws_main/src/bringup/rerobot_bringup` — launch 追加)
- 対象 ROS: ROS 2 Jazzy (変換 = `glim_env` コンテナ / 走行系 = `rerobot_env` コンテナ)
- 関連文書: 体系的解説は `../text/map3d_to_nav2/` (読本)。地図品質の前提は
  `../issue/2026-08-16_rfans_mount_angle_glim_z_collapse.md` (縦角再較正 = 保留中) と
  `../issue/2026-08-13_glim_param_tuning.md` (E11 チューニング)

## 1. 目的・概要

GLIM が作る 3D 点群地図で Nav2 自律走行をするための「接続層」。Nav2 本体は
2D 占有格子 + 2D 自己位置の世界なので、3D 地図をそのまま食えない。本機能は
Nav2 が要求する 3 入力のうち 2 つを 3D 資産から作る:

1. **地図 (/map)**: GLIM の PLY → 高さスライス → png/yaml → 既存 `nav2.launch.py` の
   `map_yaml:=` にそのまま渡す
2. **自己位置と障害物観測の入力 (/scan)**: R-Fans 3D 点群 → `pointcloud_to_laserscan` →
   /scan → 既存の AMCL + costmap がそのまま消費 (「最短案」)

Nav2 側 (nav2.launch.py / nav2_params.yaml) は**無改造**。入力の供給者を差し替えるだけの
設計なので、屋内 2D 運用 (UTM-30LX + slam_toolbox 地図) とは起動コマンドの違いだけで共存する。

実装した機能:
- 3D→2D 地図変換 (既製 `pointcloud_to_2dmap` の導入・ビルド・実行手順の確立)
- `rfans_scan.launch.py` (R-Fans 点群 → /scan 変換 launch)
- 9 号館 08-14 bag の N12 地図で通し検証 (変換 → map_server 配信 → bag 再生で /scan 生成)

スコープ外 (意図的に未対応):
- **AMCL 込みの実走行検証** — 実機が使えない期間のため。実機再開時の残タスク (§6)
- **本格案 = 3D 点群地図ローカライザの実装** — 設計 (§2.5) のみ。別 PC で進行中の
  地図高品質化が確定してから着手する判断
- **自作変換スクリプト** — 既製ツールの制約 (§6) が実用上問題になってから
  (設計は読本 `../text/map3d_to_nav2/` に記載)
- **縦角再較正** — 較正環境なしのため保留 (2026-08-17 ユーザ指示)。地図品質の根本対策は
  引き続きこれが本命

## 2. 設計の勘所

### 2.1 地図スライス帯 = /scan 高さ帯の整合 (最重要)

AMCL は「地図の占有セル」と「/scan の観測」の一致度で自己位置を推定する。地図を
床+0.3〜1.5 m でスライスしたなら、/scan も同じ帯の点から作らないと「地図に無い壁」を
観測して推定が暴れる。**帯の値を変えるときは変換コマンドの `--min/max_height` と
`rfans_scan.launch.py` の `min/max_height` を必ずセットで変える**。

- 判断: 帯は床+0.3〜1.5 m。車体がぶつかる高さ帯 = 障害物とみなすべき点。
  08-13 の GLIM 評価スライスと同じ値で、実績がある
- 注意: 変換ツールの `--min/max_height` は**地図座標の絶対 z**、launch 側は
  **base_link 基準の相対高さ**。地図側は床 z (下記 2.2) を足して指定する

### 2.2 既製ツール採用 (自作スクリプトの却下)

- 判断: 変換は既製の [koide3/pointcloud_to_2dmap](https://github.com/koide3/pointcloud_to_2dmap)
  を使う (2026-08-17 ユーザ決定)。GLIM と同作者で、まさに「3D SLAM 地図で 2D ナビ」用途の
  ツール。書くコードゼロ・保守ゼロで、制約が実用上問題になってから自作に切替える方針
- 実測: catkin 非依存の素の CMake (PCL + OpenCV + Boost program_options のみ)。
  Jazzy (Boost 1.83) では `boost::make_shared` の include 漏れでビルドエラーになるが、
  ソースを触らず `-DCMAKE_CXX_FLAGS='-include boost/make_shared.hpp'` で解決
  (submodule を pristine に保つため。`tools/README.md` 参照)

却下した代替案:
- 自作 numpy スクリプト: 未知領域の区別・局所床推定 (z ドリフト耐性) を作り込めるが、
  新規コード ~200 行の保守が発生。既製で足りるかを先に見る (フォールバックとして温存)
- octomap_server 連結 (pcl_ply2pcd → pcd_to_pointcloud → octomap_server → map_saver_cli):
  コマンドのみで済むが、完成済み静止点群ではレイキャスト前提の自由空間判定が機能しない
- bag から 2D SLAM やり直し (08-12 実績): GLIM の最適化済み地図を使わない別物になる

### 2.3 床 z の実測と地図サイズ

ツールの高さクリップは絶対 z なので、変換前に床 z を実測する (z ヒストグラム最頻値。
§4 のワンライナー)。地図は world (0,0) 中心の固定サイズなので、xy 範囲も同時に実測して
`-w -h` を決める。

- 実測 (N12): 床 z = 0.636、x −17〜40 m / y −33〜11 m → 2048 px @ 0.05 m (±51.2 m) を採用
- ⚠️ ツールの y 中心計算に `map_width` を使う実装バグがあるため **`-w` と `-h` は必ず同値**にする

### 2.4 /scan の設計 (nav2_params 無改造の成立条件)

- `target_frame: base_link` — 取付角の吸収は URDF (TF) に任せる。rfans プリセット
  (tilted45/tilted15/flat) のどれでも launch 設定は共通
- `range_max: 30.0` — nav2_params.yaml の amcl `laser_max_range: 30.0` に一致させた。
  これにより **nav2_params.yaml は 1 行も変えずに** 3D 由来 /scan で AMCL が回る
- `angle_increment: 0.0035` — R-Fans-16 の方位分解能 ~0.19°/step (30048 点/回転 ÷ 16 ビーム) 相当
- `range_min: 0.5` — 車体・マストの映り込み除外
- ⚠️ /scan は urg_node と同名。**同時起動禁止** (bringup は `lidar_2d:=false lidar_3d:=true`)

### 2.5 本格案 (3D 点群地図ローカライザ) の設計 — 未実装

AMCL を「3D 地図に対する scan matching localization」に置き換える案。TF map→odom の
供給者が交代するだけで Nav2 は引き続き無改造。実装トリガは高品質地図の確定。

- 候補 (2026-08-17 調査):
  - **[lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2) (推奨)**
    — ROS 2 ネイティブ。NDT/GICP/NDT_OMP/GICP_OMP 選択式。**/odom・/imu を補助入力に
    できる** = 廊下縮退への保険 (GLIM 水平ドリフトと同じ弱点を持つため重要)
  - [pcl_localization_ros2](https://github.com/scav-project/pcl_localization_ros2) — 同系の PCL ベース
  - [hdl_localization](https://github.com/koide3/hdl_localization) — 本家 (UKF + NDT) だが ROS 1。参照用
- 配置: main ws にソースビルドを第一候補 (PCL は libpcl-dev で入る。ndt_omp 等の依存の壁が
  あれば glim コンテナか専用薄コンテナへ)
- 地図入力: 同じ PLY→PCD (pcl_ply2pcd) を流用。初期位置は /initialpose (RViz 2D Pose Estimate)

## 3. データフロー

```
[オフライン: 地図作成 (glim コンテナ)]
GLIM dump ── offline_viewer --export_path ──▶ map.ply
map.ply ── pcl_ply2pcd ──▶ map.pcd
map.pcd ── pointcloud_to_2dmap (高さクリップ+点数閾値) ──▶ my_map.png + my_map.yaml
                                                            (maps/glim/<name>/nav2/)

[オンライン: 走行 (main コンテナ)]
rerobot_bringup (lidar_2d:=false lidar_3d:=true imu:=true ekf:=true)
  ├── /rfans_driver/rfans_points (sensor_msgs/PointCloud2, frame=rfans, 10 Hz)
  ├── /odometry/filtered + TF odom→base_link (EKF)
  └── TF base_link→rfans (robot_state_publisher, URDF プリセット)
        │
        ▼
rfans_scan.launch.py [rfans_to_scan ノード]
  └── /scan (sensor_msgs/LaserScan, frame=base_link, ~10 Hz, best-effort QoS)
        │
        ▼
nav2.launch.py (map_yaml:=…/my_map.yaml use_keepout:=false)
  ├── map_server ──▶ /map (nav_msgs/OccupancyGrid)
  ├── amcl (/scan × /map) ──▶ TF map→odom
  └── Nav2 サーバ群 ──▶ /robot_speed_cmd (Twist)
```

## 4. 使い方

### 4.1 地図変換 (glim コンテナ、コマンド直叩き)

```bash
# 1) PLY → PCD (実測 2 ms / 17.8 万点)
docker exec glim_env pcl_ply2pcd \
  /workspace/bags/9goukan/2d3d_imu/offline/glim/exp_2026-08-14/N12/N12_dense_map.ply /tmp/n12.pcd

# 2) 床 z と xy 範囲の実測 (--min/max_height と -w -h を決めるため)
docker exec glim_env bash -c "python3 -c \"
import numpy as np
raw = open('/workspace/bags/.../N12_dense_map.ply','rb').read()
pts = np.frombuffer(raw, dtype=np.float32, offset=raw.index(b'end_header')+11).reshape(-1,3)
h,e = np.histogram(pts[:,2], bins=np.arange(pts[:,2].min(), pts[:,2].max(), 0.05))
print('z_floor %.3f  x %.1f..%.1f  y %.1f..%.1f' % (e[h.argmax()]+0.025,
      pts[:,0].min(), pts[:,0].max(), pts[:,1].min(), pts[:,1].max()))\""

# 3) 変換 (min/max_height = 床 z + 0.3 / +1.5。N12 は床 0.636 → 0.94/2.14。
#    -w -h は同値必須、xy 範囲の最大絶対値×2÷resolution 以上に)
docker exec glim_env bash -c "mkdir -p /workspace/maps/glim/2026-08-14_N12/nav2 &&
  /workspace/tools/pointcloud_to_2dmap/build/pointcloud_to_2dmap \
    -r 0.05 -w 2048 -h 2048 --min_height 0.94 --max_height 2.14 \
    /tmp/n12.pcd /workspace/maps/glim/2026-08-14_N12/nav2"

# 4) map_dir 規約名に整える (image 参照の追従を忘れない)
docker exec glim_env bash -c "cd /workspace/maps/glim/2026-08-14_N12/nav2 &&
  mv map.png my_map.png && sed -i 's/^image: map.png/image: my_map.png/' map.yaml &&
  mv map.yaml my_map.yaml"
```

| pointcloud_to_2dmap 引数 | 使用値 | 説明 |
|---|---|---|
| `-r/--resolution` | 0.05 | m/pix (既存 slam_toolbox 地図と同じ) |
| `-w/-h` | 2048 | 地図サイズ [pix]。world (0,0) 中心。**必ず同値** (§2.3) |
| `--min/max_height` | 床z+0.3 / 床z+1.5 | 高さクリップ (絶対 z)。/scan の帯と整合 (§2.1) |
| `--min_points_in_pix` | 2 (既定) | 占有とみなす最小点数 |
| `--max_points_in_pix` | 5 (既定) | 黒飽和の点数 (2〜5 の間はグレー = map_server では未知扱い) |

### 4.2 走行 (main コンテナ)

```bash
# bringup (3D LiDAR + IMU + EKF。/scan 衝突回避のため lidar_2d は必ず false)
docker exec -it rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&
  ros2 launch rerobot_bringup rerobot_bringup.launch.py lidar_2d:=false lidar_3d:=true imu:=true ekf:=true"

# R-Fans → /scan 変換
docker exec -it rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&
  ros2 launch rerobot_bringup rfans_scan.launch.py"

# Nav2 (GLIM 由来地図を直接指定。keepout マスクは未作成なので false)
docker exec -it rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&
  ros2 launch rerobot_bringup nav2.launch.py \
    map_yaml:=/workspace/maps/glim/2026-08-14_N12/nav2/my_map.yaml use_keepout:=false"
```

起動後は従来どおり RViz の 2D Pose Estimate → Nav2 Goal。

### 4.3 検証済みの範囲 (2026-08-17、実機なし)

- 実測: N12 (9 号館 08-14) の変換 → map_server 配信 (`/map` 2048×2048 @0.05, origin −51.2) OK
- 実測: 08-14 bag 再生 (`--topics /rfans_driver/rfans_points /tf /tf_static`) + rfans_scan で
  /scan が ~10 Hz、1796 ビン中 1791 有効 (0.50〜25.66 m)、frame=base_link を確認
- AMCL 込みの通しは未 (§6)

## 5. 変更ファイル一覧

- `tools/pointcloud_to_2dmap` — 新規 submodule (koide3 製、pristine のまま)
- `tools/README.md` — ビルド手順メモ (`-include boost/make_shared.hpp` の理由込み)
- `.gitmodules` — submodule 追加 + `ignore = untracked` (build/ の dirty 表示抑止)
- `docker-compose.yml` — glim サービスに `./tools:/workspace/tools` マウント追加
- `docker/Dockerfile_glim` — `libpcl-dev` + `pcl-tools` 追加 (稼働コンテナは apt 適用済み)
- `docker/Dockerfile_main` — `ros-jazzy-pointcloud-to-laserscan` 追加 (同上)
- `ros2_ws_main/src/bringup/rerobot_bringup/launch/rfans_scan.launch.py` — 新規 launch
- `maps/glim/2026-08-14_N12/nav2/my_map.{png,yaml}` — 検証用の変換済み地図 (git 管理外)

## 6. 既知の制限

- **未知領域の概念がない**: 点が無いピクセルは全て「自由 (白)」になる。未探索領域にも
  経路を引けてしまう。問題化したら自作スクリプト (床点の有無で自由/未知を区別) へ切替
- **高さクリップが固定 z**: 現行 GLIM 地図の z ドリフト (壁二重化 issue の根) があると
  床が場所により滑り、スライスから壁が外れ/床が混入する。根本対策は縦角再較正 (保留中)。
  N12 (376 s 屋内) では実用範囲だった
- **AMCL 込みの実走検証が未**: 実機再開時に「2D Pose Estimate → 収束 → Nav2 Goal」の通しを行う
- ツールの yaml は `mode:` 無し (trinary 既定)・occupied 0.5/free 0.2。既存 slam_toolbox 地図
  (0.65/0.196) と閾値が違うが、グレー階調の意味が違うため意図的にツール既定のまま
- pointcloud_to_2dmap は LICENSE ファイル無し (README のみ)。利用は問題ないが再配布時は要確認
- /scan は best-effort QoS (p2l の仕様)。reliable 購読のノードには届かない (AMCL/costmap は
  sensor QoS なので問題なし)。`ros2 topic echo` で見えないときは QoS を疑う
- 傾け取付 (tilted45) では高さ帯に入る点の方位分布が偏る (後方が薄い等)。/scan の
  カバレッジは flat が最良。実走前に RViz で /scan の抜けを確認すること
