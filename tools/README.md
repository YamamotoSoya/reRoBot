<!-- claude: 2026-08-17 作成 -->
# tools/

コンテナ内で使う補助ツール置き場 (ROS ワークスペース外)。glim コンテナに
`/workspace/tools` として bind mount される (docker-compose.yml)。

## pointcloud_to_2dmap (submodule)

GLIM の 3D 点群地図 (PLY→PCD) を Nav2 map_server 用の 2D 占有格子 (png + yaml) に
変換する既製ツール (GLIM と同作者 koide3 製)。使い方・パラメータの意味は
`docs/features/2026-08-17_glim_map_to_nav2.md` と `docs/text/map3d_to_nav2/` を参照。

### ビルド (glim コンテナ内)

```bash
mkdir -p /workspace/tools/pointcloud_to_2dmap/build
cd /workspace/tools/pointcloud_to_2dmap/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-include boost/make_shared.hpp'
make
```

⚠️ `-include boost/make_shared.hpp` は必須。ソースが `boost::make_shared` を
ヘッダ include なしで使っており、旧 Boost では他ヘッダ経由で偶然通っていたが
Boost 1.83 (Jazzy) では未宣言エラーになる。**submodule のソースを直接パッチせず**
(pristine な上流 commit を gitlink に保つため)、コンパイラの強制 include で解決している。

依存 (`libpcl-dev`, `pcl-tools`, OpenCV, Boost) は Dockerfile_glim で導入済み。
build/ はホスト側に永続化されるが、`.gitignore` 対象 (バイナリはコミットしない)。

## glim_dump_to_2dmap (自作, 2026-08-20)

<!-- claude: 2026-08-20 追加 -->
GLIM の **dump ディレクトリを直接**読んで 2D 占有格子 (map.pgm + map.yaml) を作る
Python ツール (依存 numpy のみ、ビルド不要)。既製 pointcloud_to_2dmap との違いは
高さスライスの基準: マージ済み PCD では失われる「点↔センサ姿勢」の対応が dump には
残っている (submap = 点群 + T_world_origin) ので、**各点をその submap のセンサ z からの
相対高さで**切れる (`--height_mode sensor`, 既定)。地図の z がドリフトしていても
スライス帯がセンサと一緒に上下するため、絶対 z 方式のように「場所によって壁が
スライスから外れる」ことがない。坂のあるコースでもそのまま使える。

```bash
# 例: 5号館 LC 地図 (帯はセンサ相対。床は同梱手順で実測 → 床+0.3〜1.5 に相当する値を指定)
docker exec glim_env python3 /workspace/tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py \
  /workspace/bags/5goukan/2d3d_imu/offline/glim/glim_5goukan_lc_2026-08-14_0919 \
  /workspace/maps/glim/<name>/nav2 \
  -r 0.05 --map_width 6144 --map_height 6144 \
  --height_mode sensor --min_height -0.25 --max_height 0.95
```

- `--center world` (既定) で既製ツールと同じ world (0,0) 中心。`--center auto` は
  点群 bbox 中心 + サイズ自動決定 (既製ツールの「原点中心固定で巨大地図になる」制約の回避)
- `--export_pcd <path>` で全点マージの世界座標 PCD も書ける (既製ツールとの比較・
  3D ローカライザ地図用)
- 濃度変換 (`--min/max_points_in_pix`) と yaml 形式は既製ツール互換。画像は PGM
  (map_server は png/pgm どちらも可)
- 絶対 z vs センサ相対の比較実測 (5号館 08-14 LC 地図、z ドリフト +4.9 m):
  `bags/5goukan/2d3d_imu/offline/glim/2dmap_compare/compare_abs_vs_sensor.png`
  — 絶対 z はドリフト最大部で壁が全滅、センサ相対は全周で壁が残る
