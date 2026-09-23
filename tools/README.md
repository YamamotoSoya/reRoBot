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
- **解像度は `-r 0.10` を推奨** (2026-09-10 掃引)。dump の点は GLIM が 0.3 m ボクセルで
  間引いており、5 cm 画素では 1 submap から 1 画素に 1〜2 点しか落ちず点数しきい値が
  成立しない (壁が点線状・submap 間隔が開く区間で消える)。0.10 で右上 (最遠角) の占有
  17 倍・ノイズ増なし。機構と掃引表は読本 `docs/text/map3d_to_nav2/03_map_conversion.md` §3.4
- 絶対 z vs センサ相対の比較実測 (5号館 08-14 LC 地図、z ドリフト +4.9 m):
  `docs/text/map3d_to_nav2/img/2026-08-20_compare_abs_vs_sensor.png` (解説は読本 §3.4。
  元データ・両方式の map 出力は git 管理外の `bags/5goukan/2d3d_imu/offline/glim/2dmap_compare/`)
  — 絶対 z はドリフト最大部で壁が全滅、センサ相対は全周で壁が残る

## glim_traj_to_2dmap (自作, 2026-09-14)

<!-- claude: 2026-09-14 追加 -->
**bag の生点群 (間引き前) を GLIM dump の最適化済み軌跡 `traj_lidar.txt` で再投影**して
2D 占有格子を作る Python ツール (依存 numpy + rosbag2_py、ROS 2 Jazzy のコンテナ内で実行)。
glim_dump_to_2dmap が読む submap 点群は GLIM が 0.3 m ボクセルで間引いた後のもので、
5 cm 画素の濃さが「壁の物理密度」ではなく「重なった submap 数」になってしまう
(読本 §3.4 の機構)。本ツールは点の出どころを bag の生スキャン (1 回転 ≈ 3 万点) に戻し、
姿勢だけを GLIM から借りる。

ループクロージングが反映される理由: GLIM の LC は点を書き換えず **submap の姿勢を剛体で
動かす**だけで、dump 保存時に「最適化後 submap 姿勢 × submap 原点から見た各スキャンの
相対姿勢」を全スキャン分計算して `traj_lidar.txt` (TUM 形式、1 行 = 1 スキャン) に書く。
生スキャンをその姿勢で置き直せば、GLIM が自分の地図を世界座標に置くのと同じ計算になる
(`odom_lidar.txt` は LC 前のオドメトリ姿勢なので使わない)。5号館 LC dump では終端 z が
odom −4.50 m / traj −0.02 m で、traj 側だけループが閉じている。

高さ帯は既定で**センサ座標の z** (`--height_frame sensor`) で切る。実機の /scan
(`rfans_scan.launch.py` = pointcloud_to_laserscan、base_link 相対) と同じ切り方になり、
traj 姿勢のピッチ誤差 (z ドーム斜面) が帯に混入しない。

```bash
# glim_env 内 (rosbag2_py が要るので source が必要)
docker exec -it glim_env bash
source /opt/ros/jazzy/setup.bash
T=/workspace/tools/glim_traj_to_2dmap/glim_traj_to_2dmap.py
B=/workspace/bags/5goukan/2d3d_imu/online/rosbag/2026-08-14_0919
D=/workspace/bags/5goukan/2d3d_imu/offline/glim/glim_5goukan_lc_2026-08-14_0919   # dump dir (中の traj_lidar.txt を使う)

# 1) 床のセンサ座標 z を実測 (10 スキャンに 1 つ、約 15 s)。5号館 08-14 は -0.65 → 帯 -0.35〜+0.85
python3 $T $B $D /tmp/x --floor_probe --skip 10

# 2) 変換 (全 6,989 スキャンで約 2.5 分)
python3 $T $B $D /workspace/maps/glim/<name>/nav2 \
  -r 0.05 --map_width 6144 --map_height 6144 \
  --height_frame sensor --min_height -0.35 --max_height 0.85 \
  --min_points_in_pix 2 --max_points_in_pix 5
```

| 引数 | 既定 | 意味 |
|---|---|---|
| `bag_dir` / `traj` / `dest_dir` | — | bag ディレクトリ / `traj_lidar.txt` か dump ディレクトリ / 出力先 |
| `--topic` | `/rfans_driver/rfans_points` | PointCloud2 トピック |
| `--height_frame` | sensor | sensor = センサ座標 z で切る (実機 /scan と同じ) / world = 世界 z − 姿勢 z (glim_dump_to_2dmap の sensor と同じ意味) |
| `--min/max_height` | −0.45 / +0.75 | スライス帯。`--floor_probe` の出力 (床 +0.3〜+1.5) で決める |
| `--range_min/max` | 0.5 / 40 | この範囲外の点を捨てる (車体・無効点・遠方ノイズ) |
| `-r`, `--map_width/height`, `--center`, `--min/max_points_in_pix` | 0.05, 0 (自動), world, 2/5 | glim_dump_to_2dmap と同じ画素系・濃度変換 |
| `--stamp_tol` | 0.02 s | header stamp と traj stamp の許容差 (5号館 bag は差 0.00 ms で完全一致) |
| `--deskew` | off | 点ごとの `time` で前後スキャン姿勢を slerp 補間して動き補正 |
| `--skip N` | 1 | N スキャンに 1 つだけ使う (試験用) |
| `--floor_probe` | off | 地図を書かず床 z の最頻値だけ出す |
| `--save_counts <npy>` | なし | 画素点数配列を保存 (しきい値・解像度の派生を再実行なしで作る用) |

- **推奨引数 (2026-09-14 比較の結論)**: `--height_frame ground --min_height 0.3 --max_height 1.5
  --range_max 30 --deskew --min_points_in_pix 4 --max_points_in_pix 12`。ground は取付ピッチ
  (5号館 bag で実測 ≈1.2°) 由来の遠方地面の筋を吸収し、deskew は GLIM 自身の地図 (deskew 済み) と
  点配置を揃える (未補正だと回転区間で占有 IoU 0.78)。地上高指定なので /scan 側の帯と同じ単位
- 比較結果 (5号館 08-14 LC 地図): 右上/始点の占有比が submap 版 0.10 (r=0.05) / 0.37 (r=0.10) →
  生スキャン版 1.2 で場所依存の薄化が解消。ただし「実機 /scan との一致率」は hold-out 評価で
  submap 版 r=0.10 と同等 (0.79 vs 0.77) で、AMCL 実走での優劣は未検証。詳細・図・残る代替仮説は
  読本 `docs/text/map3d_to_nav2/03_map_conversion.md` §3.5
- ⚠️ dump は **LC 後に保存したもの**を渡す (`traj_lidar.txt` の終端 z が閉じているか確認。
  5号館は `glim_5goukan_lc_2026-08-14_0919/` が LC 後、`2026-08-14_0919_dump/` は LC 前)
- 処理時間: 全 6,989 スキャン (5.8 GB mcap) で 25〜45 s (glim_env、deskew 込みで 45 s)

## 99-wt901.rules (2026-09-23)

<!-- claude: 2026-09-23 追加 -->
比較用 IMU WITmotion WT901C-TTL の USB-TTL 変換 (Prolific 067b:23a3) に udev 安定名
`/dev/ttyUSB-wt901` を与える rule。導入コマンドはファイル冒頭、手順全体は
`docs/features/2026-09-23_wt901c_comparison_imu.md`。センサ本体の設定 (115200 / 200 Hz) は
Windows 公式ソフトで行った (Linux 用の設定スクリプトは検証後に不要となり削除)。
