<!-- claude: 運用手引き 第9章 テンプレ (2026-09-22)。本文は Claude が要約 (2026-09-24)。詳細は docs/text/map3d_to_nav2/ を正とする。 -->

# 第9章 map2d圧縮

GLIM で作った 3D 地図を、Nav2 (map_server / amcl) が読める **2D 占有格子 (map.pgm + map.yaml)** に落とす工程。
「ある高さ帯の点だけを真上から見て、点が多い画素を壁にする」だけだが、GLIM 地図は z がドリフトする (5号館で最大 +4.9 m) ので、
**高さ帯を世界座標ではなくセンサ基準で切る**のが要点。自作ツールが 2 つあり、点の出どころが違う。

| ツール (`tools/`) | 点の出どころ | 高さ帯の基準 | 必要なもの | 所要 | 向き |
|---|---|---|---|---|---|
| [glim_dump_to_2dmap](../../tools/glim_dump_to_2dmap/) | GLIM dump の submap 点群 (0.3 m 間引き済み) | 点の世界 z − その submap のセンサ z | dump のみ (numpy) | 数秒 | まず形を見る / bag が無い |
| [glim_traj_to_2dmap](../../tools/glim_traj_to_2dmap/) | **bag の生点群** (間引き前) | センサ座標 z、または地面平面からの高さ | dump の `traj_lidar.txt` + 元 bag (ROS 環境) | 25〜45 s | **Nav2 用の本番地図 (推奨)** |

## 9.1 仕組み

- **dump 版**: submap ごとに `T_world_origin` (そのときのセンサ姿勢) が残っているので、点を世界座標に置いたあと「センサの z」を引いてから帯で切る。帯がセンサと一緒に上下するので z ドリフトに強い。ただし点は GLIM が間引いた後なので、画素の濃さが「壁の密度」ではなく「重なった submap 数」になり、区間によって壁が点線になる。
- **traj 版**: 点は bag の生スキャン、姿勢だけを GLIM の最適化後軌跡 `traj_lidar.txt` (TUM 形式、1 行 = 1 スキャン) から借りる。GLIM のループクロージングは点を書き換えず submap 姿勢を動かすだけなので、生スキャンをこの姿勢で置き直せば LC 反映済みかつ生密度の地図になる。`--deskew` で回転中のにじみも補正できる。

なぜ 2 つ残っているか、スキャンマッチング上の得失は [読本 第3章 §3.4〜3.5](../text/map3d_to_nav2/03_map_conversion.md) を参照。

## 9.2 実行手順

前提: **LC 後に保存した dump** を使う (`traj_lidar.txt` の終端 z が閉じているか確認。LC 前後で行数も時刻範囲も同じなので取り違えやすい)。
どちらも glim コンテナで実行する。**保存先はコマンド最後の位置引数 `dest_dir`** で毎回指定する (既定値なし)。
ツールがそのディレクトリを作り、中に `map.pgm` と `map.yaml` の 2 ファイルを書く。
慣例は `/workspace/maps/2d/glim/<name>/nav2` — `maps/` は全コンテナに `./maps:/workspace/maps` で mount されているので、
ホストでは `reRoBot/maps/2d/glim/<name>/nav2/` に現れ、main コンテナの Nav2 からも同じパスで読める。
別設定で作り直すときは `<name>/nav2_thr13` のように dest_dir を変えれば並べて残せる。

### dump 版 (速い・形の確認用)

```bash
docker exec glim_env python3 /workspace/tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py \
  /workspace/bags/<場所>/.../glim/<dump_dir> \
  /workspace/maps/2d/glim/<name>/nav2 \
  -r 0.10 --height_mode sensor --min_height -0.25 --max_height 0.95
```

- 1 つ目の引数が dump、**2 つ目が保存先 (dest_dir)**

- `-r 0.10` を推奨 (0.05 だと間引きのせいで壁が点線になる)
- 帯 (`--min/max_height`) はセンサ相対。床のセンサ z を実測して「床 +0.3〜+1.5」に相当する値にする

### traj 版 (本番地図、推奨設定 R)

```bash
docker exec -it glim_env bash
source /opt/ros/jazzy/setup.bash          # rosbag2_py が要る
T=/workspace/tools/glim_traj_to_2dmap/glim_traj_to_2dmap.py
B=/workspace/bags/<場所>/.../online/rosbag/<bag_dir>
D=/workspace/bags/<場所>/.../glim/<dump_dir>      # 中の traj_lidar.txt を使う
O=/workspace/maps/2d/glim/<name>/nav2                # ← 保存先 (dest_dir)。map.pgm + map.yaml がここにできる

# 1) 床のセンサ座標 z を実測 (約 15 s)。ground モードでは自動推定されるので省略可
python3 $T $B $D /tmp/x --floor_probe --skip 10     # 地図は書かないので保存先はダミー

# 2) 変換
python3 $T $B $D $O \
  -r 0.05 --height_frame ground --min_height 0.3 --max_height 1.5 \
  --range_max 30 --deskew --min_points_in_pix 4 --max_points_in_pix 12
```

- `--height_frame ground` は地上高指定なので、実機 `/scan` (pointcloud_to_laserscan) の帯と同じ単位で揃えられる
- Nav2 用に 0.10 m 格子にするなら `-r 0.10 --min_points_in_pix 8 --max_points_in_pix 24`
- 帯を実機と完全に揃えたい (筋ノイズごと入れてよい) なら `--height_frame sensor --min_height -0.35 --max_height 0.85` (値は床実測で決める)

## 9.3 Nav2 に渡す

`nav2.launch.py` の規約は `<map_dir>/nav2/my_map.yaml` + `<map_dir>/keep_out/keep_out.yaml`。

```bash
# 規約に合わせるなら出力をリネーム (yaml 内の image: も合わせる)
cd /workspace/maps/2d/glim/<name>/nav2 && mv map.pgm my_map.pgm && sed -i 's/^image: map.pgm/image: my_map.pgm/' map.yaml && mv map.yaml my_map.yaml
# 起動 (main コンテナ)
ros2 launch rerobot_bringup nav2.launch.py map_dir:=/workspace/maps/2d/glim/<name> use_keepout:=false
# リネームせず直接指定するなら
ros2 launch rerobot_bringup nav2.launch.py map_yaml:=/workspace/maps/2d/glim/<name>/nav2/map.yaml use_keepout:=false
```

3D 地図由来なので、amcl の入力 `/scan` は 2D LiDAR ではなく `rfans_scan.launch.py` (R-Fans 点群 → LaserScan、`lidar_2d:=false` 前提) で作る。→ [第10章 Nav2](10_nav2.md) / [第11章 amcl](11_amcl.md)

## 9.4 注意

- 生スキャン版は停車・低速区間が濃くなる、歩行者が壁になり得る (撮影時に人を近づけない / keepout で潰す)
- 未知領域 = 自由 の問題は未解決 (読本 §3.6)
- dump 版と traj 版で AMCL の収束のしやすさを実走比較した結果はまだない (読本 §3.5「次の 1 実験」)

関連: [tools/README.md](../../tools/README.md) (引数一覧) / [docs/features/2026-08-17_glim_map_to_nav2.md](../features/2026-08-17_glim_map_to_nav2.md) (既製 pointcloud_to_2dmap の経緯) / [読本 map3d_to_nav2](../text/map3d_to_nav2/00_index.md)

---

← [第8章 LIO_SAM](08_lio_sam.md) | → [第10章 Nav2](10_nav2.md)
