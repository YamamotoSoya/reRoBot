<!-- claude: 運用手引き 第9章 テンプレ (2026-09-22)。本文は Claude が要約 (2026-09-24)。2026-10-02 base_link 基準・未観測マーク・keepout 生成を反映 (ユーザ依頼)。2026-10-10 §9.6 パラメータ・§9.7 帯ごとの一式を追加 (ユーザ依頼)。詳細は docs/text/map3d_to_nav2/ を正とする。 -->

# 第9章 map2d圧縮

GLIM で作った 3D 地図を、Nav2 (map_server / amcl) が読める **2D 占有格子 (map.pgm + map.yaml)** に落とす工程。
「ある高さ帯の点だけを真上から見て、点が多い画素を壁にする」だけだが、GLIM 地図は z がドリフトする (5号館で最大 +4.9 m) ので、
**高さ帯を世界座標ではなくセンサ基準で切る**のが要点。自作ツールが 2 つあり、点の出どころが違う。

| ツール (`tools/`) | 点の出どころ | 高さ帯の基準 | 必要なもの | 所要 | 向き |
|---|---|---|---|---|---|
| [glim_dump_to_2dmap](../../tools/glim_dump_to_2dmap/) | GLIM dump の submap 点群 (0.3 m 間引き済み) | **base_link (実機 /rfans/scan と同じ)**、または submap 原点 z からの相対 | dump のみ (numpy) | 数秒 | まず形を見る / bag が無い |
| [glim_traj_to_2dmap](../../tools/glim_traj_to_2dmap/) | **bag の生点群** (間引き前) | **base_link (実機 /rfans/scan と同じ)**、地面平面からの高さ、センサ座標 z | dump の `traj_lidar.txt` + 元 bag (ROS 環境) | 25〜45 s | **Nav2 用の本番地図 (推奨)** |

## 9.1 仕組み

- **dump 版**: submap ごとに `T_world_origin` (submap 原点の姿勢) が残っているので、点を世界座標に置いたあと原点の z を引いてから帯で切る (`--height_mode sensor`)。帯が車体と一緒に上下するので z ドリフトに強い。⚠️ **原点の z は LiDAR の高さではない** (09-18 5号館で床は原点基準 −0.65 m、LiDAR 基準 −0.81 m、submap ごとに ±0.2 m ぶれる)。`--height_mode base_link` (2026-10-02 追加) は submap 内スキャンの最適化後 LiDAR 姿勢 (`traj_lidar.txt` を stamp で引く) へ点を戻し、実機 /rfans/scan と同じ base_link 基準の高さで切る。ただし点は GLIM が間引いた後なので、画素の濃さが「壁の密度」ではなく「重なった submap 数」になり、区間によって壁が点線になる。
- **traj 版**: 点は bag の生スキャン、姿勢だけを GLIM の最適化後軌跡 `traj_lidar.txt` (TUM 形式、1 行 = 1 スキャン) から借りる。GLIM のループクロージングは点を書き換えず submap 姿勢を動かすだけなので、生スキャンをこの姿勢で置き直せば LC 反映済みかつ生密度の地図になる。`--deskew` で回転中のにじみも補正できる。

なぜ 2 つ残っているか、スキャンマッチング上の得失は [読本 第3章 §3.4〜3.5](../text/map3d_to_nav2/03_map_conversion.md) を参照。

## 9.2 実行手順

前提: **LC 後に保存した dump** を使う (`traj_lidar.txt` の終端 z が閉じているか確認。LC 前後で行数も時刻範囲も同じなので取り違えやすい)。
どちらも glim コンテナで実行する。<!-- claude: 2026-10-02 keepout 生成を既定の手順に組み込み (ユーザ依頼) -->
**既定で 1 コマンドで一式が出る**: 変換ツールが未観測マーク付きの地図を `<name>/raw/` に書き、続けて内部で
`map_to_keepout` を呼んで Nav2 規約どおりの `<name>/nav2/` (本体地図 `my_map.yaml`、走路の操作者跡を消した版) と `<name>/keep_out/` (keepout マスク) を書く。
出力の所有者は親ディレクトリ (ホストのユーザ) に合わせるので、そのまま GIMP で編集できる。

```
maps/2d/glim/<name>/
├─ raw/       map.pgm + map.yaml + map_params.yaml   変換結果 (未観測 = 灰 180、手を加えない原本)
├─ nav2/      map.pgm + map.yaml + my_map.yaml + map_params.yaml   Nav2 が読む本体地図
└─ keep_out/  keep_out.pgm + keep_out.yaml + keepout_params.yaml   未観測 = 進入禁止
```

`maps/` は全コンテナに `./maps:/workspace/maps` で mount されているので、ホストでは `reRoBot/maps/2d/glim/<name>/` に現れ、
main コンテナの Nav2 からも同じパスで読める。別設定で作り直すときは `<name>` を変えれば並べて残せる。
どちらのツールも同じ場所に **`map_params.yaml`** (実行コマンド全文・高さ基準・帯・距離の上限・しきい値など) を書くので、後から「どの高さで切ったか」を確認できる。

### dump 版 (速い・形の確認用)

```bash
G=/workspace/bags/glim/<bag名>_dump/<タグ>     # LC 後なら filtered (中の traj_lidar.txt も使う)
M=/workspace/maps/2d/glim/<name>

docker exec glim_env python3 /workspace/tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py $G $M \
  -r 0.10 --height_mode base_link --base_to_sensor_z 0.79396 --min_height 0.4 --max_height 1.7 --range_max 30 --mark_unknown
```

- 1 つ目の引数が dump、**2 つ目が地図一式の親 `<name>`** (`--map_only` を付けると、その場所に地図だけを書く従来動作)

- `-r 0.10` を推奨 (0.05 だと間引きのせいで壁が点線になる)
- 帯 (`--min/max_height`) は **base_link 基準 = 実機 bringup の `scan_min_height` / `scan_max_height` と同じ値** を書く。bringup の既定は 0.3 / 6.0 なので、地図を 0.3 / 1.5 で作ったら起動時に `scan_max_height:=1.5` を渡す (2026-10-08、[第14章 3.1](14_pointcloud_to_laserscan.md))
- `--range_max 30` は実機 AMCL (`laser_max_range` 30 m) と同じ距離上限。付けないと 30 m 以遠の地面が帯に入りノイズになる (base_link 基準では遠方ほど持ち上がる。原因未特定)
- `--base_to_sensor_z` は URDF `rfans_joint` の z (**bag の日付の値**: 〜09-30 は 0.80246、10-01 以降は 0.79396)。取付が flat (rpy=0) 前提
- 旧方式 `--height_mode sensor --min_height -0.25 --max_height 0.95` は submap 原点基準で、値が実機と一致しない (09-18 では地上 ≈0.40〜1.60 に相当)
- base_link 基準は 0.3 m 間引きの地面点が帯の下端に入りやすい。しきい値 (`--min/max_points_in_pix`、既定 2/5) と合わせて画像で確認する (比較: [features 2026-10-02](../features/2026-10-02_2dmap_threshold_unknown_keepout.md))

### traj 版 (本番地図、推奨設定 R)

```bash
docker exec -it glim_env bash
source /opt/ros/jazzy/setup.bash          # rosbag2_py が要る
T=/workspace/tools/glim_traj_to_2dmap/glim_traj_to_2dmap.py
B=/workspace/bags/raw/<bag名>
D=/workspace/bags/glim/<bag名>_dump/<タグ>      # 中の traj_lidar.txt を使う (LC 後なら filtered)
M=/workspace/maps/2d/glim/<name>                     # ← 地図一式の親 (raw/ nav2/ keep_out/ ができる)

# (任意) 床のセンサ座標 z を実測 (約 15 s)。ground / base_link モードでは不要
python3 $T $B $D /tmp/x --floor_probe --skip 10     # 地図は書かないので保存先はダミー

# 変換 + keepout (約 2 分)
python3 $T $B $D $M \
  -r 0.05 --height_frame ground --min_height 0.4 --max_height 1.7 \
  --range_max 30 --deskew --min_points_in_pix 4 --max_points_in_pix 12 --mark_unknown
```

- `--height_frame ground` は地面平面からの高さ。実機 `/rfans/scan` (pointcloud_to_laserscan) は車体基準 (base_link) なので、厳密に揃えるなら `--height_frame base_link --base_to_sensor_z <URDF rfans z>` (帯は実機 bringup の `scan_min_height` / `scan_max_height` と揃える)。平坦な 5号館 09-18 では両者の地図は占有一致 0.99
- Nav2 用に 0.10 m 格子にするなら `-r 0.10 --min_points_in_pix 8 --max_points_in_pix 24`
- `--height_frame sensor` は LiDAR 座標の z で切る (値 = 実機の値 − rfans 取付高)

## 9.3 未観測マークと keepout の仕組み (2026-10-02 追加)

`--mark_unknown` (既定の一式出力では自動的に有効) を付けると、センサから一度も見えていない画素を灰 180 (map_server で unknown) で塗る (各センサ位置から方位ごとに「帯内の最近点」と「全点の最遠点」の手前までを空きにする)。traj 版はスキャンごとの実姿勢で厳密、dump 版は submap 内スキャン位置からの近似 (建物内などに空きが漏れることがある)。

`map_to_keepout` (一式出力のとき変換ツールが内部で呼ぶ。単体でも使える) はその地図から keepout マスクを作る (未観測だけを禁止。壁は static_layer + inflation が担うので入れない。`--include_walls` で壁も入る)。

- `--min_unknown_area` (既定 1.0) は、この面積 [m²] 未満の未観測の塊を keepout にしない (空き域内の小穴で通路を塞がない)。`--path_clear_radius` (既定 0.4) は軌跡周りの除外半径。どちらも変換ツールにそのまま渡せる
- `--traj` を渡すと走行軌跡から 0.4 m 以内を keepout から外し、`--clean_map` はその範囲の壁を白に戻した本体地図 (`map.pgm` / `map.yaml` / `my_map.yaml`) を `nav2/` に書く → 規約名のまま 9.4 で `use_keepout:=true` で起動できる
- 理由: ロボットの後ろを歩く操作者が高さ帯に入り、**走路そのものが壁として焼き付く** (09-18 traj 版で軌跡上の 94% が占有。従来は GIMP で手消ししていた)
- 芝生・車道などの進入禁止帯は、出来た `keep_out.pgm` に手で黒を描き足す

## 9.4 Nav2 に渡す

Nav2 の地図規約は `<map_dir>/nav2/map.yaml` + `<map_dir>/keep_out/keep_out.yaml`。9.2 の手順ならそのまま揃っている。

```bash
# 起動 (main コンテナ)。keepout 込み (use_keepout は既定 true)。AMCL は /rfans/scan を読む
ros2 launch rerobot_bringup nav_amcl.launch.py map_dir:=/workspace/maps/2d/glim/<name>
# keepout なしで試すなら
ros2 launch rerobot_bringup nav_amcl.launch.py map_dir:=/workspace/maps/2d/glim/<name> use_keepout:=false
```

`--map_only` で作った地図や 2026-10-02 以前の地図 (`nav2/map.yaml` だけ) は `map_yaml:=/workspace/maps/2d/glim/<name>/nav2/map.yaml use_keepout:=false` で直接指定する。

3D 地図由来なので、amcl の入力は 2D LiDAR ではなく R-Fans 点群を 2D に落とした `/rfans/scan` (bringup に同梱、`nav_amcl` の既定)。⚠️ 互換の `nav2.launch.py` は既定が urg (`/urg_front/scan`) なので、GLIM 地図で使うなら `scan_topic:=/rfans/scan` を付ける (2026-10-08)。→ [第10章 Nav2](10_nav2.md) / [第11章 amcl](11_amcl.md)

## 9.5 注意

- 生スキャン版は停車・低速区間が濃くなる。歩行者・後ろを歩く操作者が壁になる → 9.3 の `--clean_map` (軌跡周りだけ) か GIMP で消す。横を歩いた区間の跡は残り得る
- 未知領域 = 自由 の問題は 9.3 の `--mark_unknown` で対処できる (2026-10-02)。濃淡の灰 (lo〜hi 点の画素) も map_server では unknown になるので、planner の `allow_unknown: false` ではなく keepout で禁止している
- dump 版と traj 版で AMCL の収束のしやすさを実走比較した結果はまだない (読本 §3.5「次の 1 実験」)

## 9.6 パラメータの意味と決め方 <!-- claude: 2026-10-10 追加 (ユーザ依頼)。既定値は各ツールの --help (実物) から転記 -->

### 9.6.1 原則: 高さは「base_link からの高さ」で揃える (2026-10-02 決定)

**経緯**: 以前は 2 つのツールで「どこからの高さか」が違っていた。
- dump 版の旧方式 `--height_mode sensor` は submap 原点の z が基準
- traj 版は LiDAR の座標や地面平面が基準

そのため、同じ数字を書いても切る高さがずれた。10-02 には換算を誤り、「dump 版は traj 版より 0.26 m 高い」と取り違えて比較図や地図を作り直している。さらに、実機の `/rfans/scan` (base_link 基準) と地図の帯が一致しているかも確かめられなかった。

**対策 (10-02)**: 両方のツールに **base_link 基準** のモードを追加した。

```
高さの基準をそろえる
├── dump 版 : --height_mode  base_link --base_to_sensor_z <URDF rfans_joint z>
├── traj 版 : --height_frame base_link --base_to_sensor_z <URDF rfans_joint z>
├── 実機    : /rfans/scan (pointcloud_to_laserscan, target_frame base_link) ← もともとこの基準
└── → 3 つとも「base_link からの高さ」なので、同じ数字 = 同じ高さ
      (09-18 5号館で、床は base_link 基準の中央値 +0.015 m)
```

**守る整合条件** (地図を作り直したら必ず見直す。→ [第14章 3.1](14_pointcloud_to_laserscan.md)):

| 地図 | 帯を合わせる相手 (bringup の引数) |
|---|---|
| 経路計画用 `/map` (`nav2/`) | `scan_min_height` / `scan_max_height` (`/rfans/scan`、costmap と AMCL の既定入力) |
| 自己位置推定専用 (`loc_map_yaml` → `/map_loc`) | `scan_all_min_height` / `scan_all_max_height` (`/rfans/scan_all`) |

`--height_frame ground` (traj 版、地面平面からの高さ) は、取付ピッチ由来の遠方の地面の筋を吸収する。ただし基準が base_link と違うので、比較や実機との整合を取るときは使わない。平坦な 5号館 09-18 では、ground と base_link の地図は占有一致 0.99 だった。

### 9.6.2 両ツール共通のパラメータ

| 引数 | 既定 (dump / traj) | 意味と決め方 |
|---|---|---|
| `-r` | 0.05 / 0.05 | 画素の大きさ [m]。**dump 版は 0.10 推奨** — 点が 0.3 m 間引き済みで、0.05 だと壁が点線になる (09-10 掃引)。traj 版は 0.05 (Nav2 用に 0.10 なら下のしきい値も上げる) |
| `--height_mode` (dump) / `--height_frame` (traj) | sensor / sensor | 高さの基準。**`base_link` を使う** (9.6.1) |
| `--base_to_sensor_z` | なし | base_link → rfans の高さ [m] = URDF `rfans_joint` の z。**bag の日付の値**: 〜09-30 は 0.80246、10-01 以降は 0.79396。取付 rpy=0 前提 |
| `--min_height` / `--max_height` | −0.5, 0.7 / −0.45, 0.75 | 切り出す高さ帯 [m]。base_link 基準なら地上高そのまま。既定値は旧方式向けなので**必ず指定する**。9.6.1 の整合条件に合わせる |
| `--range_max` | 0 = 無制限 / 40 | この水平距離 [m] より遠い点を捨てる。**30 を付ける** (実機 AMCL の `laser_max_range` 30 m と同じ)。base_link 基準では遠方の地面が持ち上がって帯に入る (09-18: 10〜20 m で +0.30 m、30 m 以遠の点はノイズ率 31%) |
| `--min_points_in_pix` / `--max_points_in_pix` | 2 / 5 | 点数 → 濃さの変換 (lo / hi)。画素値 = 255 − 255·(n − lo)/(hi − lo)。map.yaml は occupied 0.5 / free 0.2 で、画素値は切り捨てで 8 bit 化されるので、**n ≥ (lo + hi)/2 点で壁 (黒)** (正確には n > lo + 0.498(hi − lo))、n ≤ lo + 0.2(hi − lo) で空き (白)、その間は灰 = unknown。採用値の記録は 9.6.5 |
| `--mark_unknown` | off (一式出力では自動 on) | 一度も見えていない画素を灰 (`--unknown_value` 既定 180) で塗る。`--map_only` のときは自分で付ける |
| `--map_only` | off | 付けると「地図だけを出力先に直接書く」。付けないと出力先を地図一式の親とみなし、`raw/` (原本)・`nav2/`・`keep_out/` を書く |
| `--min_unknown_area` / `--path_clear_radius` | 1.0 / 0.4 | 一式出力時の keepout 用 (9.3)。小さい未観測の塊 [m²] は禁止しない / 軌跡からこの半径 [m] を禁止と壁から外す |
| `--center` | world | 地図の中心。world = 世界原点。auto = 点群 (traj は軌跡) の範囲の中心 |
| `--map_width` / `--map_height` | 0 = 自動 | 画素数。自動だと帯ごとに大きさ・origin が変わるが、座標は同じなので重ねて使える。手で奇数を指定すると半画素ずれる |

### 9.6.3 dump 版だけのパラメータ

| 引数 | 既定 | 意味 |
|---|---|---|
| `--traj` | `<dump>/traj_lidar.txt` | base_link 基準・`--range_max`・未観測マークで使うスキャン位置 (最適化後の軌跡) |
| `--raw_name` | raw | 一式出力の原本ディレクトリ名。`raw_dump/0.2-1.7` のように入れ子も可 (2026-10-10 追加) |
| `--export_pcd` | なし | 全点 (高さで切る前) を世界座標の PCD に書き出す → 3D 自己位置推定の地図 (`maps/3d/<name>/map.pcd`)。出力先ディレクトリは先に作っておく |
| `--free_range_max` / `--view_stride` / `--free_bins` | 30 / 10 / 720 | 未観測マークの空きを伸ばす最大距離 [m] / 視点に使うスキャンの間引き / 方位分割数 (720 = 0.5°) |

### 9.6.4 traj 版だけのパラメータ

| 引数 | 既定 | 意味 |
|---|---|---|
| `--topic` | `/rfans_driver/rfans_points` | bag から読む点群 |
| `--deskew` | off | 点ごとの `time` で前後のスキャン姿勢を補間し、動きを補正する。**付ける** (未補正だと回転区間で占有 IoU 0.78) |
| `--range_min` | 0.5 | これより近い点 (車体・無効点) を捨てる [m] |
| `--stamp_tol` | 0.02 | bag の stamp と軌跡の stamp の許容差 [s] |
| `--ground_z` / `--ground_range` | 自動 / 12 | ground モード用 (地面探索の初期値 / 平面当てに使う距離) |
| `--floor_probe` | off | 地図を書かず、床の高さだけ出す (sensor モードで帯を決めるとき用。base_link では不要) |
| `--skip` / `--save_counts` / `--save_free` | 1 / なし / なし | 試験・解析用 (N スキャンに 1 つ / 点数配列や空きマスクを .npy で保存) |

### 9.6.5 点数しきい値の採用記録 (地図の見た目で調整する)

| ツール・解像度 | lo / hi | 壁になる点数 (目安) | どこで使ったか |
|---|---|---|---|
| dump 0.10 | 0 / 2 | 1 点以上 | 10-03 つくば `2026-10-03_1126_tsukuba_dumpbase` (nav2 本体) |
| dump 0.10 | 1 / 3 | 2 点以上 | 10-02 比較: traj 版の壁を拾えた率 0.55 → 0.87、誤検出 0.10 → 0.20 ([features 10-02](../features/2026-10-02_2dmap_threshold_unknown_keepout.md) §1) |
| dump 0.10 | 2 / 5 | 4 点以上 | 既定。10-03 つくば `loc_*`、09-18 5号館 dumpbase の再生成 |
| traj 0.05 | 4 / 12 | 8 点以上 | 09-14 比較の推奨 |
| traj 0.05 | 12 / 20 | 16 点以上 | 10-02 比較: 壁・縁石・幹は残り、芝や植え込み内部の塗りが消える (24 点以上で樹冠が欠け始める) |
| traj 0.05 | 44 / 52 | 48 点以上 | 09-18 5号館 trajbase (ground 0.4〜1.7) |
| traj 0.10 | 8 / 24 | 16 点以上 | 9.2 の「Nav2 用に 0.10 m 格子にするなら」 |

点数は、帯の幅・停車時間・走行速度で変わる (帯を広げる、ゆっくり走るほど 1 画素の点が増える)。決めた値を他の場所や帯に流用するときは、必ず画像で確認する。

## 9.7 高さ帯ごとに作る (2D 自己位置推定・3D 自己位置推定と共用の一式) <!-- claude: 2026-10-10 追加 (ユーザ依頼) -->

1 つの dump から、帯の違う 2D 地図を dump 版・traj 版でそれぞれ作り、3D 自己位置推定用の 3D 地図と軌跡も同時にそろえる。**両ツールとも base_link 基準・同じ帯**にするので、dump 版と traj 版の違いは点の出どころだけになる (9.6.1)。

```
maps/2d/glim/<NAME>/
├── raw_dump/<帯>/   dump 版の原本。最初の帯から nav2/ と keep_out/ を作る
├── raw_traj/<帯>/   traj 版の原本 (濃さ均一) — 2D 自己位置推定 (loc_map_yaml) の地図候補
├── nav2/            経路計画用 /map
└── keep_out/
maps/3d/<NAME>/
├── map.pcd          3D 自己位置推定の地図 (nav_lidar3d)
└── traj_lidar.txt   初期位置の z を引く GLIM 軌跡
```

```bash
# ホストで: docker exec -it glim_env bash   (以下はコンテナ内で実行)
source /opt/ros/jazzy/setup.bash          # traj 版が rosbag2_py を使う
BAG=/workspace/bags/raw/<bag名>
DUMP=/workspace/bags/glim/<bag名>_dump/<タグ>   # LC したら保存し直した dump
NAME=<name>
Z=0.79396                                       # URDF rfans_joint z (〜09-30 の bag は 0.80246)
M=/workspace/maps/2d/glim/$NAME

mkdir -p /workspace/maps/3d/$NAME

# dump 版・最初の帯: raw_dump/0.2-1.7/ → nav2/ keep_out/ + 3D 地図 (map.pcd)
python3 /workspace/tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py $DUMP $M --raw_name raw_dump/0.2-1.7 \
  -r 0.10 --height_mode base_link --base_to_sensor_z $Z \
  --min_height 0.2 --max_height 1.7 --range_max 30 \
  --min_points_in_pix 0 --max_points_in_pix 2 \
  --export_pcd /workspace/maps/3d/$NAME/map.pcd

# dump 版・追加の帯 (地図だけ)
for BAND in 0.3-3.0 0.3-6.0; do
  python3 /workspace/tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py $DUMP $M/raw_dump/$BAND --map_only \
    -r 0.10 --height_mode base_link --base_to_sensor_z $Z \
    --min_height ${BAND%-*} --max_height ${BAND#*-} --range_max 30 \
    --min_points_in_pix 0 --max_points_in_pix 2 --mark_unknown
done

# traj 版・同じ帯 (地図だけ。帯 1 つごとに bag を読み直す)
for BAND in 0.2-1.7 0.3-3.0 0.3-6.0; do
  python3 /workspace/tools/glim_traj_to_2dmap/glim_traj_to_2dmap.py $BAG $DUMP $M/raw_traj/$BAND --map_only \
    -r 0.05 --height_frame base_link --base_to_sensor_z $Z \
    --min_height ${BAND%-*} --max_height ${BAND#*-} --range_max 30 --deskew \
    --min_points_in_pix 12 --max_points_in_pix 20 --mark_unknown
done

# 初期位置の z を引く軌跡 (3D 自己位置推定用)
cp $DUMP/traj_lidar.txt /workspace/maps/3d/$NAME/
```

- 帯は `下限-上限` の 1 つの文字列で書く。`${BAND%-*}` が下限、`${BAND#*-}` が上限を取り出すので、ディレクトリ名と切った高さが必ず一致する。帯を増やすときは `for` の行に書き足す
- nav2/ の元にする帯を変えるときは、最初の 1 本の `--raw_name raw_dump/<帯>` と `--min_height` / `--max_height` の 3 か所を一緒に変える
- しきい値は、dump 版 0/2 (10-03 つくば) と traj 版 12/20 (10-02 比較) を起点にしている。広い帯 (0.3-6.0 など) では点が増えるので、画像を見て上げる (9.6.5)
- 各帯のディレクトリに `map_params.yaml` (実行コマンド全文・帯の値) が残る
- 使うとき:
  - 3D 自己位置推定: `ros2 launch rerobot_bringup nav_lidar3d.launch.py map_name:=<NAME>` → [features 2026-10-10](../features/2026-10-10_lidar_localization_3d.md)
  - 経路計画用 `/map` の帯 (例 0.2-1.7) は、bringup の `scan_min_height:=0.2 scan_max_height:=1.7` と揃える
- 試験済みの範囲 (2026-10-10、08-31 5号館 dump): dump 版の入れ子 `--raw_name raw_dump/0.2-1.7` + 追加の帯 `--map_only` + `--export_pcd` (406 万点)。traj 版の base_link モードは 10-02 追加の既存機能で、この組み合わせでは未実行

関連: [tools/README.md](../../tools/README.md) (引数一覧) / [docs/features/2026-08-17_glim_map_to_nav2.md](../features/2026-08-17_glim_map_to_nav2.md) (既製 pointcloud_to_2dmap の経緯) / [読本 map3d_to_nav2](../text/map3d_to_nav2/00_index.md)

---

← [第8章 LIO_SAM](08_lio_sam.md) | → [第10章 Nav2](10_nav2.md)
