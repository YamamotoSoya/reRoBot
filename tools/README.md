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

<!-- claude: 2026-10-02 追加 -->
> **生成条件の記録**: glim_dump_to_2dmap / glim_traj_to_2dmap は地図と同じ出力先に `map_params.yaml` を書く
> (実行コマンド全文・高さ基準・帯・距離の上限・しきい値・全引数・結果の要約)。map_to_keepout は `keep_out/keepout_params.yaml` を書き、
> `--clean_map` 先には元の `map_params.yaml` を引き継いで掃除内容を追記する。
>
> **1 コマンドで Nav2 用一式 (既定)**: 両変換ツールは既定で dest_dir を地図一式の親として
> `raw/` (未観測マーク付き原本)・`nav2/` (走路掃除済み本体 + `my_map.yaml`)・`keep_out/` を書く (内部で map_to_keepout を呼ぶ。
> `--min_unknown_area` / `--path_clear_radius` もそのまま渡せる)。root (コンテナ) で実行しても出力の所有者は親ディレクトリに合わせる。
> 地図だけを dest_dir に書く従来動作は `--map_only` (下の例はすべて従来動作のまま `--map_only` 付き)。

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
  /workspace/bags/glim/2026-08-14_0919_5goukan_dump/manual_lc \
  /workspace/maps/2d/glim/<name>/nav2 \
  -r 0.05 --map_width 6144 --map_height 6144 \
  --height_mode sensor --min_height -0.25 --max_height 0.95 --map_only
```

- `--height_mode base_link --base_to_sensor_z <URDF rfans_joint z> --min_height 0.3 --max_height 1.5`
  (2026-10-02 追加) で**実機 /scan (`rfans_scan.launch.py`, target_frame base_link) と同じ基準・同じ値**で切れる。
  submap 内スキャンの stamp で `traj_lidar.txt` (最適化後 LiDAR 姿勢) を引き、各点を水平最寄りのスキャンの
  LiDAR 座標へ戻して z + 取付高を高さとする (取付 rpy=0 前提)。取付高は 〜09-30 の bag 0.80246 / 10-01 以降 0.79396。
  09-18 5号館で床 (base_link 基準) は中央値 +0.015 m を確認
- `--range_max 30` (2026-10-02 追加、既定 0 = 無制限): submap 内の最寄りスキャン位置から水平 30 m を超える点を捨てる
  (実機 rfans_scan・traj ツールと同じ)。base_link 基準では遠方の地面が距離に比例して持ち上がって帯に入る
  (09-18: 10〜20 m で +0.30 m、40〜60 m で +1.55 m。原因は未特定 — 車体ピッチ / R-Fans 縦角誤差 / GLIM 姿勢 /
  最寄りスキャン近似) ので、base_link では付けるのが前提。30 m 以上の点のノイズ率は 31%、未満は 1〜2%
- スキャン位置 (base_link / `--range_max` / `--mark_unknown` の視点) は `traj_lidar.txt` (最適化後) を stamp で引く。
  data.txt の `T_world_lidar` は最適化前で、09-18 では水平に中央値 1.7 m・最大 6.1 m ずれる (traj が無い dump では警告して data.txt を使う)
- ⚠️ sensor 帯の基準は **submap 原点 (`T_world_origin`) の z** で、LiDAR の高さとは一致しない
  (2026-10-02 実測: 09-18 5号館 dump で床は原点基準 中央値 −0.65 m、submap ごとに ±0.2 m ばらつく。
  同じ bag の traj ツール (LiDAR 基準) の床は −0.81 m)。帯を地上高で決めるときは dump の点で床を測る
- `--center world` (既定) で既製ツールと同じ world (0,0) 中心。`--center auto` は
  点群 bbox 中心 + サイズ自動決定 (既製ツールの「原点中心固定で巨大地図になる」制約の回避)
- `--export_pcd <path>` で全点マージの世界座標 PCD も書ける (既製ツールとの比較・
  3D ローカライザ地図用)
- `--mark_unknown` (2026-10-02 追加) で**未観測域を灰 (map_server の unknown) で塗る**。
  submap 内の各スキャン姿勢 (`data.txt` の `T_world_lidar`、`--view_stride` 個に 1 個) を視点にし、
  方位 0.5° ごとに「帯内の点 (障害物) の最近距離」と「全点の最遠距離」の小さい方までを空きとする
  (`--free_range_max` 既定 30 m)。dump には点↔スキャンの対応が無いので、視点は近似 (submap 長 ≈ 4 m)。
  建物内などに空きが漏れることがある。灰の値は `--unknown_value` (既定 180 = 占有確率 0.29。
  yaml の 0.5/0.2 では 128〜204 が unknown。定番の 205 は 0.196 で free になるので不可)
- 濃度変換 (`--min/max_points_in_pix`) と yaml 形式は既製ツール互換。画像は PGM
  (map_server は png/pgm どちらも可)
- **解像度は `-r 0.10` を推奨** (2026-09-10 掃引)。dump の点は GLIM が 0.3 m ボクセルで
  間引いており、5 cm 画素では 1 submap から 1 画素に 1〜2 点しか落ちず点数しきい値が
  成立しない (壁が点線状・submap 間隔が開く区間で消える)。0.10 で右上 (最遠角) の占有
  17 倍・ノイズ増なし。機構と掃引表は読本 `docs/text/map3d_to_nav2/03_map_conversion.md` §3.4
- 絶対 z vs センサ相対の比較実測 (5号館 08-14 LC 地図、z ドリフト +4.9 m):
  `docs/text/map3d_to_nav2/img/2026-08-20_compare_abs_vs_sensor.png` (解説は読本 §3.4。
  元データ・両方式の map 出力は git 管理外の `bags/exp/2026-09-14_2dmap_compare/`)
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
B=/workspace/bags/raw/2026-08-14_0919_5goukan
D=/workspace/bags/glim/2026-08-14_0919_5goukan_dump/manual_lc   # dump dir (中の traj_lidar.txt を使う)

# 1) 床のセンサ座標 z を実測 (10 スキャンに 1 つ、約 15 s)。5号館 08-14 は -0.65 → 帯 -0.35〜+0.85
python3 $T $B $D /tmp/x --floor_probe --skip 10

# 2) 変換 (全 6,989 スキャンで約 2.5 分)
python3 $T $B $D /workspace/maps/2d/glim/<name>/nav2 \
  -r 0.05 --map_width 6144 --map_height 6144 \
  --height_frame sensor --min_height -0.35 --max_height 0.85 \
  --min_points_in_pix 2 --max_points_in_pix 5 --map_only
```

| 引数 | 既定 | 意味 |
|---|---|---|
| `bag_dir` / `traj` / `dest_dir` | — | bag ディレクトリ / `traj_lidar.txt` か dump ディレクトリ / 出力先 |
| `--topic` | `/rfans_driver/rfans_points` | PointCloud2 トピック |
| `--height_frame` | sensor | sensor = センサ座標 z で切る (値は LiDAR 基準。実機と同じ値で書くなら base_link) / world = 世界 z − 姿勢 z (glim_dump_to_2dmap の sensor と同じ意味) / base_link = センサ z + `--base_to_sensor_z` (実機 /scan と同じ基準。2026-10-02 追加) |
| `--min/max_height` | −0.45 / +0.75 | スライス帯。`--floor_probe` の出力 (床 +0.3〜+1.5) で決める |
| `--range_min/max` | 0.5 / 40 | この範囲外の点を捨てる (車体・無効点・遠方ノイズ) |
| `-r`, `--map_width/height`, `--center`, `--min/max_points_in_pix` | 0.05, 0 (自動), world, 2/5 | glim_dump_to_2dmap と同じ画素系・濃度変換 |
| `--stamp_tol` | 0.02 s | header stamp と traj stamp の許容差 (5号館 bag は差 0.00 ms で完全一致) |
| `--deskew` | off | 点ごとの `time` で前後スキャン姿勢を slerp 補間して動き補正 |
| `--skip N` | 1 | N スキャンに 1 つだけ使う (試験用) |
| `--floor_probe` | off | 地図を書かず床 z の最頻値だけ出す |
| `--save_counts <npy>` | なし | 画素点数配列を保存 (しきい値・解像度の派生を再実行なしで作る用) |
| `--mark_unknown` | off | 未観測域を `--unknown_value` (既定 180) で塗る。スキャンごとに実姿勢から、方位 0.5° (`--free_bins`) ごとに帯内の最近点 / 全点の最遠点の手前までを空きにする (2026-10-02 追加。処理時間 約 2 倍) |
| `--save_free <npy>` | なし | 空き (観測済み) 画素マスクを保存 |

- **推奨引数 (2026-09-14 比較の結論)**: `--height_frame ground --min_height 0.3 --max_height 1.5
  --range_max 30 --deskew --min_points_in_pix 4 --max_points_in_pix 12`。ground は取付ピッチ
  (5号館 bag で実測 ≈1.2°) 由来の遠方地面の筋を吸収し、deskew は GLIM 自身の地図 (deskew 済み) と
  点配置を揃える (未補正だと回転区間で占有 IoU 0.78)。地上高指定なので /scan 側の帯と同じ単位
- 比較結果 (5号館 08-14 LC 地図): 右上/始点の占有比が submap 版 0.10 (r=0.05) / 0.37 (r=0.10) →
  生スキャン版 1.2 で場所依存の薄化が解消。ただし「実機 /scan との一致率」は hold-out 評価で
  submap 版 r=0.10 と同等 (0.79 vs 0.77) で、AMCL 実走での優劣は未検証。詳細・図・残る代替仮説は
  読本 `docs/text/map3d_to_nav2/03_map_conversion.md` §3.5
- ⚠️ dump は **LC 後に保存したもの**を渡す (`traj_lidar.txt` の終端 z が閉じているか確認。
  5号館は `glim/2026-08-14_0919_5goukan_dump/manual_lc/` が LC 後、同 `default/` は LC 前)
- 処理時間: 全 6,989 スキャン (5.8 GB mcap) で 25〜45 s (glim_env、deskew 込みで 45 s)

## map_to_keepout (自作, 2026-10-02)

<!-- claude: 2026-10-02 追加 -->
`--mark_unknown` 付きで作った 2D 地図 (上の 2 ツールの出力) から Nav2 KeepoutFilter 用マスク
(`keep_out.pgm` + `keep_out.yaml`、白黒 2 値) を作る。keepout = **未観測 (灰 180)** から、
**走行軌跡の周り (`--path_clear_radius` 既定 0.4 m) を除いたもの**。

軌跡を除く理由: ロボットの約 1.6 m 後ろを歩く操作者が毎スキャン高さ帯に入り、**走路そのものが壁として
焼き付く** (09-18 5号館 traj 版で軌跡上の 94% が占有。従来は GIMP で手消ししていた)。
`--clean_map <dir>` で同じ範囲を白に戻した本体地図 (map.pgm / map.yaml / my_map.yaml) も書く。

```bash
docker exec glim_env python3 /workspace/tools/map_to_keepout/map_to_keepout.py \
  <map.yaml> /workspace/maps/2d/glim/<name>/keep_out \
  --min_unknown_area 1.0 --traj <dump>/traj_lidar.txt --clean_map /workspace/maps/2d/glim/<name>/nav2
# → nav2.launch.py map_dir:=/workspace/maps/2d/glim/<name> でそのまま読める
```

- `--min_unknown_area` [m²] 未満の未観測の塊は keepout にしない (空き域内の小穴が通路を塞ぐのを防ぐ)
- 壁は既定で keepout に入れない (static_layer + inflation が担う。keepout の致命セルは inflation されない)。`--include_walls` で入れる
- 濃淡の灰 (lo〜hi 点) は keepout にしない。`allow_unknown: false` だとこれも通行不可になるので、未観測の禁止は keepout 側で行う
- 操作者が真後ろでなく横を歩いた区間は跡が残りうる (除去は軌跡半径内のみ)

## 99-wt901.rules (2026-09-23)

<!-- claude: 2026-09-23 追加 -->
比較用 IMU WITmotion WT901C-TTL の USB-TTL 変換 (Prolific 067b:23a3) に udev 安定名
`/dev/ttyUSB-wt901` を与える rule。導入コマンドはファイル冒頭、手順全体は
`docs/features/2026-09-23_wt901c_comparison_imu.md`。センサ本体の設定 (115200 / 200 Hz) は
Windows 公式ソフトで行った (Linux 用の設定スクリプトは検証後に不要となり削除)。

## amcl_compare (自作, 2026-10-01)

<!-- claude: 2026-10-01 追加 -->
AMCL を名前空間付きで多数並走させ、3D→2D スキャン変換の選び方 (最近点 / 最遠点 / n 番目 / 全点) や
AMCL パラメータを同じ bag 入力で比べるツール一式。AMCL は乱数で結果が揺れるため、反復 run の
失敗頻度で比べる。詳細は `tools/amcl_compare/README.md`。

## waypoint_editor (自作, 2026-10-02)

<!-- claude: 2026-10-02 追加 -->
屋外に出る前に、2D 地図 (map_server 形式 yaml) の上で waypoint を机上で打つ GUI。保存形式は
RViz Nav2 パネルの **Load WPs** と同じ (`waypoints: {waypointN: {pose: [x,y,z], orientation: [w,x,y,z]}}`
— orientation は **w が先頭**。nav2_panel.cpp jazzy で確認)。yaw は既定で次の点の方向 (Ctrl+ドラッグで点ごとに手動指定可、`yaw_manual: true` で保存。NavigateThroughPoses で効くのは最後の点の向きだけ)。
点の色で置き場所を判定 (緑 = 空き / 橙 = 未観測 / 赤 = 壁・keepout — 赤は経路計算失敗で
NavigateThroughPoses 全体が止まる)。keepout は `<map_dir>/keep_out/keep_out.yaml` を自動検出。
依存 (matplotlib TkAgg) は **rerobot_env** にある (ホストには無い)。

```bash
# rerobot_env 内 (xhost +local:docker 済み)
python3 /workspace/tools/waypoint_editor/waypoint_editor.py \
    /workspace/maps/2d/glim/<map>/nav2/my_map.yaml /workspace/maps/2d/glim/<map>/waypoints/course.yaml \
    [--traj <dump>/traj_lidar.txt]      # 走行軌跡を重ねる
# 操作: 左クリック=追加 / ドラッグ=移動 / Shift+左=区間に挿入 / 右クリック=削除 / Ctrl+ドラッグ=向き指定 / a=向き自動 / p=停止点 ON/OFF / u=戻す / s=保存 / q=終了
# GUI なし: --check (判定表、要確認があれば exit 1) / --render out.png
```

当日: nav2 起動 → AMCL 初期位置合わせ → Nav2 パネル「Waypoint / Nav Through Poses Mode」→ **Load WPs** →
「Start Nav Through Poses」。waypoint は作成時の地図の map 座標に紐づく (地図を作り直したら打ち直し)。

<!-- claude: 2026-10-04 追加 -->
**停止点 (一時停止 → キーで再開)**: エディタで点の上にカーソルを置いて `p` を押すと停止点になり (赤い四角)、
`stop: true` 付きで保存される。停止点で止めたいときは RViz パネルの Start ではなく (パネルは stop を読まずに素通りする)
`waypoint_runner.py` で走らせる。コースを停止点で区間に分け、区間ごとに NavigateThroughPoses を送る
(停止点の向き = YAML の orientation まで旋回して止まる)。

```bash
# rerobot_env 内 (nav2.launch.py 起動 + AMCL 初期位置合わせ済み。キー入力のため -it 必須)
docker exec -it rerobot_env bash -c "source /workspace/install/setup.bash && \
  ros2 run rerobot_bringup waypoint_runner.py /workspace/maps/2d/glim/<map>/waypoints/course.yaml"
# キー: g=開始/再開 (--resume-key で変更) / q=終了 (待機中) / p=一時停止 (走行中、残りの点から再開) / Ctrl-C=ゴールをキャンセルして終了
# 区間失敗 (ABORTED) も待機に入り、g で残りの点から再送。--start N で waypointN から始める
```
