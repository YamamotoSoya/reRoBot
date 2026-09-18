<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# glim_traj_to_2dmap — bag 生点群 × GLIM 最適化軌跡による 2D 地図変換

- 日付: 2026-09-14
- 対象パッケージ: `tools/glim_traj_to_2dmap/glim_traj_to_2dmap.py` (ROS ws 外の単体 Python ツール、colcon 非依存)
- 対象 ROS: ROS 2 Jazzy (`glim_env` コンテナで実行。rosbag2_py が使えれば `rerobot_env` でも可 — ただし tools/ は glim_env にのみ mount)
- 関連文書: 読本 `../text/map3d_to_nav2/03_map_conversion.md` §3.4 (submap 版の機構解剖)・§3.5 (本ツール)、先行機能 `2026-08-17_glim_map_to_nav2.md`、z ドリフト側の課題 `../issue/2026-09-10_glim_z_drift_not_vangle.md`

## 1. 目的・概要

GLIM の 3D 地図から Nav2 用 2D 占有格子を作る第 3 の方式。先行 2 方式の限界を埋める:

- 既製 `pointcloud_to_2dmap` (絶対 z スライス) は z ドリフト地図で遠方の壁が帯から外れる。
- 自作 `glim_dump_to_2dmap` (submap センサ相対スライス, 08-20) は帯の問題を解いたが、
  読む点が GLIM の 0.3 m ボクセル間引き後の submap 点群なので、5 cm 画素の濃さが
  「壁の物理密度」ではなく「重なった submap 数」になり、submap 間隔が開く区間 (5号館の
  最遠角) で壁が点線状に薄くなる (09-10 に解剖、読本 §3.4)。

本ツールは点の出どころを **bag の生スキャン (間引き前、1 回転 ≈ 3 万点)** に戻し、姿勢だけを
GLIM dump 根元の **`traj_lidar.txt` (大域最適化後 = ループクロージング反映済み、1 行 = 1 スキャン)**
から借りて再投影する。高さ帯は既定でセンサ座標の z で切るため、実機の /scan
(`rfans_scan.launch.py` = pointcloud_to_laserscan、base_link 相対) と同じ切り方になる。

実装した機能:
- bag (mcap) の PointCloud2 を逐次読み、header stamp で `traj_lidar.txt` の姿勢を引いて世界座標へ再投影 (GLIM 再実行なし)
- 高さ帯の基準を `--height_frame sensor` (センサ座標 z、既定) / `world` (世界 z − 姿勢 z) で選択
- `--floor_probe` (床のセンサ座標 z を実測)、`--deskew` (点ごとの `time` で前後姿勢を slerp 補間)、`--save_counts` (画素点数配列の保存 → しきい値・解像度の派生を再実行なしで作る)
- 出力は既製ツール互換の map.pgm + map.yaml (画素系・濃度変換は glim_dump_to_2dmap と同一)

スコープ外 (意図的に未対応):
- GLIM 側の再設定 (`submap_downsample_resolution` を細かくして dump し直す) — 本ツールで生密度が得られるため不要と判断。地図精度 (姿勢) は GLIM のまま
- 未知領域の判定 (レイキャスト) — 生スキャンには視点があるので原理的には可能だが、既製ツール互換の「点数→自由/未知/占有」に留めた。Nav2 側は keepout で補う運用 (先行 feature doc §6)
- 姿勢の傾き (z ドーム斜面のピッチ誤差) の補正 — 地図変換ではなく SLAM 側の課題 (`../issue/2026-09-10_glim_z_drift_not_vangle.md`)

## 2. 設計の勘所

### 2.1 姿勢は `traj_lidar.txt` から借り、GLIM は再実行しない

実測: GLIM のループクロージングは点を書き換えず、submap の姿勢 (T_world_origin) を剛体で
動かすだけ。dump 保存時 (`global_mapping.cpp` の save) に「最適化後 submap 姿勢 ×
submap 原点から見た各スキャンの相対姿勢」を全スキャン分計算し `traj_lidar.txt` に書く。
5号館 LC dump では終端スキャンの z が `odom_lidar.txt` −4.50 m / `traj_lidar.txt` −0.02 m で、
traj 側だけループが閉じている。bag の header stamp と traj の stamp は差 0.00 ms で完全一致
(GLIM はフレーム時刻に header stamp をそのまま使う)。

判断: 生スキャンをこの姿勢で置き直せば、GLIM が自分の (間引き済み) 地図を世界座標に置く
計算と同じ形になる。submap 内部の相対姿勢はオドメトリのままだが、それは GLIM 自身の地図も
同じなので精度は落ちない。

却下した代替案:
- GLIM の `submap_downsample_resolution` を 0.3 → 0.1 にして dump し直す: 点数が最悪 27 倍で 16 GB 機では OOM の懸念 (GLIM オフライン OOM の前例あり)、かつ keyframe 間隔依存の不均一は残る
- glim_dump_to_2dmap に「観測 submap 数」モードを足す: 密度情報そのものは戻らない (間引き後の点しか無い)
- GLIM 再実行で生点群を書き出す拡張モジュール: C++ 実装が要り、traj_lidar.txt で同じ結果が得られるので不要
- 点群地図から局所床推定 (格子ごとの床再検出) を作り込む: 生スキャンには視点があるので、スキャン単位の平面当て (ground モード) の方が単純で頑健

### 2.2 高さ帯はセンサ座標の z で切る (既定)

実測: 5号館 LC 地図の最遠角では traj 姿勢に約 4° のピッチ誤差 (z ドーム斜面) が乗り、
「世界 z − 姿勢 z」で切ると 15 m 先で帯が ±1 m 傾いて路面点が帯に入る (読本 §3.4 機構 (3))。

判断: 生スキャンなら変換前のセンサ座標 z がそのまま使えるので、姿勢の回転誤差を帯に
持ち込まずに切れる。これは実機の /scan (pointcloud_to_laserscan の base_link 相対帯) と同じ
操作でもあり、AMCL が照合する地図と観測の切り方が揃う。比較用に `--height_frame world`
(glim_dump_to_2dmap の sensor モードと同じ意味) も残した。

### 2.3 濃度変換は既製互換のまま、点数配列を保存できるようにした

判断: 生密度は submap 版より 1 桁高いので、点数しきい値 (min/max_points_in_pix) の意味が
「観測 submap 数」から本来の「密度によるノイズ除去」に戻る。適値の探索が要るため
`--save_counts` で画素点数配列を残し、しきい値・解像度 (2×2 ブロック和で 0.10) の派生を
再実行なしで作れるようにした (1 回の変換は約 2.5 分)。

## 3. データフロー

```
bag (mcap)                                   GLIM dump (offline_viewer で LC 後に保存)
  /rfans_driver/rfans_points                   <dump>/traj_lidar.txt  (TUM: stamp x y z qx qy qz qw)
  sensor_msgs/msg/PointCloud2                  ← 大域最適化後の 1 スキャン 1 姿勢 (6,957 行)
  fields x y z intensity laserid time hangle mirrorid, 30,016 pt/scan, 10 Hz
        │                                              │
        ▼ header.stamp で最近傍 (許容 --stamp_tol 0.02 s)  ▼
  [glim_traj_to_2dmap.py]
    range_min/max で無効点・車体・遠方を除去
    --height_frame sensor: センサ座標 z ∈ [min,max] を残す → w = R·p + t
    --height_frame world : w = R·p + t → (w_z − t_z) ∈ [min,max] を残す
    (--deskew: 点ごとの time で姿勢 k, k+1 を slerp 補間)
    画素 (x 右, y 上, 中心 = world 原点 or bbox 中心) に投票 → counts
    counts → 輝度 = clip(255 − 255·(counts − min)/(max − min))
        │
        ▼
  <dest>/map.pgm + map.yaml  → nav2.launch.py map_yaml:= (map_server)
```

## 4. 使い方

```bash
# glim_env 内
source /opt/ros/jazzy/setup.bash
T=/workspace/tools/glim_traj_to_2dmap/glim_traj_to_2dmap.py
B=/workspace/bags/5goukan/2d3d_imu/online/rosbag/2026-08-14_0919
D=/workspace/bags/5goukan/2d3d_imu/offline/glim/glim_5goukan_lc_2026-08-14_0919

# 床のセンサ座標 z を実測 (約 15 s) → 5号館 08-14: -0.650 m → 帯は床+0.3〜+1.5 = -0.35〜+0.85
python3 $T $B $D /tmp/x --floor_probe --skip 10

# 変換 (全 6,989 スキャン、約 2.5 分。3 本並列でも 1 本あたり RSS ≈ 380 MB)
python3 $T $B $D /workspace/maps/glim/<name>/nav2 \
  -r 0.05 --map_width 6144 --map_height 6144 \
  --height_frame sensor --min_height -0.35 --max_height 0.85 \
  --min_points_in_pix 2 --max_points_in_pix 5 --save_counts /workspace/maps/glim/<name>/counts.npy
```

引数の一覧は `tools/README.md` の glim_traj_to_2dmap 節を参照 (二重管理を避けるため本文書では省略)。

## 5. 変更ファイル一覧

- `tools/glim_traj_to_2dmap/glim_traj_to_2dmap.py` — 新規 (約 230 行、numpy + rosbag2_py)
- `tools/README.md` — glim_traj_to_2dmap 節を追加
- `docs/text/map3d_to_nav2/03_map_conversion.md` — §3.5 新設 (旧 §3.5/3.6 → §3.6/3.7)、`00_index.md` / `06` / `07` / `08` の参照追従
- `docs/text/map3d_to_nav2/img/2026-09-14_*.png` — 比較図

## 5b. 比較結果の要約 (5号館 08-14 LC 地図、詳細は読本 §3.5)

実測:
- 右上 ROI / 始点 ROI の占有面積比: submap 版 0.10 (r=0.05) / 0.37 (r=0.10) → 生スキャン版 1.2〜1.3
- 生スキャン版どうし: センサ z 帯 (J) は遠方地面の筋が出る (地面平面フィットでセンサの取付ピッチ
  中央値 −1.29°、ロール +0.06° → 系統ピッチが主因)。地上高帯 (Q/R) で筋が減る。deskew の有無で
  占有 IoU 0.78 (回転区間で点配置が変わる)
- 軌跡の左 3 m に沿う黒帯は地上高 0.3〜0.6 m の静止物 (観測時刻幅 51 s、別 run にも出現)。実体は未確認
- 「実機 /scan との一致率」: 自己包含では J 0.97 だが、critic 査読の hold-out 再計算では
  J 0.79 / Q 0.92 / B 0.77。姿勢を 0.5 m ずらしたときの落ち幅は全方式 +0.3 で識別力に差なし

判断: 地図生成の既定を R (`--height_frame ground 0.3..1.5 --range_max 30 --deskew` 点数 4..12) とする。
方式間の最終判定 (AMCL がどちらで収束しやすいか) は bag 再生 + AMCL の実験で行う (未実施)。

## 6. 既知の制限

- **AMCL 実走 (または bag 再生 + AMCL) での優劣は未検証**。一致率指標は自己包含・黒面積依存で方式選択の根拠にならないと判明 (critic 査読)
- LC 後に保存した dump が必要。LC 前の dump (`*_dump/`) と行数・時刻範囲が同じで見分けにくい — `traj_lidar.txt` 終端 z で確認する
- 密度が速度・距離依存になる。停車・低速区間が濃くなり、点数しきい値の効きが場所で変わる
- 動体 (歩行者・随伴者) が生密度で残る。左 3 m の黒帯が静止物である根拠は観測時刻幅と別 run の一致で、随伴者説は完全棄却できていない
- 未知領域 = 自由 の制約は既製ツールと共通で未解決 (生スキャンには視点があるのでレイキャストで解ける余地はある)
- 高さ帯の基準を ground にすると実機 /scan (センサ z 帯、取付ピッチの筋を含む) とは切り方が一致しない。AMCL 用は地図側が余剰でも害が少ないと判断して ground を既定にしたが、実走で要確認
- ground モードの地面探索初期値は先頭 50 スキャンの最頻値。起伏の大きいコースや先頭が坂の bag では `--ground_z` を明示する
- bag の点群トピック名・フィールド (x y z time) は R-Fans 新ドライバ前提。他 LiDAR では `--topic` と `time` フィールドの有無を確認
- 実装の初版は `np.bincount(minlength=W·H)` で 19 分かかった (修正済み、現在 25〜45 s)
