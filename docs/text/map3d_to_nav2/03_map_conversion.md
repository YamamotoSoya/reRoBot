<!-- claude: 3D地図→Nav2 接続読本 第3章 (2026-08-17) -->

# 第3章 3D→2D 地図変換 — 高さで切るという翻訳

3D 点群地図 (数十万点の x, y, z) を第 2 章の占有格子に変換する。本章の主役は
「高さスライス」という考え方と、それを実装した既製ツール pointcloud_to_2dmap の中身。
続いて z ドリフト地図で絶対 z スライスが破れる問題と、その解である自作
glim_dump_to_2dmap (センサ相対スライス、§3.4)、最後に代替経路の得失比較。

## 3.1 なぜ全投影は失敗するか

素朴な発想は「全点を真上から見て、点のあるセルを黒にする」だが、これは必ず失敗する。
3D 地図には床の点が大量に入っており (LiDAR は床を最もよく見る)、**全セルに床点が乗って
地図が真っ黒になる**。天井・木の枝など「ロボットの背より高い構造」も同様に、
通れる場所を占有として塗り潰す。

解決は「ロボットの車体が実際にぶつかる高さ帯だけを見る」こと:

```
高さスライスの考え方 (側面図)

  z ▲
    │  ～～～ 天井・枝 (z > 床+1.5) ........ 捨てる (ぶつからない)
    │ ┌─┐
    │ │壁│ ←─ 床+0.3〜1.5 の帯 ............ これだけ使う = 「壁」
    │ │ │      reRoBot の車体・マストが
    │ └─┘      物理的に占める高さ帯
    │ ═════ 床 (z ≈ 床±0.15) .............. 捨てる (障害物ではない)
    └──────────────────────────▶ x
```

帯の下限 0.3 を床ぎりぎりにしないのは、床点自体の高さばらつき (縦角誤差で
±0.2 m 程度 — [GLIM 読本](../glim/00_index.md) 事例A 周辺) が壁に化けるのを防ぐため。
上限 1.5 は車体高さ + マージン。この「床+0.3〜1.5 m」は 08-13 の GLIM 品質評価
(`docs/issue/2026-08-13_glim_param_tuning.md`) と同じ帯で、実績がある。

## 3.2 pointcloud_to_2dmap の解剖

採用した既製ツール ([koide3/pointcloud_to_2dmap](https://github.com/koide3/pointcloud_to_2dmap)、
GLIM と同作者) は、この高さスライスを 50 行弱で実装している。全ロジックは
`tools/pointcloud_to_2dmap/src/pointcloud_to_2dmap.cpp` の `MapGenerater::generate()`:

```cpp
// 1) 高さ帯フィルタ (min_height/max_height は絶対 z)
if(point.z < min_height || point.z > max_height) continue;
// 2) world (0,0) を画像中心に置いて格子へ投票
int x = point.x * m2pix + map_width / 2;
int y = -point.y * m2pix + map_width / 2;   // ⚠️ ここが map_height でなく map_width
map.at<int>(y, x)++;
// 3) 点数 → グレースケール (min_points_in_pix=2, max_points_in_pix=5 が既定)
map -= min_points_in_pix;
map.convertTo(map, CV_8UC1, -255.0 / (max_points_in_pix - min_points_in_pix), 255);
```

3) の式を第 2 章の 2 段翻訳 (occupied_thresh 0.5 / free_thresh 0.2) に通すと、
**セル内の点数がそのまま {自由, 未知, 占有} に写像される**:

| 帯内の点数 | ピクセル値 | 占有確率 p | map_server の解釈 |
|---|---|---|---|
| 0〜2 | 255 (白) | 0.0 | 自由 |
| 3 | 170 | 0.33 | **未知** (0.2 < p < 0.5) |
| 4 | 85 | 0.67 | 占有 |
| 5 以上 | 0 (黒) | ≥1.0 (飽和) | 占有 |

つまり「4 点以上で壁、3 点は保留、2 点以下はノイズ扱いで自由」。GLIM の submap は
0.3 m 間引き済みで点密度が管理されているので、この閾値がそのまま機能する
(密度が大きく違う地図では `--min/max_points_in_pix` の調整が要る)。

⚠️ 2 つの実装上の注意 (実測で確認済み):

1. **y の中心計算が `map_width` を参照している** (上のコード 2 行目)。`-w` と `-h` に
   別の値を渡すと y 方向の中心がずれる。**必ず -w = -h で使う**
2. **min/max_height は絶対 z** である。「床+0.3〜1.5」を渡すには床の z を知る必要がある
   — それが次節

## 3.3 床はどこにあるか — z の実測

GLIM 地図の z=0 は「床」ではない。z=0 は**マッピング開始時のセンサ (IMU) の位置**で、
重力方向だけが IMU で校正されている。reRoBot の N12 地図 (9 号館, 08-14) では
床は z=+0.636 にあった (事例の詳細は [第6章 事例C](06_case_studies.md))。

床 z は z ヒストグラムの最頻値で実測する — 屋内地図では床が圧倒的な最大平面なので、
5 cm ビンの最頻値がそのまま床になる:

```bash
# N12 実測: z_floor 0.636 / x −17〜40 m / y −33〜11 m (2026-08-17)
docker exec glim_env bash -c "python3 -c \"
import numpy as np
raw = open('<map.ply>','rb').read()
pts = np.frombuffer(raw, dtype=np.float32, offset=raw.index(b'end_header')+11).reshape(-1,3)
h,e = np.histogram(pts[:,2], bins=np.arange(pts[:,2].min(), pts[:,2].max(), 0.05))
print('z_floor %.3f' % (e[h.argmax()]+0.025))\""
```

同時に xy 範囲も出す。地図サイズ `-w -h` は「xy の最大絶対値 × 2 ÷ resolution」以上が
必要 (ツールは world 原点中心の固定枠で、はみ出た点は黙って捨てられる)。

⚠️ **z ドリフトのある地図では床が 1 枚ではない**。現行 GLIM 地図には縦角表の系統誤差
由来の z ドリフト (`docs/issue/2026-08-16_rfans_mount_angle_glim_z_collapse.md`) があり、
flat 取付でも −1.3%/距離で床が滑る。N12 (走行 50 m 級) では帯 1.2 m に対し
ずれ ~0.7 m で実用範囲だったが、数百 m 級の屋外地図では帯から壁が外れる —
5号館 344 m 周回 (ドリフト最大 +4.9 m) で実際に外れた。根本対策は縦角再較正
(保留中) だが、地図変換に限っては次節のセンサ相対スライスで回避できる。

## 3.4 glim_dump_to_2dmap — センサ相対スライスという解 (2026-08-20 追加)

絶対 z 方式が破れるのは「床が z=0 付近に 1 枚だけある」という前提が地図全体で
成り立たないときである。ではスライス帯を場所ごとに床へ追従させればよい — その情報は
どこにあるか。**マージ済み PCD には無い。GLIM の dump には残っている**。

```
情報の残り方の違い
├── マージ済み PCD (offline_viewer が export する map.ply/pcd)
│     点 = (x, y, z) だけ。「どこから撮ったか」は捨てられている
│     → 床追従には局所床推定 (格子ごとの床再検出) を作り込むしかない
└── GLIM dump (dump ディレクトリそのもの)
      submap = 点群 + T_world_origin (そのときのセンサ姿勢)
      → 各点に「観測したセンサの z」が付いている
      → 「点の z − センサ z」で切れば帯がドリフトと一緒に上下する
```

これを実装したのが `tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py` (自作、numpy のみ、
ビルド不要)。スライス判定は 3 行 (`tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py:88`、
行番号は 2026-08-20 時点):

```python
z_ref = T[2, 3] if args.height_mode == "sensor" else 0.0   # submap のセンサ z
rel = w[:, 2] - z_ref                                       # センサ相対高さ
keep = (rel >= args.min_height) & (rel <= args.max_height)
```

座標変換・画素への投票・点数→グレースケール変換 (§3.2 の表) は既製ツールと
同一仕様なので、map_server 側から見た出力の意味は変わらない (画像が png でなく
PGM になるだけ。map_server は両方読める)。

### 使い方

```bash
# 1) 床のセンサ相対 z を実測 (絶対 z の代わりに「センサから見て床は何 m 下か」を測る)
#    reRoBot 5号館 08-14 LC 地図の実測: -0.55 m
docker exec -i glim_env python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0, '/workspace/tools/glim_dump_to_2dmap')
from glim_dump_to_2dmap import load_submap, list_submaps
base = '/workspace/bags/<dump_dir>'
rels = []
for i in list_submaps(base):
    T, pts = load_submap(base, i)
    w = pts @ T[:3, :3].T + T[:3, 3]     # submap 点群 → 世界座標
    rels.append(w[:, 2] - T[2, 3])       # センサ z との差
rel = np.concatenate(rels)
low = rel[(rel > -2.0) & (rel < 0.5)]    # 床がありうる範囲だけでヒストグラム
h, e = np.histogram(low, bins=np.arange(-2.0, 0.5, 0.02))
print('floor (sensor-rel) = %+.3f' % (e[h.argmax()] + 0.01))
EOF

# 2) 変換 (帯 = 床相対値 + 0.3 〜 +1.5。床 -0.55 なら -0.25〜+0.95)
docker exec glim_env python3 /workspace/tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py \
  /workspace/bags/<dump_dir> /workspace/maps/glim/<name>/nav2 \
  -r 0.05 --map_width 6144 --map_height 6144 \
  --height_mode sensor --min_height -0.25 --max_height 0.95
```

| 引数 | 既定 | 意味 |
|---|---|---|
| `--height_mode` | sensor | sensor = センサ z 相対 / absolute = 既製ツールと同じ絶対 z |
| `--min/max_height` | −0.25 / +0.95 | スライス帯。height_mode の基準に対する値 |
| `--center` | world | world = 原点中心 (既製互換) / auto = bbox 中心 + サイズ自動 |
| `--map_width/height` | 0 (自動) | 0 なら bbox + 2 m マージンに自動フィット。y 中心バグは無い (別値可) |
| `--min/max_points_in_pix` | 2 / 5 | §3.2 の点数閾値と同じ意味 |
| `--export_pcd <path>` | なし | 全点マージの世界座標 PCD も書く (既製ツール入力・3D ローカライザ地図用) |

### 実測比較 — 絶対 z はドリフト部で壁が全滅する

5号館 08-14 手動 LC 地図 (344 m 周回、z ドリフト最大 +4.9 m) で、同一点群・
同一方針の帯 (床+0.3〜1.5) を両方式で切った結果
(`bags/5goukan/2d3d_imu/offline/glim/2dmap_compare/compare_abs_vs_sensor.png`):

| | 絶対 z (既製) | センサ相対 (本ツール) |
|---|---|---|
| スライス帯 | +0.13〜+1.33 (床最頻値 −0.17 基準) | −0.25〜+0.95 (センサ基準) |
| 始点付近 (ドリフト小) | 壁が出る | 壁が出る |
| 最遠部 (ドリフト +4.9 m) | **建物ごと消失** (帯が床下 5 m を切っている) | 壁・植栽とも残る |
| 占有画素数 | 43,493 | 55,364 (+27%) |

副次的な利点が 1 つ: /scan 側 (`rfans_scan.launch.py`) は元々 base_link 相対の高さ帯
なので、地図側もセンサ相対にすると**第4章 §4.3 の帯整合が「相対 vs 絶対」の変換なしに
そのまま成立**する (絶対 z 方式では床 z の加算という手動の橋渡しが要った)。

⚠️ 制約: 帯の追従は submap 単位 (数 m おき) の階段状。ドリフトが緩やか (5号館で
160 m かけて +5 m) なら段差は数 cm で無視できるが、急峻な z 誤り (ジャンプ) がある
地図では段差が帯幅に対して効いてくる。また「未探索領域 = 自由」の制約 (§3.5 参照) は
既製ツールと共通で、本ツールでも解決していない。

## 3.5 代替経路 — 既製ツールを使わない 3 つの方法

```
3D→2D 変換の 4 経路
├── 採用: pointcloud_to_2dmap (§3.2)
│     ✅ コードゼロ・GLIM と同作者・点数閾値で未知も出る
│     ⚠️ 未探索領域が「自由」になる / 高さ帯が固定 z
├── 経路A: 既製ノード連結 (コマンドのみ、カスタムコード 0 行)
│     pcl_ply2pcd → ros2 run pcl_ros pcd_to_pointcloud (点群を配信)
│     → octomap_server (z 帯フィルタ + /projected_map)
│     → ros2 run nav2_map_server map_saver_cli
│     ❌ octomap の自由空間判定は「センサ原点からのレイキャスト」前提。
│        完成済みの静止点群には視点情報が無く、自由/未知が崩れる。
│        占有セルは正しく出るので「壁だけ欲しい」なら成立
├── 経路B: bag から 2D SLAM やり直し (コマンドのみ、08-12 に実績 = 第6章 事例A)
│     bag 再生 → pointcloud_to_laserscan → slam_toolbox → map_saver_cli
│     ✅ レイキャストが本物なので自由/未知/占有が全部正しい
│     ❌ GLIM の最適化済み 3D 地図を**使わない**別物。ループ閉合や
│        3D での地図修正の恩恵がゼロ。GLIM 地図と origin も一致しない
└── 経路C: 自作スクリプト → **一部実装済み (2026-08-20)** = §3.4 glim_dump_to_2dmap
      ✅ センサ相対スライスで z ドリフトに追従 (局所床推定より安価に同じ目的を達成 —
         dump にセンサ姿勢が残っているため「推定」ではなく「参照」で済む)
      ⚠️ 未知の区別 (床点の有無で判定) は未実装のまま。実害が出たらここが次
```

判断の軸は「自由/未知の判定品質」と「保守コスト」のトレードオフである。屋内の
閉環境 + keepout 運用なら採用案で足り、z ドリフトが大きい長距離地図で壁が
帯から外れたら §3.4 のセンサ相対スライスに切り替える (5号館で切替済み)。

## 3.6 この章のまとめ

```
第3章 まとめ
├── 全投影は床点で真っ黒になる → 車体がぶつかる帯 (床+0.3〜1.5) だけ切る
├── pointcloud_to_2dmap: 点数→グレー階調→ {≤2 自由 / 3 未知 / ≥4 占有}
├── 使う前に床 z と xy 範囲を実測 (床は z=0 ではない! N12 は +0.636)
├── -w と -h は必ず同値 (y 中心計算の実装バグ)
├── z ドリフト地図は絶対 z 方式が破れる → glim_dump_to_2dmap (dump 直読み +
│   センサ相対スライス) で帯をドリフトに追従させる (5号館 +4.9 m で実証)
└── 代替: octomap 連結 (自由空間が崩れる) / bag 2D SLAM (GLIM を使わない) /
    自作の残り = 未知領域の区別 (未実装)
```

→ [第4章 /scan 化と AMCL](04_scan_and_amcl.md)
