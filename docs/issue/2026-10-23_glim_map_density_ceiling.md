<!-- claude: 2026-10-23 作成 -->
# GLIM 高密度地図の上限は「1 run あたり約 8,400 万点」— VRAM ではなく submap 生点の常駐量で決まる

**ステータス: 上限は確定 / 設定の優劣は暫定** — 上限はソース上の配列確保量から算術で出るため確定 (critic 査読済み、§3)。
どの設定が良いかは §5 の通り「鮮鋭度では ultra が明確に優位、外部パラメータの yaw 補正は差が検出できず、
大域 z は未解決」までで、断定していない。

- 対象機: GTX 1080 (VRAM 8,192 MiB) / RAM 31 GB / i7-8700K 相当 12 vCPU
- 実験ブランチ: `exp/glim-gpu-imu`
- 関連: `2026-10-08_glim_gpu_cuda_illegal_address_vram.md` (VRAM 落ち), `2026-10-10_glim_cpu_gpu_loop_closure_difference.md` (CPU/GPU 差・seg 分割の不利)

## 1. このセッションでやったこと

1. 09-18 5号館 bag を**従来の 3〜6 倍の密度**で地図化した (`hq5_dense`、4,990 万点、壁の厚み 9.6 mm)
2. そこから Nav2 用 2D 占有格子とマージ済み PCD を書き出した (`maps/2d/glim/2026-09-180915_5goukan_hq5_dense/`)
3. 「同じことを広域 (つくば 2.3 km) でできるか」を資源面から詰め、**できない**ことと**その理由**を確定した (§3)
4. 広域向けに seg を短く割る方式を設計し、`tsukuba_ultra` 設定で seg3 / seg4 / seg1 前半を作った (§5)
5. 設定どうしを、**同一設定 3 回反復で求めたノイズ床つきで**比較した (§5)

## 2. 密度を決めている 3 つの量

| 量 | 何で決まるか | 09-18 hq5_dense | tsukuba_ultra |
|---|---|---|---|
| **m あたり点数** | `submap_target_num_points ÷ submap 間隔` | 75,408 | **約 123,000** |
| **データ側の天井** | `max_num_keyframes × 1 スキャンの生点 (約 3 万)` を 2 cm 格子で重複除去 | 約 45 万点/submap | 約 120 万点/submap |
| **資源側の天井** | 1 run の総点数 (§3) | 8,400 万点 | 8,400 万点 |

実走は 10 Hz × 約 0.45 m/s = **22 スキャン/m** 出ているが、キーフレーム間隔 0.25 m では **4 本/m しか使っていない**。
`tsukuba_ultra` が密度を上げられたのは、間隔を 0.10 m にして **10 本/m** 使うようにしたから (点数上限を上げただけでは増えない)。

## 3. 上限の根拠 (critic がソースで確認)

`global_mapping.cpp:252` が submap の点群を `PointCloudGPU::clone` し、これが **GPU とホストの両方に常駐**する。

| | 1 点あたり | 内訳 |
|---|---|---|
| GPU | **64 B** | points 12 + covs 36 + normals 12 + intensity 4 |
| ホスト | **200 B** | 同じものを `Vector4d` / `Matrix4d` で |

`gtsam_points` には退避機構 `OffloadableGPU` があるが **GLIM 1.2.2 は呼んでいない** (`grep offload` が 0 件)。
したがって資源消費は回帰ではなく素の掛け算で、自由パラメータ 0 のモデルが実測 3 run を VRAM ±8% / RSS ±10% で再現した。

**実測した予算**: VRAM 7.4 GB ÷ 64 B ≒ 1.15 億点、RAM 23 GB ÷ 200 B ≒ 1.15 億点 — **両者ほぼ同着**。
安全係数 1.2 を見て **1 run あたり約 8,400 万点**を運用上の上限とする。

**つくば全体 (2.3 km, submap 585 個) を 30 万点/submap で処理すると**: GPU 配列だけで 585 × 30 万 × 64 B = **11.2 GB**、
ホスト配列だけで **33 GB**。どちらも単独で天井を超えるので**不可能**。全体 bag でやれるのは 10 万点/submap (= `hq4`) 程度が限界。

⚠️ **「時間をかければ解決する」ではない**が、**ハードウェアの壁でもない**。VRAM 側は実装由来で、
(a) voxelmap 構築後に過去 submap の GPU 配列を解放する、(b) `OffloadableGPU` を使う、
(c) `libglobal_mapping_pose_graph.so` に替える (GPU clone をしない)、のいずれかで消える。
消した場合に次に当たるのはホスト RAM で、全体 bag なら 20 万点/submap 相当。やはり 30 万点は届かない。

## 4. 広域をやるなら — seg を短く割る

m あたり密度の上限 = 8,400 万点 ÷ seg の長さ。つまり**短く割るほど濃くできる**。

| seg 長 | 上限 m あたり点数 | 生点 (約 64 万点/m) に対する保存率 |
|---|---|---|
| 1,000 m | 83,000 | 13% |
| 600 m | 146,000 | 23% |
| 400 m | 226,000 | 35% |
| 200 m | 466,000 | 73% |

bag を物理的に切る必要はない。**`glim_rosbag` の `start_offset` と `playback_duration`** で仮想分割できる
(2026-10-23 に動作確認済み: `start_offset:=200` 指定 → traj 開始が実測 200.0 s 後)。境界には手動接続用に前後 30 秒の重なりを入れる。

⚠️ **ただし seg 分割は精度面では不利**。`2026-10-10_..._loop_closure_difference.md` §5 の測定で、
全体 1 本なら長距離閉合 6,619 対が、seg1〜4 に割ると合計 924 対に減り、閉合 0 の区間は最適化で z 幅がむしろ広がった。
手動ループ閉合は seg 間の継ぎ目には効くが、**seg 内部のドリフトは直せない**。
さらに「4 つの dense な seg を 1 グラフに入れ直して同時最適化」は、submap が全部メモリに戻るので §3 の壁に再び当たる。

## 5. 設定の比較 (暫定)

### 壁の厚み 中央値 (`tools/glim_map_sharpness.py --stride 2 --voxel 0.6 --min_pts 12`、小さいほど鮮明)

| 設定 | seg3 (216 m) | seg4 (197 m) |
|---|---|---|
| `g1_old_range180` | 14.2 mm | 14.7 mm |
| `g1_imu_strong` | 13.7 mm | 15.0 mm |
| `g1_imu_strong_yawfix` | 14.1 mm | 14.9 mm |
| **`ultra`** | **11.0 mm** | **11.2 mm** |

### ノイズ床 (seg3・同一設定 3 回反復)

| | r1 | r2 | r3 | 平均 ± 標準偏差 |
|---|---|---|---|---|
| `rep_old` | 13.8 | 13.6 | 13.6 | 13.67 ± 0.12 mm |
| `rep_yawfix` | 14.3 | 13.7 | 13.9 | 13.97 ± 0.31 mm |

反復 6 本の全幅 = **0.7 mm**。これより小さい差は設定の効果と見なさない。

### 判定

1. **`ultra` を採用してよい** — 改善 −2.7 mm (seg3) / −3.5 mm (seg4) はノイズ床の 4〜5 倍で、独立 2 区間で再現。点数は 9 倍。
   ただし ultra は前処理・キーフレーム・submap 最適化・外部パラメータを**同時に 4 つ変えている**ので、どれが効いたかは不明。
2. **yaw 補正 (`yawfix`) の効果は検出できなかった** — old 13.67±0.12 対 yawfix 13.97±0.31、seg4 では 15.0 対 14.9。
   10-17 に z 幅で良く見えた (1.45 対 2.47 m) のは、反復の z 幅が 1.21〜2.37 m とばらつく範囲内の偶然。**ここに時間を使う必要はない**。
3. **大域 z は ultra でも改善していない** — seg3 1.40 m (最良) に対し seg4 6.40 m (ベースライン 2.63〜3.50 m より悪い)。
   鮮鋭度 (submap 内の局所整合) と z ドリフト (前段 odometry) は別の軸。seg4 の ultra は 1 回しか回していないので、
   **確定には seg4 で ultra をあと 2 回** (1 回 7 分) 必要。

## 6. 未解決 / 棄却できていないもの

- **seg4 の ultra だけ z が悪い**のが再現するか (n=1)
- `ultra` の 4 変更のうちどれが鮮鋭化に効いたか (10-17 の切り分けでは「submap 間隔の距離基準化」が支配的だった)
- 2 パス方式 (通常密度で軌跡を作り、生点を軌跡で飛ばして任意密度の地図にする) は原理的に可能だが**前提 3 つが未検証**:
  per-point `time` による姿勢補間 deskew (1 スキャンで並進 4.8 cm・回転最大 5.8° 動く)、
  traj stamp と per-point time の原点一致 (09-26 の stamp 量子化が 10-03 bag に 20% 残存)、
  パス 2 側の空間タイル分割 (生点 10 億点規模)
- `ros2_ws_glim/config/hq/` は hq run の後に編集されており**実行条件と一致しない** (`submap_voxelmap_levels` 2→3、
  `min_implicit_loop_overlap` 0.2→0.15)。**実行条件は常に dump 内の `config/` を見ること**

## 7. dump 一覧 (2026-10-23 整理時点の全数)

消した dump もここから再生成できる。実行条件は各 dump の `config/` が正だが、下表はそれを抜き出したもの。

| dump | 大きさ | submap | 点/submap | 点/m | 距離 | traj z幅 | 前処理 vox/目標 | KF 戦略/間隔/枚数 | submap 最適化 | submap 格子/上限 | rs | T_lidar_imu | acc_scale | so |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `2026-09-180915_5goukan_dump/cal300` | 544 MB | 37 | 293,824 | 75,019 | 145 m | 3.56 m | 0.15/50000 | DISPLACEMENT/0.25/15 | True | 0.02/300000 | 0.14 | legacy(Rz90) | 0.9705 | gpu |
| `2026-09-180915_5goukan_dump/hq5_dense` | 2,499 MB | 168 | 297,043 | 75,408 | 662 m | 5.56 m | 0.15/50000 | DISPLACEMENT/0.25/15 | True | 0.02/300000 | 0.05 | legacy(Rz90) | 0.9705 | gpu |
| `2026-10-03_1126_tsukuba_dump/composite_4map` | 1,628 MB | 640 | 49,914 | 14,179 | 2253 m | 23.81 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_dump/g1_imu_strong` | 1,755 MB | 651 | 49,945 | 14,160 | 2296 m | 27.76 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_dump/hq` | 2,072 MB | 557 | 69,857 | 17,219 | 2260 m | 7.45 m | 0.3/30000 | DISPLACEMENT/0.25/15 | False | 0.07/70000 | 0.15 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_dump/hq4` | 2,854 MB | 548 | 99,794 | 24,786 | 2206 m | 12.31 m | 0.3/30000 | DISPLACEMENT/0.25/15 | True | 0.05/100000 | 0.14 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg1_dump/g1_imu_strong` | 777 MB | 287 | 49,906 | 14,239 | 1006 m | 23.56 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg1_dump/g1_imu_strong_yawfix` | 780 MB | 288 | 49,951 | 14,299 | 1006 m | 25.62 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | yawfix | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg1_dump/g1_old_range180` | 810 MB | 301 | 49,802 | 14,866 | 1008 m | 20.28 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg1_dump/ultra_sub1` | 3,151 MB | 100 | 629,488 | 123,256 | 511 m | 4.70 m | 0.15/50000 | DISPLACEMENT/0.1/40 | True | 0.02/800000 | 0.05 | yawfix | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg1_dump/ultra_sub2` | 1,625 MB | 51 | 635,920 | 126,852 | 256 m | 2.52 m | 0.15/50000 | DISPLACEMENT/0.1/40 | True | 0.02/800000 | 0.05 | yawfix | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg2_dump/g1_imu_strong` | 603 MB | 224 | 49,997 | 14,061 | 796 m | 7.71 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg2_dump/g1_imu_strong_yawfix` | 600 MB | 223 | 49,919 | 13,972 | 797 m | 8.26 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | yawfix | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg2_dump/g1_old_range180` | 626 MB | 234 | 49,815 | 14,643 | 796 m | 6.26 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg3_dump/g1_imu_strong` | 155 MB | 58 | 49,894 | 13,420 | 216 m | 2.47 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg3_dump/g1_imu_strong_yawfix` | 154 MB | 58 | 49,307 | 13,266 | 216 m | 1.45 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | yawfix | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg3_dump/g1_old_range180` | 164 MB | 62 | 49,286 | 14,142 | 216 m | 2.50 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg3_dump/rep_old_r1` | 154 MB | 58 | 49,226 | 13,250 | 215 m | 2.15 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg3_dump/rep_old_r2` | 153 MB | 57 | 50,000 | 13,223 | 216 m | 2.04 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg3_dump/rep_old_r3` | 153 MB | 57 | 50,000 | 13,230 | 215 m | 1.73 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg3_dump/rep_yawfix_r1` | 152 MB | 57 | 49,675 | 13,146 | 215 m | 1.21 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | yawfix | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg3_dump/rep_yawfix_r2` | 153 MB | 57 | 49,907 | 13,208 | 215 m | 1.60 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | yawfix | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg3_dump/rep_yawfix_r3` | 151 MB | 56 | 50,000 | 12,992 | 216 m | 2.37 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | yawfix | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg3_dump/ultra` | 1,315 MB | 43 | 611,182 | 122,831 | 214 m | 1.40 m | 0.15/50000 | DISPLACEMENT/0.1/40 | True | 0.02/800000 | 0.05 | yawfix | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg4_dump/g1_imu_strong` | 186 MB | 71 | 49,698 | 17,923 | 197 m | 2.63 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg4_dump/g1_imu_strong_yawfix` | 183 MB | 70 | 49,689 | 17,674 | 197 m | 2.76 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | yawfix | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg4_dump/g1_old_range180` | 190 MB | 73 | 49,402 | 18,313 | 197 m | 3.50 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-03_1126_tsukuba_seg4_dump/ultra` | 1,219 MB | 39 | 625,079 | 124,613 | 196 m | 6.40 m | 0.15/50000 | DISPLACEMENT/0.1/40 | True | 0.02/800000 | 0.05 | yawfix | 0.0 | gpu |
| `2026-10-09_1611_5goukan_chal_dump/g1_imu_strong` | 354 MB | 129 | 49,924 | 14,088 | 457 m | 6.98 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-09_1643_5goukan_chal_dump/g1_imu_strong` | 348 MB | 128 | 49,912 | 14,152 | 451 m | 6.62 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |
| `2026-10-09_1643_5goukan_chal_dump/g1_imu_strong_filtered` | 326 MB | 128 | 49,912 | 14,146 | 452 m | 2.15 m | 1.0/10000 | OVERLAP/1.0/15 | False | 0.1/50000 | 0.2 | 旧(-93°) | 0.0 | gpu |

## 8. 整理と再生成 (2026-10-23)

`bags/raw/` は全削除。dump は「Drive に無く、かつ再生成コストが高いもの」だけ Drive へ退避し、残りは削除した。
**原本の bag はすべて Drive の `cit-share-bags:bags/raw/` にあるので、下表のコマンドで全て再生成できる。**

### Drive に退避したもの (`cit-share-bags:bags/glim/`)

| dump | なぜ残したか |
|---|---|
| `2026-10-03_1126_tsukuba_dump_hq.tar.zst` | 全体 bag (28.5 GB) + **実行 4.0 時間**。このセッション群で最も高い |
| `2026-10-03_1126_tsukuba_dump_hq4.tar.zst` | 全体 bag + 0.9 時間。10-17 時点の最終版 |
| `2026-10-03_1126_tsukuba_seg1_dump_ultra_sub1.tar.zst` | seg1 bag (展開 23 GB) の取得が重い |
| `2026-10-03_1126_tsukuba_seg1_dump_ultra_sub2.tar.zst` | 同上。※ 計画 503 m のうち 256 m で中断 (SIGTERM 時に保存されたもの) |
| `2026-09-180915_5goukan_dump_hq5_dense.tar.zst` | 2D 地図と PCD の生成元。bag 11.2 GB + 11 分 |

Drive に既にあったので触っていないもの: `..._tsukuba_dump_{composite_4map,g1_imu_strong,e3_baseline,gpu,filtered}`、
`..._seg{1,2,3,4}_dump_g1_imu_strong`、`2026-10-09_*_chal_dump_*`。
**`composite_4map` は手作業で合成したもので再生成手順が無い** — Drive の原本が唯一の実体。

### 削除したもの と 再生成コマンド

すべて glim コンテナ (`docker compose --profile glim_gpu up -d glim_gpu`) で実行する。
bag は `rclone cat "cit-share-bags:bags/raw/<bag>.tar.zst" | zstd -dc --long=31 | tar -xf - -C bags/raw/` で取得
(09-18 のみディレクトリ形式: `rclone copy cit-share-bags:bags/raw/2026-09-180915_5goukan bags/raw/2026-09-180915_5goukan`)。

| 削除した dump | 再生成 |
|---|---|
| `2026-09-180915_5goukan_dump/cal300` | `CONFIG=hq5_dense ./scripts/glim_hq.sh 2026-09-180915_5goukan cal300 300` |
| `2026-10-03_1126_tsukuba_seg{1,2,3,4}_dump/g1_imu_strong` | `CFG_DIR=g1_old TAG=g1_imu_strong ./scripts/glim_seg_yawfix.sh 1 2 3 4` ※ |
| `..._seg{1,2,3,4}_dump/g1_imu_strong_yawfix` | `CFG_DIR=g1_yawfix TAG=g1_imu_strong_yawfix ./scripts/glim_seg_yawfix.sh 1 2 3 4` ※ |
| `..._seg{1,2,3,4}_dump/g1_old_range180` | `CFG_DIR=g1_old_range180 TAG=g1_old_range180 ./scripts/glim_seg_yawfix.sh 1 2 3 4` ※ |
| `..._seg3_dump/rep_{old,yawfix}_r{1,2,3}` | `./scripts/glim_yaw_repeat.sh 3 3` ※ |
| `..._seg3_dump/ultra` | `CONFIG=tsukuba_ultra ./scripts/glim_hq.sh 2026-10-03_1126_tsukuba_seg3 ultra` |
| `..._seg4_dump/ultra` | `./scripts/glim_tsukuba_ultra.sh seg4` |
| `2026-10-09_*_chal_dump/*` | Drive に退避済み (再実行不要) |

※ `glim_seg_yawfix.sh` / `glim_yaw_repeat.sh` は config を `bags/exp/<名前>` から読む実装のまま。
  config 本体は **`ros2_ws_glim/config/{g1_old,g1_old_range180,g1_yawfix}` に移して版管理下に入れた** (2026-10-23)
  ので、使うときは `cp -r ros2_ws_glim/config/g1_old bags/exp/glim_g1_old_config` のように戻すか、
  スクリプトの `CFG` の参照先を `/glim_config/<名前>` に書き換える。

### 生成物として残したもの (ローカル、git 管理外)

- `maps/2d/glim/2026-09-180915_5goukan_hq5_dense/` — `map_dense.pcd` (4,990 万点, 572 MB) + `raw/` + `nav2/` + `keep_out/`
  生成: `tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py <dump> <dest> -r 0.10 --center auto --map_width 0 --map_height 0
  --height_mode base_link --base_to_sensor_z 0.80246 --range_max 30 --min_height 0.3 --max_height 1.5 --export_pcd <...>.pcd`
  (`--base_to_sensor_z` は 09-30 以前の取付高。10-01 以降の bag は 0.79396)
- `bags/exp/glim_hq/<bag>/*.{glim.log,resource.csv}` — 各 run の標準出力と 10〜15 秒間隔の RAM/VRAM 記録
