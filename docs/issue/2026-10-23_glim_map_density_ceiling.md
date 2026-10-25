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

## 9. 新習志野チャレンジ bag (2026-10-22_1622_shinnarachallenge) の試走 — 2026-10-23

bag: 63.9 分 / 3,832 s / 走行 1,697 m (車輪オドメトリ) / mcap 36 GB。うち点群トピックだけで 31.1 GB。
`/rfans_driver/rfans_points` 10.0 Hz、`/imu/data` 199.9 Hz、`/imu_wit/data` 200.6 Hz。2D は `/scan` ではなく **`/urg_front/scan`**。

| run | config | submap/点数 | odom 段 z 幅 | 最適化後 z 幅 | 始終点 (3 次元) | 照合因子 | 壁の厚み | VRAM | 所要 |
|---|---|---|---|---|---|---|---|---|---|
| `cpu_default` | 既定 (CPU) | 535 / 1,928 万 | — | 81.57 m | — | 415 対 | 44.1 mm (※) | 0 | 16 分 |
| `gpu_default` | 既定 (GPU) | 374 / 1,868 万 | 43.32 m | **35.11 m** | 44.90 m | 5,175 対 | **20.4 mm** | 2,465 MB | 25.6 分 |
| `posegraph` | GPU + pose_graph | 376 / 1,877 万 | 50.19 m | 52.51 m | 67.78 m | 0 | — | **311 MB** | 26 分 |
| `cal300` | `shinnara_ultra` 先頭 300 s | 20 / 1,500 万 | — | — | — | — | — | 2,563 MB | 4 分 |
| `cal300b` | `shinnara_ultra2` 先頭 300 s | 21 / 2,292 万 | — | — | — | — | — | 6,369 MB | 10 分 |

※ CPU 版の 44.1 mm は平面と判定できたボクセルが 55 個しかなく統計として不成立 (GPU 版は 10,028 個)。地図が崩れている指標として読む。

### 分かったこと

1. **この走行は周回。** 車輪オドメトリでは始終点が 202 m 離れて見えるが、GLIM の補正後では 24〜29 m。残りは未補正のドリフト。
   補正後軌跡で検出できる再訪は **走行 1,621 m ↔ 26 m (出発点) の 1 箇所だけ** (水平 14.5 m / z 差 33.7 m)。
2. **CPU も GPU も長距離ループ閉合が 0 対** (最大 \|i−j\| が CPU 10 / GPU 42)。つくば bag では GPU が 1,215 対を出したのと対照的。
3. **z ドリフトが約 2.2 m/100 m (傾き 1.26°) でほぼ一定。** 特定区間への集中はなく、系統的な傾き。
   → 区間を短くしても傾きは消えず絶対量が比例して減るだけ。150 m 区切りでも中央値 3.5 m 溜まる。
4. **これが全ての根。** 既定版は重なり率が 0.2 に届かず閉合せず、pose_graph 版は距離が閾値を超えて候補にすら挙がらない。
   09-27 時点で odom 段の車体固定傾きは 0.91° まで追い込めていた記録があるので、10-22 で条件が変わった可能性がある
   (`docs/issue/2026-09-10_glim_z_drift_not_vangle.md` 参照)。

### pose_graph 版 (`libglobal_mapping_pose_graph.so`) の評価 — 1 run

- **VRAM は 2,465 → 311 MB (1/8)。** critic がソースで指摘した通り、submap 点群を GPU に常駐させない
  (`global_mapping_pose_graph.cpp:254` で voxelmaps を clear、`PointCloudGPU::clone` を呼ばない)。
  **密度の制約が VRAM からホスト RAM (200 B/点、約 1 億 1,500 万点) に移る。**
- **ただしループは 1 本も張られなかった。** `find_loop_candidates` (同 310 行) は submap 間の直線距離が
  `max_neighbor_dist` を超える候補を捨てる。5.0 → 40.0 m に上げたが、この run の始終点は 3 次元で 67.8 m 離れていた。
- **地図は悪化した。** ただし odom 段から既に悪く (z 幅 43.3 → 50.2 m)、**実行ごとのばらつきと混ざっている**。
  そのうえで global_mapping が改善しなくなった (既定版は odom 48.4 → 44.9 m、pose_graph 版は 67.5 → 67.8 m と素通し)。
  既定版が持つ 5,175 本の暗黙の照合因子 (近接 submap 間の局所拘束) を失い、odom 因子だけになったため。
- **含意**: pose_graph は「VRAM を外す手段」としては成功、「地図を良くする手段」としては失敗。
  暗黙閉合 (既定) と明示閉合 (pose_graph) を両取りする設定は無い。

### sub_mapping の VRAM 構造 (ソースで確認)

`sub_mapping.cpp:276-312` のキーフレーム間因子は **全結合 × voxelmap レベル数**で作られ、
**`enable_optimization` では囲まれていない** (同 436 行が唯一の参照箇所で、そこは最適化の実行可否)。
したがって `enable_optimization: false` にしても因子は同数作られ GPU を消費する — VRAM は減らず品質だけ落ちる。

| キーフレーム数 | 対の数 | × levels 3 | 実測 VRAM (bag 107 m 時点) |
|---|---|---|---|
| 57 (`shinnara_ultra`) | 1,596 | 4,788 | 2,563 MB |
| 110 (`shinnara_ultra2`) | 5,995 | 17,985 | 6,369 MB |

VRAM を下げるなら `registration_error_factor_type` を `VGICP` (CPU) にするか、`keyframe_voxelmap_levels` を下げる。

### 密度の実測 (bag 先頭 300 s での較正)

| config | KF 間隔/枚数 | 点/submap | 間隔 | 密度 | seg4 ultra 比 | 再生速度 |
|---|---|---|---|---|---|---|
| `shinnara_ultra` | 0.07 m / 57 | 750,183 | 5.49 m | 140,636 点/m | 1.13x | 1.09x |
| `shinnara_ultra2` | 0.03 m / 110 | 1,091,606 | 5.35 m | 214,026 点/m | 1.72x | 0.40x |

⚠️ **キーフレーム間隔は走行速度で量子化される。** 0.45 m/s・10 Hz = 0.045 m/scan なので、しきい値 0.07 でも 0.05 でも
「2 スキャンに 1 枚」で同じ。全スキャンを使うには 0.045 を下回る値 (0.03) が要る。

### 再生成コマンド (削除した run)

```bash
# bag: rclone copy は不可 (tar.zst)。下記で取得
rclone cat "cit-share-bags:bags/raw/2026-10-22_1622_shinnarachallenge.tar.zst" | zstd -dc --long=31 | tar -xf - -C bags/raw/

# cpu_default (CPU コンテナ)
docker exec -d glim_env /ros_entrypoint.sh bash -c "ros2 run glim_ros glim_rosbag \
  /workspace/bags/raw/2026-10-22_1622_shinnarachallenge --ros-args -p config_path:=/glim_config \
  -p auto_quit:=true -p dump_path:=/workspace/bags/glim/2026-10-22_1622_shinnarachallenge_dump/cpu_default"

# cal300 / cal300b
CONFIG=shinnara_ultra  ./scripts/glim_hq.sh 2026-10-22_1622_shinnarachallenge cal300  300
CONFIG=shinnara_ultra2 ./scripts/glim_hq.sh 2026-10-22_1622_shinnarachallenge cal300b 300
```

### 4 分割の境界 (GLIM 補正後軌跡から算出、前後 30 秒の重なり込み)

| | start_offset | playback_duration | 区間 |
|---|---|---|---|
| S1 | 0 | 1065.6 | 0 → 406 m |
| S2 | 1005.6 | 1015.0 | 406 → 812 m |
| S3 | 1960.7 | 955.2 | 812 → 1,217 m |
| S4 | 2855.9 | 973.7 | 1,217 → 1,623 m |

⚠️ 唯一のループ (始点↔終点) は S1 と S4 にまたがるため、分割すると失われる。

## 10. 高密度化で odometry が追従喪失する — 原因確定 (2026-10-23)

**ステータス: 原因確定・修正済み。** `preprocess.random_downsample_target` が生点数を超えると
間引きが無効化され、odometry が走行途中で追従を失う。目標点数を生点数未満に戻すだけで直る。

### 症状

`shinnara_ultra` (キーフレーム 0.07 m / 57 枚、submap 格子 0.02 m、上限 90 万点、遠方 180 m、
外れ値除去 ON、目標点数 50,000) で 4 区間を処理したところ、推定された走行距離が実際より最大 40% 短くなった。

| 区間 | 車輪オドメトリ | 通常密度 | ultra |
|---|---|---|---|
| S1 | 446.9 m | 410.8 m (92%) | 408.6 m (91%) |
| S2 | 445.2 m | 434.2 m (98%) | 353.1 m (79%) |
| S3 | 463.4 m | 450.3 m (97%) | **278.6 m (60%)** |
| S4 | 427.7 m | 419.6 m (98%) | 344.8 m (81%) |

**「縮む」のではなく「途中で地図を作らなくなる」。** S3 では区間開始から 500 秒まで通常密度と
完全一致 (比 1.00) し、bag 時刻 2480 秒で 0.26 → 0.05 m/s と速度を失って回復しない。
実際のロボットは 0.50 m/s で走り続けている。z も −2.3 m に張り付き、実際の上り坂 (+2 m) を追えない。
追従を失うとキーフレーム生成条件 (0.07 m 移動) が満たされなくなるため、以後 submap が作られず
地図がそこで終わる (貼り付き点から 6 m 以内の点は 0.0% = 積み上がりは起きない)。

破綻地点は**両側を植え込みに挟まれた、進行方向に特徴のない直線通路**。
同じ場所を通常密度は問題なく通過する。

### 原因

`gtsam_points/src/gtsam_points/types/point_cloud_cpu_funcs.cpp` の `randomgrid_sampling` 冒頭:

```cpp
if (sampling_rate >= 0.99) {
  return PointCloudCPU::clone(*frame);   // 間引きせず素通し
}
```

`sampling_rate = random_downsample_target / 点数`。R-Fans の 1 スキャンは**生点 30,045 / 有効 27,114**
なので、目標 50,000 では rate = 1.84 となり **間引きが一度も実行されない**。

間引きの実体は一様ランダムではなく、**ボクセルごとの上限による空間的な均し**:

```cpp
const size_t points_per_voxel = std::ceil((sampling_rate * frame->size()) / num_voxels);
// 各ボクセルの点数が上限を超えたら std::sample で上限まで間引く
```

これが効くかどうかで点の構成が変わる (破綻地点付近のスキャンで実測):

| | 点数 | 距離中央値 | 5 m 以内 | **20 m 超の点数** | **20 m 超の重み** |
|---|---|---|---|---|---|
| 素通し (ultra) | 27,157 | 3.8 m | 69% | 525 点 | **1.9%** |
| 間引き後 (通常密度、ボクセル上限 12 点) | 5,416 | 7.8 m | 27% | **525 点** | **9.7%** |

**遠方点の絶対数は 525 点で不変。変わるのは重みだけで 5 倍。**
近傍の密なボクセルが上限で頭打ちになり、遠方の疎なボクセルは全点が残るため。

進行方向を拘束できるのは遠方の構造物だけで、足元の地面は進行方向を拘束せず、植え込みの葉は
面にならず共分散が定まらない。素通しすると前者の重みが 2% を切り、通路方向に照合が滑って解を失う。

### 切り分け (最小再現 bag 2400-2550 s / 150 秒 / 実距離 75.0 m、1 本 2 分)

| config | 変更点 | odom 距離 | 実距離比 |
|---|---|---|---|
| 通常密度 | — | 72.4 m | 97% |
| `shinnara_ultra` | — | 33.7 m | **45%** |
| `rp_tgt10k` | `random_downsample_target` 50,000 → 10,000 **のみ** | 72.3 m | **96%** |
| `rp_tgt10k_res10` | 上 + `downsample_resolution` 1.0 | 72.4 m | 96% |
| `rp_prep_default` | preprocess を全部既定へ | 72.3 m | 96% |
| `rp_k54` | `k_correspondences` 54 のみ | 46.1 m | 61% |
| `rp_noout` | 外れ値除去 off のみ | 47.5 m | 63% |
| `rp_k54_noout` | 上 2 つ同時 | 47.1 m | 63% |
| **`shinnara_fixed`** | **目標点数 10,000 + 外れ値除去 off** | **72.1 m** | **96%** |

全区間 S3 (955 秒) でも同傾向: `k_correspondences` 54 で 70%、目標点数 10,000 で 81%。

### 解決

**`ros2_ws_glim/config/shinnara_fixed`** = `shinnara_ultra` から 2 項目のみ変更。

| | 値 | 理由 |
|---|---|---|
| `random_downsample_target` | 50,000 → **10,000** | 縮退の唯一の原因。生点 27,114 未満にして間引きを有効化する |
| `enable_outlier_removal` | true → **false** | 縮退とは無関係だが、S3 実測で traj z 幅が 12.88 → 5.63 m と半減 |

sub_mapping 側 (キーフレーム 0.07 m / 57 枚、格子 0.02 m、上限 90 万点) は ultra のまま。
**最小再現で実距離比 96%、点数は通常密度の 11.8 倍。**

### 教訓

- **地図の密度を決めるのは sub_mapping 段、軌跡の精度を決めるのは preprocess → odometry 段。
  この 2 つは独立に設定できる。** 高密度地図が欲しいときに preprocess まで上げる必要はない。
- `random_downsample_target` は**生点数未満に保つこと**。超えると間引きが静かに無効化され、
  警告も出ない。R-Fans-16 の有効点は約 27,000。
- 間引きは「情報を捨てる処理」ではなく「足元の過剰な点を削って遠方の発言権を回復させる処理」。

### 棄却した仮説 (いずれも実験で否定)

1. submap 内の全結合最適化がキーフレームを引き寄せて縮める → `enable_optimization: false` で変化なし
2. キーフレーム間隔 0.07 m では視差が不足する → 0.15 m で変化なし
3. 遠方 180 m のノイズが照合を引っ張る → 100 m で変化なし。そもそも 100 m 超の点は 1 スキャン **8 点**
4. `downsample_resolution` 0.15 m が odometry のボクセル地図を変える → 1.0 m で変化なし
   (目標点数が生点を超えている間、ボクセル解像度は結果に影響しない)
5. 共分散の近傍半径が狭まる → `k_correspondences` 54 で 61%、ほぼ無効
6. フレーム全体の距離中央値が変わり odometry のボクセル解像度が変わる → 素通しでも間引き後でも
   中央値は変わらず (どちらも p=0 でボクセル 0.25 m)

## 11. shinnara_fixed による 4 区間の本番結果 (2026-10-25) — 部分的解決

| 区間 | 車輪 | 通常密度 | ultra (壊) | **fixed** | 点/m | traj z 幅 |
|---|---|---|---|---|---|---|
| S1 | 446.9 m | 410.8 (92%) | 408.6 (91%) | **418.7 (94%)** | 103,398 | 7.94 m |
| S2 | 445.2 m | 434.2 (98%) | 353.1 (79%) | **395.7 (89%)** | 106,966 | 13.22 m |
| S3 | 463.4 m | 450.3 (97%) | 278.6 (60%) | **373.3 (81%)** | 112,750 | 8.00 m |
| S4 | 427.7 m | 419.6 (98%) | 344.8 (81%) | **418.5 (98%)** | 109,321 | 3.81 m |

合計 **1 億 7,536 万点 / 平均 109,177 点/m** (通常密度の 9.6 倍)。dump は各 2.2〜2.3 GB。

### 到達点

- **S1 / S4 は解決。** 通常密度と同等以上 (S1 は 92% → 94% と上回る)。そのまま使える。
- **S2 / S3 に滑りが残る。** ただし性質が変わった — ultra は追従を失うと**恒久的に停止**したが、
  fixed は滑っても**自力で復帰する**。

| | 欠損区間 | その後 |
|---|---|---|
| S3 (ultra) | bag 2480 s 以降 | 最後まで回復せず |
| S3 (fixed) | bag 2461〜2761 s | **bag 2761 s で比 1.00 に復帰** |
| S2 (fixed) | bag 1406〜1506 s の 100 秒のみ | 復帰 |

### 最小再現の限界 (教訓)

`scripts/glim_repro.sh` (bag 2400-2550 s) では fixed が 96% を出したが、全区間では同じ場所で 0.66 に落ちた。
**再現窓は bag 2400 からまっさらな状態で開始するため、履歴を持って同じ場所に進入する本番とは条件が違う。**
2 分で回せる利点と引き換えに、この差を見落とした。再現窓での「直った」は全区間での保証にならない。

### 残る差分 (通常密度 vs fixed、odometry に効くもの)

| | 通常密度 | fixed |
|---|---|---|
| `downsample_resolution` | 1.0 m | **0.15 m** |
| `distance_far_thresh` | 100 m | 180 m (実質無効 — 100 m 超は 1 スキャン 8 点) |
| `k_correspondences` | 20 | 25 |

目標点数を 10,000 に戻したことで rate < 0.99 となり、**ここで初めて `downsample_resolution` が結果に効く**
(それ以前は素通しだったので何に設定しても同じだった)。0.15 m では points_per_voxel がほぼ 1 になり、
立体的に広がる植え込みが平面の地面や壁より多く残る。`shinnara_fixed2` (0.15 → 1.0) で検証中。

## 12. shinnara_fixed2 で縮退が完全解消 (2026-10-25)

`shinnara_fixed` に残っていた S2 / S3 の滑りは、**`downsample_resolution` を 0.15 → 1.0 m に戻すだけ**で消えた。

### 軌跡の健全性 — 設定ごとの実距離比

車輪オドメトリの走行距離を 100% としたときの、GLIM odom 段の推定走行距離。

| 設定 | S1 | S2 | S3 | S4 |
|---|---|---|---|---|
| 車輪オドメトリ (基準) | 446.9 m | 445.2 m | 463.4 m | 427.7 m |
| 通常密度 | 92% | 98% | 97% | 98% |
| `shinnara_ultra` | 91% | 79% | **60%** | 81% |
| `shinnara_fixed` | 94% | 89% | **81%** | 98% |
| **`shinnara_fixed2`** | **93%** | **97%** | **97%** | **98%** |

`fixed2` は全区間で通常密度との差が 1% 以内。S1 の 93% は通常密度も 92% なので区間固有の特性であり、
設定の問題ではない (`fixed2` の方が 1 ポイント上)。

### `fixed2` の中身

| | 点/m | 点数 | traj z 幅 | dump |
|---|---|---|---|---|
| S1 | 62,244 | 2,597 万 | 8.85 m | 1.3 GB |
| S2 | 66,005 | 2,878 万 | 17.59 m | 1.5 GB |
| S3 | 72,048 | 3,273 万 | 8.05 m | 1.7 GB |
| S4 | 67,141 | 2,863 万 | 9.90 m | 1.5 GB |
| **合計** | **平均 67,413** | **1 億 1,611 万** | — | **6.0 GB** |

走行 1,722 m。通常密度 (11,419 点/m) の **5.9 倍**。

### `seg_fixed` と `fixed2` の違い — 1 項目だけ

dump 内の実行 config を全ファイル比較した結果、差は `downsample_resolution` のみ (0.15 → 1.0 m)。
`random_downsample_target` 10,000 / `enable_outlier_removal` false / `distance_far_thresh` 180 m /
`k_correspondences` 25 / sub_mapping (キーフレーム 0.07 m・57 枚、格子 0.02 m、上限 90 万点) は同一。

`random_downsample_target` が生点 27,114 未満なので間引きが実際に動く。そのとき
`downsample_resolution` は**どの大きさのボクセルで均すか**を決める:

```
points_per_voxel = ceil(sampling_rate * 点数 / ボクセル数)
```

| ボクセル | ボクセル数 | 1 ボクセルの上限 | 残りやすいもの |
|---|---|---|---|
| 0.15 m (`seg_fixed`) | 非常に多い | **約 1 点** | 立体的に広がる**植え込み** (葉が多数のセルを占める) |
| 1.0 m (`fixed2`) | 878 個程度 | **約 12 点** | 地面・壁などの**平面構造** |

葉は 3 次元的に散らばるので占めるセル数が多く、細かいボクセルでは「セルの数だけ点が残る」= 葉が有利になる。
葉は面にならず共分散が定まらないため照合の質が落ちる。1.0 m なら平面も相応のセルを占め、構造物の比重が戻る。

### トレードオフ

| | 点/m | 実距離比 |
|---|---|---|
| `seg_fixed` (ボクセル 0.15 m) | 103,398〜112,750 | 81〜98% (S2/S3 が滑る) |
| `fixed2` (ボクセル 1.0 m) | 62,244〜72,048 | **93〜98% (全区間健全)** |

**点は 1.6 倍多いが軌跡が滑る**、という関係。植え込みの点は「地図としては情報だが、照合の材料としては邪魔」。
使うなら `fixed2`。

### 密度の到達点

| | 点/m | 生点 (約 64 万点/m) に対する保存率 |
|---|---|---|
| 従来の既定 | 11,419 | 1.8% |
| **`fixed2` (確定)** | **67,413** | **10.5%** |
| `shinnara_ultra` (幾何的に破綻) | 112,750 | 17.6% |

この上限を決めているのは資源ではなく「**odometry に渡す点を増やすと照合が壊れる**」という性質。
sub_mapping 側はさらに濃くできるが、preprocess 側は 10,000 点・ボクセル 1.0 m に抑える必要がある。

### z ドリフトについて (2026-10-25 方針決定)

**追わないことにした。** LT 大会での報告によれば、つくばチャレンジの地図では LIO 単体で 10〜20 m の
z ずれが出るのが一般的で、その発表者は RTK で補正していた。本プロジェクトの実測 (新習志野 1.6 km で 35 m、
つくば 2.3 km で 27〜80 m) もその範囲にあり、**設定の失敗ではなく LIO という手法の素の性能**と判断する。

- **GLIM 1.2.2 に GNSS を受ける口は無い** (`gnss|gps|navsat|rtk` の grep が 0 件)。RTK を積む場合は
  自作 extension module か、軌跡を後補正して地図を描き直す 2 パス方式が要る。
- **Nav2 用 2D 占有格子には実害がない。** `tools/glim_dump_to_2dmap` の `--height_mode sensor` /
  `base_link` は高さをセンサ基準で切るため、地図の z がドリフトしても帯が一緒に上下する (tools/README.md)。
- 3D 地図としての目視・3D 位置推定・seg 間の自動ループ閉合には効く。手動接続には
  `libmap_editor.so` / `libinteractive_viewer.so` が未使用のまま存在する。
