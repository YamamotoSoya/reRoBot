<!-- claude: 2026-10-11 新設 -->
# loc_eval — 自己位置推定の再生評価 (2D AMCL / emcl2 / 3D lidar_localization 共通)

bag を再生して自己位置推定を 1 構成ずつ動かし、**TF map→base_link** を GLIM の軌跡 (= 地図を作った座標系の基準) と
位置合わせなしで突き合わせる。手法が違っても出力 TF は同じなので、同じ物差しで比べられる。
指標の定義は上流 `lidar_localization_ros2/tools/benchmark/benchmark_eval.py` に揃えた。

```
基準軌跡                 make_reference.py   traj_lidar.txt → ref_base.tum (base_link 姿勢) + initialpose.yaml (先頭姿勢)
  ├ 別 bag のとき        register_maps2d.py  別 bag の 2D 地図を地図側の 2D 地図に ICP → T_map_from_src.txt (--transform で渡す、粗合わせ)
  └ 別 bag のとき (必須) refine_reference.py 各時刻のスキャンが地図に最も乗る姿勢で基準を補正 → ref_*_fit/ (剛体変換だけでは 1〜2 m 残る)
再生 1 本                replay_one.sh       部品 launch (use_sim_time) → record_pose.py (TF 記録 + /initialpose 送信) → bag play
採点                     score.py            err.csv / score.json / summary.md (群 = run 名の _rN を除いた部分、中央値と範囲)
作図                     plot.py             timeline.png (誤差の時系列) / cdf.png / map_<群>.png (地図上に誤差で色分け) / worst_<群>.png
診断                     scan_fit.py         ある時刻のスキャンを基準姿勢と推定姿勢で地図に置き、壁に乗る割合 (hit) を比べる
```

## 使い方 (main コンテナ内: ホストで `docker exec -it rerobot_env bash` してから)

```bash
E=/workspace/bags/exp/2026-10-11_loc_eval
M=/workspace/maps/2d/glim/2026-10-09_1643_5goukan_chal
B=/workspace/bags/raw/2026-10-09_1643_5goukan_chal

# 1. 基準軌跡 (地図を作った bag 自身なら変換なし)
python3 /workspace/tools/loc_eval/make_reference.py /workspace/maps/3d/2026-10-09_1643_5goukan_chal/traj_lidar.txt $E/ref_1643

# 2. 再生 1 本 (AMCL × traj 版 0.2-1.7 地図)。反復は run 名の末尾 _r1, _r2, ... を変えて直列に
bash /workspace/tools/loc_eval/replay_one.sh $B $E/1643/amcl_traj0217_r1 amcl $E/ref_1643 \
  loc_map_yaml:=$M/raw_traj/0.2-1.7/map.yaml scan_topic:=/rfans/scan
#    emcl2:   ... emcl2   $E/ref_1643 loc_map_yaml:=... scan_topic:=/rfans/scan
#    lidar3d: ... lidar3d $E/ref_1643 map_path:=/workspace/maps/3d/<名>/map.pcd traj_path:=/workspace/maps/3d/<名>/traj_lidar.txt odom_topic:=/odom
#    env: DUR=120 (先頭だけ試す) RATE=1.0 START=0 INIT_DELAY=3

# 3. 採点 + 作図
python3 /workspace/tools/loc_eval/score.py $E/ref_1643 "$E/1643/*_r*" --out $E/1643
python3 /workspace/tools/loc_eval/plot.py  $E/ref_1643 $M/raw_traj/0.2-1.7/map.yaml $E/1643/plots "$E/1643/*_r*"

# 4. 見失いの診断 (initialpose から 60 s, 90 s のスキャンを基準/推定姿勢で地図に置く)
python3 /workspace/tools/loc_eval/scan_fit.py $B $E/ref_1643 $M/raw_traj/0.2-1.7/map.yaml $E/1643/scanfit --times 60 90 --run $E/1643/amcl_traj0217_r1
```

別 bag (例 1611) で評価するとき:

```bash
# 1611 の GLIM dump から 2D 地図 (地図側と同じ帯、base_link 基準) を作っておき、地図側へ粗く合わせる
python3 /workspace/tools/loc_eval/register_maps2d.py <1611 の raw_dump/0.3-3.0/map.yaml> $M/raw_dump/0.3-3.0/map.yaml $E/reg_1611 --coarse
python3 /workspace/tools/loc_eval/make_reference.py <1611 dump>/traj_lidar.txt $E/ref_1611 --transform $E/reg_1611/T_map_from_src.txt
# 剛体変換だけでは 1〜2 m 残る (2 本の GLIM 地図は形が少し違う) → スキャン当てはめで補正 (rosbag2_py が要るので ROS を source)
source /opt/ros/jazzy/setup.bash
python3 /workspace/tools/loc_eval/refine_reference.py /workspace/bags/raw/2026-10-09_1611_5goukan_chal $E/ref_1611 $M/raw_traj/0.2-1.7/map.yaml $E/ref_1611_fit
# refine.png で補正量が滑らかなこと・hit が高いことを確認。refine.json の dev_from_trend_p95_m が「測れる誤差の下限」の目安。
# 以降は ref を $E/ref_1611_fit に替えるだけ。地図自身の bag (1643) にかけると補正 ≈ 0.05 m になる (方法の検証)
```

## 指標 (summary.md の列)

| 列 | 意味 |
|---|---|
| 収束 s | initialpose から、xy 誤差 < 0.3 m が 1 s 続くまでの秒 |
| xy 中央値 / p95 / 最大 | 水平誤差 [m] (位置合わせなし、地図座標) |
| >0.5 m / >1 m | その誤差を超えていた時間の割合 (1 m = 見失い) |
| yaw 中央 / p95 | 向きの誤差 [deg] |
| 見失い 回 / 合計 s | >1 m が 1 s 以上続いた区間の数と合計秒 |
| 申告 当たり / 誤警報 | 実誤差 >1 m のとき手法が「怪しい」と言えた割合 / <0.3 m のとき怪しいと言った割合。amcl/emcl2 = sqrt(cov_xx+cov_yy) > 0.5 m、lidar3d = /alignment_status level ≠ 0 |
| 記録率 | 基準の時間のうち推定が取れていた割合 |

## 注意

- **地図座標のまま比べる**。基準に合わせてから採点すると「違う場所に初期化してそのまま追従」が隠れる。
- 地図を作った bag 自身での評価は「地図と整合する上限値」。本番の再現性は別 bag で見る。
- AMCL / emcl2 は乱数で run ごとに結果が変わる → 反復 (3 回以上) を直列で回して中央値と範囲で見る。
  AMCL の種は configure した秒 (`meta.json` の launch_epoch)。
- 初期姿勢は record_pose.py が「bag 開始時の姿勢 + それ以降の odom 移動量」で補正して送り、TF map→base_link が一致する
  (events.txt の `init_ok`) まで 1 s ごとに送り直す。一致後は送らない (手法の見失いをこちらが直してしまわないため)。
  `init_ok` が無い run は初期化に失敗している。
- `ros2 bag play` は TTY なしだと SIGINT を無視するので replay_one.sh は SIGTERM で止める。
- `--start-offset` 付き再生は /tf_static を流さない → START>0 のときは先に /tf_static だけ短く再生する。
- est.tum の時刻は **TF 連鎖の最新共通時刻** (= map→odom と odom→base_link の古い方)。姿勢の誤差はその時刻で採点し、
  「いま」との差は鮮度 (latency.txt → summary の「鮮度」列) として別に出す。Nav2 も同じ lookup で姿勢を得るので、
  鮮度が悪い = 実運用で古い姿勢を使わされる。「いま」の時刻で採点すると処理遅れが鋸歯状の位置誤差に化ける (10-11 lidar3d で実測)。
- 記録率は TF の更新頻度に依存する (AMCL は transform_tolerance 1.0 で未来時刻を出すので 20 Hz で取れる、emcl2 は
  scan 周期、lidar3d は処理周期)。低くても異常ではない。

> 系統立てたまとめ (現象 → 仮説 → 実験の系統 → 判断基準 → 結果総表 → 判定) は `docs/issue/2026-10-11_loc_eval_2d_corner_loss_vs_3d.md`。以下は時系列の生記録。

## 2026-10-11 疎通試験で分かったこと (1643 bag 先頭 120 s)

- **dump 版 0.2-1.7 地図 (r30)** は地面のまだら占有が多く、4 m ずれた姿勢でもスキャンの 95 % が壁に乗る (基準姿勢 100 %)
  → 尤度場が平らで AMCL が 1 分で 9 m 流れた。traj 版は基準 0.99 / ずれ 0.67 で判別できる (`scan_fit.py`)。
- **emcl2** は /rfans/scan を reliable で購読し、bag の best_effort 記録と QoS 不一致で 1 本も届かなかった
  → `qos_override.yaml` で再生側を reliable に (AMCL の best_effort 購読にも届く)。
- **lidar3d** は上流プリセットのままだと 1 スキャン 0.6〜0.8 s + 局所地図再構築 (10 m ごと、約 10 s 停止) で鮮度 p95 8.5 s。
  `variants/lidar3d_fast_res{2,5}.yaml` (scan voxel 0.3 m + 局所地図切り出しなし) で処理 0.2 s・鮮度 p95 0.5 s、精度不変。
  voxel 0.5 m と局所地図縮小 (radius 60) はどちらも照合を壊す (全棄却 / 86/113 棄却)。

## 2026-10-11 夜間一括評価 (1611 bag) で分かったこと

- **AMCL が初期姿勢を失っていた** (critic 査読済み: 条件付き支持): AMCL は最初のスキャンで尤度場を作る間
  (traj 版地図 7616×4288 @0.05 m で ≈13 s、dump 版 @0.1 m で ≈3.6 s、セル数に比例) 内部の mutex を握り、**その間に
  /initialpose が届くと** main スレッド (bond の心拍を含む) がそこで止まる (initialpose を送らなかった 2 回目のブロックでは
  心拍は止まっていない)。既定 `bond_timeout` 4 s で manager が「amcl IS DOWN」→ deactivate→cleanup→再起動し、直前の
  /initialpose は消えて粒子は (0,0,0) に作り直される (replay.log に記録)。1643 の疎通試験でも起きていたが開始姿勢 ≈ (0,0)
  で見えなかった。実機でも「最初のスキャン後 ≈13 s 以内に RViz で 2D Pose Estimate を送る」と同じことが起きる
  (0.1 m 地図でも余裕 0.4 s)。→ `loc_amcl.launch.py` で `bond_timeout: 60.0` + record_pose.py の odom 補正つき再送
  (1611 先頭 120 s で検証: リセットなし、開始 +19 s に `init_ok`)。無効になった run は `1611/invalid_amcl_bondreset/` に退避。
- **別 bag の基準が 1〜2 m ずれていたのは 2D 地図 ICP の yaw 誤り** (critic 査読で「GLIM 地図が非剛体」説は棄却):
  `register_maps2d.py` の yaw 総当たりの評価値 (最近傍距離の中央値) が密なまだら占有地図では yaw に盲目で
  (0° 0.085 / −1° 0.091 m)、真の −1.0° を見逃していた。1° × 腕 90 m = 1.6 m が「経路に沿って滑らかに振れる 1〜2 m」の正体
  (スキャン当てはめの補正量を剛体フィットすると θ −1.05°、残差 0.08 m / p95 0.36 m = 地図自身の bag と同水準)。
  評価値を 2 m 打ち切り平均に替えて再登録 → yaw −1.01°、0.2 m 以内 90 %。`refine_reference.py` は残った局所ずれの補正と
  登録失敗の検出器 (補正量が位置の一次関数で説明できれば登録誤差) として使う。

## 2026-10-11〜12 全体評価 (1611 独立 bag 11 本 + 1643 地図元 bag 5 本 + 追試 2 本、critic 査読済み)

結果 (`$E/{1611,1643}/summary.md`、図 `plots/`)。基準 = `ref_1611_fit` / `ref_1643_fit`、見失い = xy 誤差 >1 m:

| 構成 | 1611 xy 中央値 | 1611 見失い | 1643 xy 中央値 | 1643 見失い | 5 m 超に落ちる位置 |
|---|---|---|---|---|---|
| lidar3d A (ndt 2.0, 全地図, voxel 0.3) | 0.07 m | 0 | 0.05 m | 0 | — |
| lidar3d B (ndt 5.0) | 0.10 m | 2 回 7 s | 0.10 m | 6 回 22 s | x≈105〜125, y≈−58 (南通路、両 bag で同じ場所、自己申告は level 0 のまま) |
| AMCL traj 0.2-1.7 (×3) | 70 m | 377〜383 s 以降復帰なし | 40 m | 411 s 以降 | x≈150〜157, y≈−26〜−30 (東端の角) |
| AMCL traj 0.3-3.0 (×3) | 73 m | 318 s 以降 | 44 m | 370 s 以降 | 同上 |
| emcl2 traj 0.2-1.7 (×3) | 70 m | 314〜319 s 以降 | 36 m | 411 s 以降 | 同上 |
| AMCL dump 0.2-1.7 | 44 m | 44 s から | — | — | 開始直後 (尤度場が平ら、疎通試験と同じ) |
| **追試** AMCL traj 0.2-1.7 alpha1〜4 0.02 (`variants/amcl_alpha002.yaml`) | **0.63 m** | 6 回 372 s、**400 s 以降は 0.4〜0.6 m に復帰** | — | — | 角では同じく 5 m まで外れるが戻る |
| **追試** AMCL × 随伴者の帯を軌跡周り 1 m 消した地図 (`clear_path.py`) | 70 m | 242 s 以降 (変化なし) | — | — | 同上 |

**確定したこと**
- **3D (ndt 2.0) は両 bag で 0.05〜0.07 m・見失い 0**。実機の既定 (`config/localization/lidar_localization.yaml`) はこの設定。
  ndt 5.0 は場所固定の 3〜4 m 外れがあり不採用。実機の CPU・TF 鮮度・初期姿勢誤差への耐性は**未評価**。
- **2D (AMCL / emcl2、現設定・現地図) は bag に依らず東端の角 (x ≥ 150) で見失い、復帰しない** (反復 3 本すべて同じ時刻 ±5 s)。
  基準軌跡・オドメトリ・再生機構は健全 (critic が再計算: TF odom→base_link の回頭ずれ ≤0.9°/10 s、基準のヨーは GLIM 軌跡と ≤0.3°、
  スキャン 8.6〜9.2 Hz で欠落なし、乱数種の違う run が同時刻に同方向へ外れる = 入力に決定論的に従っている)。
- 崩れ方: 直線通路の**両端** (x≈99〜106 の植え込みの切れ目 + 動体、x≈139〜150 の角手前) で推定が**時計回り −15〜−45°・後ろ・右**へ
  同符号でずれ (12 run 全部)、角直前の 30 s 停止で誤差が凍結したまま、占有率 60〜78% で**尤度場が飽和した東端** (ヨー ±12° でも当たり率 95% 超)
  に入って発散する。通路の内部 (x 110〜135) では誤差は小さい。

**原因は未確定 (critic: 傍証のみ)**。残っている仮説と追試の結果:
- H1 運動ノイズ ≫ odom 品質: amcl.yaml の alpha1〜4 0.2 は 0.25 m 更新ごとにヨー ≈9° (1σ) をばらまく (実測 odom は ≤0.9°/10 s)。
  **追試 alpha 0.02: 通路のヨー誤差 −10〜−20° は残る (= ヨーのずれ自体はノイズ設定のせいではない) が、角で 5 m 外れたあと復帰し
  以後 0.4〜0.6 m** → 「復帰しない」の方はノイズ設定が主因 (条件付き支持)。along は +1→+3 m と単調に増える (odom スケール +2.5% が通路で拘束されない)。
- H3 随伴者の帯 (地図に焼き込まれた後方 1〜1.5 m の帯) が同符号バイアスを作る: **追試 (帯を消した地図) で変化なし → 棄却**。
  帯は emcl2 の alpha = 0 (毎スキャン膨張リセット、退化設定) の原因としては残る。
- 未検証: 通路両端の「地図に無い点」(215 s 前方 ±45° の 32% が動体、300〜310 s は 20〜30 m 先の点 = 角の先の構造か −1° リングの地面ゴースト)
  が同符号のヨーずれを作る説。H2 東端の地図飽和 (帯を狭める / min_points_in_pix を上げる地図で静的スイープ → 再生) も未実施。
- 「2D スキャンの情報が少ない」は傍証どまり: AMCL の尤度式で測ると通路の ±12° 残存率 61〜84% vs 非通路 50〜56% で鈍いがピークは真値にある。
  `scan_fit.py` の当たり率 (0.3 m) は AMCL の尤度より平らに見えるので、平らさの議論は `fit_landscape.py --model amcl` を使う。

**次の実験 (安い順)**: ① alpha 0.02 を 1643 でも 1 本 + max_beams 180 ② 帯を狭めた地図 (0.8-1.7 / min_points_in_pix↑) で東端の静的スイープ
→ 改善すれば AMCL 再生 ③ emcl2 を alpha が 0 にならない設定 (帯を消した地図 + range_threshold) で 1 本 ④ 3D の摂動試験 (初期姿勢 ±1 m / ±10°、odom 劣化)。

診断 script: `fit_landscape.py` (基準姿勢を前後/左右/ヨーに振った当たり率・ヨー断面)、`err_decomp.py` (誤差の前後/左右/ヨー分解)、
`clear_path.py` (軌跡周り R [m] を free にした地図の複製)。

## 2026-10-12 追試: 帯一致地図とユーザ案「帯 0.3〜100 × レーザ距離 100 m × 全点」(1611 bag 13 本、critic 査読済み)

経緯: 上の評価は地図 0.2-1.7 帯 × bag の `/rfans/scan` (bringup 既定 = 帯 0.3〜6.0・range_max 150・AMCL 60 本) で**帯が合っていなかった** (ユーザ指摘)。
帯一致地図 (0.3-6.0 / 0.3-100、range 30) → ユーザ案の全点 → 切り分け、の順に 1611 で回した。
- 全点スキャンは bag に無いので点群から再生成: `ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r __node:=rfans_to_scan_all -r cloud_in:=/rfans_driver/rfans_points -r scan:=/rfans/scan_all -p use_sim_time:=true -p target_frame:=base_link -p min_height:=0.3 -p max_height:=100.0 -p angle_increment:=0.00016 -p range_min:=0.5 -p range_max:=100.0 -p use_inf:=true ...` (rfans_scan.launch.py と同じ刻み、39,270 席) + `scan_relay.py` (best_effort → reliable `/rfans/scan_all_rel`、emcl2 用)。再生成は 7 Hz (bag の /rfans/scan は 10 Hz)。
  ⚠️ `ros2 run` を `kill $!` してもノード本体が残る → `pkill -f rfans_to_scan_all` で止める (残ると次の run で publisher が 2 本になる)。
- 地図: `raw_traj/0.3-100.0_r100`, `raw_traj/0.3-6.0_r100` (glim_traj_to_2dmap `--range_max 100`、他は既存と同じ)。AMCL は `variants/amcl_range100{,_alpha002}.yaml` (laser_max_range 100)、emcl2 は `variants/emcl2_range100.yaml`。
- 実ビーム数: 全点 2000 本は席の 35〜41% しか埋まらないので AMCL が使う実ビームは ≈650〜825 本 (inf は捨てる)、60 本なら ≈22 本。近点 1795 本は全部実ビーム。

| run | スキャン | 地図 | alpha | xy 中央値 | 最大 | 東端の角 (300〜600 s) | 見失い |
|---|---|---|---|---|---|---|---|
| amcl_traj0360 / traj03100 | 近点 60 本 | 0.3-6.0 / 0.3-100 (30 m) | 0.2 | 81 / 74 m | — | 脱落 | 302〜430 s 以降 (0.2-1.7 と同じ) |
| emcl2_traj0360 | 近点 | 0.3-6.0 (30 m) | (0) | 64 m | — | 脱落 | 371 s 以降 |
| amcl_alpha002_traj0360 | 近点 60 本 | 0.3-6.0 (30 m) | 0.02 | 0.60 | 4.25 | 通過 | なし (0.2-1.7 地図では 0.63) |
| amcl_near03100r100 | 近点 60 本 | 0.3-100 r100 | 0.2 | 72 m | — | 脱落 | 363 s 以降 |
| amcl_near03100r100_b1795 | 近点 1795 本 | 同 | 0.2 | — | — | — | 80 s。**CPU 過負荷 (TF 遅れ中央値 1.0 s・最大 2.7 s) で評価不能** |
| amcl_near03100r100_b300_a002 | 近点 300 本 | 同 | 0.02 | 0.53 | 2.49 | 通過 | なし |
| amcl_all03100r100 r1 / r2 | 全点 2000 本 | 同 | 0.2 | 108 / 0.78 | 184 / 59 | **通過** (0.7 m) | r1 590 s 以降永久、r2 623・657 s に 36 / 58 m の一時飛び (南通路) |
| amcl_all03100r100_b60 | 全点 60 本 (実 ≈22) | 同 | 0.2 | 70 m | — | **通過** (1.1 m) | 547 s 以降 (南通路) |
| amcl_all0360r100 | 全点 2000 本・帯 0.3〜6.0 | 0.3-6.0 r100 | 0.2 | 74 m | — | **通過** (1.1 m) | 519 s 以降 (南通路) |
| amcl_all03100r100_a002 r1 / r2 | 全点 2000 本 | 0.3-100 r100 | 0.02 | **0.54 / 0.52** | 2.28 / 1.71 | 通過 | なし |
| amcl_all03100r100_b60_a002 | 全点 60 本 | 同 | 0.02 | 0.54 | 1.53 | 通過 | なし |
| amcl_all0360r100_a002 | 全点 2000 本・帯 0.3〜6.0 | 0.3-6.0 r100 | 0.02 | **0.51** | 5.40 (開始 20 s のみ) | 通過 | なし |
| emcl2_all03100r100 | 全点 (scan_increment 19) | 0.3-100 r100 | (0) | 58 m | — | — | 37〜143 s から (開始直後から劣化、n=1) |
| (参考) lidar3d_A | 3D ndt 2.0 | 3D | — | 0.07 | 0.69 | — | なし |

図: `$E/1611/plots_q5/` (timeline / map_* / worst_*)、全 run の表 `$E/1611/summary.md` (30 run で再採点済み)。

**確定したこと (critic 査読後の表現)**
- 帯一致地図 (0.3-6.0 / 0.3-100) は近点スキャンの角の見失いを変えない — 5 地図 × 近点 10 本が全部 x≈150 で脱落、alpha 0.02 でも 0.60 vs 0.63。**帯のずれは原因ではなかった**。
- レーザ距離 100 m (地図 r100 + laser_max_range 100) 単独も効かない (near03100r100 は従来どおり脱落)。`/rfans/scan` は元々 range_max 150 で 30 m 超のビームが 1〜7% しかなく、効くはずのない変更だった。
- **全点スキャンにすると既定 alpha でも東端の角を通過する** (4/4 vs 近点 0/10)。本数 (実 750 本でも 22 本でも) にも帯 (0.3〜100 でも 0.3〜6.0 でも) にもよらない → レバーは**選点法** (近点 = 0.2° 扇内の最近点で近接クラッタ優先、全点 = 0.009° ビンに単点でリング上の点をほぼランダムに拾う) か、再生成スキャンに付随する何か (cadence 7 Hz 等) で**未分離**。
- **既定 alpha は全点でも運用不可**: 南通路 450〜660 s で粒子分布 sqrt(cov_xx+cov_yy) が 10〜20 m に広がる (既定 alpha の全点 4 本すべて) → どこで崩れるかは乱数 (r1 永久 / r2 一時 / b60・0360 は 519〜547 s 以降永久)。lidar3d_B (ndt 5.0) も同じ通路で 3 m 飛ぶ = 幾何も寄与。
- **alpha 0.02 が主レバー**: 全構成 0.51〜0.63 m (alpha 0.02 では粒子分布 0.4〜1.1 m)。全点 0.51〜0.54 vs 近点 60 本 0.60〜0.63 vs 近点 300 本 0.53 — 差が出るのは角手前 240〜420 s のヨー (近点 60 本 −8〜−13° → 全点・近点 300 本 −2.5〜−9°) と最大値 (4.3〜5.0 → 1.5〜2.5 m) だけ。
- 2D の床 0.5 m は**全 AMCL run 共通の横 +0.5〜1.0 m バイアス** (0〜120 / 720〜840 / 1080〜1200 s、1150〜1230 s は 1.5〜2.5 m、3D は 0)。基準姿勢を ±0.5〜2 m 振った静的尤度は基準が最大 (`fit_landscape.py --model amcl --times 1140 1180`、2 地図とも) → 地図の壁位置ずれでは説明できず**未解明**。
- CPU: AMCL の負荷 ∝ 実ビーム数 × 粒子数。全点 2000 本 (実 ≈750) で 1 コア 100% が数分続くが TF 遅れは 0.044 s を保てた (r1 は私が解析を同時に走らせて 0.4〜0.8 s・更新回数 −15%。負荷なしでも 330〜390 s 等に p95 0.5〜0.8 s のスパイクあり)。近点 1795 本 (実 1795) は破綻。**実機で Nav2 + RViz と同居させるなら max_beams は 2000 でなく 300〜500 で十分** (60 本でも alpha 0.02 なら 0.54)。
- emcl2 の全点は n=1・開始直後から劣化 (alpha=0 退化設定のまま) で、全点の是非は語れない。

**ユーザ仮説「0.3〜100・100 m・全点が最も情報量が多く縮退しにくい」への答え**: 効いたのは**全点 (選点法)** で、帯の高所・遠点と 100 m レンジは同条件の 0.3〜6.0 全点・30 m と差が出なかった。縮退 (南通路の粒子拡散) は既定 alpha では全構成に残り、alpha 0.02 でだけ消える。2D 最良は 0.51〜0.54 m で 3D (0.07 m) の 7 倍、ただしその差の大半は 2D 共通の横バイアス。

**次 (安い順)**: ① 選点法の分離 — scan_all から 0.2° 扇ごとに最近点を選んだスキャン (近点と同じ選点法、全点と同じ帯・cadence) を alpha 0.02 と既定 alpha で 1 本ずつ ② 既定 alpha の全点をあと 2〜3 本 (崩れる位置の乱数幅) ③ 横バイアス +0.5〜1.0 m の由来 (粒子平均の偏り vs 地図投影 — a002 の est と /amcl_pose の粒子統計を区間別に) ④ 実機 2D 候補 = `nav_amcl_scanall` (bringup `rfans_scan_all:=true`) + alpha 0.02 + max_beams 300〜500、3D は nav_lidar3d のまま最良。
