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
