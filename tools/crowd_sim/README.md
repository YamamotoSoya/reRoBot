<!-- claude: 2026-10-24 新設 -->
# crowd_sim — スタート地点の人垣を bag に擬似注入して自己位置推定を再生評価する

つくば本番の最悪想定「スタート 10〜20 m に人が密集し、前方以外の LiDAR 視界が遮られる」を、人垣なしで録った bag から再現する。
人は地図座標に固定した円柱 (半径 0.25 m、高さ 1.75 m)。各フレームの LiDAR 位置 (GLIM `traj_lidar.txt`) からレイキャストし、
人に当たる光線の点を人の表面へ引き寄せる。頭より上を通る上向きリングの点はそのまま残る (3D 自己位置推定が頭越しに建物を見る効果を再現)。

```
crowd_inject.py     bag + traj → 人垣注入 bag (--no-crowd で同じ切り出しの基準 bag)。_crowd.json (配置・統計) と _frame.png (上面図)
run_eval.sh         pointcloud_to_laserscan ×2 (/rfans/scan, /rfans/scan_all、use_sim_time) を立てて tools/loc_eval/replay_one.sh を呼ぶ
queue.sh            2026-10-24 の一括実行 (base/crowd × AMCL 現行 / beam skip / 高帯 2.0 m / lidar3d、直列)
analyze.py          err.csv を「人垣の中 (スタート r_max+5 m 以内)」と「後」に分けて集計、timeline.png / paths_start.png
export_pcd_crop.py  GLIM dump → bbox で切り抜いた binary PCD (lidar3d の地図。2D 地図を作らない軽量版)
```

## 使い方 (main コンテナ内)

```bash
E=/workspace/bags/exp/2026-10-24_crowd_sim; T=/workspace/bags/glim/2026-10-03_1126_tsukuba_dump/composite_4map/traj_lidar.txt
python3 /workspace/tools/crowd_sim/crowd_inject.py /workspace/bags/raw/2026-10-03_1126_tsukuba_seg1 $E/bag_crowd $T --dur 300
python3 /workspace/tools/crowd_sim/crowd_inject.py /workspace/bags/raw/2026-10-03_1126_tsukuba_seg1 $E/bag_base  $T --dur 300 --no-crowd
python3 /workspace/tools/loc_eval/make_reference.py $T $E/ref
PREWARM=1 bash /workspace/tools/crowd_sim/run_eval.sh $E/bag_crowd $E/runs/crowd_amcl_r1 amcl $E/ref \
    scan_topic:=/rfans/scan_all max_beams:=2000 loc_map_yaml:=/workspace/maps/2d/glim/2026-10-03_1126_tsukuba_composite/raw_traj/0.3-100.0_r100/map.yaml
python3 /workspace/tools/loc_eval/score.py $E/ref "$E/runs/*_r*" --out $E/score
python3 /workspace/tools/crowd_sim/analyze.py $E/ref "$E/runs/*_r*" $E/bag_crowd_crowd.json $E/analysis
```

- `PREWARM=1` (replay_one.sh に 10-24 追加): AMCL はつくば composite 地図 (22784×12288 = 280 M セル) の尤度場構築に約 30 s かかり、
  その間 bag が進むと initialpose が効くのは出発後 8 m になる。paused 開始 + `play_next` で点群 1 枚だけ流し、AMCL プロセスの
  CPU が静まるのを待ってから record_pose を起動して resume する。lidar3d では不要 (queue.sh は amcl のときだけ付ける)。
- 人垣の帯: `--r_min/--r_max` (既定 10/20 m)、`--front_half_deg` (既定 45°)、`--density` (既定 0.5 人/m²)、`--seed`。
- 結果と解釈: `docs/issue/2026-10-24_start_crowd_sim_localization.md`。
