<!-- claude: 3D地図→Nav2 接続読本 第7章 (2026-08-17) -->

# 第7章 実務チェックリスト — コマンドと逆引き

理屈抜きで作業するための章。コマンドは 2026-08-17 に実際に動かしたものをそのまま。

## 7.1 地図変換 (glim コンテナ)

```bash
# 0) 前提: glim コンテナ稼働 + tools マウント (docker compose up -d glim)
#    ビルドが未なら tools/README.md の 3 行 (cmake は -include フラグ必須)

# 1) PLY → PCD
docker exec glim_env pcl_ply2pcd /workspace/bags/<...>/map.ply /tmp/map.pcd

# 2) 床 z と xy 範囲の実測 (毎回やる。床は z=0 ではない — 第6章 事例C)
docker exec glim_env bash -c "python3 -c \"
import numpy as np
raw = open('/workspace/bags/<...>/map.ply','rb').read()
pts = np.frombuffer(raw, dtype=np.float32, offset=raw.index(b'end_header')+11).reshape(-1,3)
h,e = np.histogram(pts[:,2], bins=np.arange(pts[:,2].min(), pts[:,2].max(), 0.05))
print('z_floor %.3f  x %.1f..%.1f  y %.1f..%.1f' % (e[h.argmax()]+0.025,
      pts[:,0].min(), pts[:,0].max(), pts[:,1].min(), pts[:,1].max()))\""

# 3) 変換。min/max_height = 床z+0.3 / 床z+1.5。
#    -w -h は同値必須で、max(|x|,|y|)×2÷0.05 以上に (はみ出た点は黙って捨てられる)
docker exec glim_env bash -c "mkdir -p /workspace/maps/glim/<name>/nav2 &&
  /workspace/tools/pointcloud_to_2dmap/build/pointcloud_to_2dmap \
    -r 0.05 -w 2048 -h 2048 --min_height <床z+0.3> --max_height <床z+1.5> \
    /tmp/map.pcd /workspace/maps/glim/<name>/nav2"

# 4) map_dir 規約名へ (image 参照の追従を忘れると map_server が起動失敗)
docker exec glim_env bash -c "cd /workspace/maps/glim/<name>/nav2 &&
  mv map.png my_map.png && sed -i 's/^image: map.png/image: my_map.png/' map.yaml &&
  mv map.yaml my_map.yaml"

# 5) 目視: my_map.png を開いて壁の輪郭を確認 (真っ白/真っ黒なら §7.4)
```

### 7.1b z ドリフトのある地図 (屋外・長距離) — glim_dump_to_2dmap を使う

遠方で壁が帯から外れる地図 (第3章 §3.3) は、PLY を経由せず **GLIM dump を直接**
センサ相対スライスで変換する (第3章 §3.4。2026-08-20 追加、5号館 +4.9 m ドリフトで実証):

```bash
# 1) 床のセンサ相対 z を実測 (手順は第3章 §3.4。5号館 08-14 LC は -0.55)
# 2) 変換 (帯 = 床相対値+0.3〜+1.5。dump 直読みなので PLY→PCD は不要)
docker exec glim_env python3 /workspace/tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py \
  /workspace/bags/<dump_dir> /workspace/maps/glim/<name>/nav2 \
  -r 0.05 --map_width 6144 --map_height 6144 \
  --height_mode sensor --min_height -0.25 --max_height 0.95
# 3) 出力は map.pgm + map.yaml (map_server は pgm も可)。規約名にするなら §7.1-4 と
#    同様に mv + yaml の image 行を追従 (pgm のまま my_map.pgm で問題ない)
```

## 7.2 走行 (main コンテナ、AMCL 通しは未検証 2026-08-17)

```bash
# 1) bringup — /scan 衝突回避のため lidar_2d は必ず false (第4章 §4.4)
docker exec -it rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&
  ros2 launch rerobot_bringup rerobot_bringup.launch.py lidar_2d:=false lidar_3d:=true imu:=true ekf:=true"

# 2) R-Fans → /scan
docker exec -it rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&
  ros2 launch rerobot_bringup rfans_scan.launch.py"

# 3) Nav2 (GLIM 由来地図。keepout マスク未作成なので use_keepout:=false)
docker exec -it rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&
  ros2 launch rerobot_bringup nav2.launch.py \
    map_yaml:=/workspace/maps/glim/<name>/nav2/my_map.yaml use_keepout:=false"

# 4) RViz: 2D Pose Estimate で初期姿勢 → 粒子が収束 → Nav2 Goal
```

## 7.3 ハード無しの点検 (bag で /scan 経路を確認)

```bash
# bag 再生 (bag 自身の /scan は混ぜない — topics で絞る)
docker exec -d rerobot_env bash -c "source /opt/ros/jazzy/setup.bash &&
  ros2 bag play /workspace/bags/<...> --topics /rfans_driver/rfans_points /tf /tf_static"
# 別端末で
docker exec -it rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash &&
  ros2 launch rerobot_bringup rfans_scan.launch.py"
docker exec -it rerobot_env bash -c "source /opt/ros/jazzy/setup.bash && ros2 topic hz /scan"
# ⚠️ echo するなら QoS 指定 (第4章 §4.4): ros2 topic echo --qos-reliability best_effort /scan
```

## 7.4 症状逆引き表

| 症状 | 第一容疑 | 確認・対処 |
|---|---|---|
| 変換した地図が真っ白 | 高さ帯に点が無い (床 z の誤り / 帯が空中) | §7.1-2 で床 z を再実測。`--min/max_height` は**絶対 z** (床からの相対ではない) |
| 変換した地図が真っ黒 | 帯に床点が混入 (帯下限が低すぎ / z ドリフト) | 下限を床+0.3 以上に。ドリフト地図なら §7.1b |
| 遠方だけ壁が消える | z ドリフトで帯から壁が外れた (床が場所により滑る) | §7.1b の glim_dump_to_2dmap (センサ相対スライス) へ切替。機構は第3章 §3.4 |
| 地図の一部が欠ける | `-w -h` が小さくはみ出し (無警告で捨てられる) | xy 実測値から再計算。max(絶対値)×2÷resolution 以上 |
| 地図が歪む/中心ずれ | `-w` ≠ `-h` (y 中心計算の実装バグ) | 必ず同値にする (第3章 §3.2) |
| map_server が起動失敗 | yaml の image 参照切れ (改名の追従漏れ) | §7.1-4 の sed を確認 |
| `ros2 topic echo /scan` が無言 | best-effort QoS (echo 既定は reliable) | `--qos-reliability best_effort` を付ける。AMCL には届いている |
| /scan は出るが AMCL が暴れる | 帯不整合 (地図と /scan で高さ帯が違う) / /scan 二重 (urg_node 同時起動) | §7.5 の連動点検。`ros2 node list` で urg_node がいないか |
| 自己位置が常に一定量ずれる | 地図 origin の誤り (全体平行移動) | yaml の origin と変換時の -w -h・resolution の整合を検算 (第2章 §2.3) |
| 粒子が収束しない | 初期姿勢の与え忘れ / 開けた場所で /scan が痩せている | 2D Pose Estimate をやり直す。屋外なら第5章 (本格案) の動機そのもの |

## 7.5 変更時の連動点検 (帯・距離は 2 箇所セット)

| 変えたい値 | 連動して変える場所 |
|---|---|
| 高さ帯 (0.3〜1.5) | ① 変換コマンドの `--min/max_height` (床 z 加算後) ② `rfans_scan.launch.py` の `min/max_height` 引数 |
| 最大距離 (30 m) | ① `rfans_scan.launch.py` の `range_max` ② `nav2_params.yaml` amcl `laser_max_range` (③ costmap の raytrace/obstacle_max_range も整合確認) |
| 解像度 (0.05) | 変換コマンドの `-r` のみ (map_server は yaml から読む)。costmap resolution とは独立 |
| rfans 取付プリセット | URDF + GLIM config の cp 2 発 (CLAUDE.md 参照)。**rfans_scan 側は変更不要** (base_link 変換が吸収) |

## 7.6 実機再開時の残タスク (2026-08-17 時点)

- [ ] AMCL 込みの通し検証: §7.2 の手順で 2D Pose Estimate → 収束 → Nav2 Goal
- [ ] 傾け取付 (tilted45) での /scan カバレッジ確認 (後方が薄くないか RViz で)
- [x] 屋外地図での変換 — 2026-08-20 実測完了。5号館 (344 m, ドリフト +4.9 m) で
      絶対 z 方式は遠方の壁が全滅 → glim_dump_to_2dmap (センサ相対) へ切替 (§7.1b)
- [ ] keepout マスクの作成 (未探索領域=自由の制約の運用面での補い)
- [ ] 縦角再較正 (保留中の本命 — 地図品質と z ドリフトの根本対策)

→ [00_index に戻る](00_index.md)
