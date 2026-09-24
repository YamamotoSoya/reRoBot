<!-- claude: 運用手引き 第12章 テンプレ (2026-09-22) -->

# 第12章 つくチャレ当日の実行手順

---

## 1. docker起動->launch
* 本体電源を入れる。
```
xhost +local:docker
docker compose up glim main
docker exec -it ~~~~~_env bash

ros2 launch rerobot_bringup rerobot_bringup.launch.py
ros2 launch rerobot_bringup wt901_imu.launch.py
ros2 launch rerobot_bringup joy_releop.launch.py
``` 
topicの確認
```
ros2 topic list
以下 topicの存在を確認

```

## 2. bagの記録
<<<リンク　05>>>
```
ros2 bag record -s mcap -o /workspace/bags/5goukan/2d3dimu/online/rosbag/$(date +%F_%H%M) /rfans_driver/rfans_points /rfans_driver/rfans_packets /scan /imu/data /imu_wit/data /imu_wit/mag /odom /tf /tf_static /diagnostics /robot_speed_cmd
```

## 3. 3D SLAM
### **GLIM**
#### 3-1. GLIM-cpu
glim-rosの実行
```
B=2026-08-151030 # 評価対象 bag のディレクトリ名
ros2 run glim_ros glim_rosbag /workspace/bags/5goukan/2d3dimu/online/rosbag/$B \
 --ros-args -p config_path:=/glim_config \
 -p auto_quit:=false \
 -p dump_path:=/workspace/bags/5goukan/2d3dimu/offline/glim/${B}_dump
```
dumpファイルをアップロード
```
rclone copy ローカルファイルパス cit-share-bags:リモートファイルパス
```
dumpファイルをダウンロード
```
rclone copy cit-share-bags:リモートファイルパス ローカルファイルパス
```
PROXMOXでofflineviewer,loopclosing
```
B=2026-08-14_0919
ros2 run glim_ros offline_viewer /workspace/bags/5goukan/2d3dimu/offline/glim/${B}_dump
```
filteredとして保存。アップロード、ローカルにダウンロード

#### 3-2. GLIM-gpu

### **LIO-SAM**
<<<実装待ち>>>

## 4. 2D 圧縮 (traj 版)
[第9章](09_map2d_compression.md) 参照。LC 後 (filtered) の dump を使う。glim コンテナ内で:
```
source /opt/ros/jazzy/setup.bash
T=/workspace/tools/glim_traj_to_2dmap/glim_traj_to_2dmap.py
B=/workspace/bags/5goukan/2d3dimu/online/rosbag/2026-09-180915
D=/workspace/bags/5goukan/2d3dimu/offline/glim/2026-09-180915_filtered_dump   # LC 後 dump (traj_lidar.txt 入り)
O=/workspace/maps/2d/glim/2026-09-180915/nav2                                    # 保存先

python3 $T $B $D $O \
  -r 0.05 --height_frame ground --min_height 0.3 --max_height 1.5 \
  --range_max 30 --deskew --min_points_in_pix 4 --max_points_in_pix 12
```
→ `$O/map.pgm` + `map.yaml` ができる。壁がつながっているか画像で確認。

## 5. map加工
* Nav2 規約名にリネーム (`<map_dir>/nav2/my_map.yaml`)
```
cd /workspace/maps/2d/glim/<name>/nav2
mv map.pgm my_map.pgm && sed -i 's/^image: map.pgm/image: my_map.pgm/' map.yaml && mv map.yaml my_map.yaml
```
* 版管理: `maps/` は親リポジトリの管理外なので、**`maps/2d/` 直下**に専用 git がある (3D 点群地図は `maps/3d/`、git 管理外) (2026-09-24 初期化)。ツール出力 (`map.pgm`) は**編集せず残し**、コピーした `my_map.pgm` だけ編集する。編集のたびに:
```
cd /home/nomface/reRoBot/maps/2d && git add -A && git commit -m "<name>: 何を塗ったか" && git push
git log --oneline -- glim/<name>/           # 履歴
git checkout <hash> -- glim/<name>/nav2/my_map.pgm   # 任意の版に戻す
```
  remote は GitHub の private リポジトリ [reRoBot-2Dmaps](https://github.com/YamamotoSoya/reRoBot-2Dmaps) (SSH、2026-09-24)。別 PC で使うときは `git clone git@github.com:YamamotoSoya/reRoBot-2Dmaps.git reRoBot/maps/2d` (`maps/` 自体は親リポジトリの管理外なので clone 先を合わせる)
  コンテナが書いたファイルは root 所有なので、ホストで編集する前に一度 `sudo chown -R $USER:$USER maps/2d/glim`
* 地図の手直し (任意): `my_map.pgm` を画像エディタで開き、歩行者・車などの動体を白 (自由) に、走らせたくない場所を黒 (占有) に塗る。サイズ・解像度・origin は変えない
* keepout マスク (任意、未整備): `my_map.pgm` を `<map_dir>/keep_out/keep_out.pgm` にコピーして進入禁止帯を黒で塗り、yaml も `keep_out.yaml` としてコピー (`image:` を書き換え)。使わないなら 6 で `use_keepout:=false`

## 6. Nav2,slamtoolbox反映
* GLIM 由来 (3D) 地図: 5 のディレクトリを `map_dir:=` で渡す (7 参照)。amcl の `/scan` は 2D LiDAR ではなく `rfans_scan.launch.py` で作る
* slam_toolbox 由来 (2D) 地図の場合: `maps/2d/slam_toolbox/<name>/nav2/my_map.{pgm,yaml}` に同じ規約で置き、bringup は 2D (`lidar_2d:=true`)、rfans_scan は不要
* 地図の高さ帯 (4 の `--min/max_height 0.3 1.5`) と `rfans_scan.launch.py` の既定 (0.3 / 1.5) は揃えておく。片方だけ変えない

## 7. 自律移動
main コンテナで 3 本 (別ターミナル)。`lidar_2d:=false` 必須 (`/scan` が urg_node と衝突する)
```
# 1) bringup (3D LiDAR + IMU + EKF)
ros2 launch rerobot_bringup rerobot_bringup.launch.py lidar_2d:=false lidar_3d:=true imu:=true ekf:=true
# 2) R-Fans 点群 → /scan
ros2 launch rerobot_bringup rfans_scan.launch.py
# 3) Nav2 + RViz
ros2 launch rerobot_bringup nav2.launch.py map_dir:=/workspace/maps/2d/glim/<name> use_keepout:=false
```
RViz 操作:
1. 「2D Pose Estimate」で現在地と向きをクリック → amcl の粒子 (赤矢印) が数秒で収束することを確認。しなければもう一度
2. 「Nav2 Goal」でゴールを指定 → 緑 (global plan) が出れば走行開始
3. 非常時は joy の deadman (LB) を離す / `robot_speed_cmd` が 0.5 s 途絶すると controller が自動停止
4. 走行中に見るもの: `/amcl_pose` が飛ばないか、local costmap に地図に無い障害物が出ているか

bag を同時に録るなら 2 のコマンドを 4 本目のターミナルで。

← [第11章 amcl](11_amcl.md) | → [第13章 トラブルシューティング](13_troubleshooting.md)
