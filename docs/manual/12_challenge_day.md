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
ros2 launch rerobot_bringup joy_teleop.launch.py
``` 
<!-- claude: 2026-10-01 WT901C (witmotion) は rerobot_bringup.launch.py に統合 — wt901_imu.launch.py の別起動は不要。joy_releop → joy_teleop (typo 修正) -->
* WT901C (`/imu_wit/data`) も bringup 1 本で一緒に起動する (別 launch 不要)。
* bringup 起動の約 15 秒後に **起動結果の表** が出る。全項目 `✔ OK` なら次へ。`✘ NG` / `▲ LOW` の行は `→` のヒントを見て対処し、bringup を再起動する。
topicの確認
```
ros2 topic list
以下 topicの存在を確認

```

## 2. bagの記録
<<<リンク　05>>>
<!-- claude: 2026-10-01 NTP 停止のチェックを追加 -->
* 記録前に **ホストで NTP を止める** (同期済みを確認 → `sudo timedatectl set-ntp false`、走行後に `true` で戻す)。走行中の時刻ジャンプ対策。手順は [第5章 記録前: NTP の自動時刻合わせを止める](05_bag_recording.md#記録前-ntp-の自動時刻合わせを止める)
```
ros2 bag record -s mcap -o /workspace/bags/raw/$(TZ=Asia/Tokyo date +%F_%H%M)_<場所> /rfans_driver/rfans_points /rfans_driver/rfans_packets /scan /imu/data /imu_wit/data /imu_wit/mag /odom /tf /tf_static /diagnostics /robot_speed_cmd /scan_all /amcl_pose
```
<!-- claude: 2026-10-01 全点版の AMCL 入力 /scan_all と推定結果 /amcl_pose を追加 (第14章 2.5) -->

## 3. 3D SLAM
### **GLIM**
#### 3-1. GLIM-cpu
glim-rosの実行
```
B=2026-09-27_1651_5goukan # 評価対象 bag のディレクトリ名 (bags/raw/ 直下)
ros2 run glim_ros glim_rosbag /workspace/bags/raw/$B \
 --ros-args -p config_path:=/glim_config \
 -p auto_quit:=false \
 -p dump_path:=/workspace/bags/glim/${B}_dump/default
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
B=2026-08-14_0919_5goukan
ros2 run glim_ros offline_viewer /workspace/bags/glim/${B}_dump/default
```
filteredとして保存 (`bags/glim/${B}_dump/filtered`)。アップロード、ローカルにダウンロード

#### 3-2. GLIM-gpu

### **LIO-SAM**
<<<実装待ち>>>

## 4. 2D 圧縮 (traj 版)
[第9章](09_map2d_compression.md) 参照。LC 後 (filtered) の dump を使う。glim コンテナ内で:
```
source /opt/ros/jazzy/setup.bash
T=/workspace/tools/glim_traj_to_2dmap/glim_traj_to_2dmap.py
B=/workspace/bags/raw/2026-09-180915_5goukan
D=/workspace/bags/glim/2026-09-180915_5goukan_dump/filtered   # LC 後 dump (traj_lidar.txt 入り)
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
* GLIM 由来 (3D) 地図: 5 のディレクトリを `map_dir:=` で渡す (7 参照)。amcl の入力スキャンは 2D LiDAR ではなく `rfans_scan.launch.py` で作る (当日は全点版 `/scan_all`。[第14章](14_pointcloud_to_laserscan.md))
* slam_toolbox 由来 (2D) 地図の場合: `maps/2d/slam_toolbox/<name>/nav2/my_map.{pgm,yaml}` に同じ規約で置き、bringup は 2D (`lidar_2d:=true`)、rfans_scan は不要
* 地図の高さ帯 (4 の `--min/max_height 0.3 1.5`) と `rfans_scan.launch.py` の既定 (0.3 / 1.5) は揃えておく。片方だけ変えない

## 7. 自律移動
main コンテナで 3 本 (別ターミナル)。`lidar_2d:=false` 必須 (`/scan` が urg_node と衝突する)
```
# 1) bringup (3D LiDAR + IMU + EKF)
ros2 launch rerobot_bringup rerobot_bringup.launch.py lidar_2d:=false lidar_3d:=true imu:=true ekf:=true
# 2) R-Fans 点群 → /scan (costmap 用) + /scan_all (AMCL 用、全点)
ros2 launch rerobot_bringup rfans_scan.launch.py allpoints:=true
# 3) Nav2 + RViz (AMCL は /scan_all を max_beams 2000 で読む)
ros2 launch rerobot_bringup nav2.launch.py map_dir:=/workspace/maps/2d/glim/<name> use_keepout:=false amcl_scan:=all
```
<!-- claude: 2026-10-01 全点版に変更 (ユーザ指示)。従来 (最近点) に戻すなら 2) の allpoints:=true と 3) の amcl_scan:=all を外す。引数の意味と最遠点版は第14章 -->
起動確認: `ros2 param get /amcl scan_topic` が `/scan_all`、`ros2 topic hz /scan_all` が約 10 Hz。従来の最近点に戻すなら 2) の `allpoints:=true` と 3) の `amcl_scan:=all` を外す ([第14章](14_pointcloud_to_laserscan.md))

RViz 操作:
1. 「2D Pose Estimate」で現在地と向きをクリック → amcl の粒子 (赤矢印) が数秒で収束することを確認。しなければもう一度
2. 「Nav2 Goal」でゴールを指定 → 緑 (global plan) が出れば走行開始
3. 非常時は joy の deadman (LB) を離す / `robot_speed_cmd` が 0.5 s 途絶すると controller が自動停止
4. 走行中に見るもの: `/amcl_pose` が飛ばないか、local costmap に地図に無い障害物が出ているか

bag を同時に録るなら 2 のコマンドを 4 本目のターミナルで。

← [第11章 amcl](11_amcl.md) | → [第13章 トラブルシューティング](13_troubleshooting.md)
