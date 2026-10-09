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
ros2 bag record -s mcap -o /workspace/bags/raw/$(TZ=Asia/Tokyo date +%F_%H%M)_<場所> /rfans_driver/rfans_points /rfans_driver/rfans_packets /urg_front/scan /rfans/scan /imu/data /imu_wit/data /imu_wit/mag /odom /tf /tf_static /diagnostics /robot_speed_cmd /rfans/scan_all /amcl_pose /teach_marker
```
<!-- claude: 2026-10-01 全点版の AMCL 入力 /scan_all と推定結果 /amcl_pose を追加 (第14章 2.5) -->
<!-- claude: 2026-10-08 /scan → /urg_front/scan + /rfans/scan、/scan_all → /rfans/scan_all に改名 (裸の /scan 廃止) -->
<!-- claude: 2026-10-09 停止点ティーチングの印 /teach_marker を追加 (joy の Y ボタン、第5章の topic 表) -->

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
  --range_max 30 --deskew --min_points_in_pix 4 --max_points_in_pix 12 --map_only
```
<!-- claude: 2026-10-02 ツールの既定が一式出力 (raw/ nav2/ keep_out/) になったため、この手順 (地図だけ → 5 で加工) は --map_only で従来動作に固定 -->
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
  * <!-- claude: 2026-10-02 追記 (ユーザ依頼) --> 未観測域の自動 keepout + 走路の操作者跡の除去は `tools/map_to_keepout` ([第9章 9.3](09_map2d_compression.md))。芝生などは出来たマスクに描き足す

## 6. Nav2,slamtoolbox反映
* GLIM 由来 (3D) 地図: 5 のディレクトリを `map_dir:=` で渡す (7 参照)。amcl の入力スキャンは 2D LiDAR ではなく R-Fans 由来 (当日は全点版 `/rfans/scan_all`。[第14章](14_pointcloud_to_laserscan.md))。R-Fans→2D 変換は bringup に入っている
* slam_toolbox 由来 (2D) 地図の場合: `maps/2d/slam_toolbox/<name>/nav2/my_map.{pgm,yaml}` に同じ規約で置き、Nav2 は `nav_amcl.launch.py ... scan_topic:=/urg_front/scan` (urg で作った地図なので urg のスキャンで合わせる)
* 地図の高さ帯 (4 の `--min/max_height`) と bringup の `scan_min_height` / `scan_max_height` (既定 0.3 / 6.0) と `scan_all_min_height` / `scan_all_max_height` (既定 = 前者) は揃えておく。片方だけ変えない。例: 地図を 0.3〜1.5 で作ったら bringup に `scan_max_height:=1.5` を渡す

## 7. 自律移動
<!-- claude: 2026-10-08 書き換え (ユーザ依頼) — bringup が全部入りになり rfans_scan を同梱、/scan 改名。3 本 → 2 本 -->
main コンテナで 2 本 (別ターミナル)。urg と R-Fans は名前が分かれたので同時に起動してよい (`lidar_2d:=false` は不要。costmap は両方を使う)
```
# 1) bringup (全部入り: urg + R-Fans + R-Fans→2D + IMU + EKF)。rfans_scan_all:=true で AMCL 用の全点 /rfans/scan_all も出す
ros2 launch rerobot_bringup rerobot_bringup.launch.py rfans_scan_all:=true
# 2) Nav2 + RViz (AMCL は /rfans/scan_all を max_beams 2000 で読む)
ros2 launch rerobot_bringup nav_amcl_scanall.launch.py map_dir:=/workspace/maps/2d/glim/<name> use_keepout:=false
```
⚠️ `rfans_scan.launch.py` を別に起動しないこと (bringup に入っているので二重になる)。

起動確認: `ros2 param get /amcl scan_topic` が `/rfans/scan_all`、`ros2 topic hz /rfans/scan_all` が約 10 Hz。
従来の最近点に戻すなら 1) の `rfans_scan_all:=true` を外し、2) を `nav_amcl.launch.py` にする (入力 `/rfans/scan`。[第14章](14_pointcloud_to_laserscan.md))。
自己位置推定専用地図 (屋根込みの広い帯) を使うなら 2) に `loc_map_yaml:=<map_dir>/loc_0.3-3.0/map.yaml`、1) に `scan_all_min_height:=0.3 scan_all_max_height:=3.0` を足す ([第15章](15_launch_guide.md))。

RViz 操作:
1. 「2D Pose Estimate」で現在地と向きをクリック → amcl の粒子 (赤矢印) が数秒で収束することを確認。しなければもう一度
2. 「Nav2 Goal」でゴールを指定 → 緑 (global plan) が出れば走行開始
3. 非常時は joy の deadman (LB) を離す / `robot_speed_cmd` が 0.5 s 途絶すると controller が自動停止
4. 走行中に見るもの: `/amcl_pose` が飛ばないか、local costmap に地図に無い障害物が出ているか

<!-- claude: 2026-10-04 追加 (ユーザ依頼)。仕様は docs/text/waypoint_nav/waypoint_nav.md -->
### 7.1 waypoint 走行 (停止点で止まり、キーで再開)

指定した waypoint (停止点) で止まり、キーを押すと次の停止点まで進む。仕組み・制約は
[waypoint 走行 仕様書](../text/waypoint_nav/waypoint_nav.md)。⚠️ 2026-10-04 時点で実機未検証 — 本番前に屋内で一度通すこと。

**事前 (屋内・机上)**: waypoint_editor でコースを作り、止めたい点の上で `p` → 赤い四角が付く → `s` で保存
([第10章](10_nav2.md))。出発前に `--check` で全点 `✔` と `■停止` の位置を確認する:
```
python3 /workspace/tools/waypoint_editor/waypoint_editor.py \
  /workspace/maps/2d/glim/<name>/nav2/my_map.yaml /workspace/maps/2d/glim/<name>/waypoints/<course>.yaml --check
```

**当日**: 7 の 1)〜3) を起動 → RViz の「2D Pose Estimate」で AMCL を収束させる → 4 本目のターミナルで:
```
docker exec -it rerobot_env bash -c "source /workspace/install/setup.bash && \
  ros2 run rerobot_bringup waypoint_runner.py /workspace/maps/2d/glim/<name>/waypoints/<course>.yaml"
```
* `-it` 必須 (キーを読むため)。地図は waypoint を打ったときと**同じ** `map_dir` で Nav2 を起動していること
* RViz パネルの「Start Nav Through Poses」は使わない (停止点を読まずに素通りする)

| いつ | キー | 動作 |
|---|---|---|
| 起動直後・停止点に着いた後 | `g` | 次の停止点 (または終点) まで走る |
| 待機中 | `q` | 終了 |
| 走行中 | `p` | 一時停止 (その場で止まる)。`g` で残りの点から再開 |
| いつでも | Ctrl-C | 走行中のゴールを取り消して終了 |

* 起動しただけでは走り出さない。最初も `g` 待ち
* 区間が失敗した (`✘ 区間失敗`) ときも待機に入る。障害物が退いたら `g` で残りの点から送り直す
* 途中からやり直したいとき: `q` で終了 → 末尾に `--start N` (waypointN から) を付けて起動し直す
* 再開キーを変えたいとき: `--resume-key <1文字>` (`q` と `p` は不可)
* ⚠️ 停止中は「速度指令が 0」なだけで非常停止ではない。joy や RViz の Nav2 Goal を触ると動く。危ないときは物理の非常停止

bag を同時に録るなら 2 のコマンドを 4 本目のターミナルで。

← [第11章 amcl](11_amcl.md) | → [第13章 トラブルシューティング](13_troubleshooting.md)
