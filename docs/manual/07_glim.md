<!-- claude: 運用手引き 第7章 テンプレ (2026-09-22) -->

# 第7章 GLIM

---

## GLIM 各操作
### CPU mode

② **GLIM オンライン** → online/glim/<日付_時分>/

```
  ros2 run glim_ros glim_rosnode --ros-args \
    -p config_path:=/glim_config \
    -p dump_path:=/bags/9goukan/2d3d_imu/online/glim/$(date +%F_%H%M)
```

- 終了は Ctrl-C を 1 回だけ。保存処理は終了時に走るので、連打すると 08-12 に踏んだ「values.bin 欠け」が再発します。


③ **GLIM オフライン評価** → offline/glim/<元bag名>_dump/

```
B=2026-08-15_1030 # 評価対象 bag のディレクトリ名
ros2 run glim_ros glim_rosbag /workspace/bags/9goukan/2d3d_imu/online/rosbag/$B \
 --ros-args -p config_path:=/glim_config \
 -p auto_quit:=false \
 -p dump_path:=/workspace/bags/9goukan/2d3d_imu/offline/glim/${B}_dump
```

- auto_quit:=true で bag 読み切り後に自動保存・自動終了 (手動 Ctrl-C 不要)。パラメータ実験のときは config_path を変異 config に差し替えるだけで、dump_path を offline/glim/exp_<日付>/EN/ 系に向ければ実験もこの木に収まります。
- dump を offline_viewer で開くときは、dump 内にコピーされた config の extension_modules から librviz_viewer.so を外す (08-12 に確立した回避策) のを忘れずに。

④ **GLIM offline viewer**
```
B=2026-08-14_0919
ros2 run glim_ros offline_viewer /workspace/bags/9goukan/2d3d_imu/offline/glim/${B}_dump
```

### GPU mode

## 各種パラメータ



← [第6章 SLAM_toolbox](06_slam_toolbox.md) | → [第8章 LIO_SAM](08_lio_sam.md)
