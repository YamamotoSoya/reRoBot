<!-- claude: 運用手引き 第7章 テンプレ (2026-09-22) -->

# 第7章 GLIM

---

## GLIM 各操作
### CPU mode

② **GLIM オンライン** → bags/glim/<日付_時分>_<場所>_live_dump/

```
  ros2 run glim_ros glim_rosnode --ros-args \
    -p config_path:=/glim_config \
    -p dump_path:=/workspace/bags/glim/$(TZ=Asia/Tokyo date +%F_%H%M)_<場所>_live_dump
```

- 終了は Ctrl-C を 1 回だけ。保存処理は終了時に走るので、連打すると 08-12 に踏んだ「values.bin 欠け」が再発します。


③ **GLIM オフライン評価** → bags/glim/<元bag名>_dump/<タグ>/  (タグ = default / k20 / filtered …)

```
B=2026-09-27_1651_5goukan # 評価対象 bag のディレクトリ名 (bags/raw/ 直下)
ros2 run glim_ros glim_rosbag /workspace/bags/raw/$B \
 --ros-args -p config_path:=/glim_config \
 -p auto_quit:=false \
 -p dump_path:=/workspace/bags/glim/${B}_dump/default
```

- auto_quit:=true で bag 読み切り後に自動保存・自動終了 (手動 Ctrl-C 不要)。パラメータ実験のときは config_path を変異 config に差し替えるだけで、dump_path を bags/exp/<日付>_<テーマ>/EN/ 系に向ければ実験もこの木に収まります。
- dump を offline_viewer で開くときは、dump 内にコピーされた config の extension_modules から librviz_viewer.so を外す (08-12 に確立した回避策) のを忘れずに。

④ **GLIM offline viewer**
```
B=2026-08-14_0919_5goukan
ros2 run glim_ros offline_viewer /workspace/bags/glim/${B}_dump/default
```

### GPU mode

## 各種パラメータ



← [第6章 SLAM_toolbox](06_slam_toolbox.md) | → [第8章 LIO_SAM](08_lio_sam.md)
