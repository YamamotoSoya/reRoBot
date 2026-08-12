<!-- claude: タイムスタンプ読本 第7章 (2026-08-11) -->

# 第7章 チェックリストとデバッグ — 時刻を疑う技術

最終章は道具箱である。コマンドは reRoBot の構成 (main コンテナ = `rerobot_env`) を
前提に、そのまま打てる形で書く。コンテナ内シェルは:

```bash
docker exec -it rerobot_env bash
# 以後コンテナ内で
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
```

## 7.1 稼働中スタックの時刻診断コマンド

### stamp の実在確認 — まずこれ (事例C の教訓)

```bash
ros2 topic echo /motor1/cia402_device_1/joint_states --field header.stamp --once
# sec: 0, nanosec: 0 → stamp が入っていない (事例C の状態)
```

### stamp の鮮度 (age) — 「今より何秒古いか」

```bash
ros2 topic delay /sdk_could
# average delay: 0.021  ← stamp と受信時刻の差 [s]。ここが「取得→受信」の遅延 + stamp の誤差
```

`ros2 topic delay` は `now() − header.stamp` の統計を出す。読み方:

| 表示 | 解釈 |
|---|---|
| 数 ms〜数十 ms | 健全 (転送 + 処理遅延) |
| **負の値** | stamp が未来 — 巻き戻し過多・別時刻軸・時計ずれ (即調査) |
| 一定の大きな正値 (例 0.16 s) | 系統的オフセット — 事例A 型 (基準点ずれ) を疑う |
| 増え続ける | 処理が滞留 (キュー詰まり) — 事例D 型 |

### 周波数とジッタ

```bash
ros2 topic hz /imu/data          # rate + min/max/std dev。std dev が大きい = stamp か供給のジッタ
ros2 topic hz /sdk_could
```

### 2 トピックの時刻軸整合 — GLIM 前チェック (第5章 5.5 節)

```bash
# 同時に stamp を流して目視比較 (秒の桁が同じ・進み方が同じか)
ros2 topic echo /sdk_could --field header.stamp &
ros2 topic echo /imu/data  --field header.stamp
# 桁違い (例: 片方 1.7e9 台、片方 3600 台) なら別時刻軸 — 融合不能
```

### TF の時刻健全性

```bash
ros2 run tf2_ros tf2_echo odom base_link   # 変換が引けるか + 時刻
ros2 run tf2_tools view_frames             # frames.pdf に各変換の "Most recent transform" が出る
# "extrapolation into the future/past" エラーの読み方は第4章 4.1 節の表
```

## 7.2 bag による時刻整合の検証手順 (GLIM 3D+IMU 検証用)

実走前にオフラインで時刻の健全性を確定させる手順。

```bash
# 1) 録画 (3D+IMU bringup が上がっている状態で)
ros2 bag record -o /workspace/log/glim_test /sdk_could /imu/data

# 2) 概況 — メッセージ数と期間から実効レートを暗算 (rfans ~6 Hz, imu ~100 Hz)
ros2 bag info /workspace/log/glim_test
```

```python
# 3) stamp の突き合わせ (コンテナ内 python3 で。rosbag2_py は jazzy 標準)
#    見るべきは 3 点:
#    (a) 両トピックの stamp が同じ時間帯を指すか (差が ms 級か)
#    (b) stamp が単調増加か (巻き戻り・重複がないか)
#    (c) bag の受信時刻 (t_recv) と stamp の差が安定か
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri='/workspace/log/glim_test'),
            rosbag2_py.ConverterOptions('', ''))
while reader.has_next():
    topic, data, t_recv = reader.read_next()
    if topic == '/sdk_could':
        m = deserialize_message(data, PointCloud2)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        print(f"pc2 stamp={stamp:.4f} recv-stamp={t_recv*1e-9 - stamp:+.4f}")
```

```bash
# 4) per-point time の健全性 — ドライバ起動ログに 1 回だけ出る自己診断 (main コンテナ内)
grep "per-point time span" /workspace/log/bringup3d.log
# "per-point time span of first scan: 0.16 s (13035 points; expect ~1/rps)" なら OK
# 0.0000 なら utcTime 不在 = deskew 不能 (LiDAR の時刻設定を確認)

# 5) GLIM オフライン評価 — glim コンテナに入って実行 (ファイル直読みなので use_sim_time 不要)
#    入り方 (ホストで): docker exec -it glim_env /ros_entrypoint.sh bash
#    bag パスは positional 引数 (ディレクトリ / .mcap / glob 可)。glim コンテナには
#    リポジトリの bags/ が /bags としてマウントされている
ros2 run glim_ros glim_rosbag /bags/<path/to/bag_dir> --ros-args -p config_path:=/glim_config
```

### 残留オフセットの微調整 (`imu_time_offset`)

手順 3 で「点群と IMU の stamp 差が一定値 (例 +15 ms) で安定」と分かったら、
`ros2_ws_glim/config/config_ros.json` の `imu_time_offset` (IMU 側に足す) か
`points_time_offset` (点群側に足す) で吸収する。**測ってから入れる**こと —
勘で入れたオフセットは事例A と同型のずれを自作するのと同じである。

## 7.3 新センサ・新ドライバ導入時の stamp 検収チェックリスト

導入のたびに 5 分で済む。事例 A〜E の再発防止をコマンドに落としたもの。

```
□ 1. stamp が実在するか
     ros2 topic echo <topic> --field header.stamp --once  → 0 でない (事例C)
□ 2. stamp の時刻軸が ROS 時刻か
     ros2 topic delay <topic> → 数 ms〜数十 ms の正値 (負・巨大値は別軸/未来 stamp)
□ 3. stamp は「取得時刻」か「publish 時刻」か
     ドライバのコードで now() の実行位置を確認 — バッファリングの後なら
     テンプレ2 (巻き戻し) が要る (事例A)
□ 4. 時間幅のあるデータなら、基準点は一致しているか
     per-point/beam 相対時刻の基準 (先頭 or 末尾) と stamp の指す瞬間が同じか (事例A)
□ 5. 相対時刻フィールドの器は適切か
     float32 に入るのは相対値のみ。絶対時刻が直入れされていないか (事例B)
□ 6. 単調性
     stamp が巻き戻らないか (デバイス時計のラップ・リセットの処理を確認)
□ 7. フォールバックの可観測性
     stamp 異常時の代替動作があるなら、発動が WARN 等で見えるか (事例E)
□ 8. bag 評価の時刻設定
     ros2 bag play --clock + use_sim_time:=true の組で評価しているか (第4章 4.6 節)
```

## 7.4 症状から引く逆引き表

| 症状 | 疑う順 | 章 |
|---|---|---|
| TF "extrapolation into the future" | センサ stamp が未来 / 時刻軸違い | 4.1 |
| TF "extrapolation into the past" (1970 年) | stamp=0 | 事例C/E |
| queue full で drop | 待っている時刻の供給不足 (odom/TF) | 事例D |
| 静止では正常、動くと点群/地図が歪む | stamp と per-point time の基準ずれ | 事例A |
| per-point 時刻が全点同値 | float32 精度潰れ / 時刻フィールド不在 | 事例B |
| 同期コールバックが呼ばれない/変な組 | 入力 stamp の実在・間隔制限 | 事例C |
| bag 再生でだけ全部壊れる | use_sim_time 忘れ | 4.6 |
| EKF 出力が入力より遅れて振動する | センサ stamp の系統オフセット | 4.4 |

---

本書はここで終わりだが、時刻バグは「書いた瞬間」ではなく「消費者が現れた瞬間」に
発火する (事例A・C)。新しい消費者 (SLAM・融合・可視化) を繋ぐたびに、この章の
チェックリストへ戻ってくること。

→ [目次に戻る](00_index.md)
