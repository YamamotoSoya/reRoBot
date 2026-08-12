<!-- claude: EKF センサ融合読本 第7章 (2026-08-12) -->

# 第7章 検証とチェックリスト — 融合を信じてよいかを測る

第6章の事故はすべて「実データとの突き合わせ」で発見・確定された。
この章はその突き合わせを、EKF の調整・再検証のたびに回せる定型手順に落とす。
数値の合格基準例は 2026-08-11 の再テスト (bag `ekf_test2`) の実測値
(`docs/claude/PROJECT_STATE.md` タイムライン 08-11 (3)) を使う。

## 7.1 検証プロトコル — 運動の種類別に測る

事例Dの教訓どおり、静止・直進・旋回を**別々に**測る。全テストで
`/odom` (生) と `/odometry/filtered` (融合) を両方記録し、比較する:

```bash
# main コンテナ内で記録 (テスト 1 本 = bag 1 本)
ros2 bag record -o /workspace/log/ekf_check \
  /odom /odometry/filtered /imu/data /tf /tf_static \
  /motor1/cia402_device_1/joint_states /motor2/cia402_device_2/joint_states
```

```
検証プロトコル
├── (0) 物差しの較正 (事例Cの再発防止。走行テストの前に必ず)
│   ├── 全ノードが params ファイル込みで起動しているか:
│   │     ros2 param get /epos4_odometry gear_ratio   → 5.0 であること
│   └── 既知の 1 m 移動で teleop 距離表示・/odom を照合
├── (1) 静止テスト (5 分放置)
│   ├── 測るもの: /odometry/filtered の位置・yaw の這い出し
│   ├── 合格基準例: 305 s で 14 mm / -1.1° (2026-08-11 実測)
│   └── ついでに IMU の静止統計を取り covariance の実測値を得る (§7.2)
├── (2) 直進テスト (メジャーで 10 m を実測)
│   ├── 測るもの: filtered / 生 odom / 実測 の 3 点比較
│   ├── 合格基準例: filtered 10.08 m, odom 10.19 m, 実測 10 m (誤差 0.8%)
│   └── 悪化したら: 並進は車輪由来 (第4章 §4.1) → まず車輪較正 (事例E) を疑う
├── (3) その場 360° 旋回 (マーキングして元の向きへ戻す)
│   ├── 測るもの: filtered / 生 odom の yaw 残差
│   ├── 現状値: filtered +3.9° vs odom 0.1° (事例D, 未解決)
│   └── filtered だけ悪い → yaw の情報源 (磁北) を疑う
└── (4) 相関テスト (旋回中の bag から)
    ├── 測るもの: gyro_z ⇔ 車輪 odom yaw rate の符号一致率と比
    ├── 合格基準例: 同符号率 98%、比 ~1.0 (2026-08-11 実測)
    └── 比が -1 に近い → 鏡像 = URDF 取付定義 (事例B)。1/N 倍 → スケール較正 (事例E)
```

bag からの数値抽出は `ros2 bag play` + `ros2 topic echo --csv` か、
Python で sqlite3 を直接読む。時刻整合の検証手順は
[タイムスタンプ読本 第7章 §7.2](../timestamp/07_checklist.md) と共通なので併用する。

## 7.2 covariance を実測で置き換える

現状の covariance は初期値ベタ置き ([第5章](05_rerobot_architecture.md) §5.4)。
静止テスト (1) の bag から実測 σ² を出せる:

```bash
# 静止中の IMU 統計 (例: gyro z)。平均 = バイアス、std = σ → σ² を yaml へ
ros2 topic echo /imu/data --csv --field angular_velocity.z > gyro_z.csv
# → 平均・標準偏差を計算し、bno086.yaml の angular_velocity_stddev を実測値に更新
```

`bno086.yaml:19-27` 自体に「静止状態で実測して置き換えよ」と明記されている。
注意 2 点:

- **静止 σ は下限値**である。走行中は振動で必ず悪化するので、実測 σ の
  2〜3 倍を設定するのが安全側 (小さすぎる σ² = 過信の方が害が大きい。第2章 §2.1)
- YAML に書くとき指数表記 (`1.0e-3`) を使わない ([第3章](03_feeding_the_filter.md) §3.5)

## 7.3 稼働中スタックの診断コマンド

上から順に「配管 → 値 → 品質」の順で見る:

```bash
# (1) 配管: ノードとトピックが期待どおりか
ros2 node list | grep -E "ekf|odometry|imu"        # ekf_filter_node がいるか
ros2 topic hz /odom /imu/data /odometry/filtered    # ~20 / 100 / 30 Hz
ros2 topic info /odometry/filtered                  # publisher が ekf_filter_node か

# (2) TF: odom→base_link の発行者が 1 つだけか (第4章 §4.6)
ros2 run tf2_ros tf2_echo odom base_link            # 値が滑らかに更新されるか
ros2 run tf2_tools view_frames                      # ツリー図で二重親・断裂を確認

# (3) 値: 動かして突き合わせる (teleop でゆっくり前進しながら)
ros2 topic echo /odom --field twist.twist.linear.x           # 0 でないこと (事例A!)
ros2 topic echo /odometry/filtered --field pose.pose.position
ros2 topic echo /imu/data --field angular_velocity.z         # 旋回で符号が合うこと

# (4) 品質: EKF の自信を読む (第2章 §2.3 — filtered の covariance は EKF の内部状態の抜粋)
ros2 topic echo /odometry/filtered --field pose.covariance
#   対角 [0],[7],[35] (x, y, yaw) が有限の値で安定していること。
#   単調増加し続ける → 更新が効いていない = センサが食われていない兆候
```

(3) で `/odom` の twist を必ず見ること。事例Aの再発検出はこの 1 行である。

## 7.4 新しいセンサを融合に追加するときの検収チェックリスト

- [ ] 1. **単体較正**: 融合前にセンサ単体を実測と照合した (事例E)。
      スケール (距離・角度) と符号の両方
- [ ] 2. **frame_id と URDF**: メッセージの frame_id に対応するリンクが URDF に
      あり、取付 rpy を**データ照合**で確認した (事例B: 静止重力の符号 +
      旋回相関の符号。目視・シルク印刷は不可)
- [ ] 3. **stamp 検収**: [タイムスタンプ読本 第7章 §7.3](../timestamp/07_checklist.md)
      の項目 (stamp の単調性・now() との差・0 でないこと)
- [ ] 4. **全フィールドの値域確認**: 実走 bag で、使う予定のフィールド全部に
      意味のある値が入っているか (事例A: 「存在するが常に 0」を検出できるのは
      これだけ)
- [ ] 5. **covariance**: 対角が全軸埋まっている。σ² = 0 の軸がない。
      無効フィールドは REP-145 の -1 (第3章 §3.3)
- [ ] 6. **ekf.yaml の config 行列**: 採用する状態量を 3 個区切りで確認
      ([第4章](04_robot_localization.md) §4.1)。積分済み量と導関数の二重取りがない
- [ ] 7. **TF 発行権**: 新センサのドライバが勝手に TF を出していないか
      (`ros2 topic echo /tf --once` で発行者を確認)。odom→base_link の
      発行者は 1 つ (第4章 §4.6)
- [ ] 8. **運動別テスト**: §7.1 の (1)〜(4) を一巡し、生 odom と filtered の
      両方を bag に残した

## 7.5 症状からの逆引き表

| 症状 | 第一容疑者 | 確認コマンド / 章 |
|---|---|---|
| filtered が (0,0) から動かない | /odom の twist が 0 (事例A) | `ros2 topic echo /odom --field twist.twist.linear.x` |
| filtered の yaw が生 odom の鏡像 | URDF の IMU 取付が逆 (事例B) | 旋回中の gyro_z と odom yaw rate の符号 / 第5章 §5.3 |
| 距離・速度が整数倍ずれる | スケール較正 (事例C, E) | `ros2 param get` で gear_ratio、既知距離の実測照合 |
| 旋回だけ filtered が悪い | 磁北 yaw の動的外乱 (事例D) | 旋回テスト §7.1 (3) / 第4章 §4.4 |
| ロボットの表示が 2 つの姿勢で振動 | TF 二重発行 | `view_frames` / 第4章 §4.6, 第5章 §5.2 |
| filtered が階段状・カクカク | センサが食われていない (timeout / QoS / stamp) | `ros2 topic hz` + filtered の covariance 増加 / タイムスタンプ読本 §4.4 |
| 起動直後だけ暴れてすぐ収束 | 初期共分散と相対化 (imu0_relative) の整定 | 数秒待つのが仕様。長引くなら第4章 §4.4 |
| Nav2 の挙動が進捗と経路でちぐはぐ | odom_topic の 2 箇所が不一致 | `nav2_params.yaml:97, :114` / 第5章 §5.2 |

## 7.6 この章のまとめ

```
第7章 まとめ
├── 検証は運動別 (静止・直進・旋回・相関) — 選択的な劣化が原因を指差す
├── テストの前に物差し (params 込み起動・距離表示) を較正する
├── /odom と /odometry/filtered は常に両方記録して比較する
├── covariance は静止実測 σ の 2〜3 倍で置く (過信が最悪)
├── filtered の covariance が単調増加 = センサが食われていないアラート
└── 新センサの検収 8 項目 — 特に「全フィールドの値域確認」(事例A) を飛ばさない
```

これで本書は終わり。[00_index](00_index.md) に戻る。
実装の一次解説は `docs/claude/knowledge/2026-08-11_ekf_odometry_code_walkthrough.md`、
時刻まわりは [タイムスタンプ読本](../timestamp/00_index.md) が姉妹書である。
