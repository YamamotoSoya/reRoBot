<!-- claude: 2026-08-11 の実装解説メモ。ユーザの「あとでじっくり確認したい」要望で保存。
     knowledge-check クイズは未実施 — 実施したらステータスを更新すること。 -->

# 2026-08-11 実装ウォークスルー: 車輪オドメトリ + EKF 融合

**ステータス: 未確認 (解説のみ保存、knowledge-check クイズ未実施)**

今日の実装の核は「**位置の差分から距離・角度・速度をすべて導く**」こと。
EKF まわり (共分散・config 行列・launch の上書き) は全部「その速度を
robot_localization に正しい重みで食わせる」ための配管。

対象ファイル:

- `ros2_ws_main/src/app/epos4_controller/src/epos4_odometry.cpp` (本体)
- `ros2_ws_main/src/bringup/rerobot_bringup/config/ekf.yaml` (新規)
- `ros2_ws_main/src/bringup/rerobot_bringup/launch/rerobot_bringup.launch.py` (ekf 引数)
- `ros2_ws_main/src/bringup/rerobot_bringup/config/params.yaml` (共分散パラメータ)
- `ros2_ws_main/src/app/epos4_teleop/src/teleop_keyboard.cpp` (パラメータ渡し忘れ警告)

---

## ① epos4_odometry.cpp — 車輪オドメトリノード

仕事は 1 つ: 左右モータのエンコーダ角度から (x, y, θ) を推定して `/odom` に流す。

```
/motor1/.../joint_states (右車輪, モータ軸角度 [rad])──┐
                                                      ├─→ onJointStates() ──→ /odom, TF, /joint_states
/motor2/.../joint_states (左車輪, モータ軸角度 [rad])──┘
```

### 1-1. 2 トピックを 1 コールバックに束ねる (message_filters)

左右のエンコーダは別トピックで届く。時刻のずれたペアで計算すると旋回中に
誤差が出るため、`message_filters` の ApproximateTime 同期で「タイムスタンプの
揃ったペア」を 1 回のコールバックにまとめる (`epos4_odometry.cpp:67-82`)。

```cpp
sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), m2_sub_, m1_sub_);
sync_->setMaxIntervalDuration(rclcpp::Duration(0, 50 * 1000 * 1000));
```

- `m2_sub_, m1_sub_` の順は意図的 (claude_swap): 物理配線が motor1=右輪 /
  motor2=左輪 なので、第 1 引数 `left_msg` に motor2 を流し込む。
- `setMaxIntervalDuration(50ms)`: PDO 同期周期 50 ms より離れたペアは
  「別周期のデータ」として捨てる上限。

### 1-2. モータ軸角度 → 車輪角度

`joint_states.position` はモータ軸角度 (rad)。ギヤ比 5:1 なので gear_ratio で
割って車輪角度にする (`epos4_odometry.cpp:108-112`)。`invert_left/right`
(params.yaml で両方 true) は、鏡像取付のモータのエンコーダ正方向を
「前進 = 正」に揃える符号反転。

### 1-3. 差動駆動オドメトリの数式 (心臓部)

前回コールバックからの差分で計算 (`epos4_odometry.cpp:137-149`):

```cpp
double d_left  = (pos_left  - prev_pos_left_)  * wheel_radius_;  // 弧長 = 角度差 × 半径
double d_right = (pos_right - prev_pos_right_) * wheel_radius_;
double d_s     = 0.5 * (d_left + d_right);           // 直進量 = 左右平均
double d_theta = (d_right - d_left) / tread_width_;  // 旋回量 = 左右差 ÷ トレッド幅
double mid_theta = theta_ + 0.5 * d_theta;           // 中点法 (2 次精度)
x_ += d_s * std::cos(mid_theta);
y_ += d_s * std::sin(mid_theta);
theta_ += d_theta;
theta_ = std::atan2(std::sin(theta_), std::cos(theta_));  // [-π, π] 折り返し
```

- `d_theta` の由来: 左右輪は半径の違う同心円弧を描き、弧長差 =
  `tread_width × d_theta`。右が多く進む = 左旋回 (θ 増加)。
- `mid_theta` を使う理由: 移動前の向きで cos/sin を取ると旋回中に誤差が
  系統的に内側へ溜まる。区間平均 ≒ 中間の向きで 1 桁改善 (数値積分の中点法)。
- `atan2(sin θ, cos θ)`: θ の無限累積を [-π, π] に折り返す定番イディオム。

### 1-4. 【今日のバグ修正】twist は位置差分から計算する

旧コードは「velocity 配列が非空ならそれを信用」だったが、**velocity は常に
存在して常に 0** (0x606C が bus.yml で PDO マッピングされていない)。
`empty()` チェックでは検出できず `/odom` twist はずっと 0 だった。
SLAM (位置しか使わない) では無症状で、EKF が速度を入力にした今日発症。
2026-08-11 の ekf_test bag で「pose 20 m 移動、twist 常時 0」として実証。

修正後 (`epos4_odometry.cpp:157-158`): 分岐を廃止し常に

```cpp
double v_lin = d_s / dt;
double v_ang = d_theta / dt;
```

### 1-5. 【今日の追加】共分散対角の充填

`nav_msgs/Odometry` の共分散は 6×6 行列を長さ 36 の 1 次元配列 (row-major)
で持つ。対角 (i,i) は index `i×6+i = i×7`:

```cpp
for (size_t i = 0; i < 6; ++i) {
  odom.pose.covariance[i * 7]  = pose_cov_diag_[i];
  odom.twist.covariance[i * 7] = twist_cov_diag_[i];
}
```

なぜ埋めるか: EKF は共分散の逆数で重み付けする。全ゼロ = 「誤差ゼロの完璧な
測定」の意味になり車輪 odom が IMU を押し潰す。params.yaml の
`[0.001, 0.001, 1e6, 1e6, 1e6, 0.03]` は [x y z roll pitch yaw] の順で
「x,y は信用 / z,roll,pitch は 2D では観測不能なので無視 (1e6) / yaw はそこそこ」。
(1.0e6 の指数表記は YAML 1.1 で文字列扱いになり得るため 1000000.0 と書く)

## ② launch の EKF 切替 — publish_tf の自動上書き

危険: `ekf:=true` で TF `odom→base_link` を epos4_odometry と ekf_node の
両方が出すと毎フレーム位置が跳ぶ。launch 側で自動防止
(`rerobot_bringup.launch.py:93-104`):

```python
odom_publish_tf = ParameterValue(
    PythonExpression(["'", LaunchConfiguration("ekf"), "'.lower() != 'true'"]),
    value_type=bool)
Node(..., parameters=[params_file, {"publish_tf": odom_publish_tf}])
```

3 段トリック:
1. launch 引数は常に**文字列** (`ekf:=true` は `"true"`)。
2. `PythonExpression` が実行時に `'true'.lower() != 'true'` を評価 → bool。
3. `parameters=[...]` リストは**後勝ち** — yaml の `publish_tf: true` を
   後ろの辞書が上書きする。yaml を書き換えずに launch 引数で挙動を変えられる。

ekf_node 自体は `condition=IfCondition(...)` で ekf:=false なら起動しない。

## ③ ekf.yaml — 15 bool の config 行列の読み方

15 次元状態ベクトル [x y z | roll pitch yaw | vx vy vz | vroll vpitch vyaw |
ax ay az] のうち「このセンサからどれを採用するか」。3 個ずつ区切って読む:

```yaml
odom0_config: [false, false, false,   # x, y, z          ← pose は使わない
               false, false, false,   # roll, pitch, yaw
               true,  true,  false,   # vx, vy            ← 速度を使う
               false, false, true,    # vyaw              ← yaw 角速度を使う
               false, false, false]   # ax, ay, az
```

- pose を取らない理由: `/odom` の x,y は積分済み。EKF 内でも積分するので
  誤差の二重取り込みになる。速度だけ渡すのが定石。
- `vy: true` は意図的: 差動駆動の vy は常に 0 → 「横滑りしていない」拘束
  (non-holonomic constraint) を測定として教える。
- IMU 側は yaw + vyaw のみ。`imu0_relative: true` = 起動時の向きを 0 とする
  相対角。BNO086 の絶対方位は磁北基準で磁気外乱に弱いため絶対値に依存しない。

## ④ teleop_keyboard.cpp — パラメータ渡し忘れ警告

`--params-file` を忘れると `declare_parameter("gear_ratio", 1.0)` の既定値が
使われ、エラーなしで距離表示だけ 5 倍になる (2026-08-11 実発生: 10 m → 51 m)。
対策 = **起動時に使用中の値を必ずログ表示** + 既定値 1.0 のままなら WARN。
「エラーにならず結果が狂う」ミスへの最も安い防御。

---

## 補足: 同日の関連変更 (コード外)

- `rerobot.urdf`: IMU 向きを実走 bag 3 証拠で正立 rpy=(0,0,π/2) に最終確定。
- `ros2_ws_glim/config/`: IMU レス CT-ICP → BNO086 あり LIO 構成へ切替。
  `T_lidar_imu` = 並進 (0.075, 0, -0.085) + Rz(90°) を URDF から算出
  (URDF の imu_joint を変えたら要再計算)。
