<!-- claude: EKF センサ融合読本 第3章 (2026-08-12) -->

# 第3章 融合器に食わせる側 — /odom と /imu/data の作り方

EKF は入力メッセージを疑わない。だから融合の品質は
**入力メッセージを作る側 (ドライバ・オドメトリノード) の作法**で決まる。
この章は `nav_msgs/Odometry` と `sensor_msgs/Imu` の「EKF が正しく信じられる
形」のテンプレートを一般論として示し、reRoBot の 2 つの実装を解剖する。

## 3.1 nav_msgs/Odometry の正しい作り方

メッセージの構造と、各フィールドに課された規約:

```
nav_msgs/Odometry
├── header
│   ├── stamp ........... 計測時刻 (受信時刻でも publish 時刻でもない)
│   │                     → 打ち方の型は タイムスタンプ読本 第3章
│   └── frame_id ........ pose が表現されている座標系 (通常 "odom")
├── child_frame_id ...... ⚠️ twist が表現されている座標系 (通常 "base_link")
│                         忘れやすいが EKF は twist の解釈にこれを使う
├── pose (odom 座標系)
│   ├── pose ............ 位置 + 姿勢 (quaternion)
│   └── covariance[36] .. 6x6 flat。対角に [x y z roll pitch yaw] の σ²
└── twist (base_link 座標系 = body frame)
    ├── twist ........... 並進速度 + 角速度。「ロボット自身から見た」速度
    └── covariance[36] .. 同上、[vx vy vz vroll vpitch vyaw]
```

**適用条件つきのテンプレート** (差動駆動の場合):

```cpp
// C++ — 差動駆動オドメトリの Odometry 組み立ての骨格
odom.header.stamp = stamp;              // 計測時刻 (下の 3.2 参照)
odom.header.frame_id = "odom";
odom.child_frame_id = "base_link";      // twist は body frame だと宣言
odom.pose.pose.position.x = x_;         // 積分した pose
odom.twist.twist.linear.x  = v_lin;     // body frame: 前進成分のみ
odom.twist.twist.angular.z = v_ang;
for (size_t i = 0; i < 6; ++i) {        // 対角だけでよい。ただし必ず埋める
  odom.pose.covariance[i * 7]  = pose_cov_diag[i];
  odom.twist.covariance[i * 7] = twist_cov_diag[i];
}
```

**落とし穴**: twist を odom 座標系 (世界から見た vx, vy) で入れてしまう間違い。
body frame では前進は常に `linear.x` であり、ロボットがどちらを向いていても
`linear.y` は 0 (横滑りしない限り)。reRoBot の `vy = 0` はこの性質を
逆手に取った拘束として使われる ([第4章](04_robot_localization.md) §4.3)。

## 3.2 実例1: epos4_odometry — 車輪からの /odom

`ros2_ws_main/src/app/epos4_controller/src/epos4_odometry.cpp` の処理を
入力から出力まで追う。左右モータの `joint_states` 2 本を同期して受けるところ
(message_filters ApproximateTime) は
[タイムスタンプ読本 第4章 §4.3](../timestamp/04_time_consumers.md) に委譲し、
ここでは同期済みペアが届いた後の**値の計算**を見る。

### (1) 差動駆動の運動学 (`:136-149`)

```cpp
double d_left  = (pos_left  - prev_pos_left_)  * wheel_radius_;   // :137 車輪の移動距離 [m]
double d_right = (pos_right - prev_pos_right_) * wheel_radius_;

double d_s     = 0.5 * (d_left + d_right);                        // :140 中心の進み
double d_theta = (d_right - d_left) / tread_width_;               // :141 向きの変化

double mid_theta = theta_ + 0.5 * d_theta;                        // :144 中点ヘディング
x_ += d_s * std::cos(mid_theta);                                  // :145
y_ += d_s * std::sin(mid_theta);
theta_ += d_theta;
theta_ = std::atan2(std::sin(theta_), std::cos(theta_));          // :149 [-π, π] へ折り返し
```

2 点だけ設計判断がある:

- **中点ヘディング (`mid_theta`)**: 移動前の向き `theta_` で cos/sin を取ると、
  旋回しながら進むとき誤差が常に旋回の内側へ溜まる (系統誤差)。
  区間の真ん中の向きを使うと、この誤差が 1 桁小さくなる (2 次精度)。
  1 行の変更で買える精度としては最も割が良い。
- **atan2 による折り返し**: `theta_ += d_theta` は無限に成長するので、
  `atan2(sin, cos)` で常に [-π, π] へ正規化する。if 文の羅列より安全な定石。

### (2) twist は位置差分から計算する (`:151-158`) — 事例Aの修正

```cpp
// claude_odom claude_ekf: body-frame velocities from position deltas.
// The driver's joint_states.velocity is ALWAYS zero (0x606C is not PDO-mapped
// in bus.yml) — confirmed from the 2026-08-11 ekf_test bag, where /odom pose
// moved 20 m while twist never exceeded 0.05 m/s. ...
double v_lin = d_s / dt;      // :157
double v_ang = d_theta / dt;
```

上流ドライバ (ros2_canopen) の `joint_states.velocity` は**配列としては常に
存在するが値は常に 0** (速度オブジェクト 0x606C が bus.yml で PDO マップ
されていないため)。旧実装は「velocity 配列が非空なら信用する」だったので、
`/odom` の twist は静かに 0 のまま — pose しか見ない SLAM では無症状で、
twist しか見ない EKF に繋いだ日に発症した。ここでは信頼できる情報源
(position の差分) だけから速度を導出する形に修正されている。
事故の全容は [第6章 事例A](06_case_studies.md)。

**一般化すると**: メッセージのフィールドが「存在する」ことと「意味のある値が
入っている」ことは別である。新しい上流と繋ぐときは、フィールドごとに
実データで値域を確認する ([第7章](07_verification.md) の検収チェックリスト)。

### (3) covariance はパラメータから対角充填 (`:176-180`)

第2章 §2.2 で見たとおり。値そのものはコードに埋め込まず
`params.yaml:36-37` から取り、6 要素でなければ起動時に FATAL で落とす
(`epos4_odometry.cpp:50-55`)。「黙って全ゼロで走る」より「起動しない」方が
安全側、という設計である。

### (4) stamp のフォールバック (`:114-117`)

`joint_states` の stamp が 0 なら `now()` で代用する。功罪は
[タイムスタンプ読本 第6章 事例E](../timestamp/06_case_studies.md) 参照。

## 3.3 sensor_msgs/Imu の正しい作り方

```
sensor_msgs/Imu
├── header (stamp, frame_id) ........ frame_id は IMU の取付リンク名 (例 "imu_link")
│                                     → EKF が URDF の TF で base_link へ変換する
├── orientation ..................... 姿勢 (quaternion)。チップが吐く融合済み姿勢
│   └── orientation_covariance[9] ... 3x3。⚠️ REP-145: 姿勢が無効なら [0] に -1.0
├── angular_velocity ................ ジャイロ生値 [rad/s]
│   └── angular_velocity_covariance[9]
└── linear_acceleration ............. 加速度計生値 [m/s²] (重力込み)
    └── linear_acceleration_covariance[9]
```

Odometry と違う注意点が 3 つ:

1. **frame_id は「IMU チップの座標系」を指すリンク名**であり、base_link ではない。
   軸の向き変換は EKF が URDF (TF) を引いて行う。つまり
   **URDF の取付定義が間違っていると、測定値が正しくても観測が間違う**
   ([第5章](05_rerobot_architecture.md) §5.3、[第6章 事例B](06_case_studies.md))。
2. **REP-145**: orientation を推定していない (できない) 場合は
   `orientation_covariance[0] = -1.0` を入れて「このフィールドは無効」と
   宣言する規約がある。0 のまま送ると「完璧な姿勢測定」の意味になってしまう。
3. covariance は 3x3 なので対角は index 0, 4, 8。

## 3.4 実例2: BNO086 ドライバ — /imu/data

`ros2_ws_main/src/drivers/BNO086_ROS2Board-main/ros2_ws/src/bno086_imu_driver/bno086_imu_driver/imu_node.py`。
BNO086 はチップ内で 9 軸融合をやる賢いセンサで、ドライバの仕事は
「チップの自己申告を ROS の規約に正しく写す」ことに集約される。

### (1) orientation covariance はチップの自己申告から (`:287-297`)

```python
sd = max(s.quat_accuracy_rad, self._min_orientation_stddev)   # :291
var = sd * sd
msg.orientation_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]  # :293
```

BNO086 は姿勢推定の自己精度 (`quat_accuracy_rad`) を毎サンプル報告してくる。
これをそのまま σ として使い、下限 `min_orientation_stddev: 0.01 rad` (`:112`)
でクリップする — チップが「誤差ほぼゼロ」と自称しても σ² = 0 (完璧宣言) には
させない、第2章 §2.1 の罠への防御である。姿勢が無効なときは REP-145 に従い
`[-1.0] + [0.0]*8` (`:294-297`)。

gyro / accel の covariance は設定ファイルの stddev から等方に埋める
(`:299-315`、値は `config/bno086.yaml:19-27` — 「静止状態で実測して置き換えよ」
と明記されている。実測手順は [第7章](07_verification.md))。

### (2) stamp はデバイス時刻から写像 (`:273-278`)

```python
def _stamp(self, device_us: int):
    now = self.get_clock().now()
    if not self._use_device_time:
        return now.to_msg()
    ns = self._dev_clock.to_ros_ns(device_us, now.nanoseconds)
    return rclpy.time.Time(nanoseconds=ns).to_msg()
```

受信時刻ではなく基板の µs カウンタを ROS 時刻軸へ写像する
(USB 転送のジッタが stamp に乗らない)。写像の仕組み (最小値フィルタ) は
[タイムスタンプ読本 第3章 テンプレ3](../timestamp/03_stamping_patterns.md) が正典。

### (3) 取付補正は 2 段構え

- **チップ→基板**: BNO086 チップは基板上に 180° 回して実装されている。
  `mount_yaw_deg: 180.0` (`:122`) で orientation は quaternion の後乗算、
  gyro/accel はベクトル逆回転で基板座標系へ直す (`_to_board()`, `:254-271`)。
- **基板→車体**: `/imu/data` の座標系 (= frame_id `imu_link`) から base_link
  への変換は **URDF の imu_joint** が担う。ドライバ内の補正と URDF の補正で
  役割が分かれていることを知らないと、事例Bのような迷宮に入る。

## 3.5 アンチパターン集

| ❌ アンチパターン | 何が起きるか | 対応する原則・実例 |
|---|---|---|
| covariance を埋めず全ゼロで publish | σ²=0 = 完璧宣言。そのセンサが他の全入力を押し潰す | §2.1。epos4_odometry は未設定なら起動拒否 (`:50-55`) |
| 上流の velocity フィールドを無検証で信用 | 常時 0 でも「速度 0 の測定」として堂々と流れる。融合先で発症 | §3.2 (2)、事例A |
| YAML で `1.0e6` と書く | YAML 1.1 パーサで文字列扱い → パラメータ型エラー、または黙って既定値 | `params.yaml:34-35` の注記。`1000000.0` と書く |
| orientation 無効時に covariance を 0 のまま送る | 「完璧な姿勢」として融合される | REP-145: `[0] = -1.0` (`imu_node.py:294-297`) |
| twist を odom 座標系で入れる | 向きが変わると速度の解釈が壊れる | §3.1: twist は child_frame_id (body frame) |
| stamp に受信時刻・publish 時刻を入れる | 融合器の時刻整列が実態とずれる | [タイムスタンプ読本 第3章](../timestamp/03_stamping_patterns.md) |
| ドライバ側 TF と URDF 側 TF の二重定義 | 同じ frame に 2 つの親、TF が喧嘩 | BNO086 は `publish_tf: false` (`config/bno086.yaml:8-10`)、TF は URDF に一本化 |

## 3.6 この章のまとめ

```
第3章 まとめ
├── Odometry: frame_id = pose の座標系、child_frame_id = twist の座標系 (body frame)
├── 中点ヘディングと atan2 折り返しは差動駆動オドメトリの 2 大定石
├── twist は信頼できる情報源 (位置差分) から導出する — フィールドの存在 ≠ 値の有効性
├── Imu: frame_id は取付リンク名。軸変換は URDF (TF) の仕事
├── covariance は「更新の重みそのもの」— 必ず埋める。無効宣言は REP-145 の -1.0
└── 取付補正はドライバ内 (チップ→基板) と URDF (基板→車体) の 2 段 — 混同しない
```

→ [第4章 robot_localization の機構](04_robot_localization.md) — 正しく作った
メッセージを、EKF 側がどう選り分けて食べるかを設定ファイルの言葉で読む。
