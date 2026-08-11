<!-- claude: タイムスタンプ読本 第2章 (2026-08-11) -->

# 第2章 Header とメッセージ構造 — stamp はどこに住んでいるか

## 2.1 std_msgs/Header — たった 2 フィールドの契約書

ほとんどのセンサメッセージは、先頭に `std_msgs/Header` を 1 個持つ:

```
std_msgs/Header
├── builtin_interfaces/Time stamp   ← 「このデータは いつ のものか」
└── string frame_id                 ← 「このデータは どの座標系 で表されているか」
```

stamp と frame_id が**対**になっているのは偶然ではない。ロボットの座標系は時々刻々
動くので、「どの座標系か」は「いつの話か」とセットでないと意味が確定しないからだ。
TF ([第4章](04_time_consumers.md)) はこの 2 つを鍵にして座標変換を引く。

### 最重要の規約: stamp = データ取得時刻

ROS の設計規約 (REP 103 / 各メッセージ定義のコメント) では、センサメッセージの
stamp は **「データが取得された瞬間」** を表す。次の 3 つはすべて別の時刻であり、
混同がバグの温床になる:

| 時刻 | 例 (回転式 LiDAR) | stamp に入れてよいか |
|---|---|---|
| データ取得時刻 | スキャン開始の瞬間 `T_start` | ✅ これが規約 |
| 処理完了時刻 | 点群の座標計算が終わった瞬間 | ❌ |
| publish 時刻 | `publish()` を呼んだ瞬間 | ❌ (取得とほぼ同時なら実害は小さいが、原理的に誤り) |

そして第1章で見たとおり、**stamp はドライバ作者が代入するただのフィールド**なので、
この規約は強制されない。`msg.header.stamp = this->now();` と書けば「その行が実行
された瞬間」が入る — それが取得時刻と一致するかはコードの構造次第である
([第6章 事例A](06_case_studies.md#事例a) はまさにこの不一致だった)。

## 2.2 主要メッセージの構造樹形図

reRoBot で流れている型を中心に、stamp がどこに埋まっているかを見る。

### sensor_msgs/LaserScan (2D LiDAR: /scan)

```
sensor_msgs/LaserScan
├── header
│   ├── stamp            ← ★スキャン最初のビームの取得時刻
│   └── frame_id         "laser"
├── float32 angle_min / angle_max / angle_increment
├── float32 time_increment   ← ★ビーム間の時間差 [s] — per-beam 時刻はここから復元
├── float32 scan_time         ← ★1 スキャン全体の所要時間 [s]
├── float32 range_min / range_max
└── float32[] ranges / intensities
```

LaserScan は「stamp = 最初のビーム、以後は `time_increment` ずつ後」という
**相対時刻の復元構造**を最初から型に持っている。i 番目のビームの取得時刻は
`stamp + i × time_increment`。

### sensor_msgs/PointCloud2 (3D LiDAR: /sdk_could)

```
sensor_msgs/PointCloud2
├── header
│   ├── stamp            ← ★基準時刻。per-point time があるなら「スキャン開始時刻」
│   └── frame_id         "rfans"
├── uint32 height / width         (点の数 = height × width)
├── PointField[] fields            ← ★点 1 個のメモリレイアウトの宣言 (下記)
│   ├── {name:"x",         offset:0,  datatype:FLOAT32, count:1}
│   ├── {name:"y",         offset:4,  ...}
│   ├── {name:"z",         offset:8,  ...}
│   ├── {name:"intensity", offset:16, ...}
│   └── {name:"time",      offset:28, FLOAT32}  ← ★per-point 相対時刻 [s]
├── uint32 point_step / row_step
└── uint8[] data                   ← 全点のバイト列 (fields の宣言どおりに並ぶ)
```

PointCloud2 は LaserScan と違い「点ごとの時刻」を型として持たない。代わりに
**fields という自己記述型のレイアウト宣言**があり、ドライバが任意のフィールドを
足せる。R-Fans ドライバは `time` という名前の float32 を各点に埋めている
(`rfans_driver.cpp:575` で旧名 `timeflag` から改名)。

### sensor_msgs/Imu (/imu/data)

```
sensor_msgs/Imu
├── header
│   ├── stamp            ← ★IMU サンプルの取得時刻 (BNO086 ではデバイス時刻から翻訳)
│   └── frame_id         "imu_link"
├── geometry_msgs/Quaternion orientation        + float64[9] covariance
├── geometry_msgs/Vector3    angular_velocity   + float64[9] covariance
└── geometry_msgs/Vector3    linear_acceleration + float64[9] covariance
```

IMU は 100 Hz と高頻度なので、stamp の質 (ジッタの少なさ) がそのまま融合精度に
効く。BNO086 ドライバの stamp 生成は [第3章](03_stamping_patterns.md) テンプレ3。

### sensor_msgs/JointState (/motor{1,2}/…/joint_states, /joint_states)

```
sensor_msgs/JointState
├── header
│   └── stamp            ← ★エンコーダ読み出し時刻 … のはずが reRoBot では常に 0 (事例C)
├── string[] name        ["m1_wheel", "m2_wheel"]
├── float64[] position   [rad]
├── float64[] velocity   [rad/s] (reRoBot では常に 0 — 0x606C 未 PDO マップ)
└── float64[] effort
```

### nav_msgs/Odometry (/odom, /odometry/filtered)

```
nav_msgs/Odometry
├── header
│   ├── stamp            ← ★この姿勢推定が有効な時刻
│   └── frame_id         "odom"        (pose の基準座標系)
├── string child_frame_id "base_link"  (twist の基準座標系)
├── pose  (PoseWithCovariance)
└── twist (TwistWithCovariance)
```

### geometry_msgs/TransformStamped (TF: /tf, /tf_static)

```
geometry_msgs/TransformStamped
├── header
│   ├── stamp            ← ★この変換が有効な時刻。TF バッファはこれを鍵に補間する
│   └── frame_id         親フレーム ("odom")
├── string child_frame_id 子フレーム ("base_link")
└── transform (translation + rotation)
```

TF は「stamp 付き変換の時系列」をバッファに溜め、問い合わせ時刻の前後 2 つから
**補間**して返す。だから TF の stamp が狂うと、座標変換そのものが時間的にずれる。

### 番外: Header を持たないメッセージ — geometry_msgs/Twist

```
geometry_msgs/Twist          ← /robot_speed_cmd (teleop → epos4_controller)
├── linear  (Vector3)
└── angular (Vector3)        ※ header なし = 時刻情報なし
```

指令系のメッセージには意図的に Header がないものがある。「いつの指令か」を
受信側が判定できないため、**通信が止まっても最後の指令が生き続ける**。これが
監査 Issue 1 のウォッチドッグ問題 (指令が途絶えたら停止する仕組みが必要) の
根っこであり、「時刻情報がないことも設計上の意味を持つ」例である。

## 2.3 per-point time — 「基準 + 相対」の二段構え

回転式 LiDAR のように **1 メッセージが時間幅を持つ**センサでは、時刻は二段構えになる:

```
1 スキャン分の時刻情報
├── header.stamp               ← 基準時刻 (絶対)。スキャン開始 T_start を指すべき
└── 各点の "time" フィールド    ← 基準からの相対秒 (float32)。点 i の絶対時刻 = stamp + time[i]
```

相対値にする理由は第1章 1.1 節の精度問題そのものである。絶対時刻 (~10⁹ 秒) を
float32 に入れると分解能 ~2 分で全点が同時刻に潰れるが、相対値 (0〜0.16 s) なら
float32 でも ~10 ns 分解能が出る。**大きい絶対値は Header の 2 整数に、小さい相対値
は float32 に** — 役割分担が精度を守る。

この構造の弱点は、**基準 (stamp) と相対 (time) の「基準点合わせ」を型が保証して
くれない**ことだ。stamp がスキャン末尾を指しているのに time がスキャン先頭基準だと、
全点が 1 スキャン分未来にずれる — [第6章 事例A](06_case_studies.md#事例a) で実際に
起きた事故である。

## 2.4 reRoBot のトピックと stamp の意味論

| トピック | 型 | stamp が指すべき時刻 | 実装 (2026-08-11 時点) |
|---|---|---|---|
| `/scan` | LaserScan | スキャン開始 | urg_node (実績ある外部ドライバ) ✅ |
| `/sdk_could` | PointCloud2 | スキャン開始 | 修正済み: `now() − スキャン所要時間` ✅ |
| `/imu/data` | Imu | サンプル取得 | デバイス時刻→ROS 時刻翻訳 ✅ |
| `/motor{1,2}/…/joint_states` | JointState | エンコーダ読み出し | ⚠️ **常に 0** (上流 ros2_canopen、事例C) |
| `/joint_states` | JointState | 同上 | 0 なら `now()` に置換して転記 |
| `/odom` | Odometry | 推定の有効時刻 | joint_states ペアの stamp を継承 |
| `/odometry/filtered` | Odometry | 同上 | robot_localization (外部) ✅ |
| `/tf` (odom→base_link) | TransformStamped | 同上 | /odom と同一 stamp |
| `/robot_speed_cmd` | Twist | (時刻なし) | ウォッチドッグ未実装 (監査 Issue 1) |

→ [第3章 スタンプを打つ側の記法](03_stamping_patterns.md)
