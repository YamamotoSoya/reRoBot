<!-- claude: タイムスタンプ読本 第3章 (2026-08-11) -->

# 第3章 スタンプを打つ側の記法 — 4 つのテンプレとアンチパターン

ドライバやノードを書くとき、stamp の入れ方は実質 4 パターンに集約される。
どれを選ぶかは「**データが生まれた瞬間と、コードがそれを知る瞬間の距離**」で決まる。

```
stamp の入れ方の選択樹形図
├── データ取得と同時にコードが動く?
│   ├── はい → テンプレ1: 取得直後に now()
│   └── いいえ (遅れて届く / まとめて届く)
│       ├── 遅れ幅をコードが知っている? → テンプレ2: now() から巻き戻し
│       └── デバイスが自前の時刻を付けてくる? → テンプレ3: デバイス時刻を翻訳
└── 他人のメッセージを加工して再発行する? → テンプレ4: 元 stamp を継承 (+フォールバック)
```

## 3.1 テンプレ1: 取得直後に now() — 最も基本

**適用条件**: データの取得とコードの実行がほぼ同時 (遅延が下流の要求精度より十分小さい)。

```cpp
// C++
void on_sample_ready() {              // 割り込み・読み出し直後に呼ばれる
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = this->now();     // 取得の「直後」なら now() ≈ 取得時刻
  msg.header.frame_id = "imu_link";
  // ... データ詰め ...
  pub_->publish(msg);
}
```

```python
# Python
def on_sample_ready(self):
    msg = Imu()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = "imu_link"
    self.pub.publish(msg)
```

**落とし穴**: 「取得直後」が本当に直後かはコードの構造が決める。取得から publish
までの間にバッファリング・リトライ・重い計算が挟まると、その分 stamp が未来にずれる。
テンプレ1 で書き始めたコードが、機能追加でいつの間にかテンプレ2 の適用条件に
変わっている — これが [第6章 事例A](06_case_studies.md#事例a) の構図である。

## 3.2 テンプレ2: 取得時刻への巻き戻し (now − 所要時間)

**適用条件**: データがまとめて (遅れて) 届くが、**遅れ幅を測る手段がある**。

reRoBot の実例 — `rfans_driver.cpp` `fansxyz_clound()` (2026-08-11 修正):

```cpp
// R-Fans は 1 スキャン分のパケットを受け切ってからここに来る。
// つまり now() はスキャン末尾 T_end。一方 per-point "time" はスキャン先頭基準。
// 最終点の相対時刻 = スキャン所要時間 (T_end − T_start) なので、それを引けば
// stamp ≈ T_start となり per-point time の前提と一致する。
rclcpp::Time stamp = this->now();
if (out_count_ > 0) {
  const double span = static_cast<double>(temple_1_[out_count_ - 1].timeflag);
  if (span > 0.0 && span < 1.0) {     // デバイス時刻の異常値から身を守る
    stamp = stamp - rclcpp::Duration::from_seconds(span);
  }
}
outCloud_sdk.header.stamp = stamp;
```

このテンプレの本質は **「now() が何の瞬間か」を言語化してから、取得時刻との差を
引き算で埋める**こと。差の測り方はいろいろある:

| 差の測り方 | 例 |
|---|---|
| データ自身が持つ相対時刻の最大値 | R-Fans: 最終点の per-point time |
| 既知の固定遅延 | 「このセンサは常に 8 ms 遅れて届く」(データシート値) |
| メッセージの構造から計算 | LaserScan: `scan_time` フィールド |

**必ず入れるべき防御**: 引き算の材料 (上の `span`) がセンサ由来の値なら、壊れた値が
来る前提で範囲チェックを入れる。時刻の防御を怠ると「stamp が 1 時間前」のような
メッセージが TF バッファや同期器を壊す。

## 3.3 テンプレ3: デバイス時刻 → ROS 時刻マッピング

**適用条件**: センサが自前のクロックでサンプルに時刻を付けてくる。これが最も高精度
だが、第1章 1.2 節のとおり**デバイス時刻軸のままでは ROS 世界で通用しない**ので、
ROS 時刻軸への「翻訳」が要る。

翻訳の一般形は `ROS時刻 = デバイス時刻 + オフセット` で、問題は**オフセットの推定**
に尽きる。素朴には「受信のたびに `now − device_time` を取る」だが、この値は
USB/OS のスケジューリング遅延で毎回揺れる (遅延が乗った分だけ大きくなる)。

reRoBot の実例 — BNO086 ドライバの `device_clock.py` (全 54 行) は、この揺れを
**min フィルタ + 上方向の制限付きクリープ**で処理する教科書的実装である:

```python
class DeviceClock:
    MAX_DRIFT_PPM = 100                      # 追従を許すクロックずれの上限 [ppm]

    def to_ros_ns(self, device_us: int, now_ns: int) -> int:
        if self._last_raw is not None and device_us < self._last_raw:
            self._wraps += 1                 # 32bit μs カウンタは ~71 分で一周 → 展開
        self._last_raw = device_us

        dev_ns = (device_us + (self._wraps << 32)) * 1000
        offset = now_ns - dev_ns             # 今回観測したオフセット (遅延で汚れている)

        if self._offset_ns is None or offset <= self._offset_ns:
            self._offset_ns = offset         # より小さい = より遅延が少ない観測を採用
        else:
            elapsed = max(0, dev_ns - self._last_dev_ns)
            self._offset_ns += (elapsed * self.MAX_DRIFT_PPM) // 1_000_000
            # ↑最小値に固執すると水晶の周波数差に置いていかれるので、
            #   100 ppm を上限にゆっくり上方修正を許す

        self._last_dev_ns = dev_ns
        return dev_ns + self._offset_ns
```

設計の勘所を分解すると:

```
デバイス時刻翻訳の設計課題
├── カウンタのラップ (一周) ......... wrap 回数を数えて 64bit に展開
├── 転送遅延でオフセットが汚れる .... min フィルタ: 「最速で届いた観測」が最も真値に近い
├── 水晶の個体差 (ドリフト) ......... 制限付きクリープ: 最小値への固執を 100 ppm まで緩める
└── デバイス再起動 (カウンタリセット) . 推定器ごと作り直す (imu_node.py:177-181)
```

呼び出し側 (`imu_node.py:273-283`):

```python
def _stamp(self, device_us: int):
    now = self.get_clock().now()
    if not self._use_device_time:            # パラメータで テンプレ1 へ切替可能
        return now.to_msg()
    ns = self._dev_clock.to_ros_ns(device_us, now.nanoseconds)
    return rclpy.time.Time(nanoseconds=ns).to_msg()

def _handle_imu(self, s):
    msg = Imu()
    msg.header.stamp = self._stamp(s.device_us)   # ← サンプル取得時刻が乗る
```

テンプレ1 との差は歴然で、テンプレ1 の stamp には USB 転送や OS スケジューリングの
ジッタ (数 ms、時に数十 ms) がそのまま乗るのに対し、テンプレ3 はデバイスが刻んだ
取得間隔 (100 Hz なら正確に 10 ms 刻み) が保たれる。IMU のような高頻度センサでは
この差が融合品質に直結する。

## 3.4 テンプレ4: 元 stamp の継承とフォールバック

**適用条件**: 他ノードのメッセージを加工して再発行する (odometry、フィルタ、変換ノード)。

原則は **「時刻を発明しない」**。入力データが表す瞬間は入力の stamp が知っているの
だから、出力にもそれを引き継ぐ。加工した瞬間の now() を入れると、データの実時刻が
どんどん上書きされて消えていく。

reRoBot の実例 — `epos4_odometry.cpp:114-117` + 163-165 + 213:

```cpp
// 入力 (joint_states ペア) の stamp を採用。ただし壊れた入力 (stamp=0) への保険つき
rclcpp::Time stamp = left_msg->header.stamp;
if (stamp.nanoseconds() == 0) {
  stamp = this->now();          // フォールバック: 無いよりはマシな近似
}
...
odom.header.stamp = stamp;      // /odom
tf_msg.header.stamp = stamp;    // TF odom→base_link
js.header.stamp = stamp;        // /joint_states 再発行
```

3 つの出力 (/odom・TF・/joint_states) が**同一の stamp** を持つことにも意味がある。
下流 (EKF・robot_state_publisher) から見ると「同じ瞬間の整合したスナップショット」
として扱えるからだ。

フォールバック (`stamp==0 → now()`) は「上流が規約を破ったときの延命装置」であり、
それ自体が問題を隠す副作用も持つ。功罪は [第6章 事例E](06_case_studies.md#事例e) で
詳しく扱う。

## 3.5 アンチパターン集

| ❌ アンチパターン | 何が起きるか | 対応テンプレ |
|---|---|---|
| 時間幅のあるデータに publish 時の `now()` | per-point time との基準ずれ → deskew が歪む (事例A) | テンプレ2 |
| 絶対時刻を float32 フィールドに直入れ | 分解能 ~2 分に潰れて時刻情報が消滅 (事例B) | 相対化してから入れる (2.3 節) |
| stamp を設定しない (ゼロのまま) | 時刻同期が到着順に退化 / TF が「1970 年」を引く (事例C) | テンプレ1〜3 のどれか |
| 再発行時に `now()` で上書き | データの実時刻が消え、遅延が姿勢誤差に化ける | テンプレ4 |
| 複数センサを別々の時刻軸で publish | 融合ノードが比較不能な時刻を比較する | 全センサをテンプレ3 で ROS 軸へ |
| センサ由来の時刻を無検証で使う | 異常値 1 個が同期器・TF バッファを汚染 | 範囲チェック (3.2 節の防御) |

→ [第4章 スタンプを読む側の仕組み](04_time_consumers.md)
