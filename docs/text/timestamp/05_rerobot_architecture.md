<!-- claude: タイムスタンプ読本 第5章 (2026-08-11) -->

# 第5章 reRoBot の時刻アーキテクチャ — あるべき姿と現在地

## 5.1 設計原則 4 か条

reRoBot (自律移動: 車輪 odom + IMU + 2D/3D LiDAR + EKF + Nav2/GLIM) の時刻設計は、
次の 4 か条に集約される。ここまでの章の結論の言い換えでもある:

```
時刻設計 4 か条
├── 1. 単一時刻軸 ..... 全トピックの stamp は「ホスト PC の ROS 時刻」に統一する。
│                        デバイス時刻はドライバ内で翻訳し、外に漏らさない (第1章 1.2)
├── 2. 取得時刻主義 ... stamp はデータが生まれた瞬間。処理・publish の瞬間ではない (第2章 2.1)
├── 3. 基準の一致 ..... 時間幅のあるデータは「stamp = 相対時刻の基準点」を厳守 (第2章 2.3)
└── 4. 時刻を発明しない . 再発行では元 stamp を継承。now() での上書きは最後の手段 (第3章 3.4)
```

## 5.2 3 つの時刻軸と ROS 時刻軸への写像

reRoBot には性質の違う時計が 3 つ載っており、それぞれ別の方法で ROS 時刻軸へ
写像される (原則 1 の実装)。

```mermaid
flowchart LR
    subgraph dev["デバイスの時刻軸"]
        GPS["R-Fans: GPS 週秒<br>(utcTime, double)"]
        MCU["BNO086: 32bit μs カウンタ<br>(~71 分で一周)"]
        EPOS["EPOS4: 時刻情報なし<br>(joint_states stamp=0)"]
    end
    subgraph ros["ROS 時刻軸 (ホスト PC)"]
        PC2["/sdk_could<br>stamp=スキャン開始"]
        IMU["/imu/data<br>stamp=サンプル取得"]
        JS["/odom, TF<br>stamp=now() フォールバック"]
    end
    GPS -->|"差分だけ使い絶対値は捨てる<br>stamp = now() − スキャン所要時間"| PC2
    MCU -->|"min フィルタでオフセット推定<br>(device_clock.py)"| IMU
    EPOS -->|"⚠️ 到着時 now() で代用<br>(真の読み出し時刻は不明)"| JS
```

三者三様である理由:

- **R-Fans (GPS 週秒)**: 週秒と ROS 時刻の対応を取る手段がホスト側にない (PPS 同期
  なし)。そこで絶対値は捨て、**点同士の差分** (per-point 相対時刻とスキャン所要時間)
  だけをもらう。基準点はホストの `now()` から巻き戻して作る (テンプレ2)。
- **BNO086 (μs カウンタ)**: サンプルごとにカウンタ値が付いてくるので、オフセット
  推定で軸ごと翻訳できる (テンプレ3)。3 つの中で最も高品質な stamp。
- **EPOS4 (時刻なし)**: ドライバ (ros2_canopen) が stamp を付けてくれない。取得
  時刻は原理的に復元不能で、到着時刻 `now()` での代用が現状の上限 (事例C)。

## 5.3 データフローと stamp の旅

bringup 2D+IMU+EKF 構成 (nav2d.sh 相当) での stamp の流れ:

```
stamp の系譜 (誰が打ち、誰が継承し、誰が消費するか)
├── /imu/data (100 Hz) ─── BNO086 ドライバがテンプレ3 で打つ
│   ├──→ EKF (imu0): yaw / yaw rate の観測として stamp 順処理
│   └──→ GLIM (LIO): プリインテグレーション区間の材料
├── /scan (40 Hz) ───────── urg_node が打つ (スキャン開始時刻)
│   └──→ slam_toolbox / AMCL: MessageFilter が stamp で TF を待ち合わせ
├── /sdk_could (~6 Hz) ──── rfans_driver がテンプレ2 で打つ (+ per-point time)
│   └──→ GLIM: stamp + time[i] で点の絶対時刻を復元 → deskew
├── /motor{1,2}/…/joint_states (20 Hz) ─── ⚠️ ros2_canopen が stamp=0 のまま publish
│   └──→ epos4_odometry: ApproximateTime ペアリング (退化中) → stamp==0 なら now() に置換
│         ├──→ /odom ────────── EKF (odom0): vx, vyaw の観測
│         ├──→ TF odom→base_link (EKF 無効時のみ。有効時は EKF が出す)
│         └──→ /joint_states ── robot_state_publisher: ホイール TF に転記
└── /odometry/filtered (30 Hz) ─── EKF が打つ (融合結果の有効時刻)
    └──→ Nav2 (bt_navigator / controller_server の odom_topic)
```

各トピックの品質評価 (2026-08-11 時点):

| トピック | 時刻軸 | stamp の質 | 備考 |
|---|---|---|---|
| `/imu/data` | ROS (翻訳済) | ◎ 取得時刻、ジッタ小 | テンプレ3 |
| `/scan` | ROS | ○ 実績ある外部実装 | urg_node |
| `/sdk_could` | ROS | ○ スキャン開始に補正済 | 残差 = UDP+処理遅延 (ms 級) |
| `/motor…/joint_states` | — | ❌ 常に 0 | 上流 (apt 配布物) の欠陥 |
| `/odom`, `/joint_states`, TF | ROS | △ 到着時刻で代用 | stamp=0 フォールバック |
| `/odometry/filtered` | ROS | ○ | 入力の質に依存 |

## 5.4 あるべき姿と現在地のギャップ

原則 4 か条に照らすと、残るギャップは実質 1 つ — **EPOS4 系統の stamp=0** である。

```
ギャップ分析
├── 原則1 (単一時刻軸) .... ✅ 達成 (GPS 週秒は相対化、BNO086 は翻訳済み)
├── 原則2 (取得時刻主義) .. △ /sdk_could・/imu/data は達成。
│                            joint_states 系統のみ「到着時刻」止まり (上流依存)
├── 原則3 (基準の一致) .... ✅ 達成 (事例A の修正で stamp=スキャン開始に統一)
└── 原則4 (継承) .......... ✅ 達成 (epos4_odometry は 3 出力へ同一 stamp を継承)
```

joint_states 問題の影響範囲は限定的である: PDO 周期 50 ms・CAN 転送は ms 級なので
「到着時刻 ≈ 読み出し時刻 + 数 ms」であり、EKF の観測としては許容範囲。実害が出る
のは**左右ペアリングの位相ずれ** (事例C の偽ヨー、最大 ~0.5°/更新の過渡ノイズ) で、
これは stamp があっても ApproximateTime の粒度では消えない。恒久対策の選択肢
(到着順を仕様として受け入れて堅牢化する / 上流 ros2_canopen を改修して真の stamp を
入れる) は `docs/issue/2026-08-01_odometry_stop_wobble_zero_stamp_pairing.md` 参照。

## 5.5 GLIM (3D+IMU) 構成での時刻整合チェックポイント

これから行う GLIM 実データ検証で確認すべき時刻項目 (手順の詳細は
[第7章](07_checklist.md)):

1. `/sdk_could` と `/imu/data` の stamp が**同じ ROS 時刻軸で近い値**か
   (差が数 ms 以内。数十 ms あるなら `imu_time_offset` / `points_time_offset` で補正)
2. rfans の起動ログ `per-point time span of first scan: X s` が `1/rps` 程度か
   (0.000 なら utcTime が入っておらず deskew 不能)
3. bag 再生評価時は `use_sim_time` の扱いに注意 (第4章 4.6 節。glim_rosbag なら不要)

→ [第6章 事故事例集](06_case_studies.md)
