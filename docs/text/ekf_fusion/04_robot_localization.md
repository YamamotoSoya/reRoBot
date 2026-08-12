<!-- claude: EKF センサ融合読本 第4章 (2026-08-12) -->

# 第4章 robot_localization の機構 — ekf.yaml を一行ずつ読む

第3章で正しく作ったメッセージを、EKF 側は**設定ファイルに書かれたとおりに
選り分けて**食べる。この章は reRoBot の実設定
`ros2_ws_main/src/bringup/rerobot_bringup/config/ekf.yaml` (全 51 行) を
教材に、robot_localization の設定言語を読めるようにする。読み終わると、
このファイルの 1 行 1 行が「何を信じ、何を捨てるか」の宣言だと分かる。

## 4.1 config 行列 — 15 個の bool は 3 個ずつ区切る

センサごとの `_config` は、第2章 §2.3 の 15 次元状態ベクトルに対応した
**「どの状態量をこのセンサから採用するか」のチェックボックス**である。
3 個ずつ 5 行に改行して書けば目視で読める:

```yaml
# ekf.yaml:31-38 — 車輪オドメトリ
odom0: /odom
odom0_config: [false, false, false,    # x,  y,  z        ← pose は取らない
               false, false, false,    # r,  p,  yaw      ← 姿勢も取らない
               true,  true,  false,    # vx, vy, vz       ← 並進速度を取る
               false, false, true,     # vr, vp, vyaw     ← yaw rate を取る
               false, false, false]    # ax, ay, az
```

```yaml
# ekf.yaml:40-49 — BNO086 IMU
imu0: /imu/data
imu0_config: [false, false, false,     # x,  y,  z
              false, false, true,      # r,  p,  yaw      ← yaw (姿勢) を取る
              false, false, false,     # vx, vy, vz
              false, false, true,      # vr, vp, vyaw     ← yaw rate を取る
              false, false, false]     # ax, ay, az       ← 加速度は捨てる
```

まとめると分担はこうなる:

```
状態量ごとの情報源 (reRoBot)
├── vx, vy .......... 車輪 odom のみ (IMU の二重積分は使わない — 第1章 §1.2)
├── yaw ............. IMU のみ (車輪の積分 yaw は使わない)
├── vyaw ............ 両方 → ここだけ重み付き平均が起きる (第2章 §2.1)
├── x, y ............ どのセンサからも取らない → EKF が vx, vyaw を自分で積分
└── z, roll, pitch .. two_d_mode が常時 0 に拘束 (§4.4)
```

## 4.2 設計判断の解剖 (1) — なぜ pose を取らないのか

`odom0_config` の第 1〜2 行が全部 false であることが、このファイル最大の
設計判断である。`/odom` には積分済みの pose (x, y, yaw) が入っているのに捨て、
速度だけ取る。理由は `ekf.yaml:7-9` のコメントに明文化されている:

> 車輪 odom からは「速度」だけ取る (vx, vy, vyaw)。積分済みの pose を取ると
> IMU と二重に積分誤差を持ち込むため。

順を追うと:

1. `/odom` の pose は epos4_odometry が**すでに積分した結果**で、
   第1章 §1.1 の蓄積誤差を丸ごと抱えている。
2. EKF も内部で速度を積分して位置を作る。pose も速度も両方食わせると、
   **同じ車輪の誤差が「積分済み pose」と「速度の再積分」の 2 経路から
   二重に入る**。しかも 2 経路は強く相関しているのに、EKF は独立な測定として
   扱ってしまう (第2章の非対角 = 相関を正しく書けないため)。
3. 速度だけ渡せば、積分は EKF の中で 1 回だけ。積分誤差の管理
   (共分散の膨張) も EKF が一元的にやれる。

一般則: **積分済みの量と、その導関数を、同じ情報源から両方食わせない**。
どちらか 1 つ、原則は微分側 (速度) を渡す。

## 4.3 設計判断の解剖 (2) — vy=0 は「測定」に化けた拘束

`odom0_config` で vy が true なのは一見おかしい — 差動駆動は横に
進めないのだから、車輪から vy を「測定」などできない。実はこれは
**「vy は常に 0 である」という車体の物理的性質 (非ホロノミック拘束) を、
σ² = 0.001 の自信を持った測定としてEKF に教え込む**トリックである
(`ekf.yaml:8-9`、値は `params.yaml:37`)。

これがないと、IMU のノイズや yaw の誤差が積分の過程で横方向の速度成分として
漏れ出し、ロボットが**横滑りしながら進む推定**になる。毎周期「vy = 0.000」
という測定が届くことで、横方向のドリフトはその都度押し戻される。

一般則: **モデルの拘束は、専用の設定項目がなければ「高信頼の擬似測定」として
注入できる**。robot_localization ではこの vy=0 が定石として知られている。

## 4.4 設計判断の解剖 (3) — imu0_relative と two_d_mode

```yaml
imu0_differential: false   # :46
imu0_relative: true        # :47
two_d_mode: true           # :19  z / roll / pitch とその速度を常に 0 に拘束
```

- **`imu0_relative: true`** — IMU の yaw を絶対値ではなく
  **「起動時の向きを 0 とした相対角」**として使う。BNO086 の絶対 yaw は
  磁北基準で、モータや鉄骨の磁気外乱で汚れる保証のなさがある
  (`ekf.yaml:12-14`)。相対化すれば「起動時からどれだけ回ったか」だけを
  信じることになり、磁北そのものへの依存が切れる — ただし磁気外乱による
  **動的な**揺れまでは消えない (それが事例Dの疑い。[第6章](06_case_studies.md))。
- **`differential` との違い**: `relative` は「初期値を引く」だけ。
  `differential: true` は姿勢を毎回微分して**角速度に変換してから**使う。
  どちらも絶対値依存を切る手段だが、differential は測定のたびに微分ノイズが
  乗る。reRoBot は yaw rate を別途ジャイロ直読みで取っているので、
  yaw は relative で姿勢のまま使う構成になっている。
- **`two_d_mode: true`** — z, roll, pitch とその速度を推定対象から外し
  常時 0 に拘束する。15 次元が実質 6 次元 (x, y, yaw, vx, vy, vyaw) になり、
  観測不能な軸が漂って発散する事故を構造的に防ぐ。屋内・平地の 2D ロボットでは
  ほぼ必須のスイッチ。

## 4.5 二段推定 — world_frame と「map→odom は AMCL の仕事」

```yaml
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom          # :26  map フレームの補正は AMCL / SLAM 側の仕事
```

`world_frame: odom` は「この EKF は odom 座標系の中で推定する局所推定器で
ある」という宣言である。ROS の自己位置推定は伝統的に**二段構え**になっている:

```
自己位置の二段推定 (REP-105)
├── odom→base_link ......... EKF (robot_localization) が担当
│   ├── 性質: 連続・滑らか・高レート (30 Hz)
│   ├── ただしドリフトする (デッドレコニングの積分だから)
│   └── 用途: 制御ループ (Nav2 の controller が読む側)
└── map→odom ............... AMCL / SLAM が担当
    ├── 性質: 地図と照合した絶対位置。ドリフトしない
    ├── ただし不連続 (照合のたびにジャンプする)
    └── 用途: 大域的な位置 (map→base_link = 2 段の合成で得る)
```

ジャンプする補正 (map→odom) と滑らかな積分 (odom→base_link) を
**別の変換に分離する**ことで、制御ループには滑らかな方だけが見え、
大域位置は 2 つの合成として矛盾なく得られる。EKF が `world_frame: map` で
動くモード (GPS 等の絶対測定を食わせる第 2 インスタンス) もあるが、
reRoBot では map 側は AMCL に任せている
(`nav2_params.yaml:49-55` の amcl フレーム設定と対になる)。

## 4.6 TF 発行権 — odom→base_link は誰が出すか

```yaml
publish_tf: true           # :20  odom -> base_link は EKF が出す
```

odom→base_link という**同じ変換を publish できるノードが 2 ついる**
(epos4_odometry `epos4_odometry.cpp:183-196` と EKF)。両方が publish すると、
TF ツリー上で同じ子フレームに 2 つの値が交互に書き込まれ、下流 (Nav2 の
costmap、RViz) から見るとロボットが 2 つの姿勢の間で振動する。

原則: **1 つの変換の発行者は常に 1 ノード**。reRoBot では
「EKF 有効時は EKF が正、epos4_odometry の TF は自動で止める」を
launch レベルで機械的に保証している (実装は [第5章](05_rerobot_architecture.md) §5.2)。
`/odom` トピック自体は publish し続けることに注意 — EKF の入力であり、
生オドメトリとの比較データでもある (第7章の検証で使う)。

## 4.7 残りの運転パラメータ

```yaml
frequency: 30.0            # :17 予測ステップの周期 (第2章 §2.4)
sensor_timeout: 0.2        # :18 これを超えて無入力のセンサは見放す
odom0_queue_size: 10       # :38 1 周期に処理する最大メッセージ数
imu0_queue_size: 20        # :48 IMU は 100 Hz なので深め
imu0_remove_gravitational_acceleration: true   # :49 (加速度は捨てているが作法として)
publish_acceleration: false  # :21
```

queue_size は「センサレート ÷ frequency に余裕を掛けた値」が目安
(IMU 100 Hz ÷ 30 Hz ≈ 3.3 → 20 は十分)。時刻整列・queue あふれ・
timeout の詳細な機構は
[タイムスタンプ読本 第4章 §4.4](../timestamp/04_time_consumers.md) が正典。

`process_noise_covariance` (予測ステップで共分散をどれだけ膨らませるか) は
**このファイルに書かれていない = robot_localization の既定値のまま**である
(`ekf.yaml:51` のコメントが唯一の言及)。チューニング項目としての位置づけは
[第5章 §5.4](05_rerobot_architecture.md)。

## 4.8 この章のまとめ

```
第4章 まとめ
├── config 行列は 3 個ずつ区切る: [位置 | 姿勢 | 速度 | 角速度 | 加速度]
├── 積分済み pose とその速度を同じ情報源から両方食わせない → 速度側を渡す
├── vy=0 は物理拘束を「高信頼の擬似測定」に化けさせた定石
├── imu0_relative = 起動時基準の相対 yaw (磁北への静的依存を切る。動的外乱は残る)
├── two_d_mode = 観測不能軸の発散を構造的に封じる 2D ロボットの必須スイッチ
├── world_frame: odom = 局所推定器の宣言。map→odom (ジャンプ側) は AMCL の仕事
└── 1 つの TF 変換の発行者は常に 1 ノード — EKF 有効時は epos4_odometry の TF を止める
```

→ [第5章 reRoBot の EKF アーキテクチャ](05_rerobot_architecture.md) — 設定の
言葉が読めたところで、システム全体の配管 (launch・URDF・Nav2 接続) を俯瞰する。
