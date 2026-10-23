<!-- claude: 2026-09-26 作成。BNO086_ROS2Board (ドライバ + 基板) の開発者に渡す申し送り。reRoBot 搭載での実測に基づく。 -->
# BNO086_ROS2Board ドライバ — ロボット搭載で見つかった要修正点 (申し送り、2026-09-26)

対象: `bno086_imu_driver` (ROS 2 Jazzy、USB CDC firmware)。reRoBot (差動 2 輪、R-Fans-16 + GLIM の LIO) に 2026-08 から搭載し、2026-09-24〜26 に比較用 IMU (WITmotion WT901C) と並べて実測した結果です。詳細データは末尾の参照先にあります。

## 要約 (優先度順)

| # | 問題 | 影響 | 提案 |
|---|---|---|---|
| 1 | **`auto_tare: all` (既定) が accel / gyro を含む全モーション出力に定数回転を掛ける** | `/imu/data` の accel・gyro が物理センサ座標でなくなる。yaw は起動時の磁北との差 (起動ごとに任意)、roll/pitch は起動時の姿勢 + 加速度計バイアス由来の偽傾き。LIO/SLAM の外部パラメータが起動ごとに狂い、取付向きの検証も不可能になる (当方は上下逆取付を 1.5 か月見落とした) | 既定を `off` に。姿勢を 0 から始めたい用途は **orientation だけ**をドライバ側でオフセット (quaternion の左掛け) し、accel/gyro の生値は触らない。README に「tare はモーション出力も回す (データシート §4.1.1)」を明記 |
| 2 | **`~/tare` / `~/tare_yaw` が persist=True で FRS に焼き、クリア手段がない** | 一度呼ぶと reset でも消えない定数回転が残り得る (README には「persist が効かない」とあるが、効いた場合の逃げ道がない) | 既定 persist=False、永続化は別サービスに分離、Tare Set Reorientation (identity) で消す `~/tare_clear` を追加 |
| 3 | **加速度計の較正状態が見えず、バイアスが大きい** | 静置 accel に x 軸 +0.8〜1.1 m/s² のバイアス (日内で 0.3 変動)、\|g\| が姿勢で 9.78〜9.94 と 1.6% 変動、\|g\| 平均も日で 9.86〜10.1。tare-all がこれを「傾き」として隠していた | `/diagnostics` に accel/gyro/mag 各 accuracy と「DCD 保存済みか」を出す。`save_calibration` (Save DCD 0x06) は 2026-09-26 に当方で `device acknowledged save DCD: OK (R=[0,0,0,0])` を確認 — README の「実機応答未確認」は更新可。電源断後の保持も当方で確認済み) の記述更新と、較正手順 (水平面は不要、6 面静置 + ゆっくり回転 → 保存 → 電源再投入で保持確認) を README に |
| 4 | **静止時の gyro が全軸ちょうど 0.000** (ゼロクランプ) | バイアス・ノイズ評価ができない。走行中のみ値が出るため静止窓法が使えない | SH-2 の calibrated gyro の仕様なら README に明記し、`GYROSCOPE_UNCALIBRATED` (raw) を選べるパラメータを追加 |
| 5 | **基板座標系と `mount_yaw_deg: 180` の定義が図で示されていない** | URDF の imu_link = 基板座標 (チップ座標 +180°) と理解するまでに時間がかかった。上下逆取付を静止 accel z の符号で検出する手順も tare-all では成立しない | README に基板シルクの軸・チップ配置・`mount_yaw_deg` の意味を図で。取付検証手順 (auto_tare off で静止 accel z の符号、傾けテスト) を追記 |

良かった点 (変更不要): デバイス時計→ROS 時刻の stamp (受信遅延 0.3 ms、ジッタ ±0.5 ms)、200 Hz で 23 分の走行 bag に穴なし、USB CDC の再接続処理。

## 各項目の根拠

### 1. auto_tare が accel/gyro を回す (因果実験で確定)
- 機体を 14.5° 傾けたまま `~/reset` → 2 s 後に auto_tare 発火 → **accel の傾きが 14.57° → 0.02°** に消えた (物理は傾いたまま)。水平に戻すと −14.8° 傾いて見え、機体を 90° 回してもセンサ座標で不変 = センサ固定の定数回転。
- tare-all 時代の走行 bag 3 本で、旋回中の gyro 回転軸が公開座標の z から 4.8〜7.3° 傾き、右旋回/左旋回で同符号 (= 定数回転の署名)。auto_tare off では 0.5°。
- off にして reset すると **accel z = −9.9 (上下逆)** が初めて現れた。実取付は上下逆 + yaw −93° だったが、tare-all では z が常に +9.8 に見えていた。
- 実装箇所: `imu_node.py` `_check_auto_tare` → `encode_tare(persist=False, axes=all)`。データシート BNO080/085 §4.1.1 "This orientation will then be applied to all motion outputs"。

### 2. tare サービスの persist
- `imu_node.py` L386/L391: `encode_tare(persist=True)` を `~/tare` / `~/tare_yaw` が送る。firmware `host_link.c` HL_MSG_TARE は Tare Now と Persist のみ実装、クリア (Set Reorientation) なし。
- README 533〜536 行「Persist Tare が効いていない」と併存しているが、効くファーム/個体では消せない回転が残る。

### 3. 加速度計
- 静置 accel (物理座標、auto_tare off): 09-24 (1.144, 0.155, −9.935) → 09-25 朝 (0.789, 0.159, −9.906) → 09-25 bag 末尾 (0.879, 0.06, −0.07 相当) — x バイアス 0.8〜1.1 m/s² が日内変動。姿勢 3 種で \|g\| 9.94 / 9.94 / 9.78。同時計測の WT901C は 9.84 ± 0.01。
- GLIM (LIO) ではこのバイアスが地図の水平を約 5° 傾け、走る方向で上り下りが変わる z ドリフトになる (同一 bag でバイアスを除去すると挙動が変わることを確認)。

### 4. gyro ゼロクランプ
- 静置で角速度 3 軸が厳密に 0.000、標準偏差 0。WT901C は 0.03 °/s のノイズが見える。走行中は値が出る。

### 5. 座標系
- 実取付は「部品面が下 (上下逆)、USB コネクタが車体左」。URDF は 1.5 か月間 (0, 0, +π/2) (正立) で運用されていた。auto_tare off + 静止 accel z の符号 + 前上げ/左上げの傾けテストで (π, 0, −93°) と実測。

## 参照 (reRoBot リポジトリ内)
- `docs/issue/2026-09-24_bno086_auto_tare_rotates_motion_outputs.md` — 因果実験・取付実測・GLIM への影響 (14 run)
- `docs/features/2026-09-23_wt901c_comparison_imu.md` — 比較用 IMU (WT901C) の導入と同時計測手順
- `ros2_ws_main/src/bringup/rerobot_bringup/config/bno086.yaml` — 当方の運用設定 (auto_tare: off)
- 実測データ: `bags/5goukan/2d3dimu/offline/imu_compare/` (0751 bag の両 IMU 比較図)、`bags/5goukan/2d3dimu/offline/glim/exp_2026-09-25_tareoff_0751/`

## 付録: 当方で行った加速度計の再較正 (2026-09-26 実施)

実施結果: 基板を外して机上 (視覚的に水平) で 6 面静置 + 3 軸ゆっくり回転 → `orientation_accuracy` unreliable → low (1 分) → high (3 分) → `save_calibration` → `save DCD: OK`。0°/180° 反転法 (机の傾きとバイアスを分離) で **水平バイアス 0.34 → 0.08 m/s² (2.0° → 0.5° 相当)**。**USB 抜き差し (電源断) 後もバイアス 0.09 m/s² で保持 = Save DCD は実機で有効**。

### 手順 (案)

水平面は**不要**です (BNO086 の動的較正は複数姿勢での静置と回転から内部で求めるので、床の 1° の傾きは影響しない。検証時に「視覚的に水平」の床で水平成分 <0.2 m/s² を見れば、1 m/s² のバイアスの有無は判定できる)。

1. `auto_tare: off` でドライバ起動 (reRoBot の bringup 経由なら既定)。`/diagnostics` の accel/gyro/mag accuracy を見る。
2. 基板 (車体から外せるなら外して) を **6 面それぞれ上向きで 3 秒ずつ静置** → 各軸まわりに**ゆっくり (数秒/回転) 1 回転ずつ**。accuracy が `high` になるまで繰り返す。
3. `ros2 service call /bno086_imu_driver/save_calibration std_srvs/srv/Trigger`。
4. **電源を切って入れ直し**、静置 accel を再測 (`imu_compare.py` 相当: 6 秒平均)。水平成分 <0.2 m/s²・\|g\| 9.81 ± 0.05 なら成功。保持されていなければ Save DCD が効いていない (→ 上記 3 の開発者確認事項)。
5. 走行 bag で GLIM を回し、odom 段の「LiDAR z 軸の車体固定傾き」が <1° (較正前 4.4°) になっていれば IMU 入力は正しい。
