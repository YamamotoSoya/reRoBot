<!-- claude: docs/issue — 未解決問題の調査記録。解決したらステータスを更新すること。-->
# ホイールオドメトリの回転数が信頼できない問題(エンコーダ分解能スケーリングの4倍ズレ)

- **ステータス: 未解決**(`gear_ratio: 1.25` による対症療法で運用中。根本原因は未修正)
- 日付: 2026-07-07
- 環境: Docker コンテナ `rerobot_env` / ROS 2 Jazzy / `ros2_canopen`(Cia402Driver)、maxon EPOS4 ×2 over can0
- 対象ブランチ: `main` (HEAD: `e5695fb feat: implement 2D and 3D LiDAR configurations; add launch files and parameters for epos4_controller and epos4_odometry`)
- 関連ファイル:
  - `src/external/epos4compact50-5can/maxon_epos4_ros2/config/bus_config_cia402_epos4_vel/bus.yml`(根本原因の所在 — `scale_pos_from_dev`)
  - `src/rerobot_bringup/config/params_2d.yaml` / `params_3d.yaml`(対症療法 `gear_ratio: 1.25` ×各2箇所)
  - `src/epos4_teleop/config/params.yaml`(同 `gear_ratio: 1.25`)
  - `src/epos4_controller/src/epos4_controller.cpp`(速度指令側 — rpm 計算に gear_ratio を使用)
  - `src/epos4_controller/src/epos4_odometry.cpp`(オドメトリ側 — 位置換算に gear_ratio を使用)
  - EPOS4 本体の NVM 設定(EPOS Studio で書き込まれたエンコーダパルス数 0x3010:01)— リポジトリ外

## TL;DR

ホイールオドメトリの回転数(joint_states.position から換算した車輪回転量)が物理量と合わない問題。実測から逆算すると、ズレはちょうど **4 倍**で、原因は当初疑われた「EPOS Studio でギヤ比を誤って 4:1 に設定した」ことでは**ない**。真の原因は `bus.yml` の `scale_pos_from_dev = 2π/4096` が「モータ1回転 = 4096 カウント」を仮定しているのに対し、実機エンコーダはモータ1回転あたり **1024 カウント**(= 256 パルス × 4逓倍)しか出ていないこと。「1024」という数字をパルス数と読むか4逓倍後カウント数と読むかの取り違え(クアドラチャ係数 4 の二重解釈)で、この bus.yml は maxon サンプルリポジトリ由来の値のまま実機に合わせていない。現在は `gear_ratio: 1.25`(= 真のギヤ比 5 ÷ スケーリング誤差 4)で数値上は補正されているが、物理的に意味のないパラメータ値が5箇所に散在しており、根本修正が必要。

## 症状

- **何が起きるか**: `/motor{1,2}/cia402_device_{1,2}/joint_states` の position(モータ軸角度 [rad] のはず)から計算した車輪回転数・走行距離が、実際の 1/4 になる。
- **実測**(`params_2d.yaml` のコメントに記録): 車輪を 10 回転させたとき position の生の増分が 78.55 rad = 車輪1回転あたり **2.5π rad**。
- **期待値**: ギヤ比 5:1 なら車輪1回転 = モータ5回転 = **10π rad**(≈ 31.4 rad)と報告されるはず。
- **ズレの係数**: 10π ÷ 2.5π = **ちょうど 4**。

## 対症療法(現状)

`gear_ratio` を物理値の 5.0 ではなく **1.25** に設定して補正している(履歴: 1.0 → 0.2 → 1.25)。オドメトリ距離・teleop の距離表示は数値上正しくなるが、以下の問題が残る:

- `gear_ratio` が物理的なギヤ比(motor_rev / wheel_rev = 5)を表しておらず、センサスケーリング誤差を運動学パラメータに折り込んでいる。将来ギヤやエンコーダを変えたとき必ず混乱する。
- 該当箇所が 5 つに分散: `params_2d.yaml` ×2、`params_3d.yaml` ×2、`epos4_teleop/config/params.yaml` ×1。

## 切り分けの記録

### 1. 「EPOS Studio でギヤ比 4:1 に誤設定」説 → 否定

- EPOS4 の position actual value (0x6064) は**メインセンサの物理エッジカウントの生値**。EPOS Studio のギヤ設定は制御チューニング・単位系表示に影響するだけで、0x6064 の報告値は変わらない。
- もしギヤが実際に 4:1 なら、5:1 前提とのズレは 5/4 = 1.25 倍のはず。実測のズレは 4 倍で数字が合わない。
- そもそも EPOS Studio でギヤ比は設定していなかった(ユーザ記憶)。

### 2. 実測値からエンコーダ分解能を逆算 → 当たり

`bus.yml:19` の `scale_pos_from_dev: 0.0015339 (= 2π/4096)` が報告値を作っている。実測 2.5π rad/車輪回転から逆算:

```
2.5π rad ÷ (2π/4096) = 5120 カウント / 車輪1回転
5120 ÷ 5(真のギヤ比)= 1024 カウント / モータ1回転
```

→ 実機エンコーダは **1024 inc/rev(= 256 パルス × 4逓倍)**。bus.yml の仮定 4096(= 1024 パルス × 4逓倍)と**ちょうど4倍 = クアドラチャ係数**だけ食い違う。「1024 パルス」と「1024 カウント(4逓倍済み)」の取り違えが濃厚。なお `bus.yml` は maxon のサンプル(submodule `epos4compact50-5can`)由来で、4096 は maxon デモ機の値。EDS のデフォルトは 500 パルスなので、EPOS Studio 側の設定は過去に誰かが必ず触っている。

### 3. 速度指令側の矛盾 → EPOS Studio のエンコーダ分解能誤設定を示唆

`epos4_controller.cpp` の `cmdSpeedCallback` は `gear_ratio = 1.25` でモータ rpm を計算して 0x60FF に送るため、真のギヤ比 5 に対して**指令は 4 倍不足**のはず。それでも実機がほぼ狙い通りの速度で走っているなら、**EPOS Studio 側のエンコーダパルス数 (0x3010:01) が実機の 4 倍(例: 実際 256 パルスのところ 1024 パルスと設定)**になっていて、速度ループが「4倍遅い」と誤認して4倍速く回し、指令不足と相殺されている、と説明がつく。

→ 「EPOS Studio で何か間違えた」という直感は正しい可能性が高いが、間違えたのは**ギヤ比ではなくエンコーダ分解能**。

## 未検証事項(実機で要確認)

1. **0x3010:01**(Digital incremental encoder 1 number of pulses)と **0x3000:05**(Main sensor resolution)の実値。EPOS Studio または実機で:
   ```bash
   ros2 service call /motor1/cia402_device_1/sdo_read canopen_interfaces/srv/CORead "{index: 0x3010, subindex: 1}"
   ```
2. **モータ軸を手で正確に1回転**させたときの 0x6064 増分(1024 なら上記仮説どおり)。
3. エンコーダの型番・データシートで pulses per revolution を確認(256 パルスと予想)。
4. **tpdo トピックのスケーリング有無**: CLAUDE.md は「tpdo の値は driver が `scale_vel_to_dev` でスケールする」と記述しているが、ros2_canopen の生 tpdo トピックは通常スケーリングされない(スケーリングは `target` サービス経由のみ)。0x60FF に届く単位を candump か EPOS Studio モニタで確認してから速度側を直すこと。

## 根本修正の手順(検証後)

1. EPOS Studio で 0x3010:01 を実エンコーダのパルス数(おそらく 256)に修正し NVM 保存。**この時点で速度ループの4倍相殺が消えるため、2〜3 を同時にやらないと4倍速で走る。単体では絶対に適用しないこと。**
2. `bus.yml` の `scale_pos_from_dev` を `2π/実カウント数`(1024 inc/rev なら 0.0061359)、`scale_pos_to_dev` をその逆数(163.0)に修正。submodule 内のファイルなので変更は `src/external/epos4compact50-5can` 側にコミットが必要。
3. `gear_ratio` を物理値の **5.0** に戻す(5箇所: `params_2d.yaml` ×2、`params_3d.yaml` ×2、`epos4_teleop/config/params.yaml` ×1)。コメントも「motor_rev / wheel_rev(物理ギヤ比)」に更新。
4. 検証: teleop で 10 m 直進 → メジャー実測と `/odom` / teleop 距離表示を突き合わせ。その場旋回 360° → `/odom` の yaw が 2π に戻ること。指令 0.5 m/s に対する実速度をストップウォッチで確認。

## 教訓

- ベンダサンプルの `bus.yml` のスケーリング係数(`scale_pos_from_dev` 等)はデモ機のエンコーダ前提。実機のエンコーダ仕様(パルス数×4逓倍)に必ず合わせる。
- センサスケーリングの誤差を `gear_ratio` のような運動学パラメータで補正すると、数値は合っても物理的意味が失われ、後段の全ての消費者(controller / odometry / teleop)に非物理値が伝播する。
- 「ちょうど 4 倍」のズレを見たら、まずクアドラチャ4逓倍の二重解釈(パルス数 vs カウント数)を疑う。
