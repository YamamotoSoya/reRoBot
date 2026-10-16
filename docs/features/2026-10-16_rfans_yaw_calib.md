<!-- claude: feature-doc スキルの設計文書。Claude 作成。-->

# R-Fans / urg の yaw 較正ツール (rfans_yaw_calib) と 2026-10-16 の較正結果の反映

- 日付: 2026-10-16
- 対象パッケージ: `tools/rfans_yaw_calib/` (ROS ws 外の Python ツール)、`rerobot_bringup` の URDF 4 本、`ros2_ws_glim/config/config_sensors.*.json`
- 対象 ROS: ROS 2 Jazzy (reRoBot コンテナ `rerobot_env`。rosbag2_py / scipy を使う)
- 関連文書: `docs/issue/2026-10-16_amcl_lateral_bias_rfans_yaw_offset.md` (原因調査・因果実験)、`docs/issue/2026-10-15_keepout_corridor_stall_imu_yaw_jump.md` (urg −5° を入れた経緯)

## 1. 目的・概要

R-Fans (3D LiDAR) の点群座標系の x 軸が車体の進行方向から約 5.6° 反時計回りに回っていた。URDF `rfans_joint` は yaw=0 だったので、
AMCL は粒子を LiDAR 軸の方向に進め、実機の進行方向との差が更新ごとに左へ積もって「左 0.3〜0.4 m + yaw 時計回り 1〜3°」の
車体固定バイアスになっていた。10-15 には逆に urg (UTM-30LX) を R-Fans に合わせて −5° 回していたが、回っていたのは R-Fans の方だった。

ロボットを組み立て直すたびにこの角度を測り直せるよう、直進 bag だけから R-Fans の yaw と urg の相対角を同時に出すツールを作り、
今回の較正値を URDF と GLIM 設定に反映した。

実装した機能:
- `rfans_yaw_calib.py`: 直進 bag の R-Fans スキャン対を 2D ICP で合わせ、scan frame で見た移動方向 θ_m → `rfans_joint` yaw の補正 −θ_m。
  同時刻の urg スキャンを R-Fans に ICP で合わせた相対角 δ → `laser_joint` yaw = 現 + δ + (R-Fans 補正)。現 yaw は bag の `/tf_static` から読む。
- `glim_t_lidar_imu.py`: URDF の rfans / imu の xyz rpy から GLIM `T_lidar_imu` (TUM) を計算、既存 JSON との差も表示。
- 反映: `rfans_joint` に +0.098 rad (センサ z まわり、tilted プリセットは等価 rpy に換算)、`laser_joint` yaw −0.08727 → +0.0100、GLIM 4 JSON の `T_lidar_imu`。

スコープ外 (意図的に未対応):
- URDF / JSON の自動書き換え: 値を印字するだけ。4 ファイル × 2 系統を黙って書き換えると取付プリセットの運用と衝突しやすいため手動にした。
- ε の正体の識別 (センサの方位ゼロ / 取付 / 駆動輪の向き): どれでも URDF の yaw で吸収できる。組み立てごとの再測定で安定性を見る。
- 実機での確認 (廊下の角 2 面で urg と R-Fans の壁を重ねる): 未実施。

## 2. 設計の勘所

### 2.1 真値を「車体の進行方向」に取る

実測: 1643 (地図 bag) の GLIM 軌跡で、直進 8 s 窓の「移動方向 − LiDAR yaw」= −5.63° ± 0.34 (n=66)。本ツールの ICP でも −5.90° ± 0.15 (n=98)、
1752 bag で −5.06° (n=10)。
判断: 差動二輪は横滑りできないので進行方向 = base_link の x 軸。地図も GLIM も要らず、直進 bag 1 本で測れる。
採用値は再生実験で検証した +0.098 rad (5.6°): 同一 bag で scan を +0.098 rad 回して AMCL を再生すると横ずれ中央値 0.41 → 0.10〜0.17 m、
平均 −0.32 → 0.00 m (逆符号では 0.75 m に倍増)。

却下した代替案:
- 壁に平行に置いて静止計測: base_link の機械基準 (車体側面) に依存し ±1°。
- urg を基準にする: urg も取付誤差を持ちうる (10-15 はこの方法で逆に urg を回してしまった)。

### 2.2 urg は「R-Fans との相対角 + R-Fans の補正」で動かす

実測: urg→R-Fans の相対角 δ = −4.97° (1643、laser yaw 0 のとき) / −5.10° (1752、laser yaw −5° 込み) → 相対 ≈ −5.0°。
判断: R-Fans を +5.6° 回すなら urg も同じ分回す (ユーザ指示) → laser yaw = −5.0° + 5.6° = +0.6° (+0.0100 rad)。
つまり urg はもともと車体にほぼ合っていた。

### 2.3 tilted プリセットの rpy 換算

ε はセンサ自身の z まわりの回転なので R_new = R_old · Rz(ε) (intrinsic)。flat は yaw += ε でよいが、tilted15/45 は等価な固定軸 rpy に
scipy で換算した (tilted15: (0.0262, 0.2605, 0.1014)、tilted45: (−0.0975, −0.7806, −3.0034))。ε がセンサ固有という仮定なので、傾斜取付に戻すときは再測定。

### 2.4 ICP の採否条件 (誤収束対策)

対 = 1 s 離れたスキャン、移動 0.2〜1.2 m、対間ヨー変化 ≤1.5°、inlier 率 >0.5、RMS <0.12 m。方向は円平均、MAD の 3 倍で外れ値除去。
urg 対は時刻差 ≤30 ms、max_d 0.3 m、inlier >0.4、平行移動 <0.3 m。1709 bag (旋回が多い) では直進対が 0 本で停止する = 想定どおり。

## 3. データフロー

```
bag (/tf_static, /rfans/scan_all [base_link], /urg_front/scan [laser])
  ├─ /tf_static ─ base_link→rfans, base_link→laser の現 yaw (と laser の roll π の xy ブロック)
  ├─ scan_all(t), scan_all(t+1 s) ─ ICP ─ 移動ベクトル t_k (scan frame) ─ 円平均 → θ_m → rfans 補正 = −θ_m
  └─ urg(t) → base_link (現 URDF) ─ ICP → scan_all(t) ─ 中央値 → δ → laser 新 yaw = 現 + δ − θ_m
         ↓ 印字 (rpy の yaw)                       ↓ JSON (--out)
  URDF rerobot.{,flat,tilted15,tilted45}.urdf  →  glim_t_lidar_imu.py  →  ros2_ws_glim/config/config_sensors.*.json
```

## 4. 使い方

```bash
# main コンテナ内 (ws を source)。直進 10〜20 m を 30 s ほど録る
ros2 bag record -o bags/raw/$(TZ=Asia/Tokyo date +%Y-%m-%d_%H%M)_yawcalib /tf /tf_static /rfans/scan_all /urg_front/scan
python3 tools/rfans_yaw_calib/rfans_yaw_calib.py bags/raw/<bag> --out bags/exp/<bag>_calib.json
#   [ε] 直進対 98 本: scan frame で見た移動方向 -5.90° → rfans_joint yaw の補正 +5.90°
#   [δ] urg→R-Fans の相対角 -4.97° → laser_joint yaw 新 +0.93°
python3 tools/rfans_yaw_calib/glim_t_lidar_imu.py --urdf src/bringup/rerobot_bringup/urdf/rerobot.flat.urdf --json <config_sensors.flat.json のパス>
```

| 引数 | 既定 | 説明 |
|---|---|---|
| `--rfans` | `/rfans/scan_all` | R-Fans の LaserScan (base_link 表現)。`/rfans/scan` でも可 |
| `--urg` / `--no_urg` | `/urg_front/scan` | urg の LaserScan (frame laser)。無ければ δ は省略 |
| `--pair_dt` | 1.0 s | ICP 対の時間差。速度 0.4 m/s で 0.4 m |
| `--max_rot_deg` | 1.5° | 対間ヨー変化の上限 (旋回を除く) |

URDF は symlink install なので再ビルド不要。GLIM の JSON は直接 mount。地図は LiDAR 座標のまま作られているので作り直し不要。

## 5. 変更ファイル一覧

- `tools/rfans_yaw_calib/rfans_yaw_calib.py` — 新規 (ε / δ の計測)
- `tools/rfans_yaw_calib/glim_t_lidar_imu.py` — 新規 (T_lidar_imu の計算・検算)
- `tools/README.md` — 節追加
- `ros2_ws_main/src/bringup/rerobot_bringup/urdf/rerobot.{,flat,tilted15,tilted45}.urdf` — `rfans_joint` に +0.098 rad 合成、`laser_joint` yaw +0.0100
- `ros2_ws_glim/config/config_sensors.{,flat,tilted15,tilted45}.json` — `T_lidar_imu` 再計算 (回転 5.6°、並進 ≤2 mm)
- `docs/issue/2026-10-16_amcl_lateral_bias_rfans_yaw_offset.md` — ステータス更新

## 6. 既知の制限

- 実機未確認 (10-16 時点)。次の走行前に廊下の角で urg と R-Fans の壁が重なること、AMCL の横ずれが消えることを確認する。
- 2026-10-16 より前の bag を GLIM で再処理するときは旧 `T_lidar_imu` (JSON のコメントに履歴) を使う。AMCL の bag 再生は bag の `/tf_static` が旧 URDF なので、
  新 URDF の効果を見るには `bags/exp/2026-10-16_loc_wobble/replay/` の `rot_relay.py` 方式 (scan を回す) を使う。
- R-Fans 由来の costmap 障害物はこれまで 5 m 先で 0.5 m 回って置かれていた。10-14/15 の衝突停止の解釈に影響しうる。
- 積モデル `likelihood_field_prob` への切替 (ε 補正後は横 0.17 → 0.02 m) は `amcl.yaml` を別エージェントが編集中のため未反映。
