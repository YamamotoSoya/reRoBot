<!-- claude: 運用手引き 第15章 (2026-10-08 作成、ユーザ依頼「launch の使い分けを manual にメモ」。
     同日、robot 側を全部入り 1 本 + scan 改名に変更したのに合わせて書き直し) -->

# 第15章 launch の使い分け

基本は **robot 側を 1 本上げる (常駐) → nav 側を 1 本重ねる (起動・停止を繰り返す)**。
robot 側は `rerobot_bringup.launch.py` の 1 本だけ (既定で全部起動)。nav 側は目的に合わせてファイル名で選ぶ。
各 launch が出す topic は [第4章](04_startup_teleop.md) (robot 側) と [第10章](10_nav2.md) (nav 側) の表を参照。

---

## 15.1 目的から選ぶ

```
rerobot_bringup.launch.py (全部入り、常駐) の上に:
├─ 手で動かすだけ             joy_teleop (またはキーボード teleop)
├─ 2D 地図を作る              slamtoolbox コンテナの slam.launch.py      (/urg_front/scan を読む)
├─ 3D 地図を作る / bag 記録   glim コンテナの GLIM / ros2 bag record
└─ 自律走行 (Nav2)
     ├─ GLIM 地図で走る        nav_amcl                (/rfans/scan を読む)
     │                         nav_amcl_scanall        (/rfans/scan_all。robot 側 rfans_scan_all:=true)
     │                         nav_emcl2               (/rfans/scan)
     ├─ urg の地図で走る       nav_amcl / nav_emcl2 に scan_topic:=/urg_front/scan
     └─ 3D 自己位置推定        nav_lidar3d (未実装)
```

## 15.2 robot 側: `rerobot_bringup.launch.py` (1 本だけ)

既定で urg + R-Fans + 3D→2D (`/rfans/scan`) + BNO086 + WT901C + EKF を全部起動する。外したいときだけ引数を付ける:

| こうしたいとき | 付ける引数 |
|---|---|
| urg が無い / 壊れた | `lidar_2d:=false` |
| R-Fans が無い (3D→2D も止まる) | `lidar_3d:=false` |
| 全点スキャンを使う (nav_amcl_scanall) | `rfans_scan_all:=true` (重いので既定 off) |
| WT901C が無い | `imu_wit:=false` |
| IMU なし (EKF も意味がなくなる) | `imu:=false ekf:=false` |

- 未接続のセンサを外し忘れても、そのドライバがエラーを出し続け、起動結果の表に ✘ が出るだけで他は動く。
- 全部を配信しても、受け手のいない topic は DDS が実際には送らないので通信の負担はほぼ無い。
  CPU は R-Fans の点群計算と `/rfans/scan` 変換の分だけ増える (`/rfans/scan_all` は重いので既定 off)。
- `ros2 bag record -a` は全部記録して大きくなる。topic を指定して記録すること (第5章)。

## 15.3 nav 側 (自己位置推定 + Nav2 + RViz、robot の後に上げる)

| launch | 自己位置推定 | 入力スキャン | 使う場面 |
|---|---|---|---|
| `nav_amcl` | AMCL | `scan_topic` (既定 `/rfans/scan`) | 標準。迷ったらこれ |
| `nav_amcl_scanall` | AMCL (全点 + 自己位置推定専用地図) | `/rfans/scan_all` | 屋外で最近点だけだとずれるとき。`loc_map_yaml:=` と併用 |
| `nav_emcl2` | emcl2 | `scan_topic` (既定 `/rfans/scan`) | AMCL との比較用 (未チューニング・実際の位置合わせは未確認) |
| `nav_lidar3d` | 3D (未実装) | — | 起動するとエラーで止まる |

⚠️ `scan_topic` は **地図を作ったセンサに合わせる**。GLIM 地図 = `/rfans/scan`、slam_toolbox 地図 = `/urg_front/scan`。
costmap (障害物) は自己位置推定と関係なく `/urg_front/scan` と `/rfans/scan` の両方を常に読む。

起動例 (つくばの GLIM 地図 + 全点 AMCL):

```bash
# 端末 1 (robot 側、常駐)
ros2 launch rerobot_bringup rerobot_bringup.launch.py rfans_scan_all:=true scan_all_min_height:=0.3 scan_all_max_height:=3.0
# 端末 2 (nav 側)
ros2 launch rerobot_bringup nav_amcl_scanall.launch.py \
  map_dir:=/workspace/maps/2d/glim/2026-10-03_1126_tsukuba_dumpbase \
  loc_map_yaml:=/workspace/maps/2d/glim/2026-10-03_1126_tsukuba_dumpbase/loc_0.3-3.0/map.yaml
```

⚠️ `scan_all_min/max_height` (robot 側) は、`loc_map_yaml` の地図を作ったときの帯 (この例だと `loc_0.3-3.0` = 0.3〜3.0 m) と揃える。
ずれると、自己位置推定が「地図に無い壁」を観測して暴れる。

## 15.4 単体で使う launch

| launch | 使う場面 |
|---|---|
| `joy_teleop` | ゲームパッド操作 (robot 側と一緒に) |
| `imu_check` | IMU の取付向きを RViz で確認 (モータには触らない) |
| `amcl_variants` | AMCL の設定違いを並走させて比較 (評価用、TF は出さない) |
| `realsense_imu` | RealSense を IMU として使う (LIO-SAM 用、凍結中) |

## 15.5 直接は使わない launch (部品)

| launch | 中身 |
|---|---|
| `rfans_scan` | 点群→`/rfans/scan` (+ `_all` / `_far`)。rerobot_bringup に入っている。**別途起動すると二重になる** |
| `wt901_imu` | WT901C。rerobot_bringup に入っている |
| `parts/*` | nav 側の部品 (`map_keepout` / `navigation` / `rviz_nav` / `localization/loc_*`)。部品単体のデバッグ用 |

## 15.6 互換のために残しているもの (新しく使う理由はない)

| launch | 代わりに使うもの |
|---|---|
| `nav2` | `nav_amcl` (`amcl_scan:=all` を付けていたら `nav_amcl_scanall`)。既定の入力は `/urg_front/scan` なので GLIM 地図なら `scan_topic:=/rfans/scan` |
| `rerobot_bringup_2d` / `_3d` / `_2d_imu` / `_3d_imu` / `_2d3d_imu` | `rerobot_bringup` (全部入り)。旧ラッパは EKF off など挙動が違う。削除候補 |

## 15.7 注意

- nav 側は 1 本だけ起動する。2 本上げると `map→odom` の TF を 2 か所から出してしまう。
- 裸の `/scan` は誰も出さない。過去の bag (`/scan` で記録) を再生するときは
  `--remap /scan:=/rfans/scan` (urg の日なら `/urg_front/scan`) を付ける。
- nav 側のファイルは `launch/nav/`・`launch/parts/` にあるが、`ros2 launch` にはファイル名だけを渡せばよい
  (`ros2 launch rerobot_bringup nav_amcl.launch.py`)。

---

← [第14章 pointcloud_to_laserscan](14_pointcloud_to_laserscan.md) | ↑ [目次](00_index.md)
