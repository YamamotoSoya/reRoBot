<!-- claude: 運用手引き 第14章 (2026-10-01 作成、ユーザ依頼「pointcloud_to_laserscan の章を追加して各引数の説明と起動コマンドを」) -->

# 第14章 pointcloud_to_laserscan (3D 点群 → /scan)

R-Fans の 3D 点群を、AMCL と costmap が読める 2D の `LaserScan` に変換する段。起動は `rfans_scan.launch.py` 1 本で、AMCL が読むスキャンを **最近点 (従来)・全点・最遠点** から選べる。

---

## 1. 何をしているか

```
/rfans_driver/rfans_points (1 回転 約 3 万点、frame rfans)
 └─ ① base_link へ座標変換 (取付角は URDF が吸収)
    └─ ② 高さ帯 [min_height, max_height] の点だけ残す (base_link 基準)
       └─ ③ 方位で席 (ビン) に割り振る ── LaserScan は「1 席 1 距離」しか持てない
          ├─ 最近点: 席 0.0035 rad (0.2°)、席に来た点のうち一番近い 1 点   → /scan       (costmap + 従来の AMCL)
          ├─ 全点  : 席 0.00016 rad (R-Fans の方位刻み)、ほぼ 1 点 1 席     → /scan_all   (AMCL 専用)
          └─ 最遠点: 席 0.0035 rad、席に来た点のうち一番遠い 1 点          → /scan_far   (AMCL 専用)
```

- R-Fans の 16 本のレーザは同じ方位では撃たず、方位が 0.009° ずつずれて並ぶ。0.2° の席には 16〜17 点が入るので、最近点では 15 点を捨てている。
- 全点は席を R-Fans の刻みまで細かくして、点ごとに自分の席を持たせる。点が来なかった席は `inf` (測定なし) になり、AMCL はそのビームを無視する。
- costmap (障害物) は常に `/scan` (最近点) を読む。近い障害物を見落とさないためと、3.9 万本のレイトレースは重いため。全点・最遠点は **AMCL の入力だけ** を変える。

## 2. 起動コマンド

`rfans_scan.launch.py` (スキャンを作る側) と `nav2.launch.py` の `amcl_scan` (AMCL が読む側) を **必ず対で** 指定する。bringup は `lidar_2d:=false lidar_3d:=true` (`/scan` が urg_node と衝突するため)。

### 2.1 最近点のみ (従来)

```
# ros2 launch rerobot_bringup rfans_scan.launch.py
# ros2 launch rerobot_bringup nav2.launch.py map_dir:=/workspace/maps/2d/glim/<name> use_keepout:=false
```

### 2.2 全点 (当日の標準、第12章)

```
# ros2 launch rerobot_bringup rfans_scan.launch.py allpoints:=true
# ros2 launch rerobot_bringup nav2.launch.py map_dir:=/workspace/maps/2d/glim/<name> use_keepout:=false amcl_scan:=all
```

### 2.3 最遠点のみ

```
# ros2 launch rerobot_bringup rfans_scan.launch.py farthest:=true
# ros2 launch rerobot_bringup nav2.launch.py map_dir:=/workspace/maps/2d/glim/<name> use_keepout:=false amcl_scan:=far
```

⚠️ 最遠点は bag 再生の比較で、見失わなくても平常時に約 1.2 m・6° ずれた。地図は植え込みも壁として描いているので、奥の点だけを見ると地図と合わない。比較実験用で、本番には使わない。

### 2.4 起動の確認

```
# ros2 topic hz /scan_all                  # 全点なら 約 10 Hz (最遠点は /scan_far)
# ros2 param get /amcl scan_topic          # /scan_all (全点) / /scan_far (最遠点) / scan (最近点)
# ros2 param get /amcl max_beams           # 2000 (全点) / 60 (最近点・最遠点)
```

### 2.5 bag に残す

選んだスキャンを記録トピックに足す (第5章のコマンドの末尾に追加)。入れ忘れると、AMCL が何を見ていたかを後から再現できない。`/scan_all` は 1 フレーム約 160 kB。

```
... /scan /scan_all /amcl_pose        # 全点の場合
```

## 3. 引数

### 3.1 `rfans_scan.launch.py` の launch 引数

| 引数 | 既定 | 意味 |
|---|---|---|
| `min_height` | `0.3` | 残す点の高さの下限 [m] (base_link 基準)。⚠️ 地図の高さ帯 (第9章の `--min_height`) と同じ値にする |
| `max_height` | `1.5` | 残す点の高さの上限 [m]。地図の帯と揃える条件は同じ |
| `range_max` | `30.0` | 採用する最大距離 [m]。⚠️ `nav2_params.yaml` の amcl `laser_max_range` と同じ値にする |
| `allpoints` | `false` | `true` で `/scan_all` (全点) も出す。`/scan` は出し続ける |
| `farthest` | `false` | `true` で `/scan_far` (最遠点) も出す。`/scan` は出し続ける |

`allpoints` と `farthest` は同時に `true` にしてよい (3 本とも出る)。帯と距離の 3 引数は 3 本のスキャンすべてに効く。

### 3.2 pointcloud_to_laserscan ノードのパラメータ (launch 内で固定)

`/scan` (`rfans_to_scan`) と `/scan_all` (`rfans_to_scan_all`) は ROS 標準の pointcloud_to_laserscan。

| パラメータ | `/scan` | `/scan_all` | 意味 |
|---|---|---|---|
| `target_frame` | `base_link` | 同 | 点を変換する先の座標系。高さ帯はこの座標系の z で切る |
| `transform_tolerance` | `0.1` | 同 | TF の時刻ずれの許容 [s] |
| `min_height` / `max_height` | 引数 | 引数 | 高さ帯 [m] |
| `angle_min` / `angle_max` | `−π` / `π` | 同 | スキャンの角度範囲 (全周) |
| `angle_increment` | `0.0035` (0.2°) | `0.00016` (0.009°) | **席 (方位ビン) の幅**。席数 = 2π ÷ この値 (1,796 / 39,270) |
| `scan_time` | `0.1` | 同 | 1 スキャンの時間 [s] (R-Fans 10 Hz) |
| `range_min` | `0.5` | 同 | これより近い点は捨てる [m] (車体・マストの映り込み除外) |
| `range_max` | 引数 | 引数 | これより遠い点は捨てる [m] |
| `use_inf` | `true` | 同 | 点が来なかった席を `inf` (測定なし) にする。`false` だと `range_max + inf_epsilon` が入り、AMCL が「そこに何かある」と誤解しうる |

その他 (`queue_size`、`concurrency_level`、`inf_epsilon`) は pointcloud_to_laserscan の既定のまま。

### 3.3 最遠点ノード (`rfans_scan_modes`) のパラメータ

`/scan_far` (`rfans_to_scan_far`) は自作ノード (`ros2_ws_main/src/app/rfans_scan_modes`)。pointcloud_to_laserscan は最近点しか選べないため作った。

| パラメータ | `/scan_far` の値 | 意味 |
|---|---|---|
| `mode` | `farthest` | 席の代表の選び方。`nearest` (最近点) / `farthest` (最遠点) / `nth` (近い方から `nth_k` 番目) |
| `nth_k` | (未使用) | `mode=nth` のとき何番目を採るか。0 始まりで 1 = 2 番目に近い点 |
| `cloud_topic` / `scan_topic` | `/rfans_driver/rfans_points` / `/scan_far` | 入出力 |
| `target_frame` | `base_link` | 同上 |
| `min_height` / `max_height` / `range_max` | 引数 | 同上 |
| `angle_increment` | `0.0035` | 同上 |
| `range_min` | `0.5` | 同上 |
| `use_laserids` | `[-1]` (全リング) | 使うレーザ番号 (0〜15) を限定する実験用 |
| `log_every_frames` | `100` | 何フレームごとに「席の埋まり率」をログに出すか |

### 3.4 `nav2.launch.py` の `amcl_scan`

| 値 | AMCL が読む | `max_beams` | 対になる rfans_scan の引数 |
|---|---|---|---|
| `default` (既定) | `/scan` | 60 (`nav2_params.yaml`) | なし |
| `all` | `/scan_all` | 2000 | `allpoints:=true` |
| `far` | `/scan_far` | 60 | `farthest:=true` |

`max_beams` は「1 スキャンのうち AMCL が使うビームの本数」。全席から席番号で等間隔に拾う。全点は席が 39,270 に増えるので、60 本のままだと約 660 席に 1 本しか見ず、空席 (`inf`) を引く率も上がる。そこで 2000 本 (19 席おき) にしている。19 はリング数 16 と公約数を持たないので、特定のレーザばかり拾う偏りも出ない。

## 4. 注意

- **地図の帯とスキャンの帯を揃える。** 3 方式とも帯は共通の `min_height` / `max_height`。地図 (第9章) を作り直したら両方を見直す。
- **選び方の効果は 1 回の走行では判定できない。** 植え込み通路の見失いは、同じ入力でも AMCL の起動条件しだいで出たり出なかったりする。bag 再生の比較では、全点と最近点に有意差はなかった。比較は `tools/amcl_compare/` の反復再生で行う (経緯: `docs/issue/2026-10-01_amcl_hedge_corridor_scan_reduction.md`)。
- **実走で比べるなら並走させる。** 本番を全点で走らせながら、`ros2 launch rerobot_bringup amcl_variants.launch.py only:=base` で最近点の AMCL を TF なしで裏に走らせると、同じ走行の上で比べられる (`/loc_base/amcl_pose`)。

---

← [第13章 トラブルシューティング](13_troubleshooting.md) | ↑ [目次](00_index.md)
