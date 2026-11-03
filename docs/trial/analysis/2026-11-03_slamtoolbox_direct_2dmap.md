<!-- claude: 2026-11-03 作成 (ユーザ依頼: 「3D を噛まさず slam_toolbox で 2D 地図を作ってみて」— 壁の帯の太さと AMCL の台地が変わるか) -->
# 11-01 2114 bag: slam_toolbox で直接作った 2D 地図 vs GLIM 経由の 2D 地図 (壁の厚みと尤度の台地)

<!-- claude: 2026-11-04 §7 追加 (scan_all 版: 大域整合は直るが植え込みが消える) -->

**結論: slam_toolbox で `/rfans/scan` (高さ帯 0.3〜2.0 m) から直接作った地図は壁が 1〜2 セル (0.05〜0.10 m) に薄くなり、AMCL 尤度場の「台地」の幅は横 0.48〜0.92 m → 0.20〜0.32 m、yaw 5.6〜9.6° → 3.2〜6.0° と半分以下になる。
ただし地図全体の形は数 m 歪む (AMCL 当てはめ軌跡との剛体変換後の残差 中央値 4.0 m、最大 12.9 m、30 s 区間の移動量比 p10 0.88) ので、このまま本番地図に使うのは無理。
「薄い壁」の利点だけを取るには、GLIM の軌跡 (全体整合) + 2D スキャン端点のレイキャスト (薄い壁) で地図を描き直すのが次の候補。**

台地の解説と図は `bags/exp/2026-11-01_loc_res/landscape/plateau_fig.py` (同日作成: `plateau_synth.png` 模式図、`plateau_real.png` 実データ、`plateau_mapzoom.png` 壁周り拡大)。

## 1. 対象

- bag: `bags/raw/2026-11-01_2114_5goukan_chal_auto` (1420 s、456 m、`/rfans/scan` 7883 msg ≈ 5.6 Hz、`/rfans/scan_all` 7928 msg)。実走は `nav_amcl_scanall` + GLIM 地図 `maps/2d/glim/2026-10-09_1643_5goukan_chal_fixed2_filtered/raw_traj/0.3-100.0_r180/map.yaml` (0.05 m、高さ帯 0.3〜100 m)。
- slam_toolbox 地図: `maps/2d/slam_toolbox/2026-11-03_bag1101_rfans/` (`map.pgm` 3059×3789 @ 0.05 m、`posegraph.*`、`slam_params.yaml`、`slam.log`)。
- 成果物: `bags/exp/2026-11-03_slamtoolbox_2d/` (`run_slam.sh` 再生 harness、`save_map.py`、`compare.py`、`compare_zoom.png`、`compare_overview.png`、`compare.json`、`slam_out/` = slam の /pose と /tf)。

## 2. 方法

- slam_toolbox (Jazzy 2.8.5、async、mapping) を slamtoolbox コンテナで `use_sim_time` + `scan_topic: /rfans/scan` で起動 (他は `rerobot_slamtoolbox/config/slam_toolbox.yaml` と同じ。`map_update_interval` だけ 2 → 10 s)。bag は main コンテナから 1 倍速で再生 (`/tf` は `tf_filter.py` で AMCL の map→odom を除去、`/tf_static` と `/rfans/scan` のみ)。ROS_DOMAIN_ID=151 で live から隔離。処理落ちは起動直後の 1 msg のみ (slam.log)。
- 地図保存: slamtoolbox コンテナに nav2_map_server が無いため、main 側から `/slam_toolbox/dynamic_map` を呼んで pgm/yaml を書く (`save_map.py`)。
- 軌跡の位置合わせ: slam の map→odom × bag の odom→base_link で slam 地図上の姿勢を復元し、AMCL 当てはめ軌跡 (`wob_live/rows.jsonl` の lag0、1 Hz) と剛体変換 (Umeyama) で重ねる。残差 = slam と AMCL の食い違い (どちらが正しいかはこれだけでは決まらないが、GLIM 側は 11-02 で横 0.15 m と検証済み)。
- 厚み: (a) 軌跡 15 m 以内の占有セルについて「縁からの深さ」(距離変換) の分布、(b) scan_all の各光線が最初の占有セルから次の非占有セルまで貫く長さ (= 台地に直結する「帯の厚み」)。
- 台地: landscape.py と同じ尤度 (Σpz³、σ_hit 0.2、2000 本、30 m) を横 ±1 m × yaw ±10° で計算し、L/Lmax ≥ 0.9 の幅。GLIM 地図は AMCL 当てはめ姿勢、slam 地図は slam 姿勢を中心にする。
- ⚠ 再生の落とし穴 (2026-11-03 実測): `docker exec ... bash -c "nohup cmd &"` の非対話シェルではバックグラウンド子が SIGINT を無視するので、停止は SIGTERM。lifecycle ノードは起動から約 10 s 経たないと `ros2 lifecycle` から見えない。

## 3. 結果

### 3.1 壁の厚み

| 指標 (軌跡 15 m 以内) | GLIM 0.05 m (帯 0.3〜100 m) | slam_toolbox 0.05 m (帯 0.3〜2.0 m) |
|---|---|---|
| 占有面積 | 2695 m² | 295 m² |
| 占有セルの厚み推定 中央値 | 0.17 m | **0.05 m (1 セル)** |
| 縁から 4 セル (0.2 m) 以上深いセル | 30 % | **0 %** |
| 光線が帯を貫く長さ 中央値 (T=430/600/950/1150) | 0.48 / 0.68 / 0.15 / 0.35 m | **0.15 / 0.13 / 0.15 / 0.18 m** |
| 同 p90 | 3.3 / 1.7 / 1.2 / 1.7 m | **0.28 / 0.30 / 0.38 / 0.45 m** |

GLIM の p90 が 1〜3 m なのは、建物の外壁が内側に向かって塗り潰されている (高さ帯 0.3〜100 m に建物の床・庇・内部が全部入る) ため。slam 地図は壁の線の奥が未知 (灰) で、帯の厚みは光線方向で 1〜3 セル。

### 3.2 台地の幅 (L/Lmax ≥ 0.9)

| T [s] | GLIM × scan_all (live 構成) 横 / yaw | slam × scan 横 / yaw | slam × scan_all 横 / yaw |
|---|---|---|---|
| 430 | 0.54 m / 7.2° | **0.22 m / 3.2°** | 0.28 m / 5.6° |
| 600 | 0.72 m / 5.6° | **0.26 m / 6.0°** | 0.46 m / 5.2° |
| 950 | 0.48 m / 9.6° | **0.20 m / 4.0°** | 0.20 m / 4.4° |
| 1150 | 0.92 m / 6.4° | **0.32 m / 3.2°** | 0.34 m / 4.0° |

横は 2〜3 分の 1、yaw は 1.5〜2 分の 1 に縮む。GLIM × scan (帯 0.3〜2 m のスキャンを 0.3〜100 m の地図に当てる) は 0.34〜1.62 m でむしろ広く、帯の不一致が台地を広げる方向に効く。

### 3.3 地図全体の整合 (slam 地図の弱点)

- AMCL 当てはめ軌跡との剛体変換後の残差: 中央値 3.98 m、p90 11.6 m、最大 12.9 m。残差は T < 300 s (北側の直線 150 m) で最大 13 m、その後 4 m 前後で安定 (`compare_overview.png`)。
- 30 s 区間の移動量比 slam/AMCL: 中央値 0.99、p10 0.88 → 一部区間で slam 側が 12 % 短い。向きの差は中央値 1.5°。
- 候補 (未確定): 建物沿いの長い直線はスキャンが壁 1 枚しか見ず進行方向が決まらないため、slam_toolbox がオドメトリ寄りに積算し、loop closure でも戻しきれていない。GLIM は 3D 構造で進行方向が決まる。
- スキャン端点 (0.3〜2 m 帯) の一致率 (d < 0.1 m): GLIM 94〜100 %、slam 65〜91 %。slam 地図は植え込みなど一部の地上構造を落としている (動的物体として消されたか、端点が揺れて占有にならなかったか、未確認)。拘束が減る方向なので、AMCL 再生で横ずれが本当に減るかは別途確認が要る。

### 3.4 拡大図で見えたこと (`compare_zoom.png` 左 2 列)

- T=430: GLIM では + の下側 2 m が全部占有 (建物内部まで塗り潰し)、slam では + の位置に 1〜2 セルの線、その奥は未知。帯の「手前面」の位置は両者で一致。
- T=430 の scan_all 最寄り点 (1.1 m 先) は GLIM で占有・slam で空き (最寄り占有まで 1.45 m) だった。scan (0.3〜2 m 帯) の最寄り点も同じ場所で、両地図ともスキャン全体には合っている (argmax ≈ 0) → この点は地図にない物体 (操作中のユーザなど) の可能性。拡大図の中心は「両地図で占有の最寄り終点」に変えてある。
- T=600 / 1150: GLIM で大きな黒い塊 (植え込み) になっている領域が slam ではほぼ空。

## 4. 解釈

1. **帯の太さの主因は高さ帯 0.3〜100 m そのもの** が有力。帯を 0.3〜2.0 m に絞った slam 地図では壁が 1 セルになり、09-23 の submap 傾き (H·sin δ) やゴースト点を持ち出さなくても厚みの大半が説明できる。ただし slam 地図は mapping 方式も違う (端点 + レイ消去で占有を決める) ので、「帯を絞っただけの GLIM 地図」が同じく薄くなるかは未確認。
2. 台地の幅は帯の厚みに従って縮んだ (§3.2)。11-02 の「0.025 m 地図は効かない」と合わせて、**台地を潰すレバーは分解能ではなく帯の厚み**。
3. slam 地図は局所的には理想的だが大域的に歪む。本番地図にするなら GLIM の軌跡を使って描き直す方が筋がよい。

## 5. 次の一手

- **GLIM 軌跡 + 2D スキャンのレイキャスト地図**: 地図作成 bag (10-09 1643) の GLIM 軌跡 (または 11-01 bag の AMCL 当てはめ軌跡) に `/rfans/scan` 端点を置き、端点 1 セル占有 + 光線上を空きにする log-odds 地図を Python で描く。全体整合は GLIM、壁は 1 セル。できた地図を `loc_map_yaml` に渡し、11-01 bag の AMCL 再生 (`bags/exp/2026-11-01_loc_res/replay_amcl.sh`) で横ずれ・yaw・σxy を現行と A/B する。
- その前に安価な確認: `pointcloud_to_2dmap --min_height 0.3 --max_height 2.0` で GLIM 点群から帯を絞った地図を作り、厚み指標 (§3.1) が slam 地図並みになるか (帯だけで説明できるか) を見る。
- slam 地図を直接使う案は、整合性 (残差 4〜13 m) が解決しない限り保留。

## 7. 追加 (11-04): `/rfans/scan_all` (帯 0.3〜100 m の最近点) で mapping した版

ユーザの「全点で slam_toolbox できる?」を受けて、scan_topic を `/rfans/scan_all` にして同じ bag を再生 (`maps/2d/slam_toolbox/2026-11-04_bag1101_rfans_scanall/`、`RUNREL=... SCAN=/rfans/scan_all OUTNAME=slam_out_scanall run_slam.sh`、比較は `TAG=_scanall`)。角度ビン内の全点 (同方向に複数距離) は slam_toolbox の LaserScan 入力では不可で、やるなら cartographer (PointCloud2 2D) かレイキャスト地図。

| 指標 | scan 版 (帯 0.3〜2 m) | **scan_all 版 (帯 0.3〜100 m)** |
|---|---|---|
| AMCL 軌跡との剛体残差 中央値 / p90 / 最大 | 3.98 / 11.6 / 12.9 m | **0.99 / 1.85 / 3.45 m** |
| 30 s 区間の移動量比 p10 / 向き差 | 0.88 / 1.5° | **0.98 / 0.7°** |
| 壁の厚み 中央値 / 深さ ≥ 4 セル | 0.05 m / 0 % | 0.05 m / 0 % |
| 台地幅 (slam × scan、横 / yaw、T=430/600/950/1150) | 0.22/0.26/0.20/0.32 m、3.2/6.0/4.0/3.2° | 0.16/0.24/0.20/0.22 m、3.2/6.8/4.0/3.6° |
| scan_all 端点の一致率 (d<0.1 m) GLIM / slam | 0.91〜0.98 / 0.26〜0.84 | 0.91〜0.98 / **0.25〜0.89 (600 s で 0.25、430 s で 0.26)** |
| 処理したスキャン数 (/pose) | 1587 | **445** (async が追いつかず間引き、終盤に TF 外挿エラー 3 件) |

- **大域整合は scan_all で解決**。長い直線の歪みは木や上階の構造を見ることで消えた (候補どおり、2D 帯だけでは進行方向が決まらなかった)。
- **植え込みは消える**。T=600 の拡大 (4 m × 4 m) は slam_scanall 地図では完全に空で、台地も壊れている (argmax が横 −0.5 m / yaw −4.5° の尾根)。端点が毎回ずれる植え込みは Karto の hit/pass 比で空きに倒れる。GLIM 2D 化 (蓄積点数) は塊として残す。植え込み通路 (`amcl-hedge-corridor-degeneracy`) ではこの地図だと AMCL が拘束を失う。
- 帰結: 「最近点 + 通過消去」は壁を薄くしゴーストを落とすが、植え込みも落とす。本番向けの地図は **GLIM 軌跡 + 最近点 + 蓄積ヒット数ベース (通過消去は弱く)** の折衷が候補。§5 のレイキャスト地図実験では閾値を「ヒット数」側に寄せた版も必ず作ること。
- scan_all を slam_toolbox で実時間処理するなら、ビーム間引きか再生 0.5 倍速が要る。

## 6. 再現

```bash
# ホスト (slamtoolbox コンテナを profile で起動してから)
docker compose --profile slamtoolbox up -d slamtoolbox
bags/exp/2026-11-03_slamtoolbox_2d/run_slam.sh            # 約 25 分。DUR=60 で煙試験
# main コンテナ内
python3 /workspace/bags/exp/2026-11-03_slamtoolbox_2d/compare.py          # 既定 T=430,600,950,1150
python3 /workspace/bags/exp/2026-11-01_loc_res/landscape/plateau_fig.py <bag> <rows.jsonl> 430,950 0.05=<map.yaml>,0.025=<map_r0025.yaml>
```
