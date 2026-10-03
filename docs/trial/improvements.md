# 実験走行の改善点メモ

実験走行で気づいた改善点を、走行ごとに追記していくファイル。
**本文はユーザが書く** (Claude は依頼があったときだけ編集する)。

- 新しい走行は上に追加する (新しいものが先頭)
- 対応が済んだら、状態を `[ ]` から `[x]` にし、どう直したか (コミット / docs/issue へのリンク) を書く
---
## 2026-10-04 つくば / 全周bag取得

- bag:
- 構成: 
- TODO1: ローカルbag(ループとじ)の取得
- TODO2: 自律移動実験 (navigation, amclでmapわけ運用　amclのmapの高さを分けて実験)
  - **使い方メモ (Claude 追記 2026-10-04)** — 地図生成は glim コンテナ、起動は main コンテナ
    - 共通: `D=/workspace/bags/glim/<bag>_dump/k20` (GLIM dump)、`M=/workspace/maps/2d/glim/<地図名>`、
      `Z` = base_link→rfans 高さ (**〜09-30 の bag は 0.80246、10-01 以降は 0.79396**)
    - ① 経路計画用 + keepout (1 コマンドで `raw/` `nav2/` `keep_out/` が出る。帯 0.4〜1.7、しきい値 1/3)
      `python3 /workspace/tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py $D $M -r 0.10 --height_mode base_link --base_to_sensor_z $Z --min_height 0.4 --max_height 1.7 --range_max 30 --min_points_in_pix 1 --max_points_in_pix 3`
      → 必要なら `nav2/map.pgm` を GIMP で手直し
    - ② AMCL 用 (高さ帯ごとに 1 枚。`--map_only`、しきい値は厳しめの 2/5 = 植え込みの中を塗らない)
      `python3 /workspace/tools/glim_dump_to_2dmap/glim_dump_to_2dmap.py $D $M/loc_0.3-H -r 0.10 --height_mode base_link --base_to_sensor_z $Z --min_height 0.3 --max_height H --range_max 30 --min_points_in_pix 2 --max_points_in_pix 5 --map_only`
      (H = 1.5 / 3.0 / 5.0。5goukan_dumpbase には 3 枚とも作成済み)
    - ③ 起動 (**H は ② の地図と必ず同じ値**。ずれると地図に無い壁を見て推定が暴れる)
      `ros2 launch rerobot_bringup rfans_scan.launch.py allpoints:=true amcl_min_height:=0.3 amcl_max_height:=H`
      `ros2 launch rerobot_bringup nav2.launch.py map_dir:=$M loc_map_yaml:=$M/loc_0.3-H/map.yaml amcl_scan:=all`
      → /scan (costmap 用) は 0.3〜1.5 のまま、/scan_all (AMCL 用) だけ H まで。Nav2 は /map、AMCL は /map_loc を読む
    - 試す順: H = 1.5 と 3.0 (2039 bag 再生では両方とも現行より安定、5.0 は悪化)。差が出るのは屋根付き区間のはず
- TODO3: 自律移動実験 (新しいwaypoint機能の試験)

### 改善点

- [ ]

---

## 2026-10-03 つくば / 全周bag取得

- bag:
- 構成: (bringup 引数・地図・パラメータの変更点など)
- TODO: 全周bagの取得

### 改善点

- [ ] R-Fansの回転ディップによるデータ欠損
- [ ] ロボットの角の突起。日光による熱対策パソコンあちち
- [ ] ホイールに挟み込み防止カバー
- [ ] joy_teleopeの加減速はもうちょいゆっくりでいいかも
- [ ] 足元につけたUTM-LX30をローカルコストマップにそのままぶち込んで障害物認識するつもりだったが、スタートとゴール地点にレンガの隙間から生えた20cmぐらいの葉がわさわさしていて誤認識しそう
- [ ] lidarの少し上ぐらいまで伸びて普通にひとに当たるぐらいの高さのススギがわさわさしている。
- [ ] 坂道が多く経路計画にそのままscan_allをぶち込むと見えない壁の発生する可能性が高い
- [ ] 相対高さで区切っているが機体が坂道で斜めのときはどうなる？
- [ ] 結構道が細い場所がある
- [ ] 白線で停止、横断歩道前で停止の位置制御がシビアそう
- [ ] 横断歩道前でロボットが待っている場合、停止もしくは減速処理を積まないと永遠と経路計画を繰り返してぐるぐるしそう
- [ ] 普通に横断歩道での一時停止処理を入れないと話にならない
- [ ] 30m 0.4~1.5mとかの点群だと話にならないかも。結構広いから誤マッチしそう。逆に遠くの建物などははっきりしているから範囲広げると安定しそう。
- [ ] 高さで区切るのは床や天井がmapに入って経路計画できなくなるから。それならそもそも自己位置推定と経路計画, ローカルコストマップ用のmapを分けて、経路計画は床や壁のないわかりやすい地図、自己位置推定は全高さで圧縮してエッジ成分だけ取り出したもの(点群の密度が濃いところ)もしくは床も天井も除去せずグレースケール（実際には二値でもいい）っぽい画像をすり合わせる感じでマッチングするのは？amclがそれに対応してるかも知りたい。むしろそういう前例ないのが不思議だが
- [ ] Parsec導入
- [ ] 

### メモ

#### 自己位置推定・コストマップ改修の計画 (2026-10-03 Claude と整理)

**目標の構成**

```
R-Fans 点群 (/rfans_driver/rfans_points)
├─ [自己位置推定] AMCL (likelihood_field)
│   ├─ 入力: /scan_all (全点版)、帯は 0.3〜3 m 程度 ← /scan とは別の引数にする
│   └─ 地図: AMCL 専用 /map_loc (同じ帯で作る。屋根・軒・建物の上部を含む)
│        → 屋根の下でも壁の中でも推定に影響なし (likelihood_field は自分の位置を評価しない)
├─ [経路計画] global_costmap
│   ├─ static_layer: 経路計画用 /map (今の 0.3〜1.5 m 帯 + keepout)
│   └─ obstacle_layer: /scan、書き込み距離を数 m に絞る (遠方の地面ゴーストを溜めない)
└─ [障害物回避] local_costmap (地図は使わない。今も static_layer なし)
    └─ VoxelLayer ← 点群を直接入力、高さ 0.25〜2.0 m
         ├─ 20 cm の草・屋根を高さで除外できる (点の z が残るため)
         └─ 3D のレイで消すので、死角に入った低い障害物を忘れにくい

(将来) 前段に地面除去ノード (法線 / セル内の高さの広がり)
        → VoxelLayer 入力・AMCL 用の縦構造スキャン・地図の 3 つで共有 → 坂の壁への根本対策
```

**フェーズ**

| 段階 | 内容 | 解決するもの |
|---|---|---|
| P0 すぐ | ① global obstacle_layer の `obstacle_max_range` を 25 m → 数 m に ② `rfans_scan.launch.py` の帯の引数を `/scan` 用と `/scan_all` 用に分ける | global に溜まる地面ゴースト・帯の変更が costmap に漏れる問題 |
| P1 AMCL | ① map_server を 2 つにして AMCL の `map_topic` を `/map_loc` に ② 地図と /scan_all を同じ帯で作り、上限 1.5 / 3 / 5 m を比較 | 広い場所での誤マッチ・屋根付き区間・遠方建物で yaw を安定させる |
| P2 local | obstacle_layer → VoxelLayer (PointCloud2) | 足元の死角での衝突・草・屋根 |
| P3 地面除去 | 法線 / 高さの広がりで地面を除くノードを P1・P2 の入力に共有 | 坂の壁 (相対高さの限界)・遠方の地面ゴースト |
| P4 並行・本命候補 | GLIM 3D 地図に対する 3D 自己位置推定 (NDT/GICP)。map→odom TF を出して AMCL と差し替え | 坂・ピッチ・帯の問題を 6 自由度でまとめて解消 |

- 本線は P0 → P1。P4 は今日の全周 bag の再生で並行して試作する。P4 が間に合えば P1・P3 の AMCL 側は不要 (P3 は costmap 用に残る)
- 保留: 足元の UTM-30LX (走査面 9.4 cm が 20 cm の草を横切り、高さで弾けない。P2 で代替)、濃淡画像の相関センサモデル (原理的には成立・前例あり〈Levinson 2007 等〉だが AMCL では自作が必要)

**評価方法**

- AMCL は同じ bag の反復再生 + 総当たり照合で比較する (`tools/amcl_compare`)。単発の再生では判定しない
- 地図と /scan_all の帯は必ず同じ値にする
- 確認する区間: 屋根付き区間・植え込み通路 (1948 で見失った場所)・坂・十字路

**未検証の前提 (仮説)**

- LaserScan のレイトレースが死角の障害物を消す → 低い箱に近づく bag で costmap を記録して確認
- 坂が壁に見え始める距離 = 0.3 / tan α (5° で 3.4 m、10° で 1.7 m) は計算値。ピッチ・地形の起伏は含まない
- 帯の上限を上げると AMCL が安定する → P1 ② で確認
- 屋根の下でも AMCL は問題ない → 屋根付き区間の再生で確認

**改善点との対応**: 30 m 帯の誤マッチ・地図分離 → P1 / 坂の見えない壁 → P0① (応急)・P3・P4 (根本) / 相対高さと機体の傾き → P3・P4 / 足元 UTM と草 → 保留 (P2) / ススキ → P2 (2 m までは避ける対象として正しく扱う)

