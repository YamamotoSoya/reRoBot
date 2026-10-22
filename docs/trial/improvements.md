# 実験走行の改善点メモ

実験走行で気づいた改善点を、走行ごとに追記していくファイル。
**本文はユーザが書く** (Claude は依頼があったときだけ編集する)。

- 新しい走行は上に追加する (新しいものが先頭)
- 対応が済んだら、状態を `[ ]` から `[x]` にし、どう直したか (コミット / docs/issue へのリンク) を書く
---
## LT大会
### 改善点
- [ ] すれちがいロボットの回避（追いつき、追いつかれ、すれ違い<<最難関）
車道ロードコーン右は一次的に避けるのは可能
もしくは避けて止まる
- [ ] どこまでが経路が塞がっているか、どこまでが通れるかの判断を明快に
- [ ] 人間は人間として推定。進行方向を推定し進行方向側にコストを伸ばす。ロボットは認識できない。
- [ ] waypointに障害物が重なっている可能性を考慮
- [ ] 自前mapと他チーム製mapどっちがマッチングしやすい？
- [ ] 衝突時のバック回避

## 2026-10-21 つくば / 全周bag取得

- bag:
- 構成:
- TODO1:0.3-100, r100, traj (composite_g1_imu_strong)の自律移動
- 

### 改善点（すべき）

- [ ] UTMは足元の草を見て経路が塞がれる。さらにglobal plannerには含まれていないせいで回避路を計画できずにスタック
- [x] 対策としてUTM無効で踏破
- [ ] 飛び出た草木による通路の遮断が発生
- [ ] やむを得ず、または何かしらが原因で危険域が衝突すると前後左右障害物認定によりスタック
- [ ] 安全管理責任者を右回左回しようとnavigation暴れる。
- [ ] global plannerの生成周期が遅く、生成された経路間で右往左往
- [ ] 常に真ん中を取ろうと凸凹な場所で凸凹に進む
- [ ] 安全管理責任者依田の左右に謎の物体。安全ベストの反射板が原因か
- [ ] plannnerの重さはgoal距離依存ではなくmap広さ依存
- [ ] mapを区画ごとに使い分ける、または範囲内のみ経路計算する構成が必要
- [ ] PCバッテリーが減りpowersavermodeへ。そしてeposエラー
- [ ] パーティクルがみられるように
- [ ] navigationがかくかくスムージング導入
- [x] 過去のwaypointへ戻る挙動 (後述)
> 追従 BT の RemovePassedGoals + ComputePathThroughPoses を ReactiveSequence で包み、通過判定を毎 tick 回すようにした (ff8b606)

### memo
- 過去のwaypointへ戻る挙動
10-16版で規定BTから経路がふさがった際に前の車両を追従する自作BTに切り替えた際、RemovePassedGoals（通過済みの waypoint を消すノード）を普通の Sequence に入れていた。
そのため、計画に時間がかかるつくば地図では通過判定が約 11 秒に 1 回しか回らず、waypoint を通過したことに気づかずに引き返していた。
そこでRemovePassedGoalsとComputePathThroughPoses (経路計算)をReactiveSequence ()に戻した。
これによりRemovePassedGoalsが毎tick回るため、waypointを通過したときに通り過ごすことがない。
しかし根本的な経路計画の重さはmapの大きさにあるので、今回はmapを市役所周りでカットし実行

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

- [x]   若干斜めってるからLaserを低い高さで区切ると市役所の壁映らない。
> 0.3~100mのレーザを圧縮することで高い高さの点群も取り入れる。
- [x]   建物もあるが。植木や木が建物の壁より手前にあるため、スタート地点近くは自己位置推定しにくい。
> 同上
- [x]   スタート前、コントローラ->自律移動の切り替えがスムーズにいくように（cmd_velの干渉）
> /joy_vel(joyコン),/cmd_vel(自律移動)をtwist_muxでまとめて対処
- [ ]   起動の簡素化
> 確認項目の見にくさから作成後使用せず
- [x]   途中復帰も含めてwaypointの計画
> 途中waypointからの実行も可
> waypoint_runner に `--start N` (waypointN から開始) と失敗時の自動再送 (`--auto-retry`、既定 10 回) を入れた
- [ ]   非常停止後の再起動不要へ
- [ ]   usbぶっこ抜いても止まるように
- [ ]   林原研の話、R-Fans16を用いている班はmap生成の段階ですでに2d。3dを挟まない。200mのうち20mを使用。localizationはslamtoolboxで10m。最初建物の近くでmattingしてからスタートへ。
- [ ]   建物の近くではマッチングする。keepoutのズレをにより草地走行
- [ ]   障害物検知は既知の問題
- [ ]   狭い場所では完全に膨張のいたばさみ。
- [x]   狭い空間でロボットが待つという動作の導入
- [x]   /scanallの常時表示
> nav2.rviz に `/rfans/scan_all` の表示を既定 ON で追加した (10-16、bringup 側は `rfans_scan_all:=true` が必要)
- [ ]   bagの起動コマンドが長い
- [x]   waypoint editorの位置ずれ
- [x]   ギヤの段付きシャフト非固定によるギヤの遊びからの騒音
- [x]   キャスタ固定具の再設計
- [x]   タイヤカバー、外装作成
- [ ]   ThinkPad chrome remote desktop GNOME問題
- [ ]   amclように別途価値mapを作成。地形によってamclのマッチング強さをコントロール
- [x]   emcl2の導入
> emcl2_ros2 を submodule で取り込み、nav 構成 `nav_emcl2.launch.py` で AMCL と差し替えて起動できるようにした (10-08)
- [ ]   amclは停止時に推定はしらない
```
Chrome Remote Desktop (CRD) のLinuxホストは、通常の画面とは別に、同じユーザーで仮想デスクトップセッションを起動します。すると同じユーザーのGNOMEセッションが2つ同時に動き、それぞれの gnome-settings-daemon が夜間モード、電源プロファイル、明るさ、色管理を管理しようとして、設定を奪い合う状態になりえます。「変えても3秒で戻る」症状とよく合います。
```
- [ ]   

### メモ
今日使ったコマンド
```
ros2 bag record -s mcap -o /workspace/bags/raw/$(TZ=Asia/Tokyo date +%F_%H%M)_tukuba_yakusho /rfans_driver/rfans_points /rfans_driver/rfans_packets /scan /imu/data /imu_wit/data /imu_wit/mag /odom /tf /tf_static /diagnostics /robot_speed_cmd

B=2026-10-04_1056_tukuba_yakusho

maps/2d/glim/2026-10-03_1126_tsukuba_dumpbase

python3 /workspace/tools/waypoint_editor/waypoint_editor.py \
  /workspace/maps/2d/glim/2026-10-03_1126_tsukuba_dumpbase/nav2/map.yaml /workspace/maps/2d/glim/2026-10-03_1126_tsukuba_dumpbase/waypoints/course.yaml --check
  
  
  # 1) bringup (3D LiDAR + IMU + EKF)
ros2 launch rerobot_bringup rerobot_bringup.launch.py lidar_2d:=false lidar_3d:=true imu:=true ekf:=true
# 2) R-Fans 点群 → /scan (costmap 用) + /scan_all (AMCL 用、全点)
ros2 launch rerobot_bringup rfans_scan.launch.py allpoints:=true

  
  ros2 launch rerobot_bringup nav2.launch.py map_dir:=/workspace/maps/2d/glim/2026-10-03_1126_tsukuba_dumpbase/waypoint/course.yaml
  
  ros2 launch rerobot_bringup nav2.launch.py \
  map_yaml:=maps/2d/glim/2026-10-03_1126_tsukuba_dumpbase/
  loc_map_yaml:=maps/2d/glim/2026-10-03_1126_tsukuba_dumpbase/loc_0.3-1.5/map.yaml \
  amcl_scan:=all
  
  ros2 launch rerobot_bringup nav2.launch.py \
  map_dir:=/workspace/maps/2d/glim/2026-10-03_1126_tsukuba_dumpbase \
  loc_map_yaml:=/workspace/maps/2d/glim/2026-10-03_1126_tsukuba_dumpbase/loc_0.3-1.5/map.yaml \
  amcl_scan:=all
```

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
- [x] 坂道が多く経路計画にそのままscan_allをぶち込むと見えない壁の発生する可能性が高い
> スキャンを costmap 用 `/rfans/scan` (帯 0.3〜1.5 m) と AMCL 専用 `/rfans/scan_all` に分け、scan_all は経路計画に入れないようにした (10-04・10-08)
- [ ] 相対高さで区切っているが機体が坂道で斜めのときはどうなる？
- [ ] 結構道が細い場所がある
- [ ] 白線で停止、横断歩道前で停止の位置制御がシビアそう
- [x] 横断歩道前でロボットが待っている場合、停止もしくは減速処理を積まないと永遠と経路計画を繰り返してぐるぐるしそう
> waypoint_runner の停止点 (`stop: true`) で Nav2 のゴールを終えて待機するので、待っている間は経路計画が走らない (10-04)
- [x] 普通に横断歩道での一時停止処理を入れないと話にならない
> waypoint_runner の停止点 (`stop: true`) で止まり、パッドの Start で再開する形で実装した (10-04、パッド対応は 10-16)
- [x] 30m 0.4~1.5mとかの点群だと話にならないかも。結構広いから誤マッチしそう。逆に遠くの建物などははっきりしているから範囲広げると安定しそう。
> AMCL 用スキャン・地図の帯とレンジを広げた (0.3〜100 m、r100) — 10-04 の「市役所の壁」と同じ対処
- [x] 高さで区切るのは床や天井がmapに入って経路計画できなくなるから。それならそもそも自己位置推定と経路計画, ローカルコストマップ用のmapを分けて、経路計画は床や壁のないわかりやすい地図、自己位置推定は全高さで圧縮してエッジ成分だけ取り出したもの(点群の密度が濃いところ)もしくは床も天井も除去せずグレースケール（実際には二値でもいい）っぽい画像をすり合わせる感じでマッチングするのは？amclがそれに対応してるかも知りたい。むしろそういう前例ないのが不思議だが
> 自己位置推定専用の地図 `/map_loc` を経路計画用 `/map` と分け、`nav_amcl_scanall.launch.py` の `loc_map_yaml` で AMCL だけに読ませるようにした (10-04)
- [ ] Parsec導入
- [ ] ロボットの側面にメジャーを取り付け、カメラで記録し、waypoint微調整
- [x] bag記録時に停止位置を記録(ティーチング)
> joy のボタンで `/teach_marker` を bag に打ち込む teach_marker.py を joy_teleop に同梱し、tools/teach_from_traj で waypoint に変換できるようにした (10-09〜10-11)
- [x] 一時停止後に自律移動再開できるように
> waypoint_runner でパッドの Back = 一時停止、Start = 再開とした (10-16)
- [ ] R-Fansの利用可能な上限180mも自己位置推定とmap生成に含める

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

ans用いてるチームはmap生成の時点で2d、20mくらい。自己位置推定はslamtoolboxで10mくらい。一度建物の近くでmattingしてからスタートへ。