<!-- claude: docs/issue — 未解決問題の調査記録。解決したらステータスを更新すること。-->
# rfans_driver (R-Fans-16) 計算式レビュー — 主要経路は正常、堅牢性・副次フィールドに問題

- **ステータス: 未解決**(静的レビューで検出した問題は未修正。x/y/z の主要経路に実害はなく運用は可能)
- 日付: 2026-07-07
- 環境: 静的コードレビュー(Windows ホスト、実機・ROS 2 実行なし)。対象は R-Fans-16 実機構成(`rerobot_bringup_3d.launch.py` + `params_3d.yaml`)
- 対象ブランチ: `main` (HEAD: `e5695fb feat: implement 2D and 3D LiDAR configurations; add launch files and parameters for epos4_controller and epos4_odometry`)
- 関連ファイル:
  - `src/external/StarROS2/src/rfans_driver.cpp`(LED 本数判定 / PonintFrequency 推定 / col 計算 / PointCloud2 生成)
  - `src/external/StarROS2/src/rfans_driver.h`(フィルタ・remap メンバ)
  - `src/external/StarROS2/include/rfans_driver/point_types.h`(`RFANS_XYZ_S` — PointCloud2 レイアウトの照合元)
  - `src/external/StarROS2/include/star/Shot.h`(SDK 構造体 `LasShot_S` / `SHOTS_CALCOUT_S`)
  - `src/external/StarROS2/lib/libstar.so`(XYZ 変換本体 `calcXYZ`/`calcTheta` — ソース非公開で静的検証不能)
  - `src/rerobot_bringup/config/params_3d.yaml`(`rfans_driver` セクション)
- 関連 issue/report:
  - `docs/report/2026-06-13_rfans_driver_dataid_0x37_z0_2d.md`(dataID 0x37 remap の実機確認)
  - `docs/report/2026-06-13_rfans_driver_low_fps_per_point_spin.md`

## TL;DR

rfans_driver の計算式を静的レビューした結果、**SLAM/Nav2 が消費する主要経路(x/y/z/intensity と PointCloud2 バイトレイアウト)は正しい**。PointCloud2 フィールドオフセットは `RFANS_XYZ_S`(pack(1), point_step=41)とバイト単位で一致、col 式は次元的に正しく実測値(15008 点/スキャン → 938 列 → 0.384°/列)とも整合、dataID 0x37 の LED 判定(remap 前の生 dataID)と 0x37→0x57 remap(calcXYZ 直前のコピーのみ)の適用順序も正しい。一方で未修正の問題が残る: (1) 起動直後に 21 点未満のバッチが来ると `packets.points.at(20)` が `std::out_of_range` で**ノード即死**、(2) `PonintFrequency` 推定にゼロ除算リスク + `utcTime` の単位不明(文字化けコメント、ms なら col が 1000 倍ズレ)、(3) `pulseWidth` フィールドは一度も代入されず常に 0、(4) フィールド名 "rol"(`col` の typo と思われる)。なお XYZ 変換の三角関数本体は closed SDK(`libstar.so`)内で検証不能(縦角表は 6/13 実機確認済み)。

## 検証して「正しい」と確認できたもの

### 1. PointCloud2 フィールドレイアウト vs `RFANS_XYZ_S`(バイト単位で照合)

`InitPointcloud2`(`rfans_driver.cpp:508`)のオフセット生成は offset 代入が increment の後という紛らわしい書き方だが、`point_types.h` の pack(1) 構造体と完全一致:

| field | offset | datatype | 構造体側 |
|-------|--------|----------|----------|
| x / y / z | 0 / 4 / 8 | FLOAT32 | `float x,y,z` |
| intensity | 12 | FLOAT32 | `float intent`(← `m_uInt` uint16) |
| laserid | 16 | INT32 | `int laserid` |
| timeflag | 20 | FLOAT32 | `float timeflag` |
| hangle | 24 | FLOAT32 | `float hangle` |
| pulseWidth | 28 | FLOAT32 | `float pulseWidth`(**未代入**、下記) |
| range | 32 | FLOAT32 | `float range` |
| rol | 36 | INT32 | `int col` |
| mirrorid | 40 | UINT8 | `unsigned char mirrorid` |

point_step = sizeof(RFANS_XYZ_S) = 41。`memcpy` ゼロコピー詰め込みは正しく機能する。

### 2. col(列番号)式の次元解析と実測整合

`col = hangle / (scanSpeed × LED_num × 360 / PonintFrequency)`(`rfans_driver.cpp:253`)は「水平角 ÷ 1列あたりの角度」で次元的に正しい。実測ログ(15008 点/スキャン、rps=10、LED_num=16)を代入すると 938 列 → 0.384°/列。R-Fans-16 のスペック(単一エコー約 160k 点/秒)とも整合。

### 3. dataID 0x37 対応の適用順序

- LED_num 判定(0x37 → 16 本)は remap **前**の生 dataID で実施(`rfans_driver.cpp:321, 435`)
- `0x37→0x57` remap は `calcXYZ` 直前のローカルコピー `calout.lasShot` にのみ適用(`rfans_driver.cpp:350, 465`)
- → 出力の laserid/hangle は remap の影響を受けない。整合。

### 4. パラメータの一貫性

- col 式の `scanSpeed` とデバイス回転数設定(`prog_Set`)が同じ `rps` パラメータ由来
- `params_3d.yaml` の `use_gps: false` → timeflag に `lasShot.utcTime` を使う分岐と一致
- 角度フィルタ(deg)・レンジフィルタ(m)の単位整合

### 検証不能(スコープ外)

XYZ の三角関数計算(`calcXYZ`/`calcTheta`)はプリビルド `libstar.so` 内。remap 先 0x57 の縦角表 `VAngle_V6B_16G`(±15°/2°間隔)の正しさは `docs/report/2026-06-13_rfans_driver_dataid_0x37_z0_2d.md` の実機確認に依拠。

## 未修正の問題(重要度順)

### 1. `packets.points.at(20)` によるクラッシュリスク

`rfans_driver.cpp:311`(Asyn_date)/ `:425`(syn_date)。起動後最初の 6 バッチの機種判定中に **21 点未満のバッチ**が来ると `std::out_of_range` がキャッチされずノード即死。`syn_date` は `size()>0` しかガードしていない。毎スキャン約 15000 点の実運用では踏まないが、起動直後の端数バッチで踏み得る。修正案: `size() > 20` ガード、または `points.front()` 参照に変更。

### 2. `PonintFrequency` 推定の脆さ(ゼロ除算 + 単位不明)

`rfans_driver.cpp:309`: `PonintFrequency = packet_size.at(5) / (packet_time.at(5) - packet_time.at(4))`。

- バッチ 4/5 の先頭点 `utcTime` が同値ならゼロ除算 → col が inf → int 変換は UB
- `Shot.h:50` の `utcTime` 単位コメントが GBK 文字化けで判読不能。「毫秒(ms)」なら col は 1000 倍ズレる
- **影響は col("rol" フィールド)のみ**。x/y/z には無関係で、slam_toolbox / Nav2 は col を使わないため実害は限定的

### 3. `pulseWidth` は常に 0

`calout_fansxyz`(`rfans_driver.cpp:197`)でこのフィールドだけ一度も代入されない。SDK 側に `m_cPulseWidth` は存在するが未使用(ベンダ由来の欠落)。消費側が pulseWidth を期待すると常に 0。

### 4. フィールド名 "rol"

構造体側は `col` なのに PointCloud2 上の名前は "rol"(typo と思われる)。Velodyne 慣例の "ring" でもないため、`pointcloud_to_laserscan` 等の ring 依存処理には使えない。実質 ring に相当するのは `laserid`。

### 5. 細かい点(記録のみ)

- 空スキャン時に `&temple_1_[0]` / `&data[0]` が空 vector の先頭参照(厳密には UB、実害ほぼなし)
- `timeflag` は float(32bit)なので utcTime が大きい値(epoch 秒等)だと精度不足
- publisher の QoS depth 10000 はメモリ過大
- `flag_simu` の意味が逆転(実機=1、再生=0)— 動作は正しいが紛らわしい
- LED 判定テーブルの syn/Asyn 非対称(0x83 の 32 本判定は Asyn_date のみ)— 再生と実機で判定が異なり得る

## 修正するなら

優先度は (1) `.at(20)` のサイズガード、(2) col 計算のゼロ除算ガード、の 2 点。いずれも submodule `src/external/StarROS2` 側へのコミットが必要。現状の運用(毎スキャン約 15000 点)では顕在化していないため、実機作業のタイミングに合わせて対応すればよい。

## 教訓

- ベンダドライバの検証は「主要経路(x/y/z)」と「副次フィールド(col/pulseWidth 等)」を分けて評価する。副次フィールドの欠陥は下流が使わなければ実害がない
- `vector::at()` はレンジチェック付きだが例外がキャッチされなければ即死する。センサ入力依存のインデックスアクセスにはサイズガードが必須
- closed SDK(.so)を挟む構成では「静的に検証できた範囲」と「実機確認に依拠する範囲」を明示して記録する
