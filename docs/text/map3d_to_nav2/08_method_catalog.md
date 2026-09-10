<!-- claude: 2026-09-04 追加。2026-09-03 の Web 網羅調査 (3 並列: 投影手法 / 2.5D 表現 / 3D 定位)
     を手法カタログとして章化。本文 1〜7 章が「reRoBot の 1 実装の深掘り」なのに対し、
     本章は「世の中の設計空間の全体地図」を担う参照章。-->

# 第8章 手法カタログ — 3D→2D 圧縮と定位の設計空間 (2026-09-03 調査)

第 1〜7 章は reRoBot が選んだ 1 本の道 (高さスライス + /scan 化 + AMCL) を深掘りした。
本章は視野を広げ、**世の中で考案されている手法の全体地図**を示す。目的は 2 つ:
今の実装が設計空間のどこに立っているかを知ること、そして次の一手の候補を
「思いつき」でなく「カタログからの選択」にすることである。

出典はすべて 2026-09-03 時点の Web 調査 (論文・GitHub・つくばチャレンジ参加チームの
技術記事)。リンク切れ・バージョン変化はあり得る前提で読むこと。

## 8.1 設計空間の全体樹形図 — どこで 3D を捨てるか

あらゆる手法は「**パイプラインのどの段階で 3D 情報を捨てるか**」で分類できる。
早く捨てるほど軽く単純に、遅く捨てるほど坂・植生・季節変化に頑健になる。

```
3D 地図 → 2D ナビの設計空間
├── 【I】地図をオフラインで 2D 占有格子に圧縮 (定位も計画も 2D)
│   ├── I-1 絶対 z 高さスライス ............ pointcloud_to_2dmap ★reRoBot 採用済 (第3章 §3.2)
│   ├── I-2 軌跡ベース (センサ相対) スライス .. glim_dump_to_2dmap ★reRoBot 自作済 (第3章 §3.4)
│   ├── I-3 OctoMap projected_map .......... 自由空間をレイキャストで確定できる唯一の系統
│   ├── I-4 セル内 min/max 高低差 .......... 坂に強い・縁石が見える
│   ├── I-5 地面分離ファースト ............. RANSAC / Patchwork++ / CSF / TRAVEL
│   ├── I-6 法線ベース (鉛直面のみ抽出) ..... 「2D LiDAR が見る壁」だけの地図
│   ├── I-7 密度・点数しきい値 ............. 単独では地図にならない (併用部品)
│   └── I-8 自由空間投影・多層スライス ...... 坂・立体交差の本命 (LTU-RAI, RA-L 2024)
├── 【II】2.5D 中間表現を挟む (完全には潰さない)
│   ├── II-1 標高地図 ...................... grid_map + elevation_mapping(_cupy)
│   ├── II-2 MLS 地図 (セルに複数の面) ...... 橋・高架下対応 (Triebel 2006)
│   ├── II-3 走行可能性 (traversability) 地図 . 傾斜・段差・粗さ → コスト値
│   └── II-4 メッシュナビ (2D 化しない対抗馬) . mesh_navigation (MBF 系)
├── 【III】定位だけ 3D のまま残す (2D 地図は計画専用) ← つくば上位の主流
│   ├── III-1 NDT/GICP 追跡型 .............. lidar_localization_ros2 / hdl_localization / DLL
│   ├── III-2 3D パーティクルフィルタ型 ..... mcl_3dl / mcl3d_ros
│   ├── III-3 LIO + 地図照合の密結合 ....... FAST-LIO-Localization / GLIL (ICRA 2024)
│   └── III-4 仮想 2D スキャン (ハイブリッド) . 3D 地図から擬似 /scan をレイキャスト合成
└── 【IV】観測側の 2D 化 ................... pointcloud_to_laserscan ★reRoBot 採用済 (第4章)
```

reRoBot の現行構成 (I-1/I-2 + IV + AMCL) は【I】+【IV】の組み合わせ。第 5 章の
「本格案」は【III】への移行にあたる。

## 8.2 系統 I — オフライン 3D→2D 地図圧縮

### I-1 絶対 z 高さスライス

z ∈ [min, max] の点だけ XY に投影する最も素朴な方式。第 3 章 §3.2 で解剖した
[pointcloud_to_2dmap](https://github.com/koide3/pointcloud_to_2dmap) のほか、
PassThrough + 外れ値除去の [pcd2pgm ROS2 版](https://github.com/LihanChen2004/pcd2pgm)、
つくば常連 AbudoriLab 製の
[2dmap_generator_from_pointcloud](https://github.com/abudori3939/2dmap_generator_from_pointcloud)
(Nav2 用 pgm/yaml 出力 + free space 充填のモルフォロジー後処理付き) がある。

- 弱点: スライス帯が**地図座標の絶対 z** なので、坂で帯が地面に食い込む/低障害物を
  取り逃す。z ドリフト地図では壁が帯から外れて消える (第 3 章 §3.4 の実測どおり)

### I-2 軌跡ベース (センサ相対) スライス

SLAM のキーフレーム姿勢列 = 「ロボットが実際に通った場所 = 確実に地面」をアンカーに、
**各キーフレーム座標系で相対高さスライスしてから map 座標へ合成**する。帯が地形と
z ドリフトに追従する。`glim_dump_to_2dmap` (第 3 章 §3.4) がこの方式で、
公開単体ツールは乏しく各チーム内製が主流。論文レベルで最も近い公開記述は
[Mentasti et al., arXiv:2403.13431](https://arxiv.org/html/2403.13431v1)
(走行軌跡近傍セルの強制通行可能化 + 軌跡参照の走行可能性判定)。

- 弱点: 軌跡から遠い領域は最寄りキーフレーム高の外挿になる (土手の上下で誤る)。
  キーフレームのロール/ピッチ誤差がそのまま帯の傾きになる

#### 2026-09-10 追補 — センサ相対の公開実装を探し直した結果

「同じセンサ基準で切る既製ツールは無いのか」を再調査した (Web、2026-09-10)。結論は
**GLIM 出力をオフラインで軌跡相対にスライスする単独ツールは依然として未確認**。ただし
同じ思想の実装と、問題を SLAM 側で潰す別解が見つかった。

| 実装 | 入力 | 高さ基準 | z ドリフト追従 | ROS 2 | 備考 |
|---|---|---|---|---|---|
| [RTAB-Map](https://index.ros.org/p/rtabmap_ros/) `Grid/*` | SLAM ノード毎の点群 | **ロボットフレーム相対** (`Grid/MaxGroundHeight` は "relative to the robot frame"、`Grid/NormalsSegmentation` 併用可) | ○ — ノード毎に局所格子を作り、最適化後の姿勢で合成 | Jazzy 0.23.7 | glim_dump_to_2dmap と設計思想が同じ唯一の既製品。ただしオンライン SLAM 内蔵で GLIM 地図には適用できない |
| [glim_ext](https://github.com/koide3/glim_ext) `flat_earther` | GLIM 内部 (submap) | — (近接 submap の高さを揃える拘束) | **SLAM 側で z ドリフトを潰す** | GLIM 拡張 | 単一フロア前提。有効なら絶対 z スライス (I-1) がそのまま使える。**未検証 — 5号館 bag で試す価値あり** |
| [LTU-RAI Map-Conversion](https://github.com/LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map) | OctoMap/UFOMap | 自由空間から床高を推定 | ○ (床追従) | ROS 2 ブランチ | I-8。OctoMap 化が前段に要る |
| octomap_server `filter_ground_plane` | PointCloud2 逐次 | 地面除去は base frame の RANSAC (センサ相対)、`projected_map` は絶対 z | 地面除去のみ ○ | Jazzy 2.3.1 | 投影が絶対 z なので目的には届かない |
| [pcd2pgm](https://github.com/LihanChen2004/pcd2pgm) / [pointcloud_to_grid](https://github.com/jkk-research/pointcloud_to_grid) / [nav2-keepout-zone-map-creator](https://github.com/CyberAgentAILab/nav2-keepout-zone-map-creator) | PCD / PointCloud2 | 絶対 z | × | Humble 系 | I-1 の亜種。pcd2pgm は半径外れ値除去付き |
| hdl_graph_slam / interactive_slam | — | 2D 出力なし | — | ROS 1 | [issue #219](https://github.com/koide3/hdl_graph_slam/issues/219) で「octomap 投影がループ閉じ後にずれる」が未解決のまま |

読み取り:

- 自作方式 (submap 原点相対) は RTAB-Map の「ノード毎ロボット相対格子 → 最適化姿勢で合成」と
  一致しており、設計として妥当。オフライン版が公開されていないだけ
- **問題を変換側でなく SLAM 側で消す**選択肢が glim_ext `flat_earther`。5号館の z ドーム
  (+4.9 m) が「近接 submap の高さを揃える」拘束で潰れるなら、第 3 章 §3.4 の機構 (3)
  (submap 姿勢の 4° ピッチ) も同時に軽くなる可能性がある。ただし坂のある屋外コースでは
  単一フロア前提が破れるので、つくば本番向きではなく校内実験向き
- 地面分割系 (Patchwork++ / linefit の ROS 2 移植) は**スキャン単位 (センサ中心) 前提**で、
  蓄積済み地図にそのまま掛けられない。使うなら glim_dump_to_2dmap と同様に submap 単位で
  回す設計になる (= 自作ツールの拡張として実装する形)

#### 「VQ 圧縮」について (用語の整理)

VQ (ベクトル量子化) は点群やその特徴をコードブック上の代表ベクトルの添字に置き換える
**保存容量の圧縮**であり、学習ベース (3QNet 等) と古典系 (PCL `OctreePointCloudCompression`、
Google Draco、MPEG G-PCC) がある。いずれも 3D→2D 変換 (本章の「圧縮」= 次元削減) とは
別レイヤで、占有格子の作成には寄与しない (量子化で壁が太る/欠ける副作用はある)。
ロボティクス文脈で「Voxel Quantization」という定訳は見つからず、GLIM が使う VGICP の
ボクセル化か、submap の `submap_downsample_resolution` (ボクセルダウンサンプル) を指して
いた可能性が高い。後者は第 3 章 §3.4 で見たとおり 2D 地図の濃さを直接支配している。

### I-3 OctoMap projected_map

点群を確率的オクツリーに統合し、z 帯に占有ボクセルがあるセルを占有として
`projected_map` (OccupancyGrid) を出す。**観測レイから自由空間が確率的に確定する
唯一の系統** — PCD 一発変換系は「点が無い = 未知」としか言えないが、こちらは
「レイが通り抜けた = 自由」を証明できる。AMCL の尤度場モデルでは自由/未知の
区別の実害は小さいが (第 4 章)、ビームモデルや経路計画では効く。

- 実装: [octomap_mapping](https://github.com/OctoMap/octomap_mapping) /
  [ROS2 移植 octomap_server2](https://github.com/iKrishneel/octomap_server2) /
  [.bt を任意高さでスライスする補助ツール](https://github.com/youliangtan/3D_Slam_tools)
- 弱点: 完成済み静止点群にはレイ情報が無いので効果半減 (第 3 章 §3.5 で却下した
  理由と同じ)。z 帯はやはり絶対高さ。大規模屋外ではオクツリー更新が重い

### I-4 セル内 min/max 高低差

セルごとの最低高と最高高の**差**が閾値 (乗り越え能力 ≈ 5〜10 cm) を超えたら障害物と
する。絶対 z に依存しないので**坂に本質的に強く、縁石・段差が見える** — 高さスライス
では原理的に見えない低障害物を拾える。既製ルートは
[grid_map_pcl](https://github.com/ANYbotics/grid_map/blob/master/grid_map_pcl/README.md)
(PCD → 標高地図 → `toOccupancyGrid`)。

- 弱点: 樹冠の下のセルは「地面点 + 枝点」で高低差が出て**頭上障害物が誤検出**される。
  高さ上限クリップ (下記の頭上クリアランス) との併用が実質必須 — つくばの並木道では特に

### I-5 地面分離ファースト

先に地面点群を分離除去し、残った非地面点を投影する 2 段構成。地面推定器の選択肢:

| 手法 | 中核アイデア | 大域地図に使えるか | 弱点 |
|---|---|---|---|
| RANSAC 平面 (PCL 標準) | 支配平面を推定し inlier を地面とみなす | △ 区画分割すれば | 単一平面仮定が坂・多段地面で崩壊。建物壁面を地面と誤認する事故も |
| [linefit (Himmelsbach)](https://github.com/baiyeweiguang/linefit_ground_segmentation_ros2) | 角度セクタ内の折れ線フィット | ✗ センサ高さ前提 | スキャン単位の前処理専用 |
| [Patchwork++](https://github.com/url-kaist/patchwork-plusplus) (IROS 2022) | 同心円区画ごとの地面尤度付き平面フィット。ほぼ無調整 | △ キーフレーム毎適用 | センサ中心の区画設計。屋外の坂・多段地面では現状最強クラス |
| [CSF 布シミュレーション](https://github.com/jianboqi/CSF) | 反転点群に仮想布を落とし張り付いた面 = 地形 | **✓ そのまま使える** | 布解像度より小さい縁石を均す。多層地形は下層のみ |
| [TRAVEL](https://github.com/url-kaist/TRAVEL) (RA-L 2022) | グラフで地面分離と走行可能性を同時判定 | ✗ スキャン単位 | パラメータが環境依存 |

蓄積済みの GLIM 地図に直接使えるのは CSF だけ。Patchwork++ を使うなら
キーフレーム点群ごとに適用してから合成する (I-2 と同じ分割統治)。

### I-6 法線ベース — 鉛直面 = 障害物

各点の近傍に PCA をかけて法線 (その点が乗る面の向き) を推定し、**法線が水平な点 =
鉛直面 = 壁だけ**を投影する。2D LiDAR が実際に見るのはまさに鉛直面なので、
**AMCL / スキャンマッチングとの整合が理論上最良**の地図になる。坂は法線が緩く
傾くだけなので障害物化しない。パイプライン全体の最詳の公開例は
[arXiv:2403.13431](https://arxiv.org/html/2403.13431v1) (法線分類 + 「走行経路と連結した
地面だけ採用」で屋根などの孤立水平面を棄却 + 頭上クリアランス除去)。

- 弱点: 法線推定は点密度と近傍半径に敏感。植生は法線がランダム化して取り漏らしと
  過検出の両方が起きる。近傍探索のぶん計算コストは高め

### I-7 密度・点数しきい値 (併用部品)

セル内点数で占有判定する。壁は複数スキャンリングが同一 XY セルに縦に積み重なるので
高密度になる性質を利用。pointcloud_to_2dmap の `min/max_points_in_pix` がこれで、
浮遊ノイズ・まばらな葉の抑制として**どの手法とも併用する価値がある**。

- 弱点: SLAM 地図の密度は**センサの滞在時間に依存する** — 長く止まった場所は濃く、
  通過しただけの区間は薄い。一様しきい値は成立しないので、ボクセルダウンサンプリングで
  密度を正規化してから使うのが定石

### I-8 自由空間投影・多層スライス — 坂・立体交差の本命

- **自由空間投影** ([LTU-RAI Map Conversion](https://github.com/LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map),
  RA-L 2024, [arXiv:2406.07270](https://arxiv.org/abs/2406.07270)):
  固定 z 帯を投影する代わりに、ボクセル地図の**自由空間の連続鉛直区間** (床から天井までの
  高さ) をセルごとに解析して 2D 化し、近傍セルの床高から局所勾配も推定して急斜面を
  占有にする。「標高が変化する環境で固定スライスより本質的に頑健」を実証しており、
  **公開実装における glim_dump_to_2dmap の上位互換**にあたる。入力が OctoMap/UFOMap
  前提なのが導入コスト
- **多層 2D マップ**: z 帯を複数に分けて層別地図を作り走行領域で切替える
  ([octomap_server 改造例](https://answers.ros.org/question/297045/))。立体交差を 2D
  スタックのまま扱える唯一の道だが、「今どの層にいるか」を定位側で解く必要があり
  運用が複雑。スロープで層が連続する場所は分割不能

### 系統 I の総括 — 文献が一致する 2 つの定石

1. **単一手法では完結しない**。実用パイプラインは
   「(a) 地面高の基準決め (軌跡 or 地面分離) → (b) 相対高さ帯スライス →
   (c) 密度しきい値でノイズ抑制 → (d) 頭上クリアランスで樹冠除去」の 4 段合成
2. **同じ 3D 地図から目的別に 2 枚焼き分ける**: 定位用 (腰高帯の鉛直面だけ —
   /scan と整合) と計画用 (縁石など低障害物込み — 車体が当たるもの全部)。
   第 3〜4 章の帯整合問題の発展形で、「1 枚の 2D 地図に両方の役割を負わせない」
   という分離が本質

## 8.3 系統 II — 2.5D 中間表現

| 表現 | 中身 | 実装 | reRoBot 視点の評価 |
|---|---|---|---|
| 標高地図 | セル = 高さ + 分散 (+ 法線・色などのレイヤ) | [grid_map](https://github.com/ANYbotics/grid_map) (Jazzy ブランチあり) + [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) / GPU 版 [cupy](https://github.com/leggedrobotics/elevation_mapping_cupy) | ローカル地図前提。グローバル 2D 地図の代替ではなく **local costmap の高度化**向き |
| MLS 地図 | セルごとに複数の面パッチ (橋の上下を区別) | [Triebel et al. 2006](https://cvg.cit.tum.de/_media/spezial/bib/triebel06multi.pdf)。ROS 2 のメンテ実装なし | つくばコース程度の立体交差なら不要 |
| 走行可能性地図 | 傾斜 `acos(normal_z)`・段差・粗さ → コスト [0,1] | [traversability_estimation](https://github.com/leggedrobotics/traversability_estimation) (IROS 2016, 教科書実装) / [traversability_mapping](https://github.com/TixiaoShan/traversability_mapping) (BGK 補間) / [CMU terrain_analysis](https://github.com/jizhang-cmu/ground_based_autonomy_basic) (SubT 実戦) / 学習系 [GA-Nav](https://github.com/rayguan97/GANav-offroad) ほか | 幾何法の本質的限界 =「通れる草と通れない縁石は幾何が同じ」。これを超えたい時だけ学習系。サーベイは [arXiv:2204.10883](https://arxiv.org/pdf/2204.10883) |
| メッシュナビ | 三角メッシュ上のコストレイヤ + 連続ベクトル場プランナ | [mesh_navigation](https://github.com/naturerobots/mesh_navigation) (ICRA 2021, ROS 2 対応) | 2D 圧縮の情報損失自体を回避する路線だが Move Base Flex スタック — Nav2 資産と二本立てになる。平坦主体 + 差動二輪にはオーバーキル |

ランタイムで 3D を Nav2 costmap に注入する口としては
[STVL (SpatioTemporalVoxelLayer)](https://github.com/SteveMacenski/spatio_temporal_voxel_layer)
(OpenVDB スパースボクセル + 時間減衰。voxel_layer 比 CPU ~1/4 で、3D LiDAR を
local costmap に食わせる定番) がある。⚠️ Nav2 標準 costmap の点群高さ閾値は
**世界フレーム基準**なので、起伏コースでは地面が障害物化する既知の統合課題に注意
(観測側で相対高さフィルタを済ませてから渡すのが安全 — 第 4 章の /scan 化は
これを満たしている)。

## 8.4 系統 III — 定位だけ 3D のまま残す

Nav2 は「誰かが map→odom TF を出せばよい」設計なので、AMCL を起動せず外部 3D
ローカライザに TF を出させ、map_server には**計画専用の 2D 地図**を配る構成が成立する
([Nav2 公式 GPS チュートリアル](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)
が AMCL 以外が TF を出す実例)。keepout フィルタも無変更で載る。第 5 章 §5.4 の
統合設計はこの系統の実装計画にあたる。

### なぜ主流化したか — 投影 2D 地図 + AMCL の本質的弱点

1. **視点不一致**: 投影地図は「高さ帯の合成」であり、実 2D スキャンが見る 1 断面と
   一致しない。草木・ベンチ・傾斜地で地図上の壁と実測の壁がずれ、尤度が崩れる
2. **屋外の断面の乏しさ**: センサ高さの水平断面には特徴が少なく (開けた広場・並木)、
   粒子が縦方向に滑る
3. **季節・動的変化**: 植生・駐車車両で 2D 断面は大きく変わる。3D の建物上部形状の
   方がはるかに安定

### 実装カタログ

| 実装 | 方式 | ROS 2 | 負荷 | 備考 |
|---|---|---|---|---|
| [lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2) | NDT/GICP 追跡 | **✓ ネイティブ** | 中 | **AbudoriLab が GLIM 地図 + MID-360 でつくば全コース成功** ([記事](https://www.abudorilab.com/entry/2025/02/25/232101): NDT_OMP, resolution 2.0, 斜面は車輪 odom 拘束で安定化)。第 5 章 §5.2 の★推奨と一致 |
| [hdl_localization](https://github.com/koide3/hdl_localization) | NDT + UKF (IMU 融合) | ✗ (非公式 fork のみ) | 中 | 実績最多。大域初期化は [hdl_global_localization](https://github.com/koide3/hdl_global_localization) が対 |
| Autoware [ndt_scan_matcher](https://autowarefoundation.github.io/autoware.universe_planning/pr-5583/localization/ndt_scan_matcher/) | NDT + EKF | ✓ | 中〜大 | 最も枯れているが Autoware のフレーム/メッセージ規約ごと持ち込むコスト大 |
| [mcl_3dl](https://github.com/at-wat/mcl_3dl) | 3D パーティクルフィルタ | ✗ | 小〜中 | PF なので誤収束耐性が追跡型より高い。千葉工大 RDC-Lab がつくばで使用 |
| [mcl3d_ros](https://github.com/NaokiAkai/mcl3d_ros) (赤井直紀) | 距離場 + PF に最適化を重点サンプリング融合 | ✗ | **小 (CPU 1 スレッド実証)** | 頑健さと軽さの両立 ([arXiv:2303.00216](https://arxiv.org/abs/2303.00216)) |
| [DLL](https://github.com/robotics-upo/dll) | 距離場への直接最適化 | ✗ | 小 | 追跡のみ。大域初期化・誤収束回復なし |
| [FAST_LIO_LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION) / [FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2) | LIO + 低頻度地図照合 | ✓ (後者) | 中 | IMU 必須 (BNO086 で条件は満たす)。斜面・激しい動きに強い |
| **GLIL** (GLIM 作者, [ICRA 2024](https://arxiv.org/abs/2402.05540)) | 因子グラフで LIO + scan-to-map 密結合 | — | 中 | **クローズド**。GLIM 公式に localization モードは無い ([issue #158](https://github.com/koide3/glim/issues/158) で要望のみ)。非公式 fork [glim_localization](https://github.com/se7oluti0n/glim_localization) は品質未検証 |
| [3D-BBS](https://arxiv.org/abs/2310.10023) (koide3 研) | Branch-and-Bound 大域初期化 | GPU | — | 「初期位置合わせ」専用部品。hdl_global_localization の後継格 |

### 2D MCL 側の改良版 (系統 I の地図で戦い続ける場合)

- [emcl2_ros2](https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2) (上田隆一) —
  膨張リセット付き MCL。「レーザが占有セルを貫通する粒子の率」で誤収束を検知して
  自動リセット。つくば実績多数で、AMCL の置き換え先として最小コスト
- [als_ros](https://github.com/NaokiAkai/als_ros) (赤井直紀) — 姿勢と同時に
  **推定の信頼度そのもの**を推定する MCL ([ROS2 移植](https://github.com/iASL-Gifu/als_ros2))。
  誤収束の検知・再定位トリガに使える。体系書『[LiDAR を用いた高度自己位置推定システム](https://www.coronasha.co.jp/np/isbn/9784339032406/)』(コロナ社 2022) の付随実装

### ハイブリッド — 仮想 2D スキャン

粒子姿勢ごとに 3D 地図をレイキャストして「その姿勢・そのセンサ高さで見えるはずの
2D スキャン」を合成し実スキャンと照合する古典
([Bonn 系の研究](https://www.researchgate.net/publication/235687374_USING_3D_DATA_FOR_MONTE_CARLO_LOCALIZATION_IN_COMPLEX_INDOOR_ENVIRONMENTS))。
投影で生じる視点不一致 (8.4 冒頭の弱点 1) を原理的に解消するが、完成した ROS 2
パッケージは無い。簡易版は「2D 地図をライブ /scan と同じ高さ帯の**薄い**スライスで
焼く」— glim_dump_to_2dmap の帯整合運用の延長線にある。学習系 (OverlapNet-MCL,
[range-mcl](https://github.com/PRBonn/range-mcl)) は GPU 前提の研究実装で現時点は選外。

## 8.5 つくばチャレンジの実勢 (2024)

[原祥尭氏の技術調査](https://www.docswell.com/s/ystk_hara/Z22VL9-tsukuba-challenge-2024-survey)
より:

- 地図は **3D メトリック地図 41 チーム vs 2D 35 チームで 3D が逆転**
- SLAM は **GLIM が最多** (Cartographer / SLAM Toolbox / GMapping が続く)
- ナビゲーションは **Nav2 が初めて ROS 1 Navigation を超過**
- 定位は Laser Bayes Filter (MCL 系) 21 / Laser Scan Matching (NDT 系) 16 /
  GNSS 併用 48 チーム

つまり「GLIM で 3D 地図 → 2D 地図化 → Nav2」は reRoBot 固有の構成ではなく
**今のボリュームゾーン**であり、上位チームの定番は「定位は 3D 地図のまま (系統 III)、
2D 地図は計画専用」の分離型。国内の地図変換手法は (1) 高さスライス 2 値化、
(2) 最低点 + 段差閾値、(3) pointcloud_to_laserscan 併用の 3 パターンが支配的で、
傾斜・粗さまでコスト化する流儀 (系統 II) はまだ少数派。

参考になる同型事例:

- **AbudoriLab**: GLIM 2.8 km 地図 ([記事](https://www.abudorilab.com/entry/2024/08/26/214525))
  + lidar_localization_ros2 + 自作 2D 地図生成 + Nav2。
  「GLIM は地図作成用で、そのまま定位には使えない」ことを産総研研究者との対話で確認したと
  [シンポジウム参加記](https://www.abudorilab.com/entry/2025/02/20/001115)に記載
- **チーム plat** (2022 完走): LeGO-LOAM 地図 + hdl_localization
  ([機体解説](https://zenn.dev/tamago117/articles/58001ff8399cc7))

## 8.6 reRoBot の現在地と次の一手

```
設計空間上の reRoBot (2026-09 時点)
├── 実装済
│   ├── I-1 絶対 z スライス (pointcloud_to_2dmap) ....... 第3章 §3.2
│   ├── I-2 センサ相対スライス (glim_dump_to_2dmap) ..... 第3章 §3.4 — 公開ツールに代替が乏しい妥当な内製
│   └── IV  /scan 化 (rfans_scan.launch.py) ............ 第4章
├── 設計済・未実装
│   └── III-1 lidar_localization_ros2 への AMCL 置換 .... 第5章 §5.4 — AbudoriLab の実走実績で裏付け強化
└── カタログからの増分候補 (優先順)
    ├── ① 地図の 2 枚焼き分け (§8.2 定石 2) — 定位用 (腰高鉛直面) と計画用 (I-4 の
    │      min/max 高低差で縁石込み + 頭上クリアランス)。glim_dump_to_2dmap への増分で到達可能
    ├── ② emcl2_ros2 — AMCL のまま戦う場合の最小コスト強化 (誤収束の自動検知)
    ├── ③ I-8 自由空間投影 (LTU-RAI) — 坂・立体交差が実害化したら。OctoMap 化の前処理が必要
    └── ④ URG + emcl2 を監視系として併走 — 3D 定位移行後の二重化 (つくば的常套手段)
```

## 8.7 この章のまとめ

- 設計空間は「**どこで 3D を捨てるか**」の 1 軸で整理できる: 地図で捨てる【I】、
  半分残す【II】、定位では捨てない【III】、観測で捨てる【IV】
- 系統 I の定石は「地面基準決め → 相対スライス → 密度しきい値 → 頭上クリアランス」の
  4 段合成と、定位用/計画用の **2 枚焼き分け**
- 投影 2D 地図 + AMCL の弱点 (視点不一致・断面の乏しさ・季節変化) が、つくば上位を
  「定位は 3D のまま」(系統 III) へ動かした。ROS 2 ネイティブで GLIM 地図の実走実績が
  あるのは lidar_localization_ros2 — 第 5 章の推奨はカタログ全体で見ても妥当
- reRoBot の現行実装 (I-1/I-2 + IV) は国内の最普及構成そのもの。次の増分は
  「地図の 2 枚焼き分け」か「3D 定位への移行」のどちらかで、両者は排他ではない

→ [00_index に戻る](00_index.md)
