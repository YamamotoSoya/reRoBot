<!-- claude: 3D地図→Nav2 接続読本 第5章 (2026-08-17) -->

# 第5章 3D ローカライザへの道 — AMCL を 3D ネイティブに格上げする (未実装)

最短案 (第 3・4 章) は観測を 2D に潰してから解く。本格案は自己位置だけ 3D のまま解く。
この章は本格案の機構・候補パッケージ・reRoBot への統合設計を記す。**2026-08-17 時点で
未実装** — 実装トリガは「別 PC で進行中の高品質地図の確定」である。

## 5.1 scan matching localization の機構

考え方は GLIM のオドメトリと同じ「点群レジストレーション」
([GLIM 読本 第3章](../glim/03_registration.md)) を、蓄積中の地図ではなく
**完成済みの固定地図**に対して行うことである:

```
3D scan matching localization
├── 起動時: 事前地図 (PCD) を全部メモリに載せ、近傍探索構造 (kd-tree / ボクセル) を構築
├── 毎スキャン:
│   ├── 予測: 前回姿勢 + オドメトリ/IMU で初期値を出す
│   ├── 照合: ライブ点群を NDT/GICP で地図に位置合わせ
│   │        「この点群が地図と一番重なる 6 自由度姿勢はどこか」を反復最適化
│   └── 平滑化: UKF/EKF で照合結果とオドメトリを融合 (実装による)
└── 出力: 6DoF 姿勢 → TF map→odom (AMCL の椅子にそのまま座る)
```

AMCL との対比で本質が見える:

| | AMCL (最短案) | 3D scan matching (本格案) |
|---|---|---|
| 仮説の表現 | 粒子 500〜2000 個 (多峰) | 単一姿勢 + 反復最適化 (単峰) |
| 観測 | /scan (帯 1 本の 2D) | 点群全体 (縦の構造も使う) |
| 地図 | 2D 占有格子 | 3D 点群 (PCD) |
| 高さ帯の整合問題 | ある (第4章 §4.3) | **概念ごと消える** |
| 大域リローカライズ | 粒子散布で可能 | 苦手 (初期値必須 = /initialpose) |
| 縮退環境 (長い廊下) | /scan が痩せて弱い | 照合が痩せて弱い (同根) |

「縦の構造も使う」が屋外 (つくば) で効く: 建物の張り出し・木・法面は高さ帯 1 本の
/scan には入らないが、3D 照合では拘束になる。

## 5.2 候補パッケージ (2026-08-17 調査)

```
候補の系譜
├── hdl_localization (koide3) — 本家。UKF + マルチスレッド NDT
│     ❌ ROS 1。Jazzy への自力移植は中規模作業 → 参照実装として使う
├── lidar_localization_ros2 (rsasaki0109) ★推奨
│     ✅ ROS 2 ネイティブ。NDT / GICP / NDT_OMP / GICP_OMP を選択式
│     ✅ /odom・/imu を補助入力にできる — §5.3 の縮退保険がそのまま実現できる
│     ⚠️ Jazzy での動作は要ビルド確認 (Humble 実績が中心)
└── pcl_localization_ros2 (scav-project) — 同系の PCL ベース。第 2 候補
```

- [hdl_localization](https://github.com/koide3/hdl_localization)
- [lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2)
- [pcl_localization_ros2](https://github.com/scav-project/pcl_localization_ros2)

## 5.3 縮退という共通の弱点 — 車輪 odom という保険

忘れてはならないのは、**scan matching は GLIM が苦しんだのと同じ理由で苦しむ**こと。
特徴の乏しい長い廊下では「進行方向のどこにいるか」を点群照合が拘束できない
(`docs/issue/2026-08-12_glim_horizontal_drift.md` の主題 — 主犯は縮退 × 車輪 odom 不使用)。

だから本格案の実装では「/odom を予測に使える実装を選ぶ」が最重要の選定基準になる。
lidar_localization_ros2 を推す理由がこれで、EKF の /odometry/filtered を予測初期値に
食わせれば、照合が痩せた区間は車輪 + IMU が姿勢を持ち堪える。
2D 側で slam_toolbox が車輪 odom prior のおかげで廊下でも綺麗だった事実
([slam_toolbox 読本](../slam_toolbox/00_index.md)) と同じ構図である。

## 5.4 reRoBot への統合設計 (実装時のチェックポイント)

```
統合手順の設計
├── 1. ビルド場所: main ws (ros2_ws_main/src) を第一候補
│     PCL は apt libpcl-dev で入る。ndt_omp 等の追加依存が apt で解決しないときだけ
│     専用薄コンテナへ (glim コンテナは GTSAM 版数の壁で ws を持たない方針)
├── 2. 地図入力: 第3章と同じ PLY→PCD (pcl_ply2pcd) を流用。ダウンサンプルは
│     ローカライザ側パラメータで (照合速度と精度のトレード)
├── 3. TF 出力の確認: map→odom を出す設定になっているか
│     (pose だけ出す実装なら pose→TF 変換の橋ノードが要る — ここが統合の本丸)
├── 4. AMCL の退場: nav2.launch.py の amcl / lifecycle_manager から外した
│     glim ローカライザ版 launch を別ファイルで作る (既存 2D 運用は無傷に保つ)
├── 5. 初期位置: /initialpose (RViz 2D Pose Estimate) を購読できるか。
│     できない実装ならパラメータ初期値で与える
└── 6. 検証順: bag 再生で TF map→odom の連続性 → 実機で静止→直進→旋回 →
      Nav2 接続 (2D Pose Estimate → Goal)
```

## 5.5 最短案と本格案の使い分け (現時点の見立て)

```
運用の見立て
├── 屋内 (9 号館等の閉環境) ....... 最短案で足りる見込み。壁が全周にあり /scan が濃い
├── 屋外 (つくば本番) ............. 本格案が本命。開けた場所で /scan が痩せる +
│     縦構造 (建物・木) を照合に使いたい
└── 移行パス ...................... 最短案で配線を通す (済) → 高品質地図確定 →
      本格案を差し替え (TF 供給者の交代だけなので Nav2 側は無変更)
```

## 5.6 この章のまとめ

```
第5章 まとめ
├── 本格案 = 固定 3D 地図への点群レジストレーション → TF map→odom を供給
├── AMCL 対比: 縦構造が使える・帯整合が不要 / 大域リローカライズは苦手
├── 弱点は GLIM と同根の縮退 → /odom を予測に使える実装 (lidar_localization_ros2) を推奨
├── 統合の本丸は TF map→odom の出力確認と AMCL の外し方 (別 launch で共存)
└── 実装トリガ: 高品質地図の確定。それまで最短案で配線を維持
```

→ [第6章 事例集](06_case_studies.md)
