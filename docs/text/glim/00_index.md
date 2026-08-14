<!-- claude: GLIM 読本 (2026-08-12)。ユーザ依頼「手法・概念とパラメータをできるだけ詳細に図で解説」による書籍化。
     軸足は 3D LiDAR SLAM (LIO) の一般論、事例は reRoBot 実設定 (GLIM 1.2.2 / R-Fans-16 + BNO086)。 -->

# GLIM 読本 — 3D LiDAR SLAM (LIO) の手法と設定

GLIM は「3D 版 slam_toolbox」ではない。自由度は 3 から 6 に増え、重力という新しい基準が
入り、車輪オドメトリという最強の prior を**使わない**。reRoBot では導入初日の評価で
「2D なら綺麗に解けたデータが 3D では水平に流れる」という形でこの違いが表面化した。
本書は 3D SLAM / LIO の理論を土台に、GLIM の 3 層構造と全 15 設定ファイルのパラメータを
「何を変えると何が起きるか」まで下ろして解説する。

## 本書の構成

```
GLIM 読本
├── 第1章 3D SLAM の基礎 ................. 01_3d_slam_basics.md
│   ├── 2D との違い: 6 自由度・重力・動きながらのスキャン
│   ├── LO / LIO / CT-ICP の系譜
│   └── factor graph と固定ラグ平滑化 (iSAM2)
├── 第2章 GLIM の構造 .................... 02_glim_architecture.md
│   ├── 3 層パイプライン: odometry → sub_mapping → global_mapping
│   ├── モジュール .so 差し替え設計 (config.json = 結線表)
│   └── ROS との境界 (glim_ros): topic / TF / 実行ファイル
├── 第3章 点群レジストレーションの仕組み . 03_registration.md
│   ├── ICP → GICP → VGICP の発展
│   ├── iVox — 増分ボクセルの近傍探索
│   ├── IMU プリインテグレーション
│   └── deskew — 動きながらのスキャンの歪み取り
├── 第4章 パラメータ大全 ................. 04_parameters.md
│   ├── config_ros / sensors / preprocess
│   ├── odometry (cpu / ct / gpu)
│   ├── sub_mapping / global_mapping (+pose_graph)
│   └── viewer / logging、上流デフォルト vs reRoBot 差分表
├── 第5章 reRoBot での適用 ............... 05_rerobot_setup.md
│   ├── R-Fans-16 + BNO086 の LIO 構成
│   ├── T_lidar_imu の算出 (URDF からの検算つき)
│   ├── glim_base — TF 衝突を避ける設計
│   └── コンテナ構成と「使えないモジュール」一覧
├── 第6章 事例集 ......................... 06_case_studies.md
│   ├── 事例A: 水平ドリフト (未解決・切り分け 4 段)
│   ├── 事例B: 潜伏していた時刻バグが LIO 切替で発火
│   └── 事例C: 運用の罠 (viewer ハング・dump の読み方)
└── 第7章 実務チェックリスト ............. 07_checklist.md
    ├── 起動手順 (素のコマンド)
    ├── glim_rosbag によるオフライン評価
    ├── CT-ICP 切替レシピ / dump 生成物の読み方
    └── 症状逆引き表・変更時の連動点検
```

## 読み方

- **初読は 1 → 2 → 3 章を順に**。第 4 章のパラメータ表は第 2 章の 3 層構造と
  第 3 章のレジストレーション機構を語彙として使う
- **「なぜ地図が流れたか」から入るなら第 6 章 事例A** — 本書の全章へのリンクを持つ
- **実務で繰り返し参照するのは第 4 章と第 7 章**
- 2D SLAM (slam_toolbox) との対比は姉妹書 [slam_toolbox 読本](../slam_toolbox/00_index.md)
  第 1 章・事例C が前提。時刻・per-point time の話は
  [タイムスタンプ読本](../timestamp/00_index.md) §4.5 が正

## 凡例

| 記法 | 意味 |
|---|---|
| `path/to/file.json:12` | リポジトリ内の実ファイル位置 (行番号は 2026-08-12 時点) |
| 上流デフォルト | 公式イメージ koide3/glim_ros2:jazzy 内の GLIM **1.2.2** 同梱 config の既定値 |
| ✅ / ⚠️ / ❌ | 正しい設計 / 注意つきで可 / アンチパターン |
| mermaid 図 | GitHub / VS Code の Markdown プレビューでそのまま描画される |

## 関連資料 (一次資料)

- `docs/issue/2026-08-12_glim_horizontal_drift.md` — 事例A の完全な調査記録 (PNG 5 枚つき)
- `docs/claude/PROJECT_STATE.md` — GLIM 導入年表 (07-26 採用 → 08-11 LIO 切替 → 08-12 初評価)
- `docs/text/timestamp/04_time_consumers.md` §4.5 / `06_case_studies.md` 事例A・B — GLIM の時刻面
- `docs/report/2026-08-01_rfans_driver_libstar_exception_runtime_sigabrt.md` ほか R-Fans 系 report — 入力側の経緯
- `bags/9goukan/3d_imu/offline/glim/2026-08-12_dump/` — 実行 dump の実物 (第 7 章で読み方)
- 上流: https://github.com/koide3/glim / ドキュメント https://koide3.github.io/glim/
