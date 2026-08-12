<!-- claude: タイムスタンプ解説書 (2026-08-11)。rfans スキャン末尾 stamp バグの解説を
     きっかけに、ROS 2 の時刻の扱いを体系的に学べる教材として書籍化したもの。
     軸足は ROS 2 一般論、事例は reRoBot 実コード。 -->

# ROS 2 タイムスタンプ読本 — reRoBot の実コードで学ぶ時刻設計

センサデータに付くタイムスタンプは「ROS が勝手に付けてくれる何か」ではなく、
**ドライバを書いた人が 1 行のコードで代入する、ただのフィールド**である。
だからこそ間違えられるし、間違えてもエラーは出ず、下流の SLAM やセンサ融合が
静かに狂う。本書は「時刻とは何か」から始めて、スタンプを正しく打つ側・読む側の
両方を、reRoBot で実際に起きた事故を教材にして解説する。

## 本書の構成

```
ROS 2 タイムスタンプ読本
├── 第1章 時刻の基礎 ............... 01_time_fundamentals.md
│   ├── エポックと sec/nanosec
│   ├── クロックの種類 (wall / steady / ROS time / sim time / デバイス時計)
│   └── rclcpp::Time / Duration の演算規則
├── 第2章 Header とメッセージ構造 ... 02_header_and_messages.md
│   ├── std_msgs/Header の意味論 — stamp は「取得時刻」という規約
│   ├── 主要センサメッセージの構造樹形図
│   └── per-point time (PointCloud2) の位置づけ
├── 第3章 スタンプを打つ側の記法 .... 03_stamping_patterns.md
│   ├── テンプレ1: 取得直後に now()
│   ├── テンプレ2: 取得時刻への巻き戻し (now − 所要時間)
│   ├── テンプレ3: デバイス時刻 → ROS 時刻マッピング
│   ├── テンプレ4: 変換・比較・フォールバックの定型
│   └── アンチパターン集
├── 第4章 スタンプを読む側の仕組み .. 04_time_consumers.md
│   ├── TF2 — 時刻指定つき座標変換
│   ├── message_filters — ApproximateTime 同期
│   ├── robot_localization EKF の時刻処理
│   ├── GLIM の点群時刻復元
│   └── use_sim_time と bag 再生
├── 第5章 reRoBot の時刻アーキテクチャ 05_rerobot_architecture.md
│   ├── 設計原則 4 か条
│   ├── 3 つの時刻軸と ROS 時刻軸への写像
│   └── 全トピックの時刻意味論一覧
├── 第6章 事故事例集 ............... 06_case_studies.md
│   ├── 事例A: rfans スキャン末尾スタンプ (2026-08-11 修正)
│   ├── 事例B: float32 に絶対時刻を入れて精度崩壊 (2026-08-01 修正)
│   ├── 事例C: stamp=0 で時刻同期が到着順に退化 (未修正・潜在)
│   ├── 事例D: scan queue full と message age (2026-05-26 解決)
│   └── 事例E: stamp==0 フォールバックの功罪
└── 第7章 チェックリストとデバッグ .. 07_checklist.md
    ├── 実行中スタックの時刻診断コマンド
    ├── bag による時刻整合の検証手順
    └── 新センサ導入時の stamp 検収項目
```

## 読み方

- **初読は 1 → 2 → 3 章を順に**。時刻の器 (第1章)、器の置き場所 (第2章)、
  値の入れ方 (第3章) の順で積み上がる。
- 「なぜスタンプがそんなに大事なのか」を先に知りたければ **第6章の事例Aから**
  読み始めてもよい。必要な前提には各所で章リンクを張ってある。
- 第5章・第7章は reRoBot の開発中に繰り返し参照する実務ページ。

## 凡例

| 記法 | 意味 |
|---|---|
| `path/to/file.cpp:123` | reRoBot リポジトリ内の実コード位置 (行番号は 2026-08-11 時点) |
| ✅ / ⚠️ / ❌ | 正しい設計 / 注意つきで可 / アンチパターン |
| `T_start`, `T_end` | スキャンなど時間幅を持つ計測の開始・終了時刻 |
| mermaid 図 | GitHub / VS Code の Markdown プレビューでそのまま描画される |

## 関連資料 (一次資料)

- `docs/issue/2026-08-01_odometry_stop_wobble_zero_stamp_pairing.md` — 事例Cの調査記録
- `docs/report/2026-05-26_slam_toolbox_scan_queue_full.md` — 事例Dの事後報告
- `docs/claude/PROJECT_STATE.md` タイムライン 08-11 (5) — 事例Aの修正記録
- `docs/claude/knowledge/2026-08-11_ekf_odometry_code_walkthrough.md` — EKF/odometry 実装の解説 (本書第4章と相互補完)
- [EKF センサ融合読本](../ekf_fusion/00_index.md) — 姉妹書 (2026-08-12)。本書が「時刻」、あちらが「値と不確かさ」を担当
