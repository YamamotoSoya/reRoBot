<!-- claude: docs/ 全体の入口 (2026-10-23 作成)。各ディレクトリの役割・編集ルール・索引はここが正。
     CLAUDE.md の docs 表と PROJECT_STATE §8 はここへのポインタ + 編集ルールだけに縮めた。 -->

# docs/ の地図

| 名前 | 説明 |
|---|---|
| **まず読むもの** | |
| ├─ [TODO.md](TODO.md) | 今取り組むこと (ジャンル別、ユーザ更新) |
| ├─ [claude/PROJECT_STATE.md](claude/PROJECT_STATE.md) | 現在地・既知の問題・次の作業 (Claude 用の状態メモ) |
| ├─ [monthly/00_index.md](monthly/00_index.md) | **月別の変更履歴** (2026-03〜)。何を・なぜ変えたかの索引、docs とファイルへのリンク付き |
| └─ [manual/16_software_architecture.md](manual/16_software_architecture.md) | ソフトウェア構成図 (コンテナ・ノード・トピック・TF・設定ファイルの地図) |
| **運用・手順** | |
| ├─ [manual/](manual/00_index.md) | 運用手引き (README の詳細版、17 章)。本文はユーザ執筆 |
| ├─ [../README.md](../README.md) | 最短セットアップ + ハードウェア (udev / ネットワーク) |
| └─ [../tools/README.md](../tools/README.md) | ROS ws 外のツール (地図変換・waypoint・評価・較正) |
| **問題と設計の記録** | |
| ├─ [issue/INDEX.md](issue/INDEX.md) | 調査記録。未解決は `issue/` 直下、解決・前提消失は `issue/closed/`。`*.summary.md` は長い調査の要点版 |
| ├─ [report/INDEX.md](report/INDEX.md) | 解決済みバグの事後報告 (`/debug-report`) |
| ├─ [features/INDEX.md](features/INDEX.md) | 追加機能の設計文書・仕様書 (`/feature-doc`) |
| └─ [reference/](reference/) | 機体外の設定記録 (EPOS Studio の NVM 設定、配線図 mmd、BNO086 開発者への申し送り) |
| **学ぶ** | |
| └─ [text/README.md](text/README.md) | テーマ別解説書 (書籍形式、6 冊)。timestamp / ekf_fusion / ros2_cpp / slam_toolbox / glim / map3d_to_nav2 |
| **ユーザの記録** | |
| ├─ [trial/improvements.md](trial/improvements.md) | 実験走行の改善点メモ (走行ごとに追記、ユーザ執筆)。Claude の分析は [trial/analysis/](trial/analysis/) |
| └─ `monthly/2026_6` など拡張子なし | 月次ゼミ報告 (ユーザ執筆、Claude 編集禁止) |
| **Claude の作業用** | |
| ├─ [claude/handoff/](claude/handoff/) | 別エージェントへの引き継ぎ書 (完了したら消してよい) |
| ├─ [claude/knowledge/](claude/knowledge/) | 実装の理解確認の記録 (`/knowledge-check`) |
| ├─ `claude/USER_LEVEL.md` | ユーザ知識レベル (git 管理外・コミット禁止) |
| └─ [claude/gen_docs_index.py](claude/gen_docs_index.py) | issue / report / features の INDEX と text/README を再生成するスクリプト |

## 編集ルール (要約。正は [CLAUDE.md](../CLAUDE.md))

| 誰が書く | 場所 |
|---|---|
| **ユーザ** (Claude は依頼時のみ) | `TODO.md`、`trial/improvements.md`、`monthly/` の拡張子なしファイル、`manual/` 本文 |
| **Claude** | `claude/`、`issue/`、`report/`、`features/`、`text/`、`monthly/2026_MM.md` + `00_index.md`、各 `INDEX.md` |

## 置き場のルール

- 新しい調査は `issue/YYYY-MM-DD_slug.md` (冒頭にステータス行)。解決したら → ステータス更新 → `issue/closed/` へ `git mv` → `python3 docs/claude/gen_docs_index.py`。
- 画像は本体と同じディレクトリの `img/<本体と同じ slug>/`。
- 日付接頭辞は `YYYY-MM-DD_` で統一 (manual は章番号 `NN_`)。
- 経緯 (いつ・何を・なぜ) は `monthly/2026_MM.md` に書き、PROJECT_STATE には現在地だけを書く (`/monthly-changelog`)。
