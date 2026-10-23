<!-- claude: docs/monthly/ の入口 (2026-10-23 作成、ユーザ依頼)。
     このディレクトリには 2 種類のファイルが同居する:
       (1) ユーザ執筆の月次ゼミ報告 — 拡張子なし (例: 2026_6)。Claude は編集しない。
       (2) Claude 作成の変更履歴 — 2026_MM.md (ゼロ埋め)。月ごとの「何を・なぜ変えたか」を
           親子関係つきツリー表にまとめ、docs/ 内の該当文書と変更ファイルへのリンクを張った索引。
     (2) を更新するときは git log と PROJECT_STATE タイムラインを一次資料にする。 -->

# 変更履歴 (月別) — reRoBot

プロジェクト開始 (2026-03、リポジトリ新設) から現在までに「何を・なぜ」変えたかを月ごとにまとめた索引。
各月のページは **親行 = 大まかな修正・機能追加・調査テーマ**、**子行 (`├─` `└─`) = 個別の変更や判明事項** のツリー表で、
説明欄から関連する調査記録 (`docs/issue/`)・事後報告 (`docs/report/`)・設計文書 (`docs/features/`)・変更したファイルへ直接飛べる。

## 月別ページ

| 名前 | 説明 |
|---|---|
| **2026 年** | |
| ├─ [3 月](2026_03.md) | リポジトリ新設 (コード・環境はまだ無し) |
| ├─ [4 月](2026_04.md) | Docker 環境と EPOS4 モータ制御の基礎 — epos4_controller / odometry / teleop / bringup launch / URDF |
| ├─ [5 月](2026_05.md) | 2D LiDAR (HOKUYO) 統合、rerobot_bringup 分離、slam_toolbox「queue is full」解決 |
| ├─ [6 月](2026_06.md) | 片輪不動の根治・脱力モード・左右 swap・Nav2 自律走行成功・R-Fans-16 移植・joy teleop。ユーザ執筆の [6 月進捗](2026_6) はこちら |
| ├─ [7 月](2026_07.md) | リポジトリ監査 → Docker 4 コンテナ再編、エンコーダ 4 倍ズレの根本修正、LIO-SAM / RealSense、Claude 作業基盤 |
| ├─ [8 月](2026_08.md) | GLIM 導入とドリフト調査、BNO086 IMU + EKF、R-Fans ドライバ刷新、取付プリセット、3D 地図→Nav2、車体改修 |
| ├─ [9 月](2026_09.md) | GLIM z ドリフトの正体 (GICP、k_correspondences 10→20)、EPOS 停止調査と watchdog、BNO086 auto_tare と再較正、R-Fans stamp 修正 |
| └─ [10 月](2026_10.md) | つくば本番へ向けた実戦化 — 全部入り bringup、3D 自己位置推定、twist_mux + パッド運用、追従 BT、実走 bag 解析、EPOS Fault 0x8250 |

## 読み方

- **何が今の仕様を決めたか**を知りたい → 該当する月の親行から子行のリンク (issue / features) へ。例: 「なぜ `gear_ratio` が 5.0 → 92.25 なのか」は [7 月](2026_07.md) (4 倍ズレ修正) → [8 月](2026_08.md) (車体改修)。
- **棄却された仮説**も親行の説明に残している (例: 9 月「縦角表が本命」の棄却)。同じ仮説を再調査しないための記録。
- 行末の `` `abcdef1` `` は git の短ハッシュ。`git show abcdef1 --stat` で当時の変更ファイルが見られる。
- 現在地・既知の問題・次の作業は [docs/claude/PROJECT_STATE.md](../claude/PROJECT_STATE.md)、運用手順は [docs/manual/](../manual/00_index.md)、ソフトウェア構成図は [manual 第16章](../manual/16_software_architecture.md)。

## このディレクトリの規約

| ファイル | 書き手 | 内容 | 更新 |
|---|---|---|---|
| `2026_6` (拡張子なし) | ユーザ | 月次ゼミ報告 (一次資料) | Claude は編集しない |
| `2026_MM.md` | Claude | 変更履歴 (本索引の各ページ) | `/monthly-changelog` skill で git log + PROJECT_STATE から追記。Stop hook (`.claude/hooks/check-project-state.sh`) が「当月コミットの未反映 7 日超」「月初 10 日までの前月分未締め」を自動で催促する (2026-10-23) |
| `00_index.md` | Claude | この目次 | 月を追加したら 1 行足す |
