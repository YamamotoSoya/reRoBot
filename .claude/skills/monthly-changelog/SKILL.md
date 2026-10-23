---
name: monthly-changelog
description: docs/monthly/YYYY_MM.md (Claude 作成の月別変更履歴) に、git log と PROJECT_STATE から当月の「何を・なぜ変えたか」を親子ツリー表で追記し、00_index.md と各 INDEX を更新する。Stop hook が当月コミットの未反映 7 日超・前月分の未締めを催促する。
---

# monthly-changelog — 月別変更履歴の追記

## 目的

継ぎ足し開発でも「いつ・何を・なぜ変えたか」を後から追えるように、`docs/monthly/2026_MM.md` を月ごとの索引として育てる。
PROJECT_STATE.md の §6 タイムラインは 2026-10-23 にここへ移管済み — **新しい出来事はここに書く** (PROJECT_STATE には書かない)。

## いつ動くか

- Stop hook (`.claude/hooks/check-project-state.sh`) が、当月のコミットが `docs/monthly/<当月>.md` より 7 日以上新しいとき、または月初 10 日までに前月分が前月末コミットより古いときに催促する。
- ユーザが `/monthly-changelog` と打ったとき。
- セッションで大きな変更をコミットした直後に、自発的に走らせてもよい (PROJECT_STATE 更新とセット)。

## 手順

1. **対象期間の把握**
   ```bash
   M=2026-10   # 対象月
   git log --reverse --format='%ad %h %s' --date=short --since=$M-01 --until=$(date -d "$M-01 +1 month" +%F)
   ```
   既存ファイル `docs/monthly/${M/-/_}.md` の表を読み、**最後に記載されたコミット短ハッシュ以降**だけを対象にする (重複して書かない)。

2. **一次資料を読む** (この順): 各コミットの `git show --stat <hash>` → `docs/claude/PROJECT_STATE.md` §4/§5/§10 の当月言及 → 当月日付の `docs/issue/` `docs/features/` `docs/report/` (ステータス行と結論)。

3. **親子に分ける**
   - 親 = 大まかな修正・機能追加・調査テーマ (例: 「GLIM z ドリフトの原因究明」)。説明に日付範囲 (MM-DD〜MM-DD) と結論 (棄却された仮説も 1 文)、関連 docs へのリンク。
   - 子 = 個別の変更・判明事項 (例: 「k_correspondences 10→20」)。説明に 1 行 + 変更ファイルへのリンク + コミット短ハッシュ。
   - 深さは最大 3 段。並びは時系列。

4. **書式** (既存ページと同じ。親は太字、罫線は `├─` `└─` `│　`、リンクは `docs/monthly/` 基準の相対パス):
   ```
   | **親 (MM-DD〜MM-DD)** | 何を・なぜ。[issue](../issue/....md) |
   | ├─ 子 | 1 行説明 — [file.yaml](../../ros2_ws_main/src/.../file.yaml) — `abcdef1` |
   | └─ 子 | ... |
   ```
   新しい月は `docs/monthly/2026_MM.md` を新規作成 (冒頭の HTML コメント・`# 2026 年 M 月 — <一行テーマ>`・表・`## 参照ドキュメント`・末尾の前後月ナビは既存ページをコピー)。`00_index.md` の月別表に 1 行足す。

5. **リンク検査と索引再生成**
   ```bash
   python3 docs/claude/gen_docs_index.py        # issue / report / features の INDEX.md と text/README.md
   ```
   リンクの実在は `ls` で確認する (切れリンクを残さない)。移動・改名したファイルは現在地にリンクし「(旧 …)」を添える。

6. **PROJECT_STATE との分担**: PROJECT_STATE は「今どうなっているか」(§4 到達点・§5 既知の問題・§10 次の作業) だけ。経緯はここ。両方に同じ文を書かない。

## 禁止

- ユーザ執筆の月次ゼミ報告 (拡張子なし `docs/monthly/2026_6` など) は編集しない。
- 推測で変更理由を書かない。コミットメッセージと docs から読めることだけ書き、不明なら「(理由の記録なし)」と明記する。
- `docs/trial/improvements.md` は読み取り専用 (そこから引用はしてよい)。
