#!/bin/bash
# claude: Stop hook — コード領域が docs/claude/PROJECT_STATE.md より後に
# 変更されたままターンが終わろうとしたら、Claude に更新させる。
# 比較は mtime ベース: PROJECT_STATE.md を更新 (= mtime が新しくなる) すれば
# 次の Stop からは発火しない。

input=$(cat)

# ループガード: この hook の block から続行した直後の Stop では再発火しない
if [ "$(printf '%s' "$input" | jq -r '.stop_hook_active // false')" = "true" ]; then
  exit 0
fi

cd "${CLAUDE_PROJECT_DIR:-$(git rev-parse --show-toplevel 2>/dev/null)}" || exit 0

STATE="docs/claude/PROJECT_STATE.md"
[ -f "$STATE" ] || exit 0
state_time=$(stat -c %Y "$STATE")

# 「大きな状態変化」とみなす領域 (docs/ と .claude/ は除外)
code_paths=(src Dockerfile docker-compose.yml .gitmodules .mcp.json)

newest=0
# 1) 未コミット変更 (untracked 含む) の mtime
while IFS= read -r f; do
  [ -e "$f" ] || continue
  t=$(stat -c %Y "$f" 2>/dev/null) || continue
  [ "$t" -gt "$newest" ] && newest=$t
done < <(git status --porcelain -- "${code_paths[@]}" 2>/dev/null | cut -c4- | sed 's/^"//;s/"$//;s/.* -> //')

# 2) コード領域に最後に触れたコミットの時刻 (コミット済み変更の検知)
last_commit=$(git log -1 --format=%ct -- "${code_paths[@]}" 2>/dev/null)
[ -n "$last_commit" ] && [ "$last_commit" -gt "$newest" ] && newest=$last_commit

if [ "$newest" -gt "$state_time" ]; then
  jq -n '{
    decision: "block",
    reason: "このセッション/リポジトリでコード領域 (src/, Dockerfile, docker-compose.yml, .gitmodules, .mcp.json) が docs/claude/PROJECT_STATE.md より後に変更されています。変更内容が「大きな状態変化」(機能追加・重要バグ解決・方針変更) に該当するなら、/project-state スキルの流儀で PROJECT_STATE.md の該当セクション (到達点・既知の問題・タイムライン・次の作業) を Edit で更新してください。該当しない軽微な変更、または既に内容が最新の場合は、冒頭の「最終更新」行の日付だけ今日に更新して終了してください (これでこのリマインダは止まります)。"
  }'
fi
exit 0
