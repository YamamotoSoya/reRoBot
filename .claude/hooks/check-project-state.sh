#!/bin/bash
# claude: Stop hook — 2 つの鮮度チェック。どちらかが古ければ Claude に更新させる。
#  (1) コード領域が docs/claude/PROJECT_STATE.md より後に変更されたまま → PROJECT_STATE を更新
#  (2) 直近コミットが当月の docs/monthly/YYYY_MM.md より 7 日以上新しい (または前月分が未完) → /monthly-changelog
# 比較は mtime ベース: 対象ファイルを更新 (= mtime が新しくなる) すれば次の Stop からは発火しない。
# 2026-10-23: 監視パスを 07-26 再編後の実パスに修正 (旧 `src` `Dockerfile` は存在せず一度も発火していなかった)、
#   (2) を追加、jq 依存を撤廃 (ホストに jq が無く、旧版は入力解析も出力も失敗して無言で終わっていた)。
#   ホストで動く前提: bash / git / python3 (JSON 出力に使用)。

input=$(cat)

# ループガード: この hook の block から続行した直後の Stop では再発火しない
if printf '%s' "$input" | grep -Eq '"stop_hook_active"[[:space:]]*:[[:space:]]*true'; then
  exit 0
fi

cd "${CLAUDE_PROJECT_DIR:-$(git rev-parse --show-toplevel 2>/dev/null)}" || exit 0

reasons=()

# ---------- (1) PROJECT_STATE の鮮度 ----------
STATE="docs/claude/PROJECT_STATE.md"
if [ -f "$STATE" ]; then
  state_time=$(stat -c %Y "$STATE")
  # 「大きな状態変化」とみなす領域 (docs/ と .claude/ は除外)
  code_paths=(ros2_ws_main/src/app ros2_ws_main/src/bringup ros2_ws_main/src/colcon.meta
              ros2_ws_slamtoolbox/src ros2_ws_glim/config ros2_ws_liosam/src
              docker docker-compose.yml scripts tools .gitmodules .mcp.json)
  newest=0
  # 未コミット変更 (untracked 含む) の mtime
  while IFS= read -r f; do
    [ -e "$f" ] || continue
    t=$(stat -c %Y "$f" 2>/dev/null) || continue
    [ "$t" -gt "$newest" ] && newest=$t
  done < <(git status --porcelain -- "${code_paths[@]}" 2>/dev/null | cut -c4- | sed 's/^"//;s/"$//;s/.* -> //')
  # コード領域に最後に触れたコミットの時刻
  last_commit=$(git log -1 --format=%ct -- "${code_paths[@]}" 2>/dev/null)
  [ -n "$last_commit" ] && [ "$last_commit" -gt "$newest" ] && newest=$last_commit
  if [ "$newest" -gt "$state_time" ]; then
    reasons+=("【PROJECT_STATE】コード領域 (ros2_ws_*/src, ros2_ws_glim/config, docker/, scripts/, tools/, .gitmodules) が docs/claude/PROJECT_STATE.md より後に変更されています。「大きな状態変化」(機能追加・重要バグ解決・方針変更) なら /project-state の流儀で該当セクション (§4 到達点・§5 既知の問題・§10 次の作業) を Edit で更新してください。軽微なら冒頭の「最終更新」行の日付だけ今日にすれば止まります。")
  fi
fi

# ---------- (2) 月別変更履歴 docs/monthly/YYYY_MM.md の鮮度 ----------
MONTHLY_DIR="docs/monthly"
if [ -d "$MONTHLY_DIR" ]; then
  cur=$(TZ=Asia/Tokyo date +%Y_%m)
  cur_file="$MONTHLY_DIR/$cur.md"
  cur_mtime=0; [ -f "$cur_file" ] && cur_mtime=$(stat -c %Y "$cur_file")
  first_of_month=$(TZ=Asia/Tokyo date +%Y-%m-01)
  # 当月の最新コミット
  last_cur=$(git log -1 --format=%ct --since="$first_of_month" 2>/dev/null)
  if [ -n "$last_cur" ] && [ $((last_cur - cur_mtime)) -gt $((7 * 86400)) ]; then
    upd='ファイル無し'; [ "$cur_mtime" -gt 0 ] && upd=$(date -d @"$cur_mtime" +%m-%d)
    reasons+=("【月別変更履歴】当月のコミットが $cur_file に 7 日以上反映されていません (最新コミット $(date -d @"$last_cur" +%m-%d)、履歴の更新 $upd)。/monthly-changelog skill の手順で git log と PROJECT_STATE から当月の親子ツリー表を追記し、python3 docs/claude/gen_docs_index.py で INDEX を再生成してください。")
  fi
  # 前月分の締め: 前月最後のコミットより前月ファイルが古ければ (当月 10 日まで催促)
  day=$(TZ=Asia/Tokyo date +%-d)
  if [ "$day" -le 10 ]; then
    prev=$(TZ=Asia/Tokyo date -d "$first_of_month -1 day" +%Y_%m)
    prev_file="$MONTHLY_DIR/$prev.md"
    prev_mtime=0; [ -f "$prev_file" ] && prev_mtime=$(stat -c %Y "$prev_file")
    last_prev=$(git log -1 --format=%ct --before="$first_of_month" 2>/dev/null)
    if [ -n "$last_prev" ] && [ "$last_prev" -gt "$prev_mtime" ]; then
      reasons+=("【月別変更履歴】前月分 $prev_file が前月末のコミット ($(date -d @"$last_prev" +%m-%d)) より古いままです。/monthly-changelog で前月を締めてください (末尾の『参照ドキュメント』と 00_index.md の 1 行も)。")
    fi
  fi
fi

if [ ${#reasons[@]} -gt 0 ]; then
  msg=$(printf '%s\n\n' "${reasons[@]}")
  if command -v python3 >/dev/null 2>&1; then
    python3 -c 'import json,sys; print(json.dumps({"decision":"block","reason":sys.stdin.read().strip()}, ensure_ascii=False))' <<<"$msg"
  else
    # python3 も無い環境向けの簡易エスケープ (改行→空白、" と \ をエスケープ)
    esc=$(printf '%s' "$msg" | tr '\n' ' ' | sed 's/\\/\\\\/g; s/"/\\"/g')
    printf '{"decision":"block","reason":"%s"}\n' "$esc"
  fi
fi
exit 0
